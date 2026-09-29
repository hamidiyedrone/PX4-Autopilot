#!/usr/bin/env python3
"""
Convert a *triangulated* STEP file (every ADVANCED_FACE is a planar polygon,
e.g. a mesh that was exported to STEP) into one binary STL per part, without
needing a CAD kernel.

Also writes parts.json with per-part triangle count, bounding box, surface
area and (for closed shells) enclosed volume / centroid, which is what we use
to identify parts and estimate mass properties.

Usage:
    step_tri_to_mesh.py TALON.STEP out_dir [--scale 0.001]
"""

import argparse
import json
import os
import re
import struct
import sys

import numpy as np

ENTITY_RE = re.compile(r"#(\d+)\s*=\s*([A-Z0-9_]+)\s*\((.*)\)\s*;\s*$", re.S)
REF_RE = re.compile(r"#(\d+)")
FLOAT_RE = re.compile(r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[Ee][-+]?\d+)?")

WANTED = {
	"CARTESIAN_POINT", "VERTEX_POINT", "EDGE_CURVE", "ORIENTED_EDGE", "EDGE_LOOP",
	"FACE_BOUND", "FACE_OUTER_BOUND", "ADVANCED_FACE", "OPEN_SHELL", "CLOSED_SHELL",
	"SHELL_BASED_SURFACE_MODEL", "MANIFOLD_SURFACE_SHAPE_REPRESENTATION",
	"MANIFOLD_SOLID_BREP", "ADVANCED_BREP_SHAPE_REPRESENTATION",
	"ITEM_DEFINED_TRANSFORMATION", "AXIS2_PLACEMENT_3D", "DIRECTION",
}


def iter_statements(path):
	buf = []
	with open(path, "r", errors="replace") as f:
		for line in f:
			buf.append(line)

			if line.rstrip().endswith(";"):
				yield "".join(buf)
				buf = []


def parse(path):
	ent = {}

	for stmt in iter_statements(path):
		if not stmt.startswith("#"):
			continue

		m = ENTITY_RE.match(stmt.strip())

		if not m or m.group(2) not in WANTED:
			continue

		eid, etype, args = int(m.group(1)), m.group(2), m.group(3)

		if etype in ("CARTESIAN_POINT", "DIRECTION"):
			nums = FLOAT_RE.findall(args.split(",", 1)[1])
			ent[eid] = (etype, tuple(float(x) for x in nums))

		elif etype == "ORIENTED_EDGE":
			ent[eid] = (etype, (int(REF_RE.findall(args)[0]), args.rstrip().endswith(".T.")))

		elif etype in ("FACE_BOUND", "FACE_OUTER_BOUND"):
			ent[eid] = (etype, (int(REF_RE.findall(args)[0]), args.rstrip().endswith(".T.")))

		elif etype == "ADVANCED_FACE":
			refs = [int(r) for r in REF_RE.findall(args)]
			ent[eid] = (etype, (refs[:-1], args.rstrip().endswith(".T.")))

		else:
			ent[eid] = (etype, tuple(int(r) for r in REF_RE.findall(args)))

	return ent


def face_polygons(ent, face_id):
	(bounds, same_sense) = ent[face_id][1]
	polys = []

	for b in bounds:
		loop_id, b_orient = ent[b][1]
		pts = []

		for oe in ent[loop_id][1]:
			edge_id, oe_orient = ent[oe][1]
			v1, v2 = ent[edge_id][1][0], ent[edge_id][1][1]
			start = v1 if oe_orient else v2
			pts.append(ent[ent[start][1][0]][1])

		if not b_orient:
			pts.reverse()

		if not same_sense:
			pts.reverse()

		polys.append(pts)

	return polys


def triangulate(poly):
	# faces are expected to be triangles; fan-triangulate anything bigger
	return [(poly[0], poly[i], poly[i + 1]) for i in range(1, len(poly) - 1)]


def write_stl(path, tris):
	with open(path, "wb") as f:
		f.write(b"step_tri_to_mesh".ljust(80, b"\0"))
		f.write(struct.pack("<I", len(tris)))
		v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
		n = np.cross(v1 - v0, v2 - v0)
		ln = np.linalg.norm(n, axis=1, keepdims=True)
		n = np.divide(n, ln, out=np.zeros_like(n), where=ln > 0)
		rec = np.zeros(len(tris), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
		rec["n"] = n
		rec["v"] = tris
		f.write(rec.tobytes())


def main():
	ap = argparse.ArgumentParser()
	ap.add_argument("step")
	ap.add_argument("out_dir")
	ap.add_argument("--scale", type=float, default=0.001, help="STEP unit to metre (default mm)")
	a = ap.parse_args()

	print("parsing", a.step, "...", file=sys.stderr)
	ent = parse(a.step)
	print("entities kept:", len(ent), file=sys.stderr)

	transforms = [e for e in ent.values() if e[0] == "ITEM_DEFINED_TRANSFORMATION"]

	for _, (p1, p2) in transforms:
		loc1, loc2 = ent[ent[p1][1][0]][1], ent[ent[p2][1][0]][1]

		if any(abs(x) > 1e-9 for x in loc1 + loc2):
			print("WARNING: non-identity part placement found, it is ignored", file=sys.stderr)
			break

	shells = sorted(k for k, v in ent.items() if v[0] in ("OPEN_SHELL", "CLOSED_SHELL"))
	os.makedirs(a.out_dir, exist_ok=True)
	summary = []

	for idx, sid in enumerate(shells):
		stype, faces = ent[sid]
		tris = []

		for fid in faces:
			for poly in face_polygons(ent, fid):
				tris.extend(triangulate(poly))

		tris = np.asarray(tris, dtype=np.float64) * a.scale
		name = f"part_{idx:02d}"
		write_stl(os.path.join(a.out_dir, name + ".stl"), tris)

		v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
		cr = np.cross(v1 - v0, v2 - v0)
		area = 0.5 * np.linalg.norm(cr, axis=1).sum()
		pts = tris.reshape(-1, 3)
		info = {
			"name": name,
			"step_id": sid,
			"closed": stype == "CLOSED_SHELL",
			"triangles": len(tris),
			"bbox_min": pts.min(0).round(4).tolist(),
			"bbox_max": pts.max(0).round(4).tolist(),
			"size": (pts.max(0) - pts.min(0)).round(4).tolist(),
			"area_m2": round(float(area), 5),
		}

		if info["closed"]:
			# signed tetra volumes -> volume and centroid of the enclosed solid
			sv = np.einsum("ij,ij->i", v0, np.cross(v1, v2)) / 6.0
			vol = sv.sum()
			info["volume_m3"] = round(float(abs(vol)), 7)

			if abs(vol) > 1e-12:
				info["centroid"] = (((v0 + v1 + v2) / 4.0 * sv[:, None]).sum(0) / vol).round(4).tolist()

		summary.append(info)
		print(f"{name}: {info['triangles']:7d} tris  size {info['size']}  closed={info['closed']}", file=sys.stderr)

	with open(os.path.join(a.out_dir, "parts.json"), "w") as f:
		json.dump(summary, f, indent=1)


if __name__ == "__main__":
	main()
