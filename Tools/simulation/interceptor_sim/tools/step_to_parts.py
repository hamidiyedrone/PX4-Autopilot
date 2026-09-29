#!/usr/bin/env python3
"""
Split a STEP file into its solids, export each one as STL meshes and write
parts.json with bounding box, volume, centroid and inertia (unit density) per
solid. Runs inside FreeCAD (the snap ships `freecad.cmd`):

    STEP_IN="x-uav talon.step" OUT_DIR=build/talon_parts freecad.cmd tools/step_to_parts.py

Colours (STYLED_ITEM on solids and on single faces) are read from the STEP
text directly, because FreeCAD drops them in console mode. Every solid is
exported as part_XX.stl plus one part_XX_<colour>.stl per face colour group.

Coordinates are converted from STEP millimetres to metres.
"""

import json
import os
import re

import FreeCAD  # noqa: F401  (needed to initialise Part)
import MeshPart
import Part

STEP_IN = os.environ["STEP_IN"]
OUT_DIR = os.environ["OUT_DIR"]
LIN_DEFLECTION = float(os.environ.get("LIN_DEFLECTION", "0.5"))  # mm
MM = 0.001


def parse_step_styles(path):
	"""Returns (solid ids in file order, {item id: (colour name, (r, g, b))}, entity text dict)."""
	text = open(path, errors="replace").read()
	ent = dict(re.findall(r"#(\d+)\s*=\s*(.*?);\s*\n(?=#|ENDSEC)", text, re.S))
	refs = lambda i: re.findall(r"#(\d+)", ent[i])
	styles = {}

	for k, v in ent.items():
		if not v.startswith("STYLED_ITEM"):
			continue

		r = refs(k)
		stack, seen = r[:-1], set()

		while stack:
			x = stack.pop()

			if x in seen or x not in ent:
				continue

			seen.add(x)

			if ent[x].startswith("COLOUR_RGB"):
				m = re.match(r"COLOUR_RGB\('(.*)',\s*([\d.eE+-]+),\s*([\d.eE+-]+),\s*([\d.eE+-]+)\)", ent[x].replace("\n", ""))
				styles[r[-1]] = (decode_step_string(m.group(1)), tuple(float(m.group(i)) for i in (2, 3, 4)))
				break

			stack += refs(x)

	solids = [k for k, v in sorted(ent.items(), key=lambda kv: int(kv[0])) if v.startswith("MANIFOLD_SOLID_BREP")]
	return solids, styles, ent


def step_face_key(ent, face_id):
	"""Surface type and a characteristic point [mm] of a STEP ADVANCED_FACE."""
	surf = re.findall(r"#(\d+)", ent[face_id])[-1]
	s = ent[surf].replace("\n", "")
	point = lambda pid: [float(x) for x in re.findall(r"[-+]?[\d.]+(?:[eE][-+]?\d+)?", ent[pid].split(",", 1)[1])]

	if "B_SPLINE_SURFACE" in s:
		poles = re.search(r"\(\((.*?)\)\)", s).group(1)
		pts = [point(p) for p in re.findall(r"#(\d+)", poles)]
		return "bspline", [sum(c) / len(pts) for c in zip(*pts)]

	kind = s.split("(")[0]
	axis = re.findall(r"#(\d+)", s)[0]
	return {"PLANE": "plane", "CYLINDRICAL_SURFACE": "cylinder"}.get(kind, kind.lower()), point(re.findall(r"#(\d+)", ent[axis])[0])


def freecad_face_key(face):
	s = face.Surface

	if isinstance(s, Part.BSplineSurface):
		poles = [p for row in s.getPoles() for p in row]
		return "bspline", [sum(getattr(p, c) for p in poles) / len(poles) for c in "xyz"]

	if isinstance(s, Part.Plane):
		return "plane", list(s.Position)

	if isinstance(s, Part.Cylinder):
		return "cylinder", list(s.Center)

	return type(s).__name__.lower(), None


def decode_step_string(s):
	"""ISO 10303-21 string escapes: \\X\\hh (latin-1) and \\X2\\hhhh...\\X0\\ (UTF-16)."""
	s = re.sub(r"\\X2\\((?:[0-9A-F]{4})+)\\X0\\",
		   lambda m: "".join(chr(int(m.group(1)[i:i + 4], 16)) for i in range(0, len(m.group(1)), 4)), s)
	return re.sub(r"\\X\\([0-9A-F]{2})", lambda m: chr(int(m.group(1), 16)), s)


def slug(name):
	tr = str.maketrans("çğıöşüÇĞİÖŞÜ", "cgiosuCGIOSU")
	return re.sub(r"[^a-z0-9]+", "_", name.translate(tr).lower().encode("ascii", "ignore").decode()).strip("_")


def export_mesh(shape, path):
	mesh = MeshPart.meshFromShape(Shape=shape, LinearDeflection=LIN_DEFLECTION, AngularDeflection=0.3, Relative=False)
	mesh.transform(FreeCAD.Matrix(MM, 0, 0, 0, 0, MM, 0, 0, 0, 0, MM, 0, 0, 0, 0, 1))
	mesh.write(path)
	return mesh.CountFacets


os.makedirs(OUT_DIR, exist_ok=True)

shape = Part.read(STEP_IN)
step_solids, styles, ent = parse_step_styles(STEP_IN)
assert len(step_solids) == len(shape.Solids), "solid count mismatch between STEP text and FreeCAD"

# face colour overrides: match STEP faces to FreeCAD faces by surface type + characteristic point
face_colour = {}  # (solid index, face index) -> colour

for item, colour in styles.items():
	if not ent[item].startswith("ADVANCED_FACE"):
		continue

	kind, p = step_face_key(ent, item)
	best = None

	for si, solid in enumerate(shape.Solids):
		for fi, face in enumerate(solid.Faces):
			fkind, fp = freecad_face_key(face)

			if fkind != kind or fp is None:
				continue

			d = sum((a - b) ** 2 for a, b in zip(p, fp)) ** 0.5

			if best is None or d < best[0]:
				best = (d, si, fi)

	if best is None or best[0] > 1.0:
		print("WARNING: no match for coloured face #%s (%s)" % (item, colour[0]))
		continue

	face_colour[(best[1], best[2])] = colour

summary = []

for i, solid in enumerate(shape.Solids):
	name = "part_%02d" % i
	bb = solid.BoundBox
	com = solid.CenterOfMass
	inertia = solid.MatrixOfInertia  # about CoM, unit density, mm^5
	solid_colour = styles.get(step_solids[i], ("default", (0.8, 0.8, 0.8)))

	tris = export_mesh(solid, os.path.join(OUT_DIR, name + ".stl"))

	# split by colour
	groups = {}

	for fi, face in enumerate(solid.Faces):
		groups.setdefault(face_colour.get((i, fi), solid_colour), []).append(face)

	colour_meshes = []

	for (cname, rgb), faces in groups.items():
		fname = "%s_%s.stl" % (name, slug(cname))
		export_mesh(Part.makeCompound(faces), os.path.join(OUT_DIR, fname))
		colour_meshes.append({"file": fname, "colour": cname, "rgb": rgb, "faces": len(faces)})

	summary.append({
		"name": name,
		"step_name": decode_step_string(re.search(r"'(.*?)'", ent[step_solids[i]]).group(1)),
		"colour": solid_colour[0],
		"rgb": solid_colour[1],
		"colour_meshes": colour_meshes,
		"faces": len(solid.Faces),
		"triangles": tris,
		"bbox_min": [round(bb.XMin * MM, 4), round(bb.YMin * MM, 4), round(bb.ZMin * MM, 4)],
		"bbox_max": [round(bb.XMax * MM, 4), round(bb.YMax * MM, 4), round(bb.ZMax * MM, 4)],
		"size": [round(bb.XLength * MM, 4), round(bb.YLength * MM, 4), round(bb.ZLength * MM, 4)],
		"volume_m3": solid.Volume * MM ** 3,
		"area_m2": round(solid.Area * MM ** 2, 5),
		"centroid": [round(com.x * MM, 4), round(com.y * MM, 4), round(com.z * MM, 4)],
		# inertia per unit density [m^5]; multiply by density [kg/m^3] to get kg m^2
		"inertia_unit_density": [[inertia.A[r * 4 + c] * MM ** 5 for c in range(3)] for r in range(3)],
	})

with open(os.path.join(OUT_DIR, "parts.json"), "w") as f:
	json.dump(summary, f, indent=1)

for p in summary:
	print("%s %-12s %-30s faces=%3d centroid=%s  colours: %s" % (
		p["name"], p["step_name"], p["colour"], p["faces"], p["centroid"],
		", ".join("%s(%d)" % (c["colour"], c["faces"]) for c in p["colour_meshes"])))
