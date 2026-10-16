#!/usr/bin/env python3
"""
Net projectile physics: a hexagonal cord net with steel weights at its 6 corners.

The net is a triangular lattice of point masses inside a regular hexagon, joined by cords
that only pull (tension only: a slack cord folds freely). It is integrated with position
based dynamics (Verlet + distance constraints), which stays stable for inextensible cords.

  - launch: the packed net leaves the muzzle, the corner weights splay out at the launch
    half angle, the inner net only moves along the barrel and is pulled open by the weights
  - drag: cords feel the air speed component normal to them (cylinder drag), so a net
    streaming behind the weights has little drag and an open net facing the air a lot;
    the weights have sphere drag
  - gravity, flat ground (the net stops where it lands)

Tumbling, flapping and the opening/closing of the net are not scripted: they follow from
the weights, the cords and the drag.

The integration runs in one numba compiled kernel (pip install numba): with only ~60 nodes
numpy spends its time in call overhead, compiled loops run the net ~50x faster.

No Gazebo dependency: tools/net_launcher.py runs it in sim time and draws it.
"""

import math

import numpy as np
from numba import njit

RHO_AIR = 1.225
G = np.array([0.0, 0.0, -9.81])


class NetParams:
	def __init__(self, **kw):
		self.open_diameter = 3.0      # corner to corner [m]
		self.rings = 4                # lattice rings: 1 + 3 n (n + 1) nodes
		self.ball_diameter = 0.012    # steel ball [m]
		self.ball_density = 7850.0    # steel [kg/m^3]
		self.muzzle_speed = 50.0      # weights, relative to the vehicle [m/s]
		self.splay_deg = 30.0         # weight launch angle off the barrel axis [deg]
		self.inner_spread = 0.25      # radial launch speed of the inner net, fraction of the weights'
		self.pack_radius = 0.03       # packed net radius in the canister [m]
		self.mesh_size = 0.30         # real net mesh, sets the total cord length [m]
		self.cord_diameter = 0.001    # braided cord [m]
		self.cord_density = 970.0     # Dyneema [kg/m^3]
		self.cord_fill = 0.8          # fiber fraction of the braid cross section
		self.cord_cd = 1.0            # cylinder drag coefficient (Re ~ 1000)
		self.ball_cd = 0.47           # sphere drag coefficient
		self.jitter = 0.03            # launch velocity spread (fraction), breaks the symmetry
		self.dt = 0.002               # integration step [s]
		self.iterations = 3           # constraint passes per step
		self.ground_z = 0.0           # [m]
		for k, v in kw.items():
			if not hasattr(self, k):
				raise KeyError(f"unknown net parameter '{k}'")
			setattr(self, k, type(getattr(self, k))(v))

	@property
	def ball_mass(self):
		return self.ball_density * math.pi / 6.0 * self.ball_diameter ** 3

	@property
	def cord_length(self):
		"""Total cord length of the real net (square mesh over the hexagon) [m]."""
		area = 1.5 * math.sqrt(3.0) * (self.open_diameter / 2.0) ** 2
		return 2.0 * area / self.mesh_size

	@property
	def net_mass(self):
		"""Cord mass of the real net (knots not counted) [kg]."""
		return self.cord_fill * self.cord_density * math.pi / 4.0 * self.cord_diameter ** 2 * self.cord_length


def hex_lattice(rings, radius):
	"""Triangular lattice inside a hexagon: node xy, edges (i, j), the 6 corner nodes and
	6 edge groups without shared nodes (for Gauss-Seidel constraint passes)."""
	s = radius / rings
	index = {}
	xy = []
	for q in range(-rings, rings + 1):
		for r in range(-rings, rings + 1):
			if max(abs(q), abs(r), abs(q + r)) <= rings:
				index[(q, r)] = len(xy)
				xy.append((s * (q + 0.5 * r), s * r * math.sqrt(3.0) / 2.0))

	groups = [[] for _ in range(6)]
	for (q, r), i in index.items():
		for g, (dq, dr, parity) in enumerate(((1, 0, q), (0, 1, r), (-1, 1, r))):
			j = index.get((q + dq, r + dr))
			if j is not None:
				groups[2 * g + (parity & 1)].append((i, j))

	corners = [index[c] for c in ((rings, 0), (0, rings), (-rings, rings), (-rings, 0), (0, -rings), (rings, -rings))]
	groups = [np.array(gr, dtype=int).reshape(-1, 2) for gr in groups if gr]
	edges = np.concatenate(groups)
	return np.array(xy), edges, corners, groups


@njit(cache=True)
def _integrate(x, x_prev, mass, edges, rest, share, corners, radius, grounded, cord_k, ball_k,
               dt, iterations, ground_z, steps, max_spread):
	"""Verlet steps with cord / ball drag and gravity, tension only cord constraints, ground
	stop. Updates x, x_prev, grounded in place, returns the max corner spread."""
	n = x.shape[0]
	f = np.empty((n, 3))
	v = np.empty((n, 3))
	for _ in range(steps):
		for k in range(n):
			for c in range(3):
				v[k, c] = (x[k, c] - x_prev[k, c]) / dt
				f[k, c] = 0.0
			f[k, 2] = mass[k] * G[2]

		# cords: drag of the air speed normal to the cord, half to each end
		for e in range(edges.shape[0]):
			i, j = edges[e, 0], edges[e, 1]
			d0, d1, d2 = x[j, 0] - x[i, 0], x[j, 1] - x[i, 1], x[j, 2] - x[i, 2]
			length = math.sqrt(d0 * d0 + d1 * d1 + d2 * d2)
			inv = 1.0 / max(length, 1e-9)
			t0, t1, t2 = d0 * inv, d1 * inv, d2 * inv
			u0, u1, u2 = 0.5 * (v[i, 0] + v[j, 0]), 0.5 * (v[i, 1] + v[j, 1]), 0.5 * (v[i, 2] + v[j, 2])
			ut = u0 * t0 + u1 * t1 + u2 * t2
			n0, n1, n2 = u0 - ut * t0, u1 - ut * t1, u2 - ut * t2
			k = -0.5 * cord_k * min(length, rest[e]) * math.sqrt(n0 * n0 + n1 * n1 + n2 * n2)
			f[i, 0] += k * n0
			f[i, 1] += k * n1
			f[i, 2] += k * n2
			f[j, 0] += k * n0
			f[j, 1] += k * n1
			f[j, 2] += k * n2

		# balls: sphere drag
		for c in corners:
			k = -ball_k * math.sqrt(v[c, 0] ** 2 + v[c, 1] ** 2 + v[c, 2] ** 2)
			for a in range(3):
				f[c, a] += k * v[c, a]

		for k in range(n):
			for c in range(3):
				x_new = 2.0 * x[k, c] - x_prev[k, c] + f[k, c] / mass[k] * dt * dt
				x_prev[k, c] = x[k, c]
				x[k, c] = x_new

		# cords only pull
		for _ in range(iterations):
			for e in range(edges.shape[0]):
				i, j = edges[e, 0], edges[e, 1]
				d0, d1, d2 = x[j, 0] - x[i, 0], x[j, 1] - x[i, 1], x[j, 2] - x[i, 2]
				length = math.sqrt(d0 * d0 + d1 * d1 + d2 * d2)
				if length > rest[e]:
					s = (length - rest[e]) / length
					si, sj = share[e, 0] * s, share[e, 1] * s
					x[i, 0] += si * d0
					x[i, 1] += si * d1
					x[i, 2] += si * d2
					x[j, 0] -= sj * d0
					x[j, 1] -= sj * d1
					x[j, 2] -= sj * d2

		# ground: stop where it lands
		for k in range(n):
			if x[k, 2] < ground_z + radius[k]:
				x[k, 2] = ground_z + radius[k]
				for c in range(3):
					x_prev[k, c] = x[k, c]
				grounded[k] = True

		# mean distance between opposite corners
		spread = 0.0
		h = len(corners) // 2
		for q in range(h):
			a, b = corners[q], corners[q + h]
			spread += math.sqrt((x[a, 0] - x[b, 0]) ** 2 + (x[a, 1] - x[b, 1]) ** 2 + (x[a, 2] - x[b, 2]) ** 2)
		max_spread = max(max_spread, spread / h)
	return max_spread


class Net:
	def __init__(self, params, muzzle, axis, v_vehicle, rng=None):
		p = self.p = params
		rng = rng or np.random.default_rng()

		# edges come grouped (no shared nodes within a group): one sequential constraint
		# sweep over them is a Gauss-Seidel pass per group
		xy, self.edges, self.corners, _ = hex_lattice(p.rings, p.open_diameter / 2.0)
		self.corners = np.array(self.corners)
		n = len(xy)
		self.rest = np.full(len(self.edges), p.open_diameter / 2.0 / p.rings)

		self.mass = np.full(n, p.net_mass / n)
		self.mass[self.corners] += p.ball_mass
		wi, wj = 1.0 / self.mass[self.edges[:, 0]], 1.0 / self.mass[self.edges[:, 1]]
		self._share = np.stack([wi / (wi + wj), wj / (wi + wj)], axis=1)   # correction share of i, j

		# net frame: z along the barrel
		axis = np.asarray(axis, dtype=float)
		axis /= np.linalg.norm(axis)
		ref = np.array([0.0, 0.0, 1.0]) if abs(axis[2]) < 0.95 else np.array([1.0, 0.0, 0.0])
		e1 = np.cross(axis, ref)
		e1 /= np.linalg.norm(e1)
		e2 = np.cross(axis, e1)
		self.axis = axis

		radial = xy[:, 0:1] * e1 + xy[:, 1:2] * e2              # world offsets of the open net
		rho = np.linalg.norm(xy, axis=1) / (p.open_diameter / 2.0)
		radial_dir = radial / np.maximum(np.linalg.norm(radial, axis=1), 1e-9)[:, None]

		self.muzzle = np.asarray(muzzle, dtype=float)
		self.x = self.muzzle + radial * (p.pack_radius / (p.open_diameter / 2.0))

		splay = math.radians(p.splay_deg)
		v_ax = p.muzzle_speed * math.cos(splay)
		v_rad = p.muzzle_speed * math.sin(splay) * np.where(np.isin(np.arange(n), self.corners), 1.0,
		                                                     p.inner_spread * rho)
		# jitter only the launch velocity relative to the vehicle, not the vehicle's own velocity
		v = v_ax * axis + v_rad[:, None] * radial_dir
		v *= 1.0 + p.jitter * rng.standard_normal((n, 1))
		v += p.jitter * p.muzzle_speed * 0.3 * rng.standard_normal((n, 3))
		v += np.asarray(v_vehicle, dtype=float)
		self.v = v
		self.x_prev = self.x - v * p.dt

		# drag: the lattice stands for the whole real net, scale its cord length up to it
		self._cord_k = 0.5 * RHO_AIR * p.cord_cd * p.cord_diameter * p.cord_length / self.rest.sum()
		self._ball_k = 0.5 * RHO_AIR * p.ball_cd * math.pi / 4.0 * p.ball_diameter ** 2

		self._radius = np.full(n, 0.002)
		self._radius[self.corners] = p.ball_diameter / 2.0
		self.t = 0.0
		self.grounded = np.zeros(n, dtype=bool)
		self.max_spread = 0.0

	def step(self):
		self._integrate(1)

	def _integrate(self, steps):
		p = self.p
		self.max_spread = _integrate(self.x, self.x_prev, self.mass, self.edges, self.rest, self._share,
		                             self.corners, self._radius, self.grounded, self._cord_k, self._ball_k,
		                             p.dt, p.iterations, p.ground_z, steps, self.max_spread)
		self.v = (self.x - self.x_prev) / p.dt
		self.t += steps * p.dt

	def advance(self, duration):
		"""Integrate for duration [s] (whole steps; the remainder is left for the next call).
		A net lying on the ground is not integrated any more, its clock still runs."""
		steps = max(0, int(duration / self.p.dt))
		if self.on_ground():
			self.t += steps * self.p.dt
			return
		if steps:
			self._integrate(steps)

	# --- state ---

	def centroid(self):
		return (self.mass[:, None] * self.x).sum(axis=0) / self.mass.sum()

	def velocity(self):
		return (self.mass[:, None] * self.v).sum(axis=0) / self.mass.sum()

	def spread(self):
		"""Mean distance between opposite corners [m]."""
		c = self.x[self.corners]
		return float(np.mean(np.linalg.norm(c[:3] - c[3:], axis=1)))

	def on_ground(self):
		return bool(np.all(self.grounded))

	def phase(self):
		if self.on_ground():
			return "ground"
		d = self.p.open_diameter
		if self.max_spread < 0.85 * d:
			return "opening"
		return "open" if self.spread() >= 0.6 * d else "collapsed"
