"""Planar (SE(2)) pose-graph optimisation with scipy.optimize.least_squares.

Nodes are keyframe poses (x, y, yaw); edges carry the relative pose of node j
measured in node i plus a diagonal information vector (w_x, w_y, w_yaw).
Node 0 is held fixed.  A Huber loss keeps a single wrong loop closure from
tearing the map apart.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List

import numpy as np
from scipy import sparse
from scipy.optimize import least_squares


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Edge:
    i: int
    j: int
    rel: np.ndarray          # (3,) pose of j in i
    info: np.ndarray         # (3,) diagonal information (1/sigma^2)
    kind: str = "odom"       # odom | loop | weak


@dataclass
class PoseGraphSE2:
    nodes: List[np.ndarray] = field(default_factory=list)
    edges: List[Edge] = field(default_factory=list)

    def add_node(self, pose) -> int:
        self.nodes.append(np.array(pose, dtype=float).reshape(3))
        return len(self.nodes) - 1

    def add_edge(self, i: int, j: int, rel, info, kind: str = "odom") -> Edge:
        e = Edge(int(i), int(j), np.array(rel, dtype=float).reshape(3), np.array(info, dtype=float).reshape(3), kind)
        self.edges.append(e)
        return e

    @property
    def n_loops(self) -> int:
        return sum(1 for e in self.edges if e.kind == "loop")

    # ------------------------------------------------------------ optimisation
    def optimize(self, max_nfev: int = 100, huber_scale: float = 5.0, verbose: bool = False) -> dict:
        n = len(self.nodes)
        if n < 2 or not self.edges:
            return {"nodes": n, "edges": len(self.edges), "cost0": 0.0, "cost": 0.0, "nfev": 0}
        I = np.array([e.i for e in self.edges])
        J = np.array([e.j for e in self.edges])
        Z = np.stack([e.rel for e in self.edges])
        W = np.sqrt(np.stack([e.info for e in self.edges]))
        m = len(self.edges)

        def unpack(x):
            P = np.empty((n, 3))
            P[0] = self.nodes[0]
            P[1:] = x.reshape(-1, 3)
            return P

        def residuals(x):
            P = unpack(x)
            pi, pj = P[I], P[J]
            c, s = np.cos(pi[:, 2]), np.sin(pi[:, 2])
            dx, dy = pj[:, 0] - pi[:, 0], pj[:, 1] - pi[:, 1]
            ex = c * dx + s * dy - Z[:, 0]
            ey = -s * dx + c * dy - Z[:, 1]
            et = wrap(pj[:, 2] - pi[:, 2] - Z[:, 2])
            return (np.stack([ex, ey, et], axis=1) * W).ravel()

        def jacobian(x):
            P = unpack(x)
            pi, pj = P[I], P[J]
            c, s = np.cos(pi[:, 2]), np.sin(pi[:, 2])
            dx, dy = pj[:, 0] - pi[:, 0], pj[:, 1] - pi[:, 1]
            rows, cols, vals = [], [], []

            def put(r_off, node_idx, col_off, val):
                # node 0 is fixed -> its columns are dropped
                sel = node_idx > 0
                rows.append(np.arange(m)[sel] * 3 + r_off)
                cols.append((node_idx[sel] - 1) * 3 + col_off)
                vals.append(val[sel])

            wx, wy, wt = W[:, 0], W[:, 1], W[:, 2]
            # d ex / d(xi, yi, yawi, xj, yj)
            put(0, I, 0, -c * wx); put(0, I, 1, -s * wx); put(0, I, 2, (-s * dx + c * dy) * wx)
            put(0, J, 0, c * wx); put(0, J, 1, s * wx)
            # d ey
            put(1, I, 0, s * wy); put(1, I, 1, -c * wy); put(1, I, 2, (-c * dx - s * dy) * wy)
            put(1, J, 0, -s * wy); put(1, J, 1, c * wy)
            # d et
            put(2, I, 2, -np.ones(m) * wt); put(2, J, 2, np.ones(m) * wt)
            r = np.concatenate(rows); cidx = np.concatenate(cols); v = np.concatenate(vals)
            return sparse.csr_matrix((v, (r, cidx)), shape=(3 * m, 3 * (n - 1)))

        x0 = np.stack(self.nodes[1:]).ravel()
        cost0 = float(0.5 * np.sum(residuals(x0) ** 2))
        res = least_squares(residuals, x0, jac=jacobian, method="trf", loss="huber", f_scale=huber_scale,
                            tr_solver="lsmr", max_nfev=max_nfev, x_scale="jac", verbose=1 if verbose else 0)
        P = unpack(res.x)
        P[:, 2] = wrap(P[:, 2])
        for k in range(n):
            self.nodes[k] = P[k]
        return {"nodes": n, "edges": m, "loops": self.n_loops, "cost0": cost0, "cost": float(res.cost),
                "nfev": int(res.nfev), "status": int(res.status)}

    def poses(self) -> np.ndarray:
        return np.stack(self.nodes) if self.nodes else np.zeros((0, 3))
