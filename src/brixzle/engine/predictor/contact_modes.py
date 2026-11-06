"""Planar contact modes and stability, by linear programming (Balkcom & Trinkle 2002; Pang & Trinkle 2000).

One fixed and one moving rigid body in the XZ plane, at rest (velocity product terms neglected), as in
Balkcom & Trinkle §2: the contact accelerations are a = A c + B g with A = Jᵀ M⁻¹ J, B = Jᵀ M⁻¹,
where J maps contact forces c = [c_n..., c_t...] to a wrench (fx, fz, m) about the centre of mass.
Each contact takes one interaction (Table 2):

- 'n' rolling (sticking): a_n = a_t = 0, |c_t| <= mu c_n
- '+' / '-' sliding along +t / -t: a_n = 0, a_t >= k / <= -k, c_t = -/+ mu c_n (friction opposes)
- 's' separating: a_n >= k, c = 0
- 'a' approaching (a near, untouched fixel): a_n <= -k, c = 0
- 'S' passive: a_n >= 0, c = 0 (may separate or stay, never penetrate; for contacts left out of
  mode enumeration)

k is a small positive bound standing in for the strict inequalities (§5.6). Rather than build cones
(§3, §5), each question is a membership test: one small LP per mode.

Stability (Pang & Trinkle 2000, Definition 2): weakly stable if some contact forces in the friction
cones balance the load; strongly stable if every solution of the dynamic model has zero acceleration.
Theorem 1: if the normal forces alone balance the load (frictionless stability), it is strongly stable
for every mu. Weak-but-not-strong stability is indeterminate: some solutions move, some don't.

Units are the caller's; the predictor uses mm, and the load g normalised to the body's weight.
"""

from dataclasses import dataclass
import itertools

import numpy as np
from scipy.optimize import linprog, nnls

K_STRICT = 1e-4   # strict-inequality bound (§5.6), in units of the load; well above the LP tolerance (~1e-7)
MOVING = 1e-4     # a mode "moves" only if the body acceleration exceeds this (units of the load)
_LP = dict(method="highs")
_MODES = {}  # (k, n_free) -> ordered mode list


@dataclass(frozen=True)
class Contact:
    point: tuple   # (x, z) in the world
    normal: tuple  # unit, from the fixed body into the moving one (direction of the normal force)
    alternatives: tuple = ()  # other admissible normals (a convex corner on a convex corner)


def _feasible(c, A_ub, b_ub, A_eq, b_eq, bounds):
    res = linprog(c, A_ub=A_ub if len(A_ub) else None, b_ub=b_ub if len(b_ub) else None,
                  A_eq=A_eq if len(A_eq) else None, b_eq=b_eq if len(b_eq) else None, bounds=bounds, **_LP)
    return res if res.status == 0 else None


class ContactSystem:
    """Contacts on one moving body. ``com`` (x, z); ``M`` 3x3 generalised mass for (x, z, theta)
    about the centre of mass; ``L`` scales moments into forces for the margins."""

    def __init__(self, contacts, com, M, L, n_free=None):
        """``n_free``: enumerate modes over the first n_free contacts only; the rest are 'S'."""
        self.contacts = list(contacts)
        self.n_free = len(self.contacts) if n_free is None else n_free
        self.k = len(self.contacts)
        self.com = np.asarray(com, float)
        self.M = np.asarray(M, float)
        self.Minv = np.linalg.inv(self.M)
        self.L = float(L)
        n = np.array([c.normal for c in self.contacts], float).reshape(-1, 2)
        n /= np.linalg.norm(n, axis=1, keepdims=True)
        t = np.column_stack([-n[:, 1], n[:, 0]])
        r = np.array([c.point for c in self.contacts], float).reshape(-1, 2) - self.com
        self.n, self.t, self.r = n, t, r

        def cols(d):
            return np.vstack([d[:, 0], d[:, 1], r[:, 0] * d[:, 1] - r[:, 1] * d[:, 0]])
        self.Wn, self.Wt = cols(n), cols(t)          # 3 x k each
        self.J = np.hstack([self.Wn, self.Wt])       # 3 x 2k
        self.A = self.J.T @ self.Minv @ self.J
        self.B = self.J.T @ self.Minv
        self.rank = int(np.linalg.matrix_rank(self.J)) if self.k else 0

    # -- modes --

    def feasible(self, mode, g, mu, k=K_STRICT, spin=0):
        """Contact forces c (2k,) consistent with ``mode`` (one letter per contact) under load g, or None.

        ``spin`` (+1/-1) asks for nonzero body acceleration along the null space of Jᵀ (pivoting
        with every contact rolling); only needed for an all-'n' mode when J is rank deficient."""
        kk = self.k
        g = np.asarray(g, float)
        Bg = self.B @ g
        A_ub, b_ub, A_eq, b_eq = [], [], [], []
        bounds = [(0, None)] * kk + [(None, None)] * kk
        for i, m in enumerate(mode):
            an, at = self.A[i], self.A[kk + i]
            if m == "n":
                A_eq += [an, at]; b_eq += [-Bg[i], -Bg[kk + i]]
                row = np.zeros(2 * kk); row[kk + i] = 1; row[i] = -mu
                A_ub.append(row.copy()); b_ub.append(0.)
                row[kk + i] = -1
                A_ub.append(row); b_ub.append(0.)
            elif m in "+-":
                sign = 1 if m == "+" else -1
                A_eq.append(an); b_eq.append(-Bg[i])
                A_ub.append(-sign * at); b_ub.append(sign * Bg[kk + i] - k)
                row = np.zeros(2 * kk); row[kk + i] = 1; row[i] = sign * mu
                A_eq.append(row); b_eq.append(0.)
            elif m in "saS":
                bounds[i] = (0, 0); bounds[kk + i] = (0, 0)
                if m == "S":
                    A_ub.append(-an); b_ub.append(Bg[i])
                elif m == "s":
                    A_ub.append(-an); b_ub.append(Bg[i] - k)
                else:
                    A_ub.append(an); b_ub.append(-Bg[i] - k)
            else:
                raise ValueError(f"unknown interaction {m!r}")
        if spin:
            null = np.linalg.svd(self.J.T)[2][self.rank:]  # rows: null-space directions of Jᵀ
            if not len(null):
                return None
            v = null[0] @ self.Minv  # acc = Minv (J c + g); its null component must be nonzero
            A_ub.append(-spin * (v @ self.J)); b_ub.append(spin * (v @ g) - k)
        res = _feasible(np.zeros(2 * kk), np.array(A_ub), np.array(b_ub), np.array(A_eq), np.array(b_eq), bounds)
        return None if res is None else res.x

    def solve_mode(self, mode, g, mu, tol=1e-7):
        """Contact forces for a mode without an LP: the friction cones as edge generators make every
        force constraint a sign constraint (sliding: c = c_n (n -/+ mu t); rolling: two edge weights;
        separating: none), so the mode's equalities (a_n = 0 when touching, a_t = 0 when rolling) are
        a non-negative least squares problem. A zero residual gives forces; then the mode's strict
        inequalities are checked. Returns c (2k,) or None. Same model as ``feasible``."""
        kk = self.k
        g = np.asarray(g, float)
        Bg = self.B @ g
        cols, rows = [], []  # generator columns (2k-vectors of c) and constrained rows of a
        for i, m in enumerate(mode):
            if m in "+-":
                v = np.zeros(2 * kk); v[i] = 1; v[kk + i] = -mu if m == "+" else mu
                cols.append(v); rows.append(i)
            elif m == "n":
                for sgn in (1, -1):
                    v = np.zeros(2 * kk); v[i] = 1; v[kk + i] = sgn * mu
                    cols.append(v)
                rows += [i, kk + i]
        if cols:
            Gm = np.array(cols).T                      # 2k x m
            A = self.A[rows] @ Gm
            lam, res = nnls(A, -Bg[rows]) if rows else (np.zeros(len(cols)), 0.)
            if res > 1e-7 * max(1., float(np.linalg.norm(Bg[rows]))):
                return None
            c = Gm @ lam
        else:
            c = np.zeros(2 * kk)
        a = self.A @ c + Bg
        for i, m in enumerate(mode):
            ok = {"n": True, "+": a[kk + i] >= K_STRICT, "-": a[kk + i] <= -K_STRICT,
                  "s": a[i] >= K_STRICT, "S": a[i] >= -tol, "a": a[i] <= -K_STRICT}[m]
            if not ok:
                return None
        return c

    def acceleration(self, c, g):
        """Body acceleration (ax, az, alpha) for contact forces c."""
        return self.Minv @ (self.J @ c + np.asarray(g, float))

    def _modes(self):
        """All modes, those with the fewest rolling contacts first (sliding is the common case)."""
        key = (self.k, self.n_free)
        if key not in _MODES:
            _MODES[key] = self._make_modes()
        return list(_MODES[key])

    def _make_modes(self):
        modes = ["".join(m) + "S" * (self.k - self.n_free) for m in itertools.product("+-sn", repeat=self.n_free)]
        return sorted(modes, key=lambda m: (m.count("n"), m.count("s")))

    def moving_modes(self, g, mu, first=False, prefer=None):
        """[(mode, acceleration)] of the modes with nonzero body acceleration under g."""
        out = []
        order = self._modes()
        if prefer in order:
            order.remove(prefer); order.insert(0, prefer)
        for mode in order:
            if set(mode) <= {"n", "S"}:
                if self.rank >= 3:
                    continue  # all rolling with full rank: zero acceleration, not a moving mode
                cands = [self.feasible(mode, g, mu, spin=s) for s in (1, -1)]  # pivoting: LP
            else:
                cands = [self.solve_mode(mode, g, mu)]  # NNLS (same constraints, no LP)
            for c in cands:
                if c is not None and np.linalg.norm(self.acceleration(c, g)) > MOVING:
                    out.append((mode, self.acceleration(c, g)))
                    if first:
                        return out
                    break
        return out

    def moving_mode(self, g, mu, prefer=None):
        found = self.moving_modes(g, mu, first=True, prefer=prefer)
        return found[0] if found else (None, None)

    def least_constraint(self, g, mu):
        """Contact forces and acceleration by Gauss's least constraint over the friction cones.

        In 2D each friction cone is exactly the span of its two edges, so minimising
        |J c + g|^2 in the M^-1 metric over the cones is a non-negative least squares problem in
        the edge weights. This is the convex relaxation of Coulomb contact used by time-stepping
        schemes (Anitescu & Potra); it picks one motion without enumerating modes. Returns (c, acc).
        """
        kk = self.k
        E = np.hstack([self.Wn + mu * self.Wt, self.Wn - mu * self.Wt])  # 3 x 2k cone edges
        Lc = np.linalg.cholesky(self.Minv).T
        lam, _ = nnls(Lc @ E, -Lc @ np.asarray(g, float))
        c = np.r_[lam[:kk] + lam[kk:], mu * (lam[:kk] - lam[kk:])]
        return c, self.acceleration(c, g)

    def infer_mode(self, c, acc, tol=1e-6):
        """The contact mode a solution shows: separating, sliding (sign of a_t) or rolling."""
        kk = self.k
        a = self.J.T @ acc
        out = []
        for i in range(kk):
            if c[i] <= tol and a[i] > tol:
                out.append("s")
            elif a[kk + i] > tol:
                out.append("+")
            elif a[kk + i] < -tol:
                out.append("-")
            else:
                out.append("n")
        return "".join(out)

    def motion(self, g, mu, prefer=None):
        """(mode, acceleration) for a body that is not weakly stable.

        The least-constraint solution picks the motion; its mode is then checked with the exact
        mode LP on every contact, so the step follows Coulomb's law (sliding forces on the cone
        edge, no spurious separation). Falls back to enumerating modes, then to the relaxed
        solution with its normal separation removed."""
        c, acc = self.least_constraint(g, mu)
        modes = []
        for tol in (1e-6, 1e-4, 1e-3):  # the inferred mode, read at looser tolerances
            mode = self.infer_mode(c, acc, tol)
            if set(mode) <= {"n", "S"} or mode in modes:
                continue
            modes.append(mode)
            exact = self.solve_mode(mode, g, mu)  # fast: equalities only
            if exact is not None and np.linalg.norm(self.acceleration(exact, g)) > MOVING:
                return mode, self.acceleration(exact, g)
        # Neighbours: the inferred mode with one contact's interaction changed (the usual miss is a
        # single contact read as sliding instead of separating, or the reverse).
        free = self.n_free
        near = [m[:i] + x + m[i + 1:] for m in modes for i in range(free) for x in "+-sn" if x != m[i]]
        if prefer and prefer in self._modes():
            near.insert(0, prefer)
        for mode in dict.fromkeys(modes + near):
            if set(mode) <= {"n", "S"}:
                continue
            exact = self.solve_mode(mode, g, mu)
            if exact is not None and np.linalg.norm(self.acceleration(exact, g)) > MOVING:
                return mode, self.acceleration(exact, g)
        if self.n_free <= 3:  # enumerate the rest (at most 64 LPs)
            m, a = self.moving_mode(g, mu, prefer=prefer)
            if m is not None:
                return m, a
        # Relaxed solution ('~' in the trace), with its spurious separation at loaded contacts
        # (a_n = mu |a_t| in the convex relaxation) projected out in the M metric.
        loaded = np.flatnonzero(c[:self.k] > 1e-9)
        if len(loaded):
            N = self.Wn[:, loaded].T
            S = N @ self.Minv @ N.T
            acc = acc - self.Minv @ N.T @ np.linalg.lstsq(S, N @ acc, rcond=None)[0]
        return self.infer_mode(c, acc) + "~", acc

    # -- stability --

    def _wrench_rows(self):
        return np.diag([1., 1., 1. / self.L])  # moment scaled into force units

    def _in_cone(self, E, g):
        """Is -g a non-negative combination of the columns of E? Exactly, by NNLS: the residual is
        zero iff it is (moments scaled by 1/L so the components are comparable)."""
        S = self._wrench_rows()
        _, res = nnls(S @ E, -S @ np.asarray(g, float))
        return res <= 1e-9 * max(1.0, float(np.linalg.norm(S @ np.asarray(g, float))))

    def weakly_stable(self, g, mu):
        """Some contact forces in the friction cones balance g (Pang & Trinkle, weak stability).
        In 2D each cone is spanned by its two edges, so this is a cone-membership test."""
        if not self.k:
            return False
        return self._in_cone(np.hstack([self.Wn + mu * self.Wt, self.Wn - mu * self.Wt]), g)

    def frictionless_stable(self, g):
        """Normal forces alone balance g: strongly stable for every mu (Pang & Trinkle Theorem 1)."""
        if not self.k:
            return False
        return self._in_cone(self.Wn, g)

    def strongly_stable(self, g, mu):
        """Weakly stable and no mode with nonzero acceleration (Balkcom & Trinkle §7)."""
        if self.frictionless_stable(g):
            return True
        return self.weakly_stable(g, mu) and not self.moving_modes(g, mu, first=True)

    def stick_margin(self, g, mu):
        """How far g lies outside the set of loads the friction cones can balance (> 0: it moves).

        Outside: min over cone forces of |g + J c|_inf. Inside: minus the largest r such that every
        corner of the box g +/- r (moment scaled by L) is still balanced."""
        kk = self.k
        S = self._wrench_rows()
        g = np.asarray(g, float)
        if not kk:
            return float(np.max(np.abs(S @ g)))
        cone = []
        for i in range(kk):
            for s in (1, -1):
                row = np.zeros(2 * kk); row[kk + i] = s; row[i] = -mu
                cone.append(row)
        cone = np.array(cone)
        # Outside: variables (c, t); min t s.t. -t <= S (J c + g) <= t.
        SJ = S @ self.J
        A_ub = np.vstack([np.hstack([cone, np.zeros((2 * kk, 1))]),
                          np.hstack([SJ, -np.ones((3, 1))]), np.hstack([-SJ, -np.ones((3, 1))])])
        b_ub = np.concatenate([np.zeros(2 * kk), -S @ g, S @ g])
        res = _feasible(np.r_[np.zeros(2 * kk), 1.], A_ub, b_ub, [], [], [(0, None)] * kk + [(None, None)] * kk + [(0, None)])
        if res is not None and res.fun > 1e-9:
            return float(res.fun)
        return -self._ball(g, mu)

    def _ball(self, g, mu=None):
        """Largest r with g + r v balanced for all 8 box corners v (normal forces only if mu is None)."""
        kk = self.k
        if not kk:
            return 0.
        corners = np.array(list(itertools.product((-1., 1.), repeat=3))) @ np.diag([1., 1., self.L])
        W = self.Wn if mu is None else self.J
        nv = W.shape[1]
        nvar = 8 * nv + 1
        A_eq, b_eq, A_ub, b_ub = [], [], [], []
        g = np.asarray(g, float)
        for j, v in enumerate(corners):
            block = np.zeros((3, nvar))
            block[:, j * nv:(j + 1) * nv] = W
            block[:, -1] = v
            A_eq.append(block); b_eq.append(-g)
            if mu is not None:
                for i in range(kk):
                    for s in (1, -1):
                        row = np.zeros(nvar); row[j * nv + kk + i] = s; row[j * nv + i] = -mu
                        A_ub.append(row); b_ub.append(0.)
        bounds = []
        for _ in range(8):
            bounds += [(0, None)] * kk + ([] if mu is None else [(None, None)] * kk)
        bounds.append((0, None))
        cost = np.zeros(nvar); cost[-1] = -1.
        res = _feasible(cost, np.array(A_ub) if A_ub else [], np.array(b_ub) if b_ub else [],
                        np.vstack(A_eq), np.concatenate(b_eq), bounds)
        return float(-res.fun) if res is not None else 0.

    def seat_margin(self, g):
        """Largest disturbance (fraction of the load, moment scaled by L) the seat resists with normal
        forces alone: a conservative ball in the strongly stable cone (Balkcom & Trinkle §8, using
        Pang & Trinkle Theorem 1)."""
        return self._ball(g, None) if self.frictionless_stable(g) else 0.

    # -- geometry of the contact set --

    def opposing(self, tol=-0.5):
        """Two contacts push from roughly opposite sides (normals at more than 120 degrees)."""
        return any(self.n[i] @ self.n[j] < tol for i in range(self.k) for j in range(i + 1, self.k))

    def wedged(self, mu):
        """Some pair can squeeze the body between them: the line joining two contacts lies inside both
        friction cones (Whitney 1982 §4: wedging when two-point contact comes with l < mu d)."""
        half = np.arctan(mu)
        for i in range(self.k):
            for j in range(i + 1, self.k):
                d = np.asarray(self.contacts[j].point, float) - np.asarray(self.contacts[i].point, float)
                if np.linalg.norm(d) < 1e-9:
                    continue
                d /= np.linalg.norm(d)
                if np.arccos(np.clip(self.n[i] @ d, -1, 1)) < half and np.arccos(np.clip(-self.n[j] @ d, -1, 1)) < half:
                    return True
        return False
