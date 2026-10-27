"""Derive the Bailey tail-rotor torque coefficients from Bailey's assumptions.

F. J. Bailey, Jr., NACA Report 716 (1941): a rotor of b hinged, rectangular,
linearly twisted blades with no cyclic pitch, lift c_l = a alpha inboard of
the tip-loss radius BR only, profile drag c_d = d0 + d1 alpha + d2 alpha^2 out
to the tip (p. 209), flapping beta = a0 - a1 cos psi - b1 sin psi - ... about
a hinge on the axis (p. 210), the weight moment M_W dropped (Bailey: "the
last four terms of equation (9) are negligible", p. 212). Its torque is
C_Q = C_Qd - C_Qa (p. 212-213): with sigma the solidity,

    (1/a)(2 C_Qa / sigma) = t41 lam^2 + t42 lam th0 + t43 lam th1
                          + t44 th0^2 + t45 th0 th1 + t46 th1^2      (Eq. 9)
    2 C_Qd / sigma = d0 t51 + d1 (t52 lam + t53 th0 + t54 th1)
                   + d2 (t55 lam^2 + t56 lam th0 + t57 lam th1
                         + t58 th0^2 + t59 th0 th1 + t510 th1^2)     (Eq. 11)

Eyas keeps the terms to mu^2, as GenHel's tail-rotor thrust does (CR-166309
p. 5.4-7; D-028), so the second flapping harmonics (whose terms here start
at mu^3) drop out. The reversed-velocity region does not: its lam^2 part is
mu^2 (below). This script derives the mu^0 and mu^2 parts from the
blade-element integrals and checks them against Bailey's appendix
(pp. 223-224, read from the page images), then prints them; it exits non-zero
on a mismatch.

Per blade element at radius x R and azimuth psi, all speeds over Omega R:
    u_T = x + mu sin psi,  u_P = lam - x dbeta/dpsi - mu beta cos psi,
    theta = th0 + x th1,   alpha = u_P / u_T + theta.
Lift per span (small angles) is (1/2) rho (Omega R)^2 c a (u_T u_P + u_T^2 theta),
normal to the disk; tilted by u_P / u_T it accelerates the rotor by
(1/2) rho (Omega R)^2 c a (u_P^2 + u_T u_P theta). Drag accelerates it by
-(1/2) rho (Omega R)^2 c u_T^2 c_d. Flapping: beta'' + beta = (gamma/2)
int_0^B x (u_T u_P + u_T^2 theta) dx, gamma = c rho a R^4 / I_1 (p. 210).

Run: .venv/bin/python tools/derive_bailey_torque.py
"""

import sys

import sympy as sp

x, psi, mu, lam, th0, th1, B, g = sp.symbols("x psi mu lambda theta0 theta1 B gamma", real=True)
a0, a1, b1 = sp.symbols("a0 a1 b1")
ORDER = 2  # keep mu^0 .. mu^ORDER


def avg(expr):
    """Mean over one revolution."""
    return sp.integrate(sp.expand(expr), (psi, 0, 2 * sp.pi)) / (2 * sp.pi)


def harmonic(expr, f):
    """2/(2 pi) int expr f dpsi: a Fourier coefficient."""
    return sp.integrate(sp.expand(expr * f), (psi, 0, 2 * sp.pi)) / sp.pi


def truncate(expr):
    """Taylor series in mu to ORDER, exact rational coefficients."""
    return sp.expand(sp.series(expr, mu, 0, ORDER + 1).removeO())


beta = a0 - a1 * sp.cos(psi) - b1 * sp.sin(psi)
u_T = x + mu * sp.sin(psi)
u_P = lam - x * sp.diff(beta, psi) - mu * beta * sp.cos(psi)
theta = th0 + x * th1

# Flapping by harmonic balance: beta'' + beta has no first harmonic, so the
# aerodynamic moment's first harmonics vanish, and its mean is a0.
moment = (g / 2) * sp.integrate(sp.expand(x * (u_T * u_P + u_T**2 * theta)), (x, 0, B))
eqs = [
    sp.Eq(a0, avg(moment)),
    sp.Eq(0, harmonic(moment, sp.cos(psi))),
    sp.Eq(0, harmonic(moment, sp.sin(psi))),
]
flap = sp.solve(eqs, [a0, a1, b1], dict=True)[0]
flap = {k: sp.simplify(v) for k, v in flap.items()}


def reversed_region(integrand):
    """Mean over the revolution of int x integrand dx over the reversed-velocity
    region, u_T < 0: psi in (pi, 2 pi), x from 0 to -mu sin psi (p. 213)."""
    inner = sp.integrate(sp.expand(x * integrand), (x, 0, -mu * sp.sin(psi)))
    return sp.integrate(sp.expand(inner), (psi, sp.pi, 2 * sp.pi)) / (2 * sp.pi)


# Accelerating torque: lift inboard of B; decelerating: drag to the tip.
# Where the flow is reversed, the lift's in-plane part changes sign with u_T
# (the lift is normal to the flow, |u_T| (u_P + u_T theta)), as does the
# drag's direction; there Bailey takes alpha' = -alpha (p. 213), so the d0 and
# d2 parts of c_d keep their sign and the d1 part flips with alpha. Each
# region term is the polynomial's, subtracted twice. Its lowest order is
# mu^2 lam^2 (the region's area is O(mu^2) and u_P^2 -> lam^2 at the hub).
w_n = u_P + u_T * theta  # u_T alpha: polynomial, so u_T^2 c_d is too
accel_poly = u_P**2 + u_T * u_P * theta
accel = avg(sp.integrate(sp.expand(x * accel_poly), (x, 0, B)))
accel -= 2 * reversed_region(accel_poly)
d0, d1, d2 = sp.symbols("delta0 delta1 delta2")
decel = avg(
    sp.integrate(sp.expand(x * (d0 * u_T**2 + d1 * u_T * w_n + d2 * w_n**2)), (x, 0, 1))
)
decel -= 2 * reversed_region(d0 * u_T**2 + d2 * w_n**2)
accel = truncate(accel.subs(flap))
decel = truncate(decel.subs(flap))

monomials = {
    "t41": lam**2, "t42": lam * th0, "t43": lam * th1,
    "t44": th0**2, "t45": th0 * th1, "t46": th1**2,
}
derived = {}
poly = sp.Poly(accel, lam, th0, th1)
for name, m in monomials.items():
    derived[name] = sp.simplify(poly.coeff_monomial(m))
poly = sp.Poly(decel, d0, d1, d2, lam, th0, th1)
derived["t51"] = poly.coeff_monomial(d0)
for name, m in {"t52": lam, "t53": th0, "t54": th1}.items():
    derived[name] = poly.coeff_monomial(d1 * m)
for name, m in {
    "t55": lam**2, "t56": lam * th0, "t57": lam * th1,
    "t58": th0**2, "t59": th0 * th1, "t510": th1**2,
}.items():
    derived[name] = poly.coeff_monomial(d2 * m)

# Bailey's appendix (NACA Report 716 pp. 223-224), the mu^0 and mu^2 parts.
R = sp.Rational
bailey = {
    "t41": B**2 / 2 + (R(5, 4) + g**2 * B**8 / 1296) * mu**2,
    "t42": B**3 / 3 + (R(8, 3) * B + g**2 * B**9 / 864) * mu**2,
    "t43": B**4 / 4 + (2 * B**2 + g**2 * B**10 / 1080) * mu**2,
    "t44": (R(8, 9) * B**2 + g**2 * B**10 / 2304) * mu**2,
    "t45": (R(4, 3) * B**3 + g**2 * B**11 / 1440) * mu**2,
    "t46": (B**4 / 2 + g**2 * B**12 / 3600) * mu**2,
    "t51": R(1, 4) + mu**2 / 4,
    "t52": R(1, 3),
    "t53": R(1, 4) + mu**2 / 4,
    "t54": R(1, 5) + mu**2 / 6,
    "t55": R(1, 2)
    + (-R(1, 4) + 1 / B**2 + 1 / (2 * B**4) + g**2 * (B**4 / 162 - B**5 / 81 + B**6 / 144)) * mu**2,
    "t56": R(2, 3)
    + (R(4, 3) / B + R(4, 3) / B**3 + g**2 * (B**5 / 108 - B**6 / 54 + B**7 / 96)) * mu**2,
    "t57": R(1, 2)
    + (1 + 1 / B**2 + g**2 * (B**6 / 135 - 2 * B**7 / 135 + B**8 / 120)) * mu**2,
    "t58": R(1, 4)
    + (R(1, 4) + R(8, 9) / B**2 + g**2 * (B**6 / 288 - B**7 / 144 + B**8 / 256)) * mu**2,
    "t59": R(2, 5)
    + (R(1, 3) + R(4, 3) / B + g**2 * (B**7 / 180 - B**8 / 90 + B**9 / 160)) * mu**2,
    "t510": R(1, 6) + (R(5, 8) + g**2 * (B**8 / 450 - B**9 / 225 + B**10 / 400)) * mu**2,
}

ok = True
for name in bailey:
    diff = sp.simplify(sp.expand(derived[name] - bailey[name]))
    status = "matches Bailey" if diff == 0 else f"DIFFERS by {diff}"
    ok = ok and diff == 0
    print(f"{name:5s} = {sp.collect(sp.expand(derived[name]), mu)}    [{status}]")
sys.exit(0 if ok else 1)
