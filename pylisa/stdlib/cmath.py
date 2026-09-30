"""Structural replica of the ``cmath`` module (Python 3.14).

Purely a function/constant library operating on complex numbers -- no
classes are defined by the real module.
"""

from __future__ import annotations

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

pi: float
e: float
tau: float
inf: float
infj: complex
nan: float
nanj: complex

# ---------------------------------------------------------------------------
# Conversions to and from polar coordinates
# ---------------------------------------------------------------------------

def phase(x: complex, /) -> float:
    # Return the phase (argument) of x, in radians.
    pass

def polar(x: complex, /) -> tuple[float, float]:
    # Return the polar representation of x as (modulus, phase).
    pass

def rect(r: float, phi: float, /) -> complex:
    # Return the complex number with polar coordinates (modulus r, phase phi).
    pass

# ---------------------------------------------------------------------------
# Power and logarithmic functions
# ---------------------------------------------------------------------------

def exp(x: complex, /) -> complex:
    # Return e raised to the power x.
    pass

def log(x: complex, base: complex = ..., /) -> complex:
    # Return the logarithm of x to the given base (natural log by default).
    pass

def log10(x: complex, /) -> complex:
    # Return the base-10 logarithm of x.
    pass

def sqrt(x: complex, /) -> complex:
    # Return the square root of x.
    pass

# ---------------------------------------------------------------------------
# Trigonometric functions
# ---------------------------------------------------------------------------

def acos(x: complex, /) -> complex:
    # Return the arc cosine of x.
    pass

def asin(x: complex, /) -> complex:
    # Return the arc sine of x.
    pass

def atan(x: complex, /) -> complex:
    # Return the arc tangent of x.
    pass

def cos(x: complex, /) -> complex:
    # Return the cosine of x.
    pass

def sin(x: complex, /) -> complex:
    # Return the sine of x.
    pass

def tan(x: complex, /) -> complex:
    # Return the tangent of x.
    pass

# ---------------------------------------------------------------------------
# Hyperbolic functions
# ---------------------------------------------------------------------------

def acosh(x: complex, /) -> complex:
    # Return the inverse hyperbolic cosine of x.
    pass

def asinh(x: complex, /) -> complex:
    # Return the inverse hyperbolic sine of x.
    pass

def atanh(x: complex, /) -> complex:
    # Return the inverse hyperbolic tangent of x.
    pass

def cosh(x: complex, /) -> complex:
    # Return the hyperbolic cosine of x.
    pass

def sinh(x: complex, /) -> complex:
    # Return the hyperbolic sine of x.
    pass

def tanh(x: complex, /) -> complex:
    # Return the hyperbolic tangent of x.
    pass

# ---------------------------------------------------------------------------
# Classification functions
# ---------------------------------------------------------------------------

def isfinite(x: complex, /) -> bool:
    # Return whether both components of x are finite.
    pass

def isinf(x: complex, /) -> bool:
    # Return whether either component of x is infinite.
    pass

def isnan(x: complex, /) -> bool:
    # Return whether either component of x is a NaN.
    pass

def isclose(a: complex, b: complex, *, rel_tol: float = 1e-09, abs_tol: float = 0.0) -> bool:
    # Return whether a and b are approximately equal within given tolerances.
    pass
