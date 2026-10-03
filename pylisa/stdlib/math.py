"""Structural replica of the ``math`` module (Python 3.14).

Purely a function/constant library -- no classes are defined by the real
module, so none are stubbed here.
"""

from __future__ import annotations

from typing import Iterable, SupportsFloat, SupportsIndex

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

pi: float
e: float
tau: float
inf: float
nan: float

# ---------------------------------------------------------------------------
# Number-theoretic and representation functions
# ---------------------------------------------------------------------------

def ceil(x: SupportsFloat, /) -> int:
    # Return the smallest integer greater than or equal to x.
    pass

def comb(n: SupportsIndex, k: SupportsIndex, /) -> int:
    # Return the number of ways to choose k items from n without repetition or order.
    pass

def copysign(x: SupportsFloat, y: SupportsFloat, /) -> float:
    # Return a float with the magnitude of x and the sign of y.
    pass

def fabs(x: SupportsFloat, /) -> float:
    # Return the absolute value of x as a float.
    pass

def factorial(n: SupportsIndex, /) -> int:
    # Return n factorial.
    pass

def floor(x: SupportsFloat, /) -> int:
    # Return the largest integer less than or equal to x.
    pass

def fmod(x: SupportsFloat, y: SupportsFloat, /) -> float:
    # Return the C-library floating-point remainder of x / y.
    pass

def frexp(x: SupportsFloat, /) -> tuple[float, int]:
    # Return the mantissa and exponent of x as (m, e) such that x == m * 2**e.
    pass

def fsum(iterable: Iterable[SupportsFloat], /) -> float:
    # Return an accurate floating-point sum of values in iterable.
    pass

def gcd(*integers: SupportsIndex) -> int:
    # Return the greatest common divisor of the given integers.
    pass

def isclose(a: SupportsFloat, b: SupportsFloat, *, rel_tol: float = 1e-09, abs_tol: float = 0.0) -> bool:
    # Return whether a and b are approximately equal within given tolerances.
    pass

def isfinite(x: SupportsFloat, /) -> bool:
    # Return whether x is neither infinite nor NaN.
    pass

def isinf(x: SupportsFloat, /) -> bool:
    # Return whether x is positive or negative infinity.
    pass

def isnan(x: SupportsFloat, /) -> bool:
    # Return whether x is a NaN (not a number).
    pass

def isqrt(n: SupportsIndex, /) -> int:
    # Return the integer square root of a non-negative integer n.
    pass

def lcm(*integers: SupportsIndex) -> int:
    # Return the least common multiple of the given integers.
    pass

def ldexp(x: SupportsFloat, i: SupportsIndex, /) -> float:
    # Return x * (2**i); the inverse of frexp().
    pass

def modf(x: SupportsFloat, /) -> tuple[float, float]:
    # Return the fractional and integer parts of x, both as floats.
    pass

def nextafter(x: SupportsFloat, y: SupportsFloat, /, *, steps: SupportsIndex | None = None) -> float:
    # Return the floating-point value steps representable values after x, toward y.
    pass

def perm(n: SupportsIndex, k: SupportsIndex | None = None, /) -> int:
    # Return the number of ways to arrange k items out of n, order mattering.
    pass

def prod(iterable: Iterable[SupportsFloat], /, *, start: SupportsFloat = 1) -> float:
    # Return the product of all elements in iterable, starting from start.
    pass

def remainder(x: SupportsFloat, y: SupportsFloat, /) -> float:
    # Return the IEEE 754-style remainder of x with respect to y.
    pass

def sumprod(p: Iterable[SupportsFloat], q: Iterable[SupportsFloat], /) -> float:
    # Return the accurate sum of products of corresponding elements of p and q.
    pass

def trunc(x: SupportsFloat, /) -> int:
    # Return x truncated toward zero to the nearest integer.
    pass

def ulp(x: SupportsFloat, /) -> float:
    # Return the value of the least significant bit of x.
    pass

# ---------------------------------------------------------------------------
# Power and logarithmic functions
# ---------------------------------------------------------------------------

def cbrt(x: SupportsFloat, /) -> float:
    # Return the cube root of x.
    pass

def exp(x: SupportsFloat, /) -> float:
    # Return e raised to the power x.
    pass

def exp2(x: SupportsFloat, /) -> float:
    # Return 2 raised to the power x.
    pass

def expm1(x: SupportsFloat, /) -> float:
    # Return e**x - 1, computed accurately for small x.
    pass

def log(x: SupportsFloat, base: SupportsFloat = ..., /) -> float:
    # Return the logarithm of x to the given base (natural log by default).
    pass

def log1p(x: SupportsFloat, /) -> float:
    # Return the natural logarithm of 1 + x, computed accurately for small x.
    pass

def log2(x: SupportsFloat, /) -> float:
    # Return the base-2 logarithm of x.
    pass

def log10(x: SupportsFloat, /) -> float:
    # Return the base-10 logarithm of x.
    pass

def pow(x: SupportsFloat, y: SupportsFloat, /) -> float:
    # Return x raised to the power y, as a float.
    pass

def sqrt(x: SupportsFloat, /) -> float:
    # Return the square root of x.
    pass

# ---------------------------------------------------------------------------
# Trigonometric functions
# ---------------------------------------------------------------------------

def acos(x: SupportsFloat, /) -> float:
    # Return the arc cosine of x, in radians.
    pass

def asin(x: SupportsFloat, /) -> float:
    # Return the arc sine of x, in radians.
    pass

def atan(x: SupportsFloat, /) -> float:
    # Return the arc tangent of x, in radians.
    pass

def atan2(y: SupportsFloat, x: SupportsFloat, /) -> float:
    # Return atan(y / x), using the signs of both arguments to pick the quadrant.
    pass

def cos(x: SupportsFloat, /) -> float:
    # Return the cosine of x radians.
    pass

def dist(p: Iterable[SupportsFloat], q: Iterable[SupportsFloat], /) -> float:
    # Return the Euclidean distance between points p and q.
    pass

def hypot(*coordinates: SupportsFloat) -> float:
    # Return the Euclidean norm of the given coordinates.
    pass

def sin(x: SupportsFloat, /) -> float:
    # Return the sine of x radians.
    pass

def tan(x: SupportsFloat, /) -> float:
    # Return the tangent of x radians.
    pass

# ---------------------------------------------------------------------------
# Angular conversion
# ---------------------------------------------------------------------------

def degrees(x: SupportsFloat, /) -> float:
    # Convert angle x from radians to degrees.
    pass

def radians(x: SupportsFloat, /) -> float:
    # Convert angle x from degrees to radians.
    pass

# ---------------------------------------------------------------------------
# Hyperbolic functions
# ---------------------------------------------------------------------------

def acosh(x: SupportsFloat, /) -> float:
    # Return the inverse hyperbolic cosine of x.
    pass

def asinh(x: SupportsFloat, /) -> float:
    # Return the inverse hyperbolic sine of x.
    pass

def atanh(x: SupportsFloat, /) -> float:
    # Return the inverse hyperbolic tangent of x.
    pass

def cosh(x: SupportsFloat, /) -> float:
    # Return the hyperbolic cosine of x.
    pass

def sinh(x: SupportsFloat, /) -> float:
    # Return the hyperbolic sine of x.
    pass

def tanh(x: SupportsFloat, /) -> float:
    # Return the hyperbolic tangent of x.
    pass

# ---------------------------------------------------------------------------
# Special functions
# ---------------------------------------------------------------------------

def erf(x: SupportsFloat, /) -> float:
    # Return the error function at x.
    pass

def erfc(x: SupportsFloat, /) -> float:
    # Return the complementary error function at x.
    pass

def gamma(x: SupportsFloat, /) -> float:
    # Return the gamma function at x.
    pass

def lgamma(x: SupportsFloat, /) -> float:
    # Return the natural logarithm of the absolute value of the gamma function at x.
    pass
