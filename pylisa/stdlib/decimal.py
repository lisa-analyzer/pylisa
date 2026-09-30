"""Structural replica of the ``decimal`` module (Python 3.14)."""

from __future__ import annotations

from typing import Any, Sequence

# ---------------------------------------------------------------------------
# Rounding mode constants
# ---------------------------------------------------------------------------

ROUND_CEILING: str
ROUND_DOWN: str
ROUND_FLOOR: str
ROUND_HALF_DOWN: str
ROUND_HALF_EVEN: str
ROUND_HALF_UP: str
ROUND_UP: str
ROUND_05UP: str

HAVE_THREADS: bool
HAVE_CONTEXTVAR: bool
MAX_EMAX: int
MIN_EMIN: int
MAX_PREC: int
MIN_ETINY: int


# ---------------------------------------------------------------------------
# Exceptions (signals)
# ---------------------------------------------------------------------------

class DecimalException(ArithmeticError):
    pass


class Clamped(DecimalException):
    pass


class InvalidOperation(DecimalException):
    pass


class ConversionSyntax(InvalidOperation):
    pass


class DivisionImpossible(InvalidOperation):
    pass


class DivisionUndefined(InvalidOperation, ZeroDivisionError):
    pass


class InvalidContext(InvalidOperation):
    pass


class DivisionByZero(DecimalException, ZeroDivisionError):
    pass


class Inexact(DecimalException):
    pass


class Rounded(DecimalException):
    pass


class Subnormal(DecimalException):
    pass


class Overflow(Inexact, Rounded):
    pass


class Underflow(Inexact, Rounded, Subnormal):
    pass


class FloatOperation(DecimalException, TypeError):
    pass


# ---------------------------------------------------------------------------
# Decimal
# ---------------------------------------------------------------------------

class Decimal(object):
    def __init__(self, value: Any = "0", context: "Context | None" = None) -> None:
        # Initialize the decimal from a string, integer, float or tuple.
        pass

    @classmethod
    def from_float(cls, f: float, /) -> "Decimal":
        # Create a Decimal from a float, preserving its exact binary value.
        pass

    def as_tuple(self) -> tuple[int, tuple[int, ...], int]:
        # Return a (sign, digits, exponent) named tuple representation.
        pass

    def as_integer_ratio(self) -> tuple[int, int]:
        # Return a pair of integers whose ratio exactly equals this decimal.
        pass

    def to_eng_string(self, context: "Context | None" = None) -> str:
        # Convert to a string, using engineering notation if needed.
        pass

    def to_integral(self, rounding: str | None = None, context: "Context | None" = None) -> "Decimal":
        # Round to the nearest integer without signaling Inexact or Rounded.
        pass

    def to_integral_exact(self, rounding: str | None = None, context: "Context | None" = None) -> "Decimal":
        # Round to the nearest integer, signaling Inexact and Rounded if needed.
        pass

    def to_integral_value(self, rounding: str | None = None, context: "Context | None" = None) -> "Decimal":
        # Round to the nearest integer without signaling Inexact or Rounded.
        pass

    def adjusted(self) -> int:
        # Return the exponent of the leading digit in scientific notation.
        pass

    def canonical(self) -> "Decimal":
        # Return the canonical form of the decimal (it is always already canonical).
        pass

    def compare(self, other: Any, context: "Context | None" = None) -> "Decimal":
        # Compare self and other, returning a Decimal -1, 0 or 1.
        pass

    def compare_signal(self, other: Any, context: "Context | None" = None) -> "Decimal":
        # Like compare(), but always signals if either operand is a NaN.
        pass

    def compare_total(self, other: Any, context: "Context | None" = None) -> "Decimal":
        # Compare self and other using their abstract representation, total ordering.
        pass

    def compare_total_mag(self, other: Any, context: "Context | None" = None) -> "Decimal":
        # Like compare_total(), but comparing absolute values.
        pass

    def conjugate(self) -> "Decimal":
        # Return self, since decimals have no imaginary component.
        pass

    def copy_abs(self) -> "Decimal":
        # Return a copy with the sign set to positive.
        pass

    def copy_negate(self) -> "Decimal":
        # Return a copy with the sign inverted.
        pass

    def copy_sign(self, other: Any, context: "Context | None" = None) -> "Decimal":
        # Return a copy of self with the sign taken from other.
        pass

    def exp(self, context: "Context | None" = None) -> "Decimal":
        # Return e raised to the power of self.
        pass

    def fma(self, other: Any, third: Any, context: "Context | None" = None) -> "Decimal":
        # Return self * other + third with a single, correctly-rounded operation.
        pass

    def is_canonical(self) -> bool:
        # Return whether self is in canonical form (always True for Decimal).
        pass

    def is_finite(self) -> bool:
        # Return whether self is neither infinite nor a NaN.
        pass

    def is_infinite(self) -> bool:
        # Return whether self is positive or negative infinity.
        pass

    def is_nan(self) -> bool:
        # Return whether self is a quiet or signaling NaN.
        pass

    def is_normal(self, context: "Context | None" = None) -> bool:
        # Return whether self is a normal, finite, nonzero number.
        pass

    def is_qnan(self) -> bool:
        # Return whether self is a quiet NaN.
        pass

    def is_signed(self) -> bool:
        # Return whether self has a negative sign.
        pass

    def is_snan(self) -> bool:
        # Return whether self is a signaling NaN.
        pass

    def is_subnormal(self, context: "Context | None" = None) -> bool:
        # Return whether self is subnormal.
        pass

    def is_zero(self) -> bool:
        # Return whether self is zero.
        pass

    def ln(self, context: "Context | None" = None) -> "Decimal":
        # Return the natural logarithm of self.
        pass

    def log10(self, context: "Context | None" = None) -> "Decimal":
        # Return the base-10 logarithm of self.
        pass

    def logb(self, context: "Context | None" = None) -> "Decimal":
        # Return the exponent of the leading digit of self.
        pass

    def logical_and(self, other: Any, context: "Context | None" = None) -> "Decimal":
        # Return the digit-wise logical AND of self and other.
        pass

    def logical_invert(self, context: "Context | None" = None) -> "Decimal":
        # Return the digit-wise logical inversion of self.
        pass

    def logical_or(self, other: Any, context: "Context | None" = None) -> "Decimal":
        # Return the digit-wise logical OR of self and other.
        pass

    def logical_xor(self, other: Any, context: "Context | None" = None) -> "Decimal":
        # Return the digit-wise logical XOR of self and other.
        pass

    def max(self, other: Any, context: "Context | None" = None) -> "Decimal":
        # Return the larger of self and other.
        pass

    def max_mag(self, other: Any, context: "Context | None" = None) -> "Decimal":
        # Return the operand with the larger absolute value.
        pass

    def min(self, other: Any, context: "Context | None" = None) -> "Decimal":
        # Return the smaller of self and other.
        pass

    def min_mag(self, other: Any, context: "Context | None" = None) -> "Decimal":
        # Return the operand with the smaller absolute value.
        pass

    def next_minus(self, context: "Context | None" = None) -> "Decimal":
        # Return the largest representable number smaller than self.
        pass

    def next_plus(self, context: "Context | None" = None) -> "Decimal":
        # Return the smallest representable number larger than self.
        pass

    def next_toward(self, other: Any, context: "Context | None" = None) -> "Decimal":
        # Return the next representable number of self toward other.
        pass

    def normalize(self, context: "Context | None" = None) -> "Decimal":
        # Return an equivalent value with trailing zeros removed.
        pass

    def number_class(self, context: "Context | None" = None) -> str:
        # Return a string describing the class of self (e.g. "+Normal").
        pass

    def quantize(self, exp: Any, rounding: str | None = None, context: "Context | None" = None) -> "Decimal":
        # Return a value equal to self after rounding to the exponent of exp.
        pass

    def radix(self) -> "Decimal":
        # Return Decimal(10), the radix used by this arithmetic.
        pass

    def remainder_near(self, other: Any, context: "Context | None" = None) -> "Decimal":
        # Return the remainder from dividing self by other, rounded to nearest.
        pass

    def rotate(self, other: Any, context: "Context | None" = None) -> "Decimal":
        # Return self with digits rotated by the number of places given by other.
        pass

    def same_quantum(self, other: Any, context: "Context | None" = None) -> bool:
        # Return whether self and other have the same exponent.
        pass

    def scaleb(self, other: Any, context: "Context | None" = None) -> "Decimal":
        # Return self with its exponent adjusted by other.
        pass

    def shift(self, other: Any, context: "Context | None" = None) -> "Decimal":
        # Return self with digits shifted by the number of places given by other.
        pass

    def sqrt(self, context: "Context | None" = None) -> "Decimal":
        # Return the square root of self.
        pass

    def __abs__(self) -> "Decimal":
        # Return the absolute value of self.
        pass

    def __add__(self, other: Any) -> "Decimal":
        # Return self + other.
        pass

    def __radd__(self, other: Any) -> "Decimal":
        # Return other + self.
        pass

    def __sub__(self, other: Any) -> "Decimal":
        # Return self - other.
        pass

    def __rsub__(self, other: Any) -> "Decimal":
        # Return other - self.
        pass

    def __mul__(self, other: Any) -> "Decimal":
        # Return self * other.
        pass

    def __rmul__(self, other: Any) -> "Decimal":
        # Return other * self.
        pass

    def __truediv__(self, other: Any) -> "Decimal":
        # Return self / other.
        pass

    def __rtruediv__(self, other: Any) -> "Decimal":
        # Return other / self.
        pass

    def __floordiv__(self, other: Any) -> "Decimal":
        # Return self // other.
        pass

    def __rfloordiv__(self, other: Any) -> "Decimal":
        # Return other // self.
        pass

    def __mod__(self, other: Any) -> "Decimal":
        # Return self % other.
        pass

    def __rmod__(self, other: Any) -> "Decimal":
        # Return other % self.
        pass

    def __divmod__(self, other: Any) -> tuple["Decimal", "Decimal"]:
        # Return (self // other, self % other).
        pass

    def __pow__(self, other: Any, modulo: Any = None) -> "Decimal":
        # Return self ** other, optionally modulo a third value.
        pass

    def __rpow__(self, other: Any) -> "Decimal":
        # Return other ** self.
        pass

    def __neg__(self) -> "Decimal":
        # Return -self.
        pass

    def __pos__(self) -> "Decimal":
        # Return +self.
        pass

    def __eq__(self, other: Any) -> bool:
        # Return whether self equals other.
        pass

    def __lt__(self, other: Any) -> bool:
        # Return whether self is less than other.
        pass

    def __le__(self, other: Any) -> bool:
        # Return whether self is less than or equal to other.
        pass

    def __gt__(self, other: Any) -> bool:
        # Return whether self is greater than other.
        pass

    def __ge__(self, other: Any) -> bool:
        # Return whether self is greater than or equal to other.
        pass

    def __hash__(self) -> int:
        # Return the hash of self, consistent with numerically equal ints/floats.
        pass

    def __int__(self) -> int:
        # Return self truncated to a built-in int.
        pass

    def __float__(self) -> float:
        # Return self converted to a built-in float.
        pass

    def __round__(self, ndigits: int | None = None) -> Any:
        # Round self to ndigits precision (or to an integer if omitted).
        pass

    def __str__(self) -> str:
        # Return the string representation of self.
        pass

    def __repr__(self) -> str:
        # Return the official string representation of self.
        pass

    def __reduce__(self) -> Any:
        # Return state information used for pickling.
        pass

    def __copy__(self) -> "Decimal":
        # Return self, since Decimal instances are immutable.
        pass

    def __deepcopy__(self, memo: Any) -> "Decimal":
        # Return self, since Decimal instances are immutable.
        pass


# ---------------------------------------------------------------------------
# Context
# ---------------------------------------------------------------------------

class Context(object):
    prec: int
    rounding: str
    Emin: int
    Emax: int
    capitals: int
    clamp: int
    traps: dict[type, bool]
    flags: dict[type, bool]

    def __init__(
        self,
        prec: int | None = None,
        rounding: str | None = None,
        Emin: int | None = None,
        Emax: int | None = None,
        capitals: int | None = None,
        clamp: int | None = None,
        flags: Sequence[type] | dict[type, bool] | None = None,
        traps: Sequence[type] | dict[type, bool] | None = None,
    ) -> None:
        # Initialize the arithmetic context (precision, rounding, traps, ...).
        pass

    def clear_flags(self) -> None:
        # Reset all status flags to False.
        pass

    def clear_traps(self) -> None:
        # Reset all trap enablers to False.
        pass

    def copy(self) -> "Context":
        # Return a copy of this context.
        pass

    def copy_decimal(self, value: Decimal) -> Decimal:
        # Return a copy of a Decimal value.
        pass

    def create_decimal(self, value: Any = "0") -> Decimal:
        # Create a new Decimal from value, applying this context's precision.
        pass

    def create_decimal_from_float(self, f: float) -> Decimal:
        # Create a new Decimal from a float, applying this context's precision.
        pass

    def Etiny(self) -> int:
        # Return the minimum exponent of a subnormal result for this context.
        pass

    def Etop(self) -> int:
        # Return the maximum exponent for a normal result for this context.
        pass

    def abs(self, x: Any) -> Decimal:
        # Return the absolute value of x, per this context.
        pass

    def add(self, x: Any, y: Any) -> Decimal:
        # Return x + y, per this context.
        pass

    def canonical(self, x: Any) -> Decimal:
        # Return the canonical form of x.
        pass

    def compare(self, x: Any, y: Any) -> Decimal:
        # Compare x and y, per this context.
        pass

    def compare_signal(self, x: Any, y: Any) -> Decimal:
        # Compare x and y, always signaling on NaN operands.
        pass

    def compare_total(self, x: Any, y: Any) -> Decimal:
        # Compare x and y using total ordering.
        pass

    def compare_total_mag(self, x: Any, y: Any) -> Decimal:
        # Compare the absolute values of x and y using total ordering.
        pass

    def copy_abs(self, x: Any) -> Decimal:
        # Return a copy of x with a positive sign.
        pass

    def copy_negate(self, x: Any) -> Decimal:
        # Return a copy of x with the sign inverted.
        pass

    def copy_sign(self, x: Any, y: Any) -> Decimal:
        # Return a copy of x with the sign taken from y.
        pass

    def divide(self, x: Any, y: Any) -> Decimal:
        # Return x / y, per this context.
        pass

    def divide_int(self, x: Any, y: Any) -> Decimal:
        # Return the integer part of x / y.
        pass

    def divmod(self, x: Any, y: Any) -> tuple[Decimal, Decimal]:
        # Return (x // y, x % y), per this context.
        pass

    def exp(self, x: Any) -> Decimal:
        # Return e raised to the power of x.
        pass

    def fma(self, x: Any, y: Any, z: Any) -> Decimal:
        # Return x * y + z with a single, correctly-rounded operation.
        pass

    def is_canonical(self, x: Any) -> bool:
        # Return whether x is in canonical form.
        pass

    def is_finite(self, x: Any) -> bool:
        # Return whether x is neither infinite nor a NaN.
        pass

    def is_infinite(self, x: Any) -> bool:
        # Return whether x is positive or negative infinity.
        pass

    def is_nan(self, x: Any) -> bool:
        # Return whether x is a quiet or signaling NaN.
        pass

    def is_normal(self, x: Any) -> bool:
        # Return whether x is a normal, finite, nonzero number.
        pass

    def is_qnan(self, x: Any) -> bool:
        # Return whether x is a quiet NaN.
        pass

    def is_signed(self, x: Any) -> bool:
        # Return whether x has a negative sign.
        pass

    def is_snan(self, x: Any) -> bool:
        # Return whether x is a signaling NaN.
        pass

    def is_subnormal(self, x: Any) -> bool:
        # Return whether x is subnormal.
        pass

    def is_zero(self, x: Any) -> bool:
        # Return whether x is zero.
        pass

    def ln(self, x: Any) -> Decimal:
        # Return the natural logarithm of x.
        pass

    def log10(self, x: Any) -> Decimal:
        # Return the base-10 logarithm of x.
        pass

    def logb(self, x: Any) -> Decimal:
        # Return the exponent of the leading digit of x.
        pass

    def logical_and(self, x: Any, y: Any) -> Decimal:
        # Return the digit-wise logical AND of x and y.
        pass

    def logical_invert(self, x: Any) -> Decimal:
        # Return the digit-wise logical inversion of x.
        pass

    def logical_or(self, x: Any, y: Any) -> Decimal:
        # Return the digit-wise logical OR of x and y.
        pass

    def logical_xor(self, x: Any, y: Any) -> Decimal:
        # Return the digit-wise logical XOR of x and y.
        pass

    def max(self, x: Any, y: Any) -> Decimal:
        # Return the larger of x and y.
        pass

    def max_mag(self, x: Any, y: Any) -> Decimal:
        # Return the operand with the larger absolute value.
        pass

    def min(self, x: Any, y: Any) -> Decimal:
        # Return the smaller of x and y.
        pass

    def min_mag(self, x: Any, y: Any) -> Decimal:
        # Return the operand with the smaller absolute value.
        pass

    def minus(self, x: Any) -> Decimal:
        # Return -x, per this context (a NaN-propagating negation).
        pass

    def multiply(self, x: Any, y: Any) -> Decimal:
        # Return x * y, per this context.
        pass

    def next_minus(self, x: Any) -> Decimal:
        # Return the largest representable number smaller than x.
        pass

    def next_plus(self, x: Any) -> Decimal:
        # Return the smallest representable number larger than x.
        pass

    def next_toward(self, x: Any, y: Any) -> Decimal:
        # Return the next representable number of x toward y.
        pass

    def normalize(self, x: Any) -> Decimal:
        # Return x with trailing zeros removed.
        pass

    def number_class(self, x: Any) -> str:
        # Return a string describing the class of x (e.g. "+Normal").
        pass

    def plus(self, x: Any) -> Decimal:
        # Return +x, per this context (a NaN-propagating no-op that rounds).
        pass

    def power(self, x: Any, y: Any, modulo: Any = None) -> Decimal:
        # Return x ** y, optionally modulo a third value.
        pass

    def quantize(self, x: Any, y: Any) -> Decimal:
        # Return x rounded to have the same exponent as y.
        pass

    def radix(self) -> Decimal:
        # Return Decimal(10), the radix used by this arithmetic.
        pass

    def remainder(self, x: Any, y: Any) -> Decimal:
        # Return the remainder from dividing x by y.
        pass

    def remainder_near(self, x: Any, y: Any) -> Decimal:
        # Return the remainder from dividing x by y, rounded to nearest.
        pass

    def rotate(self, x: Any, y: Any) -> Decimal:
        # Return x with digits rotated by the number of places given by y.
        pass

    def same_quantum(self, x: Any, y: Any) -> bool:
        # Return whether x and y have the same exponent.
        pass

    def scaleb(self, x: Any, y: Any) -> Decimal:
        # Return x with its exponent adjusted by y.
        pass

    def shift(self, x: Any, y: Any) -> Decimal:
        # Return x with digits shifted by the number of places given by y.
        pass

    def sqrt(self, x: Any) -> Decimal:
        # Return the square root of x.
        pass

    def subtract(self, x: Any, y: Any) -> Decimal:
        # Return x - y, per this context.
        pass

    def to_eng_string(self, x: Any) -> str:
        # Convert x to a string, using engineering notation if needed.
        pass

    def to_integral_exact(self, x: Any) -> Decimal:
        # Round x to the nearest integer, signaling Inexact and Rounded if needed.
        pass

    def to_sci_string(self, x: Any) -> str:
        # Convert x to a string, using scientific notation if needed.
        pass


# ---------------------------------------------------------------------------
# Context management
# ---------------------------------------------------------------------------

class localcontext(object):
    def __init__(
        self,
        ctx: Context | None = None,
        prec: int | None = None,
        rounding: str | None = None,
        Emin: int | None = None,
        Emax: int | None = None,
        capitals: int | None = None,
        clamp: int | None = None,
        traps: Sequence[type] | dict[type, bool] | None = None,
    ) -> None:
        # Prepare a temporary decimal context to install for the ``with`` block.
        pass

    def __enter__(self) -> Context:
        # Install the temporary context as the thread's active context.
        pass

    def __exit__(self, *exc: Any) -> None:
        # Restore the previously active decimal context.
        pass


def getcontext() -> Context:
    # Return the current thread's active decimal context.
    pass

def setcontext(context: Context) -> None:
    # Set the current thread's active decimal context.
    pass


DefaultContext: Context
BasicContext: Context
ExtendedContext: Context
