"""Structural replica of the ``numbers`` module (Python 3.14).

The numeric tower: Number -> Complex -> Real -> Rational -> Integral.
All bodies are ``pass``; abstract methods are marked with
``@abstractmethod`` to mirror the real ABC hierarchy.
"""

from __future__ import annotations

from abc import ABCMeta, abstractmethod
from typing import Any


class Number(metaclass=ABCMeta):
    __slots__ = ()

    @abstractmethod
    def __hash__(self) -> int:
        # Return the number's hash value.
        pass


class Complex(Number):
    __slots__ = ()

    @property
    @abstractmethod
    def real(self) -> Any:
        # Return the real part of the number.
        pass

    @property
    @abstractmethod
    def imag(self) -> Any:
        # Return the imaginary part of the number.
        pass

    @abstractmethod
    def __complex__(self) -> complex:
        # Return self converted to a built-in complex.
        pass

    def __bool__(self) -> bool:
        # Return whether the number is nonzero.
        pass

    @abstractmethod
    def __add__(self, other: Any) -> Any:
        # Return self + other.
        pass

    @abstractmethod
    def __radd__(self, other: Any) -> Any:
        # Return other + self.
        pass

    @abstractmethod
    def __neg__(self) -> Any:
        # Return -self.
        pass

    @abstractmethod
    def __pos__(self) -> Any:
        # Return +self.
        pass

    def __sub__(self, other: Any) -> Any:
        # Return self - other.
        pass

    def __rsub__(self, other: Any) -> Any:
        # Return other - self.
        pass

    @abstractmethod
    def __mul__(self, other: Any) -> Any:
        # Return self * other.
        pass

    @abstractmethod
    def __rmul__(self, other: Any) -> Any:
        # Return other * self.
        pass

    @abstractmethod
    def __truediv__(self, other: Any) -> Any:
        # Return self / other.
        pass

    @abstractmethod
    def __rtruediv__(self, other: Any) -> Any:
        # Return other / self.
        pass

    @abstractmethod
    def __pow__(self, exponent: Any) -> Any:
        # Return self ** exponent.
        pass

    @abstractmethod
    def __rpow__(self, base: Any) -> Any:
        # Return base ** self.
        pass

    @abstractmethod
    def __abs__(self) -> Any:
        # Return the magnitude of self.
        pass

    @abstractmethod
    def conjugate(self) -> Any:
        # Return the complex conjugate of self.
        pass

    @abstractmethod
    def __eq__(self, other: Any) -> bool:
        # Return whether self equals other.
        pass


class Real(Complex):
    __slots__ = ()

    @abstractmethod
    def __float__(self) -> float:
        # Return self converted to a built-in float.
        pass

    @abstractmethod
    def __trunc__(self) -> int:
        # Return self truncated toward zero to an integer.
        pass

    @abstractmethod
    def __floor__(self) -> int:
        # Return the greatest integer less than or equal to self.
        pass

    @abstractmethod
    def __ceil__(self) -> int:
        # Return the least integer greater than or equal to self.
        pass

    @abstractmethod
    def __round__(self, ndigits: int | None = None) -> Any:
        # Round self to ndigits precision (or to an integer if omitted).
        pass

    def __divmod__(self, other: Any) -> tuple[Any, Any]:
        # Return (self // other, self % other).
        pass

    def __rdivmod__(self, other: Any) -> tuple[Any, Any]:
        # Return (other // self, other % self).
        pass

    @abstractmethod
    def __floordiv__(self, other: Any) -> Any:
        # Return self // other.
        pass

    @abstractmethod
    def __rfloordiv__(self, other: Any) -> Any:
        # Return other // self.
        pass

    @abstractmethod
    def __mod__(self, other: Any) -> Any:
        # Return self % other.
        pass

    @abstractmethod
    def __rmod__(self, other: Any) -> Any:
        # Return other % self.
        pass

    @abstractmethod
    def __lt__(self, other: Any) -> bool:
        # Return whether self is less than other.
        pass

    @abstractmethod
    def __le__(self, other: Any) -> bool:
        # Return whether self is less than or equal to other.
        pass

    def __complex__(self) -> complex:
        # Return self converted to a built-in complex with zero imaginary part.
        pass

    @property
    def real(self) -> Any:
        # Return self, since a real number is its own real part.
        pass

    @property
    def imag(self) -> Any:
        # Return zero, the imaginary part of a real number.
        pass

    def conjugate(self) -> Any:
        # Return self, since real numbers are their own conjugate.
        pass


class Rational(Real):
    __slots__ = ()

    @property
    @abstractmethod
    def numerator(self) -> int:
        # Return the numerator of the number in lowest terms.
        pass

    @property
    @abstractmethod
    def denominator(self) -> int:
        # Return the denominator of the number in lowest terms.
        pass

    def __float__(self) -> float:
        # Return the value as a float, computed from numerator / denominator.
        pass


class Integral(Rational):
    __slots__ = ()

    @abstractmethod
    def __int__(self) -> int:
        # Return self converted to a built-in int.
        pass

    def __index__(self) -> int:
        # Return self converted losslessly to an integer, e.g. for slicing.
        pass

    @abstractmethod
    def __pow__(self, exponent: Any, modulus: Any = None) -> Any:
        # Return self ** exponent, optionally modulo modulus.
        pass

    @abstractmethod
    def __lshift__(self, other: Any) -> Any:
        # Return self shifted left by other bits.
        pass

    @abstractmethod
    def __rlshift__(self, other: Any) -> Any:
        # Return other shifted left by self bits.
        pass

    @abstractmethod
    def __rshift__(self, other: Any) -> Any:
        # Return self shifted right by other bits.
        pass

    @abstractmethod
    def __rrshift__(self, other: Any) -> Any:
        # Return other shifted right by self bits.
        pass

    @abstractmethod
    def __and__(self, other: Any) -> Any:
        # Return the bitwise AND of self and other.
        pass

    @abstractmethod
    def __rand__(self, other: Any) -> Any:
        # Return the bitwise AND of other and self.
        pass

    @abstractmethod
    def __xor__(self, other: Any) -> Any:
        # Return the bitwise XOR of self and other.
        pass

    @abstractmethod
    def __rxor__(self, other: Any) -> Any:
        # Return the bitwise XOR of other and self.
        pass

    @abstractmethod
    def __or__(self, other: Any) -> Any:
        # Return the bitwise OR of self and other.
        pass

    @abstractmethod
    def __ror__(self, other: Any) -> Any:
        # Return the bitwise OR of other and self.
        pass

    @abstractmethod
    def __invert__(self) -> Any:
        # Return the bitwise inversion of self.
        pass

    def __float__(self) -> float:
        # Return self converted to a built-in float.
        pass

    @property
    def numerator(self) -> int:
        # Return self, since an integer is its own numerator.
        pass

    @property
    def denominator(self) -> int:
        # Return 1, the denominator of an integer.
        pass
