"""Structural replica of the ``builtins`` module (Python 3.14).

Covers the "Built-in Functions", "Built-in Constants", "Built-in Types",
and "Built-in Exceptions" sections of the Python standard library
reference. Every callable body is ``pass`` -- only names, signatures,
class hierarchies and attributes are represented.
"""

from __future__ import annotations

from typing import Any, Callable, Generic, Iterable, Iterator, Mapping, TypeVar

_T = TypeVar("_T")
_S = TypeVar("_S")

# ---------------------------------------------------------------------------
# Built-in Constants
# ---------------------------------------------------------------------------
# NOTE: True, False and None are keyword literals in real Python and cannot
# be declared as names (doing so is a SyntaxError). They are omitted here;
# everything else that is a real, assignable/annotatable name is kept.

NotImplemented: "NotImplementedType"
Ellipsis: "EllipsisType"

# __debug__ is likewise omitted: real Python special-cases it the same way
# as True/False/None and rejects any assignment or annotation of the name
# (SyntaxError: cannot assign to __debug__), even bare `__debug__: bool`.
# It is a read-only bool, true unless the interpreter is run with -O.

# quit/exit/copyright/credits/license/help are added to builtins by the
# site module at interpreter start-up, not by builtins itself.


# ---------------------------------------------------------------------------
# Built-in Functions (the ones that are genuinely functions, not type
# constructors -- constructors are modelled as classes further below)
# ---------------------------------------------------------------------------

def abs(x: Any, /) -> Any:
    # Return the absolute value of x.
    pass

def aiter(async_iterable: Any, /) -> Any:
    # Return an asynchronous iterator for an asynchronous iterable.
    pass

def all(iterable: Iterable[object], /) -> bool:
    # Return True if all elements of the iterable are truthy (or it is empty).
    pass

def anext(async_iterator: Any, /, default: Any = ...) -> Any:
    # Retrieve the next item from an asynchronous iterator, or default.
    pass

def any(iterable: Iterable[object], /) -> bool:
    # Return True if any element of the iterable is truthy.
    pass

def ascii(obj: Any, /) -> str:
    # Return a repr()-like string with non-ASCII characters escaped.
    pass

def bin(number: Any, /) -> str:
    # Convert an integer to a binary string prefixed with '0b'.
    pass

def breakpoint(*args: Any, **kws: Any) -> None:
    # Drop into the debugger at the call site.
    pass

def callable(obj: Any, /) -> bool:
    # Return True if the object appears to be callable.
    pass

def chr(i: int, /) -> str:
    # Return the string of one character whose code point is i.
    pass

def compile(
    source: Any,
    filename: Any,
    mode: str,
    flags: int = 0,
    dont_inherit: bool = False,
    optimize: int = -1,
) -> Any:
    # Compile source into a code object or AST object.
    pass

def delattr(obj: Any, name: str, /) -> None:
    # Delete the named attribute from an object.
    pass

def dir(*args: Any) -> list[str]:
    # Return a list of names in the current scope, or of an object's attributes.
    pass

def divmod(x: Any, y: Any, /) -> tuple[Any, Any]:
    # Return the pair (x // y, x % y).
    pass

def eval(source: Any, globals: dict[str, Any] | None = None, locals: Mapping[str, Any] | None = None, /) -> Any:
    # Evaluate a Python expression and return its value.
    pass

def exec(source: Any, globals: dict[str, Any] | None = None, locals: Mapping[str, Any] | None = None, /, *, closure: tuple[Any, ...] | None = None) -> None:
    # Execute Python code dynamically.
    pass

def format(value: Any, format_spec: str = "", /) -> str:
    # Convert a value to a formatted string according to format_spec.
    pass

def getattr(obj: Any, name: str, default: Any = ..., /) -> Any:
    # Return the value of the named attribute of an object.
    pass

def globals() -> dict[str, Any]:
    # Return the dictionary of the current module's global namespace.
    pass

def hasattr(obj: Any, name: str, /) -> bool:
    # Return True if the object has the named attribute.
    pass

def hash(obj: Any, /) -> int:
    # Return the hash value of an object.
    pass

def help(request: Any = ...) -> None:
    # Start the interactive help system or show help for an object.
    pass

def hex(number: Any, /) -> str:
    # Convert an integer to a hexadecimal string prefixed with '0x'.
    pass

def id(obj: Any, /) -> int:
    # Return the identity of an object (unique for its lifetime).
    pass

def input(prompt: Any = "", /) -> str:
    # Read a line from standard input, optionally after printing a prompt.
    pass

def isinstance(obj: Any, class_or_tuple: Any, /) -> bool:
    # Return True if the object is an instance of the class or tuple of classes.
    pass

def issubclass(cls: type, class_or_tuple: Any, /) -> bool:
    # Return True if the class is a subclass of the class or tuple of classes.
    pass

def iter(*args: Any) -> Iterator[Any]:
    # Return an iterator over an iterable, or from a callable/sentinel pair.
    pass

def len(obj: Any, /) -> int:
    # Return the number of items in a container.
    pass

def locals() -> dict[str, Any]:
    # Return the dictionary of the current local namespace.
    pass

def max(*args: Any, key: Callable[[Any], Any] | None = None, default: Any = ...) -> Any:
    # Return the largest of the given arguments or items in an iterable.
    pass

def min(*args: Any, key: Callable[[Any], Any] | None = None, default: Any = ...) -> Any:
    # Return the smallest of the given arguments or items in an iterable.
    pass

def next(iterator: Iterator[Any], default: Any = ..., /) -> Any:
    # Retrieve the next item from an iterator, or default if exhausted.
    pass

def oct(number: Any, /) -> str:
    # Convert an integer to an octal string prefixed with '0o'.
    pass

def open(
    file: Any,
    mode: str = "r",
    buffering: int = -1,
    encoding: str | None = None,
    errors: str | None = None,
    newline: str | None = None,
    closefd: bool = True,
    opener: Callable[[str, int], int] | None = None,
) -> Any:
    # Open a file and return a corresponding file object.
    pass

def ord(c: str, /) -> int:
    # Return the Unicode code point of a one-character string.
    pass

def pow(base: Any, exp: Any, mod: Any = None) -> Any:
    # Return base raised to the power exp, optionally modulo mod.
    pass

def print(*values: Any, sep: str | None = " ", end: str | None = "\n", file: Any = None, flush: bool = False) -> None:
    # Write values to a text stream, separated by sep and followed by end.
    pass

def repr(obj: Any, /) -> str:
    # Return a printable, unambiguous string representation of an object.
    pass

def round(number: Any, ndigits: int | None = None) -> Any:
    # Round a number to ndigits precision after the decimal point.
    pass

def setattr(obj: Any, name: str, value: Any, /) -> None:
    # Set the named attribute of an object to a value.
    pass

def sorted(iterable: Iterable[_T], /, *, key: Callable[[_T], Any] | None = None, reverse: bool = False) -> list[_T]:
    # Return a new sorted list from the items in the iterable.
    pass

def sum(iterable: Iterable[Any], /, start: Any = 0) -> Any:
    # Sum the items of an iterable, starting from start.
    pass

def vars(obj: Any = ...) -> dict[str, Any]:
    # Return the __dict__ of an object, or the local namespace if omitted.
    pass

def __import__(
    name: str,
    globals: Mapping[str, Any] | None = None,
    locals: Mapping[str, Any] | None = None,
    fromlist: Iterable[str] = (),
    level: int = 0,
) -> Any:
    # Import a module; invoked by the ``import`` statement.
    pass


# ---------------------------------------------------------------------------
# Built-in Types
# ---------------------------------------------------------------------------

class object:
    __dict__: dict[str, Any]
    __class__: type
    __doc__: str | None
    __module__: str

    def __init__(self) -> None:
        # Initialize a newly created instance (default no-op).
        pass

    def __new__(cls) -> "object":
        # Create and return a new instance of the class.
        pass

    def __repr__(self) -> str:
        # Return the official, unambiguous string representation.
        pass

    def __str__(self) -> str:
        # Return the informal, user-facing string representation.
        pass

    def __format__(self, format_spec: str) -> str:
        # Return a formatted string representation using format_spec.
        pass

    def __hash__(self) -> int:
        # Return the object's hash value.
        pass

    def __eq__(self, other: Any) -> bool:
        # Return whether this object equals other.
        pass

    def __ne__(self, other: Any) -> bool:
        # Return whether this object does not equal other.
        pass

    def __lt__(self, other: Any) -> bool:
        # Return whether this object is less than other.
        pass

    def __le__(self, other: Any) -> bool:
        # Return whether this object is less than or equal to other.
        pass

    def __gt__(self, other: Any) -> bool:
        # Return whether this object is greater than other.
        pass

    def __ge__(self, other: Any) -> bool:
        # Return whether this object is greater than or equal to other.
        pass

    def __sizeof__(self) -> int:
        # Return the size of the object in memory, in bytes.
        pass

    def __getattribute__(self, name: str) -> Any:
        # Return the value of the named attribute.
        pass

    def __setattr__(self, name: str, value: Any) -> None:
        # Set the named attribute to a value.
        pass

    def __delattr__(self, name: str) -> None:
        # Delete the named attribute.
        pass

    def __dir__(self) -> Iterable[str]:
        # Return a list of valid attribute names for the object.
        pass

    def __reduce__(self) -> Any:
        # Return state information used for pickling.
        pass

    def __reduce_ex__(self, protocol: int) -> Any:
        # Return state information for pickling, given a protocol version.
        pass

    def __init_subclass__(cls) -> None:
        # Hook called automatically whenever the class is subclassed.
        pass

    def __subclasshook__(cls, subclass: type) -> bool:
        # Customize issubclass() checks for this class.
        pass


class type(object):
    __name__: str
    __qualname__: str
    __module__: str
    __bases__: tuple["type", ...]
    __mro__: tuple["type", ...]
    __dict__: dict[str, Any]

    def __init__(self, name_or_object: Any, bases: tuple[type, ...] = ..., namespace: dict[str, Any] = ...) -> None:
        # Initialize a type object, or return the type of an object.
        pass

    def __new__(cls, name: str, bases: tuple[type, ...], namespace: dict[str, Any]) -> "type":
        # Create a new class object with the given name, bases and namespace.
        pass

    def __call__(self, *args: Any, **kwargs: Any) -> Any:
        # Create and return a new instance by calling the type.
        pass

    def __instancecheck__(self, instance: Any) -> bool:
        # Return whether an object is an instance of this type.
        pass

    def __subclasscheck__(self, subclass: type) -> bool:
        # Return whether a class is a subclass of this type.
        pass

    def mro(self) -> list["type"]:
        # Return the method resolution order for this class.
        pass


class int(object):
    def __init__(self, x: Any = 0, base: int = 10) -> None:
        # Initialize the integer from x, parsed in the given base if x is a string.
        pass

    def bit_length(self) -> int:
        # Return the number of bits needed to represent the integer in binary.
        pass

    def bit_count(self) -> int:
        # Return the number of set bits (population count) in the integer.
        pass

    def to_bytes(self, length: int = 1, byteorder: str = "big", *, signed: bool = False) -> bytes:
        # Return an array of bytes representing the integer.
        pass

    @classmethod
    def from_bytes(cls, bytes: Any, byteorder: str = "big", *, signed: bool = False) -> "int":
        # Return the integer represented by the given array of bytes.
        pass

    def as_integer_ratio(self) -> tuple[int, int]:
        # Return a pair of integers whose ratio equals this integer.
        pass

    def is_integer(self) -> bool:
        # Return True (integers are always integral).
        pass

    def __add__(self, other: Any) -> Any:
        # Return self + other.
        pass

    def __sub__(self, other: Any) -> Any:
        # Return self - other.
        pass

    def __mul__(self, other: Any) -> Any:
        # Return self * other.
        pass

    def __truediv__(self, other: Any) -> Any:
        # Return self / other as a float.
        pass

    def __floordiv__(self, other: Any) -> Any:
        # Return self // other, floor division.
        pass

    def __mod__(self, other: Any) -> Any:
        # Return self % other.
        pass

    def __pow__(self, other: Any, modulo: Any = None) -> Any:
        # Return self ** other, optionally modulo a third value.
        pass

    def __and__(self, other: Any) -> Any:
        # Return the bitwise AND of self and other.
        pass

    def __or__(self, other: Any) -> Any:
        # Return the bitwise OR of self and other.
        pass

    def __xor__(self, other: Any) -> Any:
        # Return the bitwise XOR of self and other.
        pass

    def __lshift__(self, other: Any) -> Any:
        # Return self shifted left by other bits.
        pass

    def __rshift__(self, other: Any) -> Any:
        # Return self shifted right by other bits.
        pass

    def __invert__(self) -> Any:
        # Return the bitwise inversion of self (~self).
        pass

    def __neg__(self) -> Any:
        # Return -self.
        pass

    def __pos__(self) -> Any:
        # Return +self.
        pass

    def __abs__(self) -> Any:
        # Return the absolute value of self.
        pass

    def __int__(self) -> int:
        # Return self converted to int.
        pass

    def __float__(self) -> float:
        # Return self converted to float.
        pass

    def __index__(self) -> int:
        # Return self converted losslessly to an integer, e.g. for slicing.
        pass

    def __bool__(self) -> bool:
        # Return whether the integer is nonzero.
        pass


class bool(int):
    def __init__(self, x: Any = False) -> None:
        # Initialize the boolean from the truthiness of x.
        pass

    def __and__(self, other: Any) -> Any:
        # Return the logical/bitwise AND of self and other.
        pass

    def __or__(self, other: Any) -> Any:
        # Return the logical/bitwise OR of self and other.
        pass

    def __xor__(self, other: Any) -> Any:
        # Return the logical/bitwise XOR of self and other.
        pass


class float(object):
    def __init__(self, x: Any = 0.0) -> None:
        # Initialize the float from x.
        pass

    def as_integer_ratio(self) -> tuple[int, int]:
        # Return a pair of integers whose ratio exactly equals this float.
        pass

    def is_integer(self) -> bool:
        # Return whether the float has no fractional part.
        pass

    def hex(self) -> str:
        # Return a hexadecimal string representation of the float.
        pass

    @classmethod
    def fromhex(cls, s: str) -> "float":
        # Create a float from a hexadecimal string.
        pass

    def __add__(self, other: Any) -> Any:
        # Return self + other.
        pass

    def __sub__(self, other: Any) -> Any:
        # Return self - other.
        pass

    def __mul__(self, other: Any) -> Any:
        # Return self * other.
        pass

    def __truediv__(self, other: Any) -> Any:
        # Return self / other.
        pass

    def __floordiv__(self, other: Any) -> Any:
        # Return self // other, floor division.
        pass

    def __mod__(self, other: Any) -> Any:
        # Return self % other.
        pass

    def __pow__(self, other: Any, modulo: Any = None) -> Any:
        # Return self ** other.
        pass

    def __neg__(self) -> Any:
        # Return -self.
        pass

    def __pos__(self) -> Any:
        # Return +self.
        pass

    def __abs__(self) -> Any:
        # Return the absolute value of self.
        pass

    def __int__(self) -> int:
        # Return self converted to int (truncating toward zero).
        pass

    def __float__(self) -> float:
        # Return self converted to float.
        pass

    def __bool__(self) -> bool:
        # Return whether the float is nonzero.
        pass


class complex(object):
    real: float
    imag: float

    def __init__(self, real: Any = 0, imag: Any = 0) -> None:
        # Initialize the complex number from a real and imaginary part.
        pass

    def conjugate(self) -> "complex":
        # Return the complex conjugate.
        pass

    def __add__(self, other: Any) -> Any:
        # Return self + other.
        pass

    def __sub__(self, other: Any) -> Any:
        # Return self - other.
        pass

    def __mul__(self, other: Any) -> Any:
        # Return self * other.
        pass

    def __truediv__(self, other: Any) -> Any:
        # Return self / other.
        pass

    def __pow__(self, other: Any, modulo: Any = None) -> Any:
        # Return self ** other.
        pass

    def __neg__(self) -> Any:
        # Return -self.
        pass

    def __pos__(self) -> Any:
        # Return +self.
        pass

    def __abs__(self) -> float:
        # Return the magnitude of the complex number.
        pass

    def __bool__(self) -> bool:
        # Return whether the complex number is nonzero.
        pass


class str(Generic[_T], object):
    def __init__(self, object: Any = "", encoding: str = "utf-8", errors: str = "strict") -> None:
        # Initialize the string from an object, decoding bytes if needed.
        pass

    def capitalize(self) -> "str":
        # Return a copy with the first character uppercased and the rest lowercased.
        pass

    def casefold(self) -> "str":
        # Return a casefolded copy, suitable for caseless matching.
        pass

    def center(self, width: int, fillchar: str = " ") -> "str":
        # Return a copy centered in a string of length width, padded with fillchar.
        pass

    def count(self, sub: str, start: int | None = None, end: int | None = None) -> int:
        # Return the number of non-overlapping occurrences of sub.
        pass

    def encode(self, encoding: str = "utf-8", errors: str = "strict") -> bytes:
        # Return the string encoded to bytes using encoding.
        pass

    def endswith(self, suffix: Any, start: int | None = None, end: int | None = None) -> bool:
        # Return whether the string ends with the given suffix.
        pass

    def expandtabs(self, tabsize: int = 8) -> "str":
        # Return a copy with tab characters expanded using spaces.
        pass

    def find(self, sub: str, start: int | None = None, end: int | None = None) -> int:
        # Return the lowest index where sub is found, or -1 if not found.
        pass

    def format(self, *args: Any, **kwargs: Any) -> "str":
        # Return a copy with replacement fields filled in from args/kwargs.
        pass

    def format_map(self, mapping: Mapping[str, Any]) -> "str":
        # Return a copy with replacement fields filled in from a mapping.
        pass

    def index(self, sub: str, start: int | None = None, end: int | None = None) -> int:
        # Like find(), but raise ValueError if sub is not found.
        pass

    def isalnum(self) -> bool:
        # Return whether all characters are alphanumeric and there is at least one.
        pass

    def isalpha(self) -> bool:
        # Return whether all characters are alphabetic and there is at least one.
        pass

    def isascii(self) -> bool:
        # Return whether all characters are ASCII.
        pass

    def isdecimal(self) -> bool:
        # Return whether all characters are decimal characters.
        pass

    def isdigit(self) -> bool:
        # Return whether all characters are digits.
        pass

    def isidentifier(self) -> bool:
        # Return whether the string is a valid Python identifier.
        pass

    def islower(self) -> bool:
        # Return whether all cased characters are lowercase.
        pass

    def isnumeric(self) -> bool:
        # Return whether all characters are numeric characters.
        pass

    def isprintable(self) -> bool:
        # Return whether all characters are printable.
        pass

    def isspace(self) -> bool:
        # Return whether all characters are whitespace.
        pass

    def istitle(self) -> bool:
        # Return whether the string is titlecased.
        pass

    def isupper(self) -> bool:
        # Return whether all cased characters are uppercase.
        pass

    def join(self, iterable: Iterable[str]) -> "str":
        # Return the strings in iterable joined together, separated by self.
        pass

    def ljust(self, width: int, fillchar: str = " ") -> "str":
        # Return the string left-justified in a field of the given width.
        pass

    def lower(self) -> "str":
        # Return a lowercased copy of the string.
        pass

    def lstrip(self, chars: str | None = None) -> "str":
        # Return a copy with leading whitespace (or chars) removed.
        pass

    def maketrans(*args: Any) -> dict[int, Any]:
        # Return a translation table usable with str.translate().
        pass

    def partition(self, sep: str) -> tuple["str", "str", "str"]:
        # Split on the first occurrence of sep into (before, sep, after).
        pass

    def removeprefix(self, prefix: str) -> "str":
        # Return a copy with the given prefix removed, if present.
        pass

    def removesuffix(self, suffix: str) -> "str":
        # Return a copy with the given suffix removed, if present.
        pass

    def replace(self, old: str, new: str, count: int = -1) -> "str":
        # Return a copy with occurrences of old replaced by new.
        pass

    def rfind(self, sub: str, start: int | None = None, end: int | None = None) -> int:
        # Return the highest index where sub is found, or -1 if not found.
        pass

    def rindex(self, sub: str, start: int | None = None, end: int | None = None) -> int:
        # Like rfind(), but raise ValueError if sub is not found.
        pass

    def rjust(self, width: int, fillchar: str = " ") -> "str":
        # Return the string right-justified in a field of the given width.
        pass

    def rpartition(self, sep: str) -> tuple["str", "str", "str"]:
        # Split on the last occurrence of sep into (before, sep, after).
        pass

    def rsplit(self, sep: str | None = None, maxsplit: int = -1) -> list["str"]:
        # Split the string from the right, using sep as the delimiter.
        pass

    def rstrip(self, chars: str | None = None) -> "str":
        # Return a copy with trailing whitespace (or chars) removed.
        pass

    def split(self, sep: str | None = None, maxsplit: int = -1) -> list["str"]:
        # Split the string using sep as the delimiter.
        pass

    def splitlines(self, keepends: bool = False) -> list["str"]:
        # Split the string at line boundaries.
        pass

    def startswith(self, prefix: Any, start: int | None = None, end: int | None = None) -> bool:
        # Return whether the string starts with the given prefix.
        pass

    def strip(self, chars: str | None = None) -> "str":
        # Return a copy with leading and trailing whitespace (or chars) removed.
        pass

    def swapcase(self) -> "str":
        # Return a copy with uppercase characters lowercased and vice versa.
        pass

    def title(self) -> "str":
        # Return a titlecased copy of the string.
        pass

    def translate(self, table: Any) -> "str":
        # Return a copy with characters mapped through the given table.
        pass

    def upper(self) -> "str":
        # Return an uppercased copy of the string.
        pass

    def zfill(self, width: int) -> "str":
        # Return the string left-padded with zeros to the given width.
        pass

    def __add__(self, other: Any) -> "str":
        # Return the concatenation of self and other.
        pass

    def __mul__(self, n: int) -> "str":
        # Return the string repeated n times.
        pass

    def __contains__(self, item: str) -> bool:
        # Return whether item is a substring of self.
        pass

    def __getitem__(self, index: Any) -> "str":
        # Return the character or slice at the given index.
        pass

    def __iter__(self) -> Iterator["str"]:
        # Return an iterator over the characters of the string.
        pass

    def __len__(self) -> int:
        # Return the number of characters in the string.
        pass


class bytes(object):
    def __init__(self, source: Any = b"", encoding: str | None = None, errors: str = "strict") -> None:
        # Initialize the immutable byte sequence from source.
        pass

    def decode(self, encoding: str = "utf-8", errors: str = "strict") -> str:
        # Return the bytes decoded to a string using encoding.
        pass

    def hex(self, sep: str = "", bytes_per_sep: int = 1) -> str:
        # Return a string of two hexadecimal digits per byte.
        pass

    @classmethod
    def fromhex(cls, string: str) -> "bytes":
        # Create a bytes object from a string of hexadecimal digits.
        pass

    def count(self, sub: Any, start: int | None = None, end: int | None = None) -> int:
        # Return the number of non-overlapping occurrences of sub.
        pass

    def find(self, sub: Any, start: int | None = None, end: int | None = None) -> int:
        # Return the lowest index where sub is found, or -1 if not found.
        pass

    def index(self, sub: Any, start: int | None = None, end: int | None = None) -> int:
        # Like find(), but raise ValueError if sub is not found.
        pass

    def join(self, iterable: Iterable[Any]) -> "bytes":
        # Return the byte sequences in iterable joined together, separated by self.
        pass

    def partition(self, sep: Any) -> tuple["bytes", "bytes", "bytes"]:
        # Split on the first occurrence of sep into (before, sep, after).
        pass

    def replace(self, old: Any, new: Any, count: int = -1) -> "bytes":
        # Return a copy with occurrences of old replaced by new.
        pass

    def split(self, sep: Any = None, maxsplit: int = -1) -> list["bytes"]:
        # Split the byte sequence using sep as the delimiter.
        pass

    def startswith(self, prefix: Any, start: int | None = None, end: int | None = None) -> bool:
        # Return whether the byte sequence starts with the given prefix.
        pass

    def endswith(self, suffix: Any, start: int | None = None, end: int | None = None) -> bool:
        # Return whether the byte sequence ends with the given suffix.
        pass

    def strip(self, bytes: Any = None) -> "bytes":
        # Return a copy with leading and trailing bytes stripped.
        pass

    def translate(self, table: Any, delete: bytes = b"") -> "bytes":
        # Return a copy with bytes mapped through the given table.
        pass

    def __add__(self, other: Any) -> "bytes":
        # Return the concatenation of self and other.
        pass

    def __mul__(self, n: int) -> "bytes":
        # Return the byte sequence repeated n times.
        pass

    def __getitem__(self, index: Any) -> Any:
        # Return the byte or slice at the given index.
        pass

    def __iter__(self) -> Iterator[int]:
        # Return an iterator over the integer byte values.
        pass

    def __len__(self) -> int:
        # Return the number of bytes.
        pass


class bytearray(object):
    def __init__(self, source: Any = b"", encoding: str | None = None, errors: str = "strict") -> None:
        # Initialize the mutable byte sequence from source.
        pass

    def decode(self, encoding: str = "utf-8", errors: str = "strict") -> str:
        # Return the bytes decoded to a string using encoding.
        pass

    def append(self, item: int) -> None:
        # Append a single byte to the end.
        pass

    def extend(self, iterable: Iterable[int]) -> None:
        # Append all bytes from the iterable to the end.
        pass

    def insert(self, index: int, item: int) -> None:
        # Insert a byte before the given index.
        pass

    def remove(self, value: int) -> None:
        # Remove the first occurrence of a byte value.
        pass

    def pop(self, index: int = -1) -> int:
        # Remove and return the byte at the given index.
        pass

    def clear(self) -> None:
        # Remove all bytes.
        pass

    def reverse(self) -> None:
        # Reverse the bytes in place.
        pass

    def copy(self) -> "bytearray":
        # Return a shallow copy of the byte array.
        pass

    def hex(self, sep: str = "", bytes_per_sep: int = 1) -> str:
        # Return a string of two hexadecimal digits per byte.
        pass

    @classmethod
    def fromhex(cls, string: str) -> "bytearray":
        # Create a bytearray from a string of hexadecimal digits.
        pass

    def join(self, iterable: Iterable[Any]) -> "bytearray":
        # Return the byte sequences in iterable joined together, separated by self.
        pass

    def __add__(self, other: Any) -> "bytearray":
        # Return the concatenation of self and other.
        pass

    def __iadd__(self, other: Any) -> "bytearray":
        # Extend self in place with the contents of other.
        pass

    def __getitem__(self, index: Any) -> Any:
        # Return the byte or slice at the given index.
        pass

    def __setitem__(self, index: Any, value: Any) -> None:
        # Set the byte or slice at the given index.
        pass

    def __delitem__(self, index: Any) -> None:
        # Delete the byte or slice at the given index.
        pass

    def __iter__(self) -> Iterator[int]:
        # Return an iterator over the integer byte values.
        pass

    def __len__(self) -> int:
        # Return the number of bytes.
        pass


class memoryview(object):
    obj: Any
    nbytes: int
    readonly: bool
    itemsize: int
    ndim: int
    shape: tuple[int, ...] | None
    strides: tuple[int, ...] | None
    format: str

    def __init__(self, obj: Any) -> None:
        # Create a memoryview exposing the buffer of obj without copying.
        pass

    def tobytes(self, order: str | None = "C") -> bytes:
        # Return the data as a bytes object.
        pass

    def tolist(self) -> list[Any]:
        # Return the data as a flat list of elements.
        pass

    def toreadonly(self) -> "memoryview":
        # Return a read-only version of the memoryview.
        pass

    def release(self) -> None:
        # Release the underlying buffer.
        pass

    def hex(self, sep: str = "", bytes_per_sep: int = 1) -> str:
        # Return a string of two hexadecimal digits per byte.
        pass

    def cast(self, format: str, shape: tuple[int, ...] | None = None) -> "memoryview":
        # Return a new view of the same memory with a different format/shape.
        pass

    def __getitem__(self, index: Any) -> Any:
        # Return the element or sub-view at the given index.
        pass

    def __setitem__(self, index: Any, value: Any) -> None:
        # Set the element or sub-view at the given index.
        pass

    def __len__(self) -> int:
        # Return the length of the exposed memory.
        pass

    def __enter__(self) -> "memoryview":
        # Enter the context manager, returning self.
        pass

    def __exit__(self, *exc: Any) -> None:
        # Release the buffer on context manager exit.
        pass


class list(Generic[_T], object):
    def __init__(self, iterable: Iterable[_T] = ()) -> None:
        # Initialize the list from the items of iterable.
        pass

    def append(self, object: _T) -> None:
        # Add an item to the end of the list.
        pass

    def extend(self, iterable: Iterable[_T]) -> None:
        # Extend the list by appending all items from iterable.
        pass

    def insert(self, index: int, object: _T) -> None:
        # Insert an item before the given index.
        pass

    def remove(self, value: _T) -> None:
        # Remove the first item equal to value.
        pass

    def pop(self, index: int = -1) -> _T:
        # Remove and return the item at the given index.
        pass

    def clear(self) -> None:
        # Remove all items from the list.
        pass

    def index(self, value: _T, start: int = 0, stop: int = ...) -> int:
        # Return the index of the first item equal to value.
        pass

    def count(self, value: _T) -> int:
        # Return the number of occurrences of value.
        pass

    def sort(self, *, key: Callable[[_T], Any] | None = None, reverse: bool = False) -> None:
        # Sort the list in place.
        pass

    def reverse(self) -> None:
        # Reverse the list in place.
        pass

    def copy(self) -> "list[_T]":
        # Return a shallow copy of the list.
        pass

    def __add__(self, other: "list[_T]") -> "list[_T]":
        # Return the concatenation of self and other.
        pass

    def __iadd__(self, other: Iterable[_T]) -> "list[_T]":
        # Extend self in place with the contents of other.
        pass

    def __mul__(self, n: int) -> "list[_T]":
        # Return the list repeated n times.
        pass

    def __getitem__(self, index: Any) -> Any:
        # Return the item or slice at the given index.
        pass

    def __setitem__(self, index: Any, value: Any) -> None:
        # Set the item or slice at the given index.
        pass

    def __delitem__(self, index: Any) -> None:
        # Delete the item or slice at the given index.
        pass

    def __iter__(self) -> Iterator[_T]:
        # Return an iterator over the items of the list.
        pass

    def __len__(self) -> int:
        # Return the number of items in the list.
        pass

    def __contains__(self, item: Any) -> bool:
        # Return whether item is present in the list.
        pass


class tuple(Generic[_T], object):
    def __init__(self, iterable: Iterable[_T] = ()) -> None:
        # Initialize the tuple from the items of iterable.
        pass

    def count(self, value: Any) -> int:
        # Return the number of occurrences of value.
        pass

    def index(self, value: Any, start: int = 0, stop: int = ...) -> int:
        # Return the index of the first item equal to value.
        pass

    def __add__(self, other: "tuple[_T, ...]") -> "tuple[_T, ...]":
        # Return the concatenation of self and other.
        pass

    def __mul__(self, n: int) -> "tuple[_T, ...]":
        # Return the tuple repeated n times.
        pass

    def __getitem__(self, index: Any) -> Any:
        # Return the item or slice at the given index.
        pass

    def __iter__(self) -> Iterator[_T]:
        # Return an iterator over the items of the tuple.
        pass

    def __len__(self) -> int:
        # Return the number of items in the tuple.
        pass

    def __contains__(self, item: Any) -> bool:
        # Return whether item is present in the tuple.
        pass


class range(object):
    start: int
    stop: int
    step: int

    def __init__(self, stop_or_start: int, stop: int = ..., step: int = 1) -> None:
        # Initialize an immutable sequence of numbers from start to stop by step.
        pass

    def count(self, value: int) -> int:
        # Return the number of occurrences of value (0 or 1).
        pass

    def index(self, value: int) -> int:
        # Return the index of value within the range.
        pass

    def __getitem__(self, index: Any) -> Any:
        # Return the item or slice at the given index.
        pass

    def __iter__(self) -> Iterator[int]:
        # Return an iterator over the numbers in the range.
        pass

    def __len__(self) -> int:
        # Return the number of items in the range.
        pass

    def __contains__(self, item: Any) -> bool:
        # Return whether item is one of the numbers in the range.
        pass

    def __reversed__(self) -> Iterator[int]:
        # Return an iterator over the range in reverse order.
        pass


class dict(Generic[_T, _S], object):
    def __init__(self, *args: Any, **kwargs: _S) -> None:
        # Initialize the dictionary from a mapping/iterable and/or keyword args.
        pass

    def clear(self) -> None:
        # Remove all items from the dictionary.
        pass

    def copy(self) -> "dict[_T, _S]":
        # Return a shallow copy of the dictionary.
        pass

    @classmethod
    def fromkeys(cls, iterable: Iterable[_T], value: _S = None) -> "dict[_T, _S]":
        # Create a new dictionary with keys from iterable, all set to value.
        pass

    def get(self, key: _T, default: Any = None) -> Any:
        # Return the value for key, or default if the key is absent.
        pass

    def items(self) -> Iterable[tuple[_T, _S]]:
        # Return a view of the dictionary's (key, value) pairs.
        pass

    def keys(self) -> Iterable[_T]:
        # Return a view of the dictionary's keys.
        pass

    def values(self) -> Iterable[_S]:
        # Return a view of the dictionary's values.
        pass

    def pop(self, key: _T, default: Any = ...) -> Any:
        # Remove and return the value for key, or default if absent.
        pass

    def popitem(self) -> tuple[_T, _S]:
        # Remove and return a (key, value) pair.
        pass

    def setdefault(self, key: _T, default: _S = None) -> _S:
        # Return the value for key, inserting it with default if absent.
        pass

    def update(self, other: Any = (), **kwargs: _S) -> None:
        # Update the dictionary with items from other and/or keyword args.
        pass

    def __getitem__(self, key: _T) -> _S:
        # Return the value associated with key.
        pass

    def __setitem__(self, key: _T, value: _S) -> None:
        # Associate value with key.
        pass

    def __delitem__(self, key: _T) -> None:
        # Remove the item associated with key.
        pass

    def __iter__(self) -> Iterator[_T]:
        # Return an iterator over the dictionary's keys.
        pass

    def __len__(self) -> int:
        # Return the number of items in the dictionary.
        pass

    def __contains__(self, key: Any) -> bool:
        # Return whether key is present in the dictionary.
        pass

    def __or__(self, other: "dict[_T, _S]") -> "dict[_T, _S]":
        # Return a new dictionary merging self and other.
        pass

    def __ior__(self, other: "dict[_T, _S]") -> "dict[_T, _S]":
        # Update self in place by merging in other.
        pass


class set(Generic[_T], object):
    def __init__(self, iterable: Iterable[_T] = ()) -> None:
        # Initialize the mutable set from the items of iterable.
        pass

    def add(self, element: _T) -> None:
        # Add an element to the set.
        pass

    def remove(self, element: _T) -> None:
        # Remove an element, raising KeyError if it is absent.
        pass

    def discard(self, element: _T) -> None:
        # Remove an element if it is present, without raising otherwise.
        pass

    def pop(self) -> _T:
        # Remove and return an arbitrary element.
        pass

    def clear(self) -> None:
        # Remove all elements from the set.
        pass

    def copy(self) -> "set[_T]":
        # Return a shallow copy of the set.
        pass

    def union(self, *others: Iterable[_T]) -> "set[_T]":
        # Return a new set with elements from self and all others.
        pass

    def intersection(self, *others: Iterable[Any]) -> "set[_T]":
        # Return a new set with elements common to self and all others.
        pass

    def difference(self, *others: Iterable[Any]) -> "set[_T]":
        # Return a new set with elements in self but not in others.
        pass

    def symmetric_difference(self, other: Iterable[_T]) -> "set[_T]":
        # Return a new set with elements in exactly one of self, other.
        pass

    def update(self, *others: Iterable[_T]) -> None:
        # Update the set, adding elements from all others.
        pass

    def intersection_update(self, *others: Iterable[Any]) -> None:
        # Update the set, keeping only elements found in all others.
        pass

    def difference_update(self, *others: Iterable[Any]) -> None:
        # Update the set, removing elements found in others.
        pass

    def symmetric_difference_update(self, other: Iterable[_T]) -> None:
        # Update the set, keeping elements in exactly one of self, other.
        pass

    def isdisjoint(self, other: Iterable[Any]) -> bool:
        # Return whether self and other have no elements in common.
        pass

    def issubset(self, other: Iterable[Any]) -> bool:
        # Return whether every element of self is in other.
        pass

    def issuperset(self, other: Iterable[Any]) -> bool:
        # Return whether every element of other is in self.
        pass

    def __or__(self, other: "set[_T]") -> "set[_T]":
        # Return the union of self and other.
        pass

    def __and__(self, other: "set[_T]") -> "set[_T]":
        # Return the intersection of self and other.
        pass

    def __sub__(self, other: "set[_T]") -> "set[_T]":
        # Return the difference of self and other.
        pass

    def __xor__(self, other: "set[_T]") -> "set[_T]":
        # Return the symmetric difference of self and other.
        pass

    def __iter__(self) -> Iterator[_T]:
        # Return an iterator over the elements of the set.
        pass

    def __len__(self) -> int:
        # Return the number of elements in the set.
        pass

    def __contains__(self, item: Any) -> bool:
        # Return whether item is a member of the set.
        pass


class frozenset(Generic[_T], object):
    def __init__(self, iterable: Iterable[_T] = ()) -> None:
        # Initialize the immutable set from the items of iterable.
        pass

    def copy(self) -> "frozenset[_T]":
        # Return a shallow copy of the frozenset (or self, since immutable).
        pass

    def union(self, *others: Iterable[_T]) -> "frozenset[_T]":
        # Return a new frozenset with elements from self and all others.
        pass

    def intersection(self, *others: Iterable[Any]) -> "frozenset[_T]":
        # Return a new frozenset with elements common to self and all others.
        pass

    def difference(self, *others: Iterable[Any]) -> "frozenset[_T]":
        # Return a new frozenset with elements in self but not in others.
        pass

    def symmetric_difference(self, other: Iterable[_T]) -> "frozenset[_T]":
        # Return a new frozenset with elements in exactly one of self, other.
        pass

    def isdisjoint(self, other: Iterable[Any]) -> bool:
        # Return whether self and other have no elements in common.
        pass

    def issubset(self, other: Iterable[Any]) -> bool:
        # Return whether every element of self is in other.
        pass

    def issuperset(self, other: Iterable[Any]) -> bool:
        # Return whether every element of other is in self.
        pass

    def __or__(self, other: "frozenset[_T]") -> "frozenset[_T]":
        # Return the union of self and other.
        pass

    def __and__(self, other: "frozenset[_T]") -> "frozenset[_T]":
        # Return the intersection of self and other.
        pass

    def __sub__(self, other: "frozenset[_T]") -> "frozenset[_T]":
        # Return the difference of self and other.
        pass

    def __xor__(self, other: "frozenset[_T]") -> "frozenset[_T]":
        # Return the symmetric difference of self and other.
        pass

    def __iter__(self) -> Iterator[_T]:
        # Return an iterator over the elements of the frozenset.
        pass

    def __len__(self) -> int:
        # Return the number of elements in the frozenset.
        pass

    def __contains__(self, item: Any) -> bool:
        # Return whether item is a member of the frozenset.
        pass


class slice(object):
    start: Any
    stop: Any
    step: Any

    def __init__(self, start_or_stop: Any, stop: Any = ..., step: Any = None) -> None:
        # Create a slice object representing a range of indices.
        pass

    def indices(self, length: int) -> tuple[int, int, int]:
        # Return (start, stop, step) normalized for a sequence of given length.
        pass


class enumerate(Generic[_T], object):
    def __init__(self, iterable: Iterable[_T], start: int = 0) -> None:
        # Create an iterator yielding (index, item) pairs, index starting at start.
        pass

    def __iter__(self) -> "enumerate[_T]":
        # Return self, since enumerate objects are their own iterator.
        pass

    def __next__(self) -> tuple[int, _T]:
        # Return the next (index, item) pair.
        pass


class filter(Generic[_T], object):
    def __init__(self, function: Callable[[_T], bool] | None, iterable: Iterable[_T]) -> None:
        # Create an iterator yielding items for which function returns true.
        pass

    def __iter__(self) -> "filter[_T]":
        # Return self, since filter objects are their own iterator.
        pass

    def __next__(self) -> _T:
        # Return the next item that satisfies the predicate.
        pass


class map(Generic[_T], object):
    def __init__(self, func: Callable[..., _T], *iterables: Iterable[Any]) -> None:
        # Create an iterator applying func to items from one or more iterables.
        pass

    def __iter__(self) -> "map[_T]":
        # Return self, since map objects are their own iterator.
        pass

    def __next__(self) -> _T:
        # Return the next result of applying func to the input items.
        pass


class zip(Generic[_T], object):
    def __init__(self, *iterables: Iterable[Any], strict: bool = False) -> None:
        # Create an iterator aggregating elements from each of the iterables.
        pass

    def __iter__(self) -> "zip[_T]":
        # Return self, since zip objects are their own iterator.
        pass

    def __next__(self) -> _T:
        # Return the next tuple of aggregated elements.
        pass


class reversed(Generic[_T], object):
    def __init__(self, sequence: Any) -> None:
        # Create an iterator over the sequence in reverse order.
        pass

    def __iter__(self) -> "reversed[_T]":
        # Return self, since reversed objects are their own iterator.
        pass

    def __next__(self) -> _T:
        # Return the next item, iterating from the end of the sequence.
        pass

    def __length_hint__(self) -> int:
        # Return an estimate of the number of items remaining.
        pass


class property(object):
    fget: Callable[[Any], Any] | None
    fset: Callable[[Any, Any], None] | None
    fdel: Callable[[Any], None] | None
    __doc__: str | None

    def __init__(
        self,
        fget: Callable[[Any], Any] | None = None,
        fset: Callable[[Any, Any], None] | None = None,
        fdel: Callable[[Any], None] | None = None,
        doc: str | None = None,
    ) -> None:
        # Create a managed attribute backed by getter/setter/deleter functions.
        pass

    def getter(self, fget: Callable[[Any], Any]) -> "property":
        # Return a copy of the property with a new getter function.
        pass

    def setter(self, fset: Callable[[Any, Any], None]) -> "property":
        # Return a copy of the property with a new setter function.
        pass

    def deleter(self, fdel: Callable[[Any], None]) -> "property":
        # Return a copy of the property with a new deleter function.
        pass

    def __get__(self, obj: Any, objtype: type | None = None) -> Any:
        # Invoke the getter to retrieve the property's value.
        pass

    def __set__(self, obj: Any, value: Any) -> None:
        # Invoke the setter to assign the property's value.
        pass

    def __delete__(self, obj: Any) -> None:
        # Invoke the deleter to remove the property's value.
        pass


class classmethod(Generic[_T], object):
    __func__: Callable[..., _T]
    __isabstractmethod__: bool

    def __init__(self, f: Callable[..., _T]) -> None:
        # Wrap a function as a class method, receiving the class as first argument.
        pass

    def __get__(self, obj: Any, objtype: type | None = None) -> Callable[..., _T]:
        # Bind the wrapped function to the owning class.
        pass


class staticmethod(Generic[_T], object):
    __func__: Callable[..., _T]
    __isabstractmethod__: bool

    def __init__(self, f: Callable[..., _T]) -> None:
        # Wrap a function as a static method, receiving no implicit first argument.
        pass

    def __get__(self, obj: Any, objtype: type | None = None) -> Callable[..., _T]:
        # Return the wrapped function unbound.
        pass


class super(object):
    def __init__(self, type_: type = ..., obj_or_type: Any = ...) -> None:
        # Return a proxy delegating attribute lookups to a parent or sibling class.
        pass


class NoneType(object):
    pass


class NotImplementedType(object):
    pass


class EllipsisType(object):
    pass


class function(object):
    __name__: str
    __qualname__: str
    __doc__: str | None
    __module__: str
    __defaults__: tuple[Any, ...] | None
    __kwdefaults__: dict[str, Any] | None
    __code__: "code"
    __globals__: dict[str, Any]
    __dict__: dict[str, Any]
    __closure__: tuple[Any, ...] | None
    __annotations__: dict[str, Any]

    def __call__(self, *args: Any, **kwargs: Any) -> Any:
        # Call the function with the given arguments.
        pass

    def __get__(self, obj: Any, objtype: type | None = None) -> "method":
        # Bind the function to an instance, producing a bound method.
        pass


class method(object):
    __self__: Any
    __func__: function
    __name__: str
    __doc__: str | None

    def __call__(self, *args: Any, **kwargs: Any) -> Any:
        # Call the bound method, implicitly passing __self__ as the first argument.
        pass


class code(object):
    co_name: str
    co_filename: str
    co_argcount: int
    co_varnames: tuple[str, ...]
    co_consts: tuple[Any, ...]
    co_flags: int


class cell(object):
    cell_contents: Any


class generator(Generic[_T], object):
    gi_running: bool
    gi_frame: Any
    gi_code: code

    def __iter__(self) -> "generator[_T]":
        # Return self, since generators are their own iterator.
        pass

    def __next__(self) -> _T:
        # Resume the generator and return the next yielded value.
        pass

    def send(self, value: Any) -> _T:
        # Resume the generator, sending a value into the paused yield expression.
        pass

    def throw(self, typ: Any, val: Any = None, tb: Any = None) -> _T:
        # Raise an exception at the point where the generator was paused.
        pass

    def close(self) -> None:
        # Raise GeneratorExit inside the generator to stop it.
        pass


class coroutine(object):
    cr_running: bool
    cr_frame: Any
    cr_code: code

    def send(self, value: Any) -> Any:
        # Resume the coroutine, sending a value into the paused await expression.
        pass

    def throw(self, typ: Any, val: Any = None, tb: Any = None) -> Any:
        # Raise an exception at the point where the coroutine was paused.
        pass

    def close(self) -> None:
        # Raise GeneratorExit inside the coroutine to stop it.
        pass

    def __await__(self) -> Iterator[Any]:
        # Return an iterator used to drive the coroutine with await.
        pass


class async_generator(Generic[_T], object):
    ag_running: bool
    ag_frame: Any
    ag_code: code

    def __aiter__(self) -> "async_generator[_T]":
        # Return self, since async generators are their own async iterator.
        pass

    def __anext__(self) -> Any:
        # Resume the async generator and return the next yielded value.
        pass

    def asend(self, value: Any) -> Any:
        # Resume the async generator, sending a value into the paused yield.
        pass

    def athrow(self, typ: Any, val: Any = None, tb: Any = None) -> Any:
        # Raise an exception at the point where the async generator was paused.
        pass

    def aclose(self) -> Any:
        # Raise GeneratorExit inside the async generator to stop it.
        pass


class module(object):
    __name__: str
    __doc__: str | None
    __file__: str | None
    __dict__: dict[str, Any]
    __loader__: Any
    __package__: str | None
    __spec__: Any


# ---------------------------------------------------------------------------
# Built-in Exceptions
# ---------------------------------------------------------------------------

class BaseException(object):
    args: tuple[Any, ...]
    __cause__: "BaseException | None"
    __context__: "BaseException | None"
    __suppress_context__: bool
    __traceback__: Any
    __notes__: list[str]

    def __init__(self, *args: Any) -> None:
        # Initialize the exception with its positional arguments.
        pass

    def with_traceback(self, tb: Any) -> "BaseException":
        # Set __traceback__ to tb and return self.
        pass

    def add_note(self, note: str) -> None:
        # Add a note to the exception's __notes__ list.
        pass


class BaseExceptionGroup(BaseException, Generic[_T]):
    message: str
    exceptions: tuple[_T, ...]

    def __init__(self, message: str, exceptions: Iterable[_T]) -> None:
        # Initialize the group with a message and the wrapped exceptions.
        pass

    def subgroup(self, condition: Any) -> "BaseExceptionGroup[_T] | None":
        # Return a subgroup of exceptions matching condition, or None.
        pass

    def split(self, condition: Any) -> tuple["BaseExceptionGroup[_T] | None", "BaseExceptionGroup[_T] | None"]:
        # Split the group into (matching, non-matching) subgroups.
        pass

    def derive(self, excs: Iterable[_T]) -> "BaseExceptionGroup[_T]":
        # Return a new exception group of the same type wrapping excs.
        pass


class GeneratorExit(BaseException):
    pass


class KeyboardInterrupt(BaseException):
    pass


class SystemExit(BaseException):
    code: Any


class Exception(BaseException):
    pass


class ExceptionGroup(BaseExceptionGroup, Exception):
    pass


class ArithmeticError(Exception):
    pass


class FloatingPointError(ArithmeticError):
    pass


class OverflowError(ArithmeticError):
    pass


class ZeroDivisionError(ArithmeticError):
    pass


class AssertionError(Exception):
    pass


class AttributeError(Exception):
    name: str
    obj: Any

    def __init__(self, *args: Any, name: str | None = None, obj: Any = None) -> None:
        # Initialize the exception, recording the missing attribute's name and object.
        pass


class BufferError(Exception):
    pass


class EOFError(Exception):
    pass


class ImportError(Exception):
    msg: str
    name: str | None
    path: str | None
    name_from: str | None

    def __init__(self, *args: Any, name: str | None = None, path: str | None = None, name_from: str | None = None) -> None:
        # Initialize the exception, recording the module name and path involved.
        pass


class ModuleNotFoundError(ImportError):
    pass


class LookupError(Exception):
    pass


class IndexError(LookupError):
    pass


class KeyError(LookupError):
    pass


class MemoryError(Exception):
    pass


class NameError(Exception):
    name: str

    def __init__(self, *args: Any, name: str | None = None) -> None:
        # Initialize the exception, recording the undefined name.
        pass


class UnboundLocalError(NameError):
    pass


class OSError(Exception):
    errno: int | None
    strerror: str | None
    filename: str | None
    filename2: str | None
    winerror: int | None
    characters_written: int

    def __init__(self, *args: Any) -> None:
        # Initialize the exception, typically with an errno/strerror/filename.
        pass


IOError = OSError
EnvironmentError = OSError


class BlockingIOError(OSError):
    pass


class ChildProcessError(OSError):
    pass


class ConnectionError(OSError):
    pass


class BrokenPipeError(ConnectionError):
    pass


class ConnectionAbortedError(ConnectionError):
    pass


class ConnectionRefusedError(ConnectionError):
    pass


class ConnectionResetError(ConnectionError):
    pass


class FileExistsError(OSError):
    pass


class FileNotFoundError(OSError):
    pass


class InterruptedError(OSError):
    pass


class IsADirectoryError(OSError):
    pass


class NotADirectoryError(OSError):
    pass


class PermissionError(OSError):
    pass


class ProcessLookupError(OSError):
    pass


class TimeoutError(OSError):
    pass


class ReferenceError(Exception):
    pass


class RuntimeError(Exception):
    pass


class NotImplementedError(RuntimeError):
    pass


class PythonFinalizationError(RuntimeError):
    pass


class RecursionError(RuntimeError):
    pass


class StopAsyncIteration(Exception):
    pass


class StopIteration(Exception):
    value: Any

    def __init__(self, *args: Any) -> None:
        # Initialize the exception, recording the generator's return value.
        pass


class SyntaxError(Exception):
    msg: str
    filename: str | None
    lineno: int | None
    offset: int | None
    text: str | None
    end_lineno: int | None
    end_offset: int | None


class IndentationError(SyntaxError):
    pass


class TabError(IndentationError):
    pass


class SystemError(Exception):
    pass


class TypeError(Exception):
    pass


class ValueError(Exception):
    pass


class UnicodeError(ValueError):
    pass


class UnicodeDecodeError(UnicodeError):
    encoding: str
    object: bytes
    start: int
    end: int
    reason: str

    def __init__(self, encoding: str, object: bytes, start: int, end: int, reason: str) -> None:
        # Initialize the exception with the encoding and the offending byte range.
        pass


class UnicodeEncodeError(UnicodeError):
    encoding: str
    object: str
    start: int
    end: int
    reason: str

    def __init__(self, encoding: str, object: str, start: int, end: int, reason: str) -> None:
        # Initialize the exception with the encoding and the offending character range.
        pass


class UnicodeTranslateError(UnicodeError):
    object: str
    start: int
    end: int
    reason: str

    def __init__(self, object: str, start: int, end: int, reason: str) -> None:
        # Initialize the exception with the offending character range.
        pass


class Warning(Exception):
    pass


class BytesWarning(Warning):
    pass


class DeprecationWarning(Warning):
    pass


class EncodingWarning(Warning):
    pass


class FutureWarning(Warning):
    pass


class ImportWarning(Warning):
    pass


class PendingDeprecationWarning(Warning):
    pass


class ResourceWarning(Warning):
    pass


class RuntimeWarning(Warning):
    pass


class SyntaxWarning(Warning):
    pass


class UnicodeWarning(Warning):
    pass


class UserWarning(Warning):
    pass
