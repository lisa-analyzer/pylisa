"""Structural replica of the ``abc`` module (Python 3.14)."""

from __future__ import annotations

from typing import Any, Callable, TypeVar

_F = TypeVar("_F", bound=Callable[..., Any])


class ABCMeta(type):
    def __new__(mcls, name: str, bases: tuple[type, ...], namespace: dict[str, Any], /, **kwargs: Any) -> "ABCMeta":
        # Create a new class, registering its abstract methods for enforcement.
        pass

    def __instancecheck__(cls, instance: Any) -> bool:
        # Return whether an object is a (virtual or real) instance of cls.
        pass

    def __subclasscheck__(cls, subclass: type) -> bool:
        # Return whether a class is a (virtual or real) subclass of cls.
        pass

    def register(cls, subclass: type) -> type:
        # Register subclass as a "virtual" subclass of cls.
        pass


class ABC(metaclass=ABCMeta):
    __slots__ = ()


def abstractmethod(funcobj: _F) -> _F:
    # Mark a method as abstract, preventing instantiation until it is overridden.
    pass


class abstractclassmethod(classmethod):
    __isabstractmethod__: bool

    def __init__(self, callable: Callable[..., Any]) -> None:
        # Combine classmethod and abstractmethod (deprecated, use both decorators instead).
        pass


class abstractstaticmethod(staticmethod):
    __isabstractmethod__: bool

    def __init__(self, callable: Callable[..., Any]) -> None:
        # Combine staticmethod and abstractmethod (deprecated, use both decorators instead).
        pass


class abstractproperty(property):
    __isabstractmethod__: bool
    # Combine property and abstractmethod (deprecated, use both decorators instead).


def get_cache_token() -> Any:
    # Return an opaque token that changes whenever a class registration occurs.
    pass


def update_abstractmethods(cls: type) -> type:
    # Recalculate a class's set of abstract methods after it was modified.
    pass
