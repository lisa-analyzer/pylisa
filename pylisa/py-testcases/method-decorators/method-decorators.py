import _sv_verifier
from abc import ABC, abstractmethod


# static methods are called without the receiver, through the class or an object
class S:
    @staticmethod
    def add(x, y):
        return x + y

    def get(self):
        return S.add(1, 2) + self.add(3, 4)


assert S.add(1, 2) == 3
assert S().add(1, 2) == 3
assert S().get() == 10


# a method that does not accept the arguments: TypeError
class M:
    def f():
        return 1


if _sv_verifier.nondet_bool():
    M().f()
assert M.f() == 1


# abstract classes cannot be instantiated while they have abstract methods
class Shape(ABC):
    @abstractmethod
    def area(self):
        pass


class Rect(Shape):
    def __init__(self, w):
        self.w = w

    def area(self):
        return self.w * 2


class Wrong(Shape):
    def aera(self):
        return 0


class Plain:
    @abstractmethod
    def area(self):
        pass


r = Rect(3)
assert r.area() == 6
assert isinstance(r, Shape)
assert not isinstance(Plain(), Shape)
p = Plain()
if _sv_verifier.nondet_bool():
    Shape()
if _sv_verifier.nondet_bool():
    Wrong()
