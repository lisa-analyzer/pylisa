import sys
import testnatives


class Base:
    def __init__(self, v):
        self.v = v


class Sub(Base):
    def __init__(self, v):
        super().__init__(v)  # @super


class Explicit(Base):
    def __init__(self, v):
        Base.__init__(self, v)  # @explicit


class Child(testnatives.Plain):
    def __init__(self):
        super().__init__()  # @native


s = Sub(1)  # @sub
e = Explicit(2)  # @explicit_call
c = Child()  # @child
k = Sub if len(sys.argv) > 1 else Explicit
o = k(3)  # @either
