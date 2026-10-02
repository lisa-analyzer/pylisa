import sys


class Base:
    def __init__(self):
        self.tag = 1


def other(self):
    self.tag = 2


class C(Base):
    if len(sys.argv) > 1:
        def __new__(cls):
            C.__init__ = other
            return object.__new__(cls)
    else:
        def __new__(cls):
            return object.__new__(cls)


c = C()  # @two
