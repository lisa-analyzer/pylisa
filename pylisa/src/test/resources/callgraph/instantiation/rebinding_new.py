class Base:
    def __init__(self):
        self.tag = 1


def other(self):
    self.tag = 2


class C(Base):
    def __new__(cls):
        C.__init__ = other
        return object.__new__(cls)


c = C()  # @rebound
