from somelib import Base


class P(Base):
    pass


c = input()
if c == 'a':
    class B(bytes):
        pass
else:
    class B:
        pass


class C(B):
    pass


class Plain:
    pass


r1 = isinstance(P(), bytes)  # @unresolved_base
r2 = isinstance(C(), bytes)  # @ambiguous_base
r3 = isinstance(Plain(), Plain)  # @own_class
r4 = isinstance(Plain(), bytes)  # @other_class


class Meta(type):
    def __instancecheck__(cls, obj):
        return True


class Anything(metaclass=Meta):
    pass


class Base2:
    pass


Base2 = bytes


class Derived(Base2):
    pass


r5 = isinstance("x", Anything)  # @metaclass
r6 = isinstance(Derived(), bytes)  # @rebound_base
