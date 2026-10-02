import testnatives


class A:
    def m(self, v):
        return v


class Child(testnatives.Plain):
    def __init__(self):
        super().__init__()  # @super


a = A()
x = a.m(1)  # @method
y = A.m(a, 2)  # @unbound
z = testnatives.echo(3)  # @module
p = testnatives.Plain()  # @construct
q = p.echo(4)  # @native_method
c = Child()  # @child


def call(r, v):
    return r.m(v)  # @mixed


call(a, 1)
call(A, a)
