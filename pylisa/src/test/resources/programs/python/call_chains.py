class A:
    def f(self):
        return self

    def g(self):
        return 1


x = A()
chain = (x
         .f())
two = x.f().g()
