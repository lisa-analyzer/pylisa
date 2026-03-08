class A:
    def __init__(self):
        self.x = 10
    @staticmethod
    def foo():
        return 20

a = A()
b = a.foo() # b is a integer and the value is 20
c = A.foo()
d = a.x
e = A().x
print(b)
print(c)
print(d)
print(e)
