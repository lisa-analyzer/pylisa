def f():
    return 10

def g():
    return 20

a = f() # 10

b = g() # 20
x = g() # 20
aa = f() # 20
print(x)

class A:
    def h():
        return 30
    def i():
        return 40
B = A
g = A.h
c = A.h() # 30
y = g() # 30
A.h = g
z = B.i() # 40
b = B()
b.i = f()
zz = b.h()
print(y)
