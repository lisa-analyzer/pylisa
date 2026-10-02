class Class1:
    Y = 20
    def __init__(self):
        self.x = 10
    def test(self):
        return 70
class1 = Class1()

f = class1.x
a = class1.Y
b = Class1.Y
Class1.Y = 100
c = class1.Y
d = Class1.Y
class1.Y = 200
e = class1.Y
g = Class1.Y
print(a)
print(b)
print(c)
print(d)
print(e)
print(g)
h = class1.test()
i = Class1.test(class1)
print(h)
print(i)