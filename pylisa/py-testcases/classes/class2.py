class Class2:
    Y = 20
    def __init__(self):
        self.x = 10

    def test(self):
        return self.x
class2 = Class2()
f = class2.x
a = class2.Y
b = Class2.Y
Class2.Y = 100
c = class2.Y
d = Class2.Y
class2.Y = 200
e = class2.Y
f = Class2.Y
g = class2.test()
print(a)
print(b)
print(c)
print(d)
print(e)
print(f)
print(g)