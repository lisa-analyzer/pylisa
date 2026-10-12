def five():
    return 5

def pair():
    return (1, 2)

lst = [1, 2, 3]
lst[0] = five()
x = 1
x += five()
y = "a"
y *= five()
a = b = five()
c = lst[1] = five()
lst[2] += five()
print(five())
d, e = pair()
t = f, g = pair()
h = t[0]
