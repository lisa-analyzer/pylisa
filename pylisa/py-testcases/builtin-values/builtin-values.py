class P:
    pass

def call(f, x):
    return f(x)

# builtin functions are values, that can be stored and called
q = len
a = q("abc")
b = q((1, 2))
p = P()
p.l = len
c = p.l("abcd")
s = str
d = s(5)
i = int
e = i("42")
r = repr
f = r("x")
# and passed to functions
g = call(len, "hello")
inp = input()
# TypeError: object of type 'int' has no len()
if inp == "k":
    k = q(5)
# a name assigned somewhere in the program is a plain variable: inside
# get, bytes is the global variable (whose value is unknown there), not
# the builtin function
def get():
    return bytes

bytes = 3
m = get()
last = 1
