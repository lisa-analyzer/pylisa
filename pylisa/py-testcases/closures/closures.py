# a counter, modified through nonlocal
def make_counter():
    n = 0

    def inc():
        nonlocal n
        n = n + 1
        return n

    return inc


c = make_counter()
a = c()
b = c()


# late binding: the cell is read when the function is called
def late():
    x = 1

    def get():
        return x

    x = 2
    return get()


l = late()


# two closures sharing a cell
def shared():
    v = 0

    def setv(k):
        nonlocal v
        v = k

        def getv():
            return v

        return getv

    return setv


setter = shared()
getter = setter(5)
p = getter()


# three levels of nesting
def outer(w):
    def middle():
        def inner():
            return w

        return inner()

    return middle()


o = outer(7)


# lambdas capturing variables
def adder(k):
    return lambda x: x + k


add3 = adder(3)
r = add3(4)


# a captured parameter, and a closure called directly
def scale(f):
    def apply(x):
        return x * f

    return apply


t = scale(2)(5)

# two definitions of the same function, in different branches
flag = input()
if flag:
    def choose():
        return 1
else:
    def choose():
        return 2
q = choose()

# lambdas inside a comprehension capture the iteration variable
fs = [lambda: i for i in [1, 2, 3]]
u = fs[0]()
last = 1
