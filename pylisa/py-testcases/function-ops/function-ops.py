def greet(name):
    return "Hello, " + name

def other(x):
    return x

say_hello = greet
c = input()
eq1 = greet == 1
ne1 = greet != "a"
eq2 = greet == say_hello
eq3 = greet == other
ne3 = greet != other
if c == "add":
    a1 = greet + 1
if c == "radd":
    a2 = 1 + greet
if c == "mul":
    a3 = greet * 2
if c == "lt":
    a4 = greet < other
if c == "neg":
    a5 = -greet
after = 1
