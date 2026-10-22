import math
from os import path

# imports bind names
m = math
p = path

# global declarations: the binding is visible after the call
counter = 0


def increment():
    global counter
    counter = counter + 1


increment()
after_global = counter


# a local shadowing a builtin does not affect the builtin
def shadow_builtin():
    len = 5
    return len


shadowed = shadow_builtin()
builtin = len("ab")

flag = input()
if flag:
    maybe = 1
# NameError if flag is empty
read_maybe = maybe


# UnboundLocalError if flag is empty
def unbound_local():
    if flag:
        y = 2
    return y


u = unbound_local()

# del unbinds the name: NameError if flag is not empty
deleted = 3
if flag:
    del deleted
read_deleted = deleted


# a recursive function
def fact(n):
    if n <= 1:
        return 1
    return n * fact(n - 1)


f5 = fact(5)

# exec might bind or delete any global: reading a global after it might raise
# NameError, and its value is unknown
before_exec = 1
exec(flag)
after_exec = before_exec

# NameError: never defined
if flag:
    undefined_name()
last = 1
