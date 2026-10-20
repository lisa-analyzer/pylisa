import helpers
import helpers as h
import pkg.sub
from helpers import func as hfunc, Box
from pkg import sub


def func():
    # a function of this module, with the same name as the imported ones
    return "main"


assert func() == "main"
assert helpers.func() == 42
assert h.func() == 42
assert hfunc() == 42
assert pkg.func() == "pkg"
assert pkg.sub.func() == 2.5
assert sub.func() == 2.5
f = helpers.func
assert f() == 42
b = Box()
assert b.get() == 1
# the value of a variable of a module is not known: any exception might be raised
x = sub.alias()
# TypeError: get expects self
y = Box.get()
