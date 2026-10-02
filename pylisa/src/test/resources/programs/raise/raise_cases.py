class MyError(Exception):
    pass


def builtin_class():
    raise ValueError("bad")


def user_class():
    raise MyError()


def bare_name():
    raise KeyError


def guarded(c):
    if c == 'g':
        raise TypeError("t")
    return 1


c = input()
if c == 'a':
    builtin_class()  # @builtin
if c == 'b':
    user_class()  # @user
if c == 'c':
    bare_name()  # @bare
r = guarded(c)  # @guarded
