import sys

def f1():
    return 1

def f2():
    return 2

def f3():
    return 3

def f4():
    return 4

def f5():
    return 5

def f6():
    return 6

def f7():
    return 7

def f8():
    return 8

def f9():
    return 9

def f10():
    return 10

def f11():
    return 11

def f12():
    return 12

def f13():
    return 13

def f14():
    return 14

def f15():
    return 15

class K1:
    pass

class K2:
    pass

class K3:
    pass

class K4:
    pass

class K5:
    pass

class K6:
    pass

def pick(n):
    if n == 1:
        return f1
    if n == 2:
        return f2
    if n == 3:
        return f3
    if n == 4:
        return f4
    if n == 5:
        return f5
    if n == 6:
        return f6
    if n == 7:
        return f7
    if n == 8:
        return f8
    if n == 9:
        return f9
    if n == 10:
        return f10
    if n == 11:
        return f11
    if n == 12:
        return f12
    if n == 13:
        return f13
    if n == 14:
        return f14
    if n == 15:
        return f15
    if n == 16:
        return K1()
    if n == 17:
        return K2()
    if n == 18:
        return K3()
    if n == 19:
        return K4()
    if n == 20:
        return K5()
    if n == 21:
        return K6()
    return f1


k = pick(len(sys.argv))
w = k()  # @mix
