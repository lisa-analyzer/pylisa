import sys

class C1:
    pass

class C2:
    pass

class C3:
    pass

class C4:
    pass

class C5:
    pass

class C6:
    pass

class C7:
    pass

class C8:
    pass

class C9:
    pass

class C10:
    pass

class C11:
    pass

class C12:
    pass

class C13:
    pass

class C14:
    pass

class C15:
    pass

class C16:
    pass

class C17:
    pass

class C18:
    pass

class C19:
    pass

class C20:
    pass

class C21:
    pass


def pick(n):
    if n == 1:
        return C1()
    if n == 2:
        return C2()
    if n == 3:
        return C3()
    if n == 4:
        return C4()
    if n == 5:
        return C5()
    if n == 6:
        return C6()
    if n == 7:
        return C7()
    if n == 8:
        return C8()
    if n == 9:
        return C9()
    if n == 10:
        return C10()
    if n == 11:
        return C11()
    if n == 12:
        return C12()
    if n == 13:
        return C13()
    if n == 14:
        return C14()
    if n == 15:
        return C15()
    if n == 16:
        return C16()
    if n == 17:
        return C17()
    if n == 18:
        return C18()
    if n == 19:
        return C19()
    if n == 20:
        return C20()
    if n == 21:
        return C21()
    return C1()


v = pick(len(sys.argv))
w = v()  # @over
