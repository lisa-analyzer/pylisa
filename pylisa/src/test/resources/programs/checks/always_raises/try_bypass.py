class Obj:
    pass


def g():
    r = None
    try:
        r = Obj()
        raise ValueError("v")
    except ValueError:
        pass
    return r


def h():
    return g()


x = h()  # @stored
