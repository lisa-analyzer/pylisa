class Box:
    pass


def conditional_expressions():
    x = 'a' if False else 'b'
    assert x == 'b'
    y = 'a' if True else 'b'
    assert y == 'a'


def short_circuit():
    box = Box()
    box.calls = 0
    z = False and set_calls(box)
    assert box.calls == 0
    assert z == False
    w = 0 or 'fallback'
    assert w == 'fallback'
    v = 3 and 'last'
    assert v == 'last'


def set_calls(box):
    box.calls = 1
    return True


def chained_comparisons():
    x = 20
    assert not (0 < x < 10)
    assert 0 < x < 30
    box = Box()
    box.calls = 0
    r = 5 < 1 < set_calls(box)
    assert box.calls == 0


def unary_operators():
    flag = True
    assert -flag == -1
    assert +flag == 1
    assert -0.5 == -(0.5)


conditional_expressions()
short_circuit()
chained_comparisons()
unary_operators()
