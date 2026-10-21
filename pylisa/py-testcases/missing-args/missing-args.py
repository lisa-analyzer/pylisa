import _sv_verifier

# omitted arguments
assert int() == 0
assert int("12") == 12
assert int("ff", 16) == 255
assert float() == 0.0
assert list() == []
assert bytes() == b""
assert bytes("a", "ascii") == b"a"
assert "aaa".replace("a", "b") == "bbb"
assert "aaa".replace("a", "b", 1) == "baa"
assert "a".encode() == b"a"
assert b"a".decode() == "a"
r = range(2, 5)

# None is a valid argument for other builtins
assert "abc".find("b", None, None) == 1

# an explicit None is not an omitted argument: TypeError
c = _sv_verifier.nondet_int()
if c == 0:
    int(None)
elif c == 1:
    int("1", None)
elif c == 2:
    float(None)
elif c == 3:
    list(None)
elif c == 4:
    bytes(None)
elif c == 5:
    bytes("a", None)
elif c == 6:
    range(3, None)
elif c == 7:
    "a".replace("a", "b", None)
elif c == 8:
    "a".encode(None)
elif c == 9:
    b"a".decode(None)
elif c == 10:
    int(base=10)
