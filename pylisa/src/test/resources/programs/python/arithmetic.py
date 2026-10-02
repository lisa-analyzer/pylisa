assert 10 - 3 - 2 == 5
assert 8 / 4 / 2 == 1.0
assert 2 * 3 % 4 == 2
a = 1
b = 2
c = 3
assert a - b + c == 2
flag = True
n = flag + 1
assert n == 2
assert -flag == -1
assert 0.1 + 0.2 == 0.30000000000000004
assert 2 ** 62 == 4611686018427387904
assert -7 % 3 == 2
assert 0x1e == 30
assert 9007199254740993 != 9007199254740992.0
x = 20
assert not (0 < x < 10)
assert 0 < x < 30  # @chain
items = [0] * 3
joined = [1] + [2]
assert '' * 10 ** 12 == ''
assert 'ab' * 2 == 'abab'
