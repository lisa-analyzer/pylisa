def never_called(x):
    assert x == 99  # @never


y = 1
assert y == 1  # @called
