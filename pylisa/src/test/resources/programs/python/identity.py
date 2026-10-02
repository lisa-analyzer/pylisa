x = None
assert x is None  # @none_is_none
y = 'a'
assert y is not None  # @string_is_not_none
if y is None:
    z = 1
else:
    z = 2
assert z == 2  # @branch_on_identity
