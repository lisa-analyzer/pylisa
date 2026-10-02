def check(x):
    if x is None:
        raise ValueError("none")
    return x


def helper(v):
    return check(v)  # @inner


c = input()
if c == 'a':
    check(None)  # @always
if c == 'b':
    helper(None)  # @through_helper
if c == 'c':
    helper(1)  # @completes
check(c)  # @string
