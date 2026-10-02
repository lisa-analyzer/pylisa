"""helper is named here, in a docstring."""


def helper():
    return 1


later = helper
later()
helper()  # @call
named = helper  # @assigned
if helper:  # @condition
    pass
if input():
    either = helper
else:
    either = 1
either  # @either
