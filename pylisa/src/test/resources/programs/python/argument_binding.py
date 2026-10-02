def f(a, b, c=3):
    return b


def g(a, *, k=5):
    return k


def h(a, b=2):
    return b


class K:
    def m(self, a, b=2):
        return b


assert f(1, 2) == 2  # @positional
assert f(1, 2, 4) == 2  # @all_positional
assert g(1) == 5  # @keyword_only_default
assert g(1, k=6) == 6  # @keyword_only_given
assert h(1) == 2  # @positional_default
assert K().m(1) == 2  # @method_default
