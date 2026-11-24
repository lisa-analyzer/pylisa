# The errors raised where the operands might have several types, or several
# values: each error is raised exactly where it happens, so that its warning
# can be confirmed.
import _sv_verifier

FLAG = _sv_verifier.nondet_bool()


def index_of_two_types():
    # IndexError exactly when the index is the int, TypeError when it is the
    # str
    lst = [1, 2]
    i = 5 if FLAG else "a"
    return lst[i]


def store_of_two_types():
    # as above, when assigning
    lst = [1, 2]
    i = 5 if FLAG else "a"
    lst[i] = 3


def f(**kwargs):
    return kwargs


def double_star_never_dict():
    # TypeError in all the executions: no value is a dictionary
    return f(**(5 if FLAG else "a"))


def double_star_sometimes_dict():
    # TypeError only where the value is not a dictionary (not confirmed: the
    # arguments are assigned to the parameters once the scope of the callee
    # is entered, where the executions computing each value are not known)
    d = {"x": 1}
    return f(**(d if FLAG else 5))


class A:
    def m(self):
        return 1


class B:
    def m(self):
        return 2


def bound_wrong_arity():
    # TypeError in all the executions: no receiver accepts the arguments
    # (not confirmed: the call has two targets, A.m and B.m, and the error is
    # raised under an unknown condition for calls with several targets)
    o = A() if FLAG else B()
    g = o.m
    return g(1, 2)


def min_of_combinations():
    # TypeError exactly where the first argument is the str
    return min("s" if FLAG else 0, 1)


if _sv_verifier.nondet_bool():
    index_of_two_types()
elif _sv_verifier.nondet_bool():
    store_of_two_types()
elif _sv_verifier.nondet_bool():
    double_star_never_dict()
elif _sv_verifier.nondet_bool():
    double_star_sometimes_dict()
elif _sv_verifier.nondet_bool():
    bound_wrong_arity()
else:
    min_of_combinations()
