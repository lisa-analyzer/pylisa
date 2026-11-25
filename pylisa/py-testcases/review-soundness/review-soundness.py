# Executions that the semantics of some statements used to drop, by raising
# errors in all of them (second review of the triage, pylisa S1-S6): the
# assertion failing after each statement is reachable, so its warning must not
# be refuted.
import _sv_verifier

FLAG = _sv_verifier.nondet_bool()


def tuple_slice():
    # a slice of a tuple is a tuple: no TypeError
    t = (1, 2, 3)
    u = t[1:]
    assert False


def tuple_str_index():
    # TypeError in all the executions: the index is neither an int nor a
    # slice
    t = (1, 2)
    return t["a"]


class C:
    def append(self):
        pass


def multicall_arity():
    # TypeError only where x is a C (C.append takes no argument), while the
    # list continues normally
    x = C() if FLAG else []
    x.append(1)
    assert False


def mutated_truthiness():
    # the truthiness of a list changes with its contents
    lst = []
    if _sv_verifier.nondet_bool():
        lst.append(0)
    a = not lst
    lst.append(1)
    b = not lst
    if a and not b:
        assert False


def nan_power():
    # NaN is neither negative nor non-negative
    y = _sv_verifier.nondet_float() ** 0.5
    if y != y:
        assert False


def str_or_list_index():
    # the IndexError of the str is raised only where x is the str
    x = "ab" if FLAG else [1, 2, 3]
    if not FLAG:
        return x[2]
    return 0


def float_division():
    # 0.1 + 0.2 - 0.3 is not 0 in python
    d = 0.1 + 0.2 - 0.3
    z = 1 / d
    assert False


if _sv_verifier.nondet_bool():
    tuple_slice()
elif _sv_verifier.nondet_bool():
    tuple_str_index()
elif _sv_verifier.nondet_bool():
    multicall_arity()
elif _sv_verifier.nondet_bool():
    mutated_truthiness()
elif _sv_verifier.nondet_bool():
    nan_power()
elif _sv_verifier.nondet_bool():
    str_or_list_index()
else:
    float_division()
