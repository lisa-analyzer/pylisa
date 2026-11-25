# Regressions of the second review of the triage (pylisa, §4.5): tuple
# slicing, the arity error of a call with a CFG and a native target, and the
# str/list mix in a subscript are in review-soundness.
import _sv_verifier

FLAG = _sv_verifier.nondet_bool()


def nan_base():
    # NaN is neither negative nor non-negative: NaN ** 0.5 is NaN
    y = _sv_verifier.nondet_float() ** 0.5
    if y != y:
        assert False


def exponent_sign():
    # an int for a non-negative exponent, a float for a negative one
    n = -1 if FLAG else 1
    r = 2 ** n
    if r < 1:
        assert False


def zero_to_negative():
    # ZeroDivisionError only for a negative exponent
    n = _sv_verifier.nondet_int()
    if n < -1:
        return 0
    return 0 ** n


def min_of_list():
    # TypeError where the list holds an int and a str, then the code goes on
    # for the other list
    lst = [3, "a"] if FLAG else [1, 2]
    m = min(lst)
    assert False


def tuple_store():
    # tuples do not support item assignment
    t = (1, 2)
    t[0] = 3


if _sv_verifier.nondet_bool():
    nan_base()
elif _sv_verifier.nondet_bool():
    exponent_sign()
elif _sv_verifier.nondet_bool():
    zero_to_negative()
elif _sv_verifier.nondet_bool():
    min_of_list()
else:
    tuple_store()
