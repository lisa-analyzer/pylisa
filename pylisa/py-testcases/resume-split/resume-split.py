# An operand chosen before a call whose callee is expanded: when the replay
# resumes after the call, the subscript is still restricted to the executions
# choosing each list, so that the IndexError, raised exactly where the list is
# the short one, can be confirmed.
import _sv_verifier

FLAG = _sv_verifier.nondet_bool()


def g(n):
    return n + 5


def index_after_call():
    n = _sv_verifier.nondet_int()
    if n < 0 or n > 2:
        return 0
    short = [1, 2, 3, 4, 5, 6]
    long = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    # the index is in [5, 7]: out of range for the short list when n > 0
    return (short if FLAG else long)[g(n)]


index_after_call()
