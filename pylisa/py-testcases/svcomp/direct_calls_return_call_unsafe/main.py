# This file is part of the SV-Benchmarks collection of verification tasks:
# https://gitlab.com/sosy-lab/benchmarking/sv-benchmarks
#
# SPDX-FileCopyrightText: 2023 Paderborn University
# SPDX-FileCopyrightText: 2026 Luca Negrini (Ca' Foscari University of Venice)
#
# SPDX-License-Identifier: MIT

# A function `func` is called and returns a function `return_func` (or `return_func2`, chosen nondeterministically)
# which is later called directly in the form func()().
# BUG (unsafe variant): return_func2 returns the result of nested_return_func2() (an int) instead of the function,
# so when FLAG is false func()()() calls an int and raises a TypeError.
import _sv_verifier


def return_func():
    def nested_return_func():
        return "Hello from nested_return_func"

    return nested_return_func


def return_func2():
    def nested_return_func2():
        return 42

    return nested_return_func2()


FLAG = _sv_verifier.nondet_bool()


def func():
    if FLAG:
        return return_func
    return return_func2


a = func()()
b = func()()()
assert callable(a)
assert isinstance(b, str) == FLAG
if FLAG:
    assert b == "Hello from nested_return_func"
else:
    assert b == 42
