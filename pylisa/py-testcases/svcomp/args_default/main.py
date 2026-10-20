# This file is part of the SV-Benchmarks collection of verification tasks:
# https://gitlab.com/sosy-lab/benchmarking/sv-benchmarks
#
# SPDX-FileCopyrightText: 2023 Paderborn University
# SPDX-FileCopyrightText: 2026 Luca Negrini (Ca' Foscari University of Venice)
#
# SPDX-License-Identifier: MIT

# A function func is defined which takes as a parameter a function which it later calls. It also has a default value assigned
# The 'param_func' function returns a string value.
# The 'param_func2' function returns a (nondeterministic) integer value.
# Whether func is called with an explicit argument or with the default one is chosen nondeterministically.
import _sv_verifier


def param_func():
    return "Hello from param_func"


def param_func2():
    return _sv_verifier.nondet_int()


def func(a=param_func2):
    return a()


if _sv_verifier.nondet_bool():
    b = func(param_func)
    _sv_verifier.check_type(b, str)
    assert b == "Hello from param_func"
else:
    b = func()
    _sv_verifier.check_type(b, int)
_sv_verifier.check_type(b, str | int)
c = func()
_sv_verifier.check_type(c, int)
assert not isinstance(c, str)
