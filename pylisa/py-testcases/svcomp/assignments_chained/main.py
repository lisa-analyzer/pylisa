# This file is part of the SV-Benchmarks collection of verification tasks:
# https://gitlab.com/sosy-lab/benchmarking/sv-benchmarks
#
# SPDX-FileCopyrightText: 2023 Paderborn University
# SPDX-FileCopyrightText: 2026 Luca Negrini (Ca' Foscari University of Venice)
#
# SPDX-License-Identifier: MIT

# Two variables are assigned a function via chained assignment.
# The second chained assignment happens only on a nondeterministic branch.

import _sv_verifier


def func1():
    return "Hello from func1"


def func2():
    return 42


a = b = func1

c = b()
assert a is b
assert c == "Hello from func1"

if _sv_verifier.nondet_bool():
    a = b = func2

assert a is b
d = a()
assert d == b()
if a is func2:
    assert d == 42
else:
    assert d == "Hello from func1"
