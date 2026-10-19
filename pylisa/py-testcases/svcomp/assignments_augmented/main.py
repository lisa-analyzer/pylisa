# This file is part of the SV-Benchmarks collection of verification tasks:
# https://gitlab.com/sosy-lab/benchmarking/sv-benchmarks
#
# SPDX-FileCopyrightText: 2023 Paderborn University
# SPDX-FileCopyrightText: 2026 Luca Negrini (Ca' Foscari University of Venice)
#
# SPDX-License-Identifier: MIT

# A program to demonstrate the use of augmented assignments.
# Augmented assignments are used to assign the result of func1, called on a nondeterministic integer.
import _sv_verifier


def func1(a):
    a += 3
    a *= 2
    return a


n = _sv_verifier.nondet_int()
b = func1(n)
_sv_verifier.check_type(b, int)
assert b == 2 * (n + 3)
assert b % 2 == 0
