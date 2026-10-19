# This file is part of the SV-Benchmarks collection of verification tasks:
# https://gitlab.com/sosy-lab/benchmarking/sv-benchmarks
#
# SPDX-FileCopyrightText: 2023 Paderborn University
# SPDX-FileCopyrightText: 2026 Luca Negrini (Ca' Foscari University of Venice)
#
# SPDX-License-Identifier: MIT

# A class is instantiated and its function is called; the function returns
# a str or an int depending on a nondeterministic argument.

import _sv_verifier


class MyClass:
    def func(self, flag):
        if flag:
            return "Hello from func in MyClass"
        return 42


flag = _sv_verifier.nondet_bool()
a = MyClass()
b = a.func(flag)
_sv_verifier.check_type(a, MyClass)
if flag:
    _sv_verifier.check_type(b, str)
    assert b == "Hello from func in MyClass"
else:
    _sv_verifier.check_type(b, int)
    assert b == 42
