a = 3
b = 5
c1 = a < b
c2 = a >= b
c3 = a == 3.0
c4 = 2 ** 70 == 2 ** 70
c5 = 2 ** 70 < 2 ** 70 + 1
c6 = 2 ** 53 + 1 == 2.0 ** 53
c7 = 0.5 < 1
c8 = -0.0 == 0
nan = float("nan")
n1 = nan == nan
n2 = nan != nan
n3 = nan < 1
n4 = nan >= nan
inf = float("inf")
i1 = inf > 2 ** 70
i2 = -inf < -5
t1 = True == 1
t2 = False < True
t3 = True + True > 1
s1 = "abc" < "abd"
s2 = "abc" < "ab"
s3 = "Z" < "a"
s4 = "" < "a"
s5 = "é" > "z"
s6 = "\U0001F600" > "￿"
s7 = "abc" == "abc"
s8 = "abc" != "abd"
s9 = "1" == 1
b1 = b"abc" < b"abd"
b2 = b"\xff" > b"a"
b3 = b"a" == "a"
b4 = b"" <= b""
