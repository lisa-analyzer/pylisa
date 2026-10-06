class D:
    def __divmod__(self, o):
        return 1

    def __rdivmod__(self, o):
        return 2

t1 = divmod(7, 2)
q1 = t1[0]
r1 = t1[1]
t2 = divmod(-7, 2)
q2 = t2[0]
r2 = t2[1]
t3 = divmod(7, -2)
q3 = t3[0]
r3 = t3[1]
t4 = divmod(7.5, 2)
q4 = t4[0]
r4 = t4[1]
t5 = divmod(-7.5, 2.0)
q5 = t5[0]
r5 = t5[1]
t6 = divmod(7, -2.5)
q6 = t6[0]
r6 = t6[1]
t7 = divmod(0, 5)
q7 = t7[0]
r7 = t7[1]
n = len(t1)
d = D()
u1 = divmod(d, 3)
u2 = divmod(3, d)
c1 = input()
if c1:
    e1 = divmod(7, 0)
if c1:
    e2 = divmod(7.0, 0.0)
if c1:
    e3 = divmod("a", 2)
if c1:
    e4 = divmod(2, "a")
if c1:
    e5 = t1[2]
after = 1
