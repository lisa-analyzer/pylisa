class C:
    pass


x = 'a'  # @x
o = C()
o.f = 'b'  # @f
o2 = C()  # @two


def g(p):
    q = p + '/'  # @q
    return q


y = g('ns')
if input() == 'x':
    t = 'a'
else:
    t = 'b'
u = t  # @branch
