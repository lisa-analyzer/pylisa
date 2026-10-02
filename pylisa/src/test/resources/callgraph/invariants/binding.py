class C:
    def m(self, v=7):
        return v

    @staticmethod
    def sm(v=8):
        return v


c = C()
f = c.m
a = f(1)  # @through
b = C.m(c, 2)  # @unbound
d = c.sm(3)  # @static
