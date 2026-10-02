class T:
    def __new__(cls, n):
        o = object.__new__(cls)
        if n > 0:
            o.c = T(n - 1)
        return o


t = T(2)  # @outer
