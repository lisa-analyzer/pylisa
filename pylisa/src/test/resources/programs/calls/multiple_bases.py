class A:
    pass


class B:
    pass


class M(A, B):
    def __init__(self):
        self.x = 1


m = M()  # @multiple
