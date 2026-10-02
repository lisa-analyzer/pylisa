class A:
    def get(self):
        return self

    def value(self):
        return 1


def make():
    return A()


v = make().value()
