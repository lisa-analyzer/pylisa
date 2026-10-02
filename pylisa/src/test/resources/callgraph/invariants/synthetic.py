import testnatives


def keep(f):
    return f


@keep  # @decorator
def handler():
    return 1


class Child(testnatives.Plain):
    def __init__(self):
        super().__init__()  # @super


n = Child()  # @child
h = handler()
