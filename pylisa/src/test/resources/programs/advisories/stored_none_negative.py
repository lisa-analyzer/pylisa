def maybe(flag):
    if flag:
        return 1


def forever():
    while True:
        pass


def numbers():
    yield 1


sometimes = maybe(input())  # @sometimes
generator = numbers()  # @generator
never = forever()  # @bottom
