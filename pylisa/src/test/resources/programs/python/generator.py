def gen():
    yield 1


def gen_with_return():
    yield 1
    return


g = gen()  # @generator
h = gen_with_return()  # @generator_return
