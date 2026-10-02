def check(x):
    assert x == 1  # @inside


def main():
    check()  # @call
    assert False  # @after


main()
