import testnatives


def build():
    return testnatives.Widget('w')


def main():
    w = build()  # @outer
    return w


main()
