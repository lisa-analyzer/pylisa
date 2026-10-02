import testnatives


def check(bag):
    return 'a' in bag


def main():
    found = check(testnatives.Bag())  # @membership
    return found


main()
