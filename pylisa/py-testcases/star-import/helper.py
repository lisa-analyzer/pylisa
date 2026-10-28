def one():
    return 1


def text():
    return "a"


# shadows the builtin max for the modules importing all the names of this one
max = 64
_hidden = 3
