def require(s):
    if s is None:
        raise ValueError("none")
    return s


def convert(s):
    # also called by whoever holds the list below, with other arguments
    return require(s)  # @inside


handlers = [convert]
convert(None)  # @direct
