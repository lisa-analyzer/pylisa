import mystery


@mystery.deco  # @decorator
def handler():
    return 1


h = handler()  # @call
