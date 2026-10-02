from testnatives import echo


def nothing():
    pass


returned = echo(None)  # @native
implicit = nothing()  # @function
