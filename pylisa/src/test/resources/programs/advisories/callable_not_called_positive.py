from testnatives import echo


def helper():
    return 1


class Thing:

    def method(self):
        return 2


thing = Thing()
helper  # @function
Thing.method  # @method
thing.method  # @bound
Thing  # @class
echo  # @native
