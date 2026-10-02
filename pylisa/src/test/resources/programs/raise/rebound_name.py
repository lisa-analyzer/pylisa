class ValueError(Exception):
    pass


def not_implemented():
    raise NotImplementedError()


c = input()
if c == 'a':
    raise ValueError()  # @rebound
if c == 'b':
    not_implemented()  # @not_implemented
