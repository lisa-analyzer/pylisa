import testnatives


def relay(v):
    return testnatives.echo(v)  # @inner


p = relay(1)  # @first
q = relay("two")  # @second
i = 0
while i < 3:
    r = relay(i)  # @loop
    i = i + 1
