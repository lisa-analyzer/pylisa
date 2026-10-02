import testnatives

c = input()
if c == 'a':
    # raises, or runs until the program is stopped
    testnatives.raise_or_block()  # @loop
if c == 'b':
    testnatives.fail()  # @fails
