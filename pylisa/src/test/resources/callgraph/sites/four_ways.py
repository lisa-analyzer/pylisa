import testnatives
from testnatives import echo

a = testnatives.echo(1)  # @module
f = testnatives.echo
b = f(2)  # @variable
c = echo(3)  # @imported
d = testnatives.echo(4)  # @again
