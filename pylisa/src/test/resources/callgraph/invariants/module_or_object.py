import sys
import testnatives


class Holder:
    def echo(self, v):
        return v


r = testnatives if len(sys.argv) > 1 else Holder()
x = r.echo(1)  # @either
