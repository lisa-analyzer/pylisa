import sys
import testnatives

if len(sys.argv) > 1:
    testnatives.fail()
testnatives.stuck()  # @stuck
