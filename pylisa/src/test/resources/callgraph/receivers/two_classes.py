import sys
import testnatives


class A(testnatives.Plain):
    pass


class B(testnatives.Plain):
    pass


K = A if len(sys.argv) > 1 else B
k = K()  # @two_classes
p = A()  # @one_class
