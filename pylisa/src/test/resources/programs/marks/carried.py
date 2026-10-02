import testnatives
from unknown import anything

testnatives.may_fail()  # @before
if anything():
    testnatives.carry()
testnatives.may_fail()  # @some_paths
testnatives.carry()
testnatives.may_fail()  # @after
