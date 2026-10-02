import testnatives


def stop():
    testnatives.carry()


stop()
testnatives.may_fail()  # @after_callee
