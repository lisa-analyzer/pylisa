import testnatives

testnatives.marked_and_plain()  # @both
testnatives.joined_marked_first()  # @marked_first
testnatives.joined_plain_first()  # @plain_first
testnatives.marked_with_unreachable()  # @with_unreachable


def helper():
    testnatives.marked_only()  # @inside


helper()  # @caller
