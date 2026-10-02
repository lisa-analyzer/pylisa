x = 'a'
assert x == 'a'  # @proved
assert x != 'b'  # @proved_ne
assert x == 'a', 'with a message'  # @proved_message
if x == 'b':
    assert x == 'a'  # @unreachable
assert x == 'b'  # @fails
y = input()
assert y == 'a'  # @after_failure
