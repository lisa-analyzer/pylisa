ok = True
try:
    int('a')
except ValueError:
    ok = False
assert ok  # @after_try
