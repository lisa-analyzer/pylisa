n = int(input())
s = 's' * (n != 1)
assert s == '' or s == 's'  # @plural
t = True * 'ab'
assert t == 'ab'  # @bool_count
