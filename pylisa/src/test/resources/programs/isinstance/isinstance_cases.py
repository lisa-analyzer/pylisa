s = 'a'
n = None
b = True
i = 3
unknown = input()
r1 = isinstance(s, bytes)  # @str_bytes
r2 = isinstance(n, bytes)  # @none_bytes
r3 = isinstance(s, object)  # @str_object
r4 = isinstance(unknown, bytes)  # @unknown_bytes
x = s if unknown else n
r5 = isinstance(x, bytes)  # @str_or_none_bytes
if not isinstance(x, bytes):
    y = 1  # @taken
