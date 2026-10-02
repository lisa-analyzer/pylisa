def count(n):
    if n <= 0:
        return 0
    return count(n - 1) + 1


x = count(3)  # @direct
g = count
y = g(3)  # @variable
