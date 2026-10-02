class T:
    def __init__(s, n):
        s.c = T(n - 1) if n > 0 else None  # @nested


t = T(2)  # @outer
