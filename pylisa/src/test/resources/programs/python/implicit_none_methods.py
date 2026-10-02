class Box:
    def put(self, x):
        self.x = x


def pick(n):
    if n == 1:
        v = 1
    elif n == 2:
        v = 2
    else:
        v = 3


b = Box()
p = b.put(1)  # @method
q = pick(len([]))  # @branches
