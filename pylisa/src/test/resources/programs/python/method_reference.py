def take(f):
    return f


class L:
    def cb(self, m):
        pass

    def go(self):
        x = take(self.cb)  # @x
        y = self.cb  # @y


L().go()
