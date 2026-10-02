class Greeter:
    def greet(self, name):
        return name


g = Greeter()
x = g.greet("a")  # @direct
f = g.greet
y = f("b")  # @through
