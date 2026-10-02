#from unknown_module import unknown_class

class CSup():
    def __init__(self, x):
        super().__init__() # should call object
        self.y = x

cs = CSup(1000)
a = cs.y
print(a) # this should be 1000

class CSup2(CSup):
    def __init__(self):
        super().__init__(2000)
cs2 = CSup2()
b = cs2.y
print(b) # this should be 2000

class CSup3(unknown_class):
    def __init__(self, z):
        super().__init__(z) # this should call the synthetic super for unknown ancestor, that will call object.__init__.

cs3 = CSup3(25)
c = cs3.z # this should be #TOP#