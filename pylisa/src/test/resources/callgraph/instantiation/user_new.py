class U:
    def __new__(cls):
        return object.__new__(cls)

    def __init__(self):
        self.tag = 1


u = U()  # @user
