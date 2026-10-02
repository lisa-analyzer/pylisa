class Ctx:
    def __enter__(self):
        return self

    def __exit__(self, a, b, c):
        return False


with Ctx() as c:
    pass
