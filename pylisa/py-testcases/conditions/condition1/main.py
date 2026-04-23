from config import settings
x = 3
if not settings.DEBUG:
    y = 5
else:
    y = 4

def test():
    if not settings.DEBUG:
        return 0
    else:
        return 1

y = test()