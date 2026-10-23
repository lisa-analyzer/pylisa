# the functions of this module read its globals, that are bound by the code of
# the module, analyzed separately from the importing module
def text():
    return "a"


K = 5


def get():
    return text()


def k_value():
    return K
