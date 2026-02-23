"""Test file for class decorators"""

def my_decorator(cls):
    """A simple class decorator"""
    return cls


@my_decorator
class SimpleClass:
    """A class with a simple decorator"""
    
    def __init__(self):
        self.value = 42
    
    def get_value(self):
        return self.value


def parametrized_decorator(param):
    """A parametrized class decorator"""
    def decorator(cls):
        cls.decorated_param = param
        return cls
    return decorator


@parametrized_decorator("test")
class DecoratedClass:
    """A class with a parametrized decorator"""
    
    def __init__(self):
        self.name = "decorated"


def multi_decorator_one(cls):
    """First decorator"""
    return cls


def multi_decorator_two(cls):
    """Second decorator"""
    return cls


@multi_decorator_one
@multi_decorator_two
class MultiDecoratedClass:
    """A class with multiple decorators"""
    pass
