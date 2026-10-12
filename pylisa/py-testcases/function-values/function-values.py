def greet(name):
    return f"Hello, {name}!"

say_hello = greet
print(say_hello("Ada"))  # Hello, Ada!
r = say_hello("Ada")
s = greet("Bob")
