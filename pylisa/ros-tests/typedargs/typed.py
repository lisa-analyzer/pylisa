def typedF(x: int | str) -> int | str:
  print(type(x))
  return x

x = typedF(3.5)
print(x)