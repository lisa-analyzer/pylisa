def f(a, b, *, x, y):
	print("a: " + str(a))
f(10,20, x=30,y=40)
f(10,20,y=40,x=30)
f(10,x=30,b=20,y=40)
f(x=30,y=40,b=20,a=10)
