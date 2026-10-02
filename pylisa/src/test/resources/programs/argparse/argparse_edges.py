import argparse

p = argparse.ArgumentParser()
p.add_argument("--fast", dest="speed", action="store_true")
p.add_argument("--speed", default="n")
p.add_argument("-o", "--out")
a = p.parse_args()  # @same_dest

q = argparse.ArgumentParser()
q.add_argument("-m")
q.set_defaults(x=1)
b = q.parse_args()  # @after_defaults

r = argparse.ArgumentParser()
r.add_argument(dest="x")
c = r.parse_args()  # @dest_only

s = argparse.ArgumentParser()
s.add_argument("--x", action="store_true", default=None)
d = s.parse_args()  # @store_true_none

t = argparse.ArgumentParser()
sub = t.add_subparsers(dest="cmd")
e = t.parse_args()  # @subparsers

u = argparse.ArgumentParser(exit_on_error=False)
u.add_argument("-m")
f = u.parse_args()  # @no_exit

v = argparse.ArgumentParser()
v.add_argument("-n", type=int)
g = v.parse_args()  # @converter
