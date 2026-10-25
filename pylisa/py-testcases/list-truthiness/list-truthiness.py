# a list that is truthy (e.g. if xs:), or that an element has been appended to,
# surely has an element: its first and last elements are in range
import _sv_verifier

xs = [1, 2]
xs.pop()
if xs:
    y = xs.pop()
else:
    # xs is empty: IndexError
    z = xs.pop()
s = _sv_verifier.nondet_str()
words = s.split()
while words and (w := words.pop()):
    print(w)
st = []
st.append(1)
a = st.pop()
# st might be empty: IndexError might be raised
b = st.pop()
ys = _sv_verifier.nondet_list(_sv_verifier.nondet_int)
while ys:
    v = ys[-1]
    ys.pop()
# ys is empty
if _sv_verifier.nondet_bool():
    u = ys[0]
