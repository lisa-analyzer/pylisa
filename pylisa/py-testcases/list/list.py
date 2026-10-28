lst = [1, 2, 3]
first = lst[0]
second = lst[1]

lst[0] = 99
third = lst[0]


def add(l, v):
    l.append(v)


fixed = [1, 2]
n_fixed = len(fixed)

empty = []
n_empty = len(empty)
r = empty.append(1)
n_appended = len(empty)

lst.append("x")
lst.append(None)
alias = lst
alias.append(2.5)
n_lst = len(lst)

other = [7]
add(other, "y")
n_other = len(other)

# an element assignment in unreachable code assigns nothing
u = [0, 0]
w = 5
if w == 5:
    pass
else:
    u[w] = 1
