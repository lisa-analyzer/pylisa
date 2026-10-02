def drain(q):
    while (item := q.pop()):
        handle(item)
