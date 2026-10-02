flag = True

if flag:

    class Secret:
        TAG = "from_true_branch"

        def reveal(self):
            return "A"
else:

    class Secret:
        TAG = "from_false_branch"

        def reveal(self):
            return "B"

x = Secret()
