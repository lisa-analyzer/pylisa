def require(m):
    if m is None:
        raise ValueError("none")
    return m


class Sink:
    def write(self, m):
        return require(m)


def on_msg(m, sink):
    if sink is None:
        return
    # in the contexts the analysis sees, this call is never made
    sink.write(m)


callbacks = [on_msg]
on_msg("x", None)
Sink().write(None)  # @direct
