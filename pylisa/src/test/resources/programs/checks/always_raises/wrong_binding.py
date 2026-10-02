import argparse


def unpack(key, iv, payload, mac=False):
    if not isinstance(payload, bytes):
        raise TypeError("payload must be bytes")
    return payload


parser = argparse.ArgumentParser()
parser.add_argument("-k", "--key", required=True)
parser.add_argument("-m", "--mac", required=False)
args = parser.parse_args()
data = input()
# the flag is passed as the payload: it is None or a str, never bytes
unpack(args.key, data, args.mac)  # @unpack
