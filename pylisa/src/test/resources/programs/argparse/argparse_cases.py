import argparse
parser = argparse.ArgumentParser(description="d")
group = parser.add_mutually_exclusive_group(required=True)
group.add_argument("-e", "--encrypt", action="store_true", help="h")
parser.add_argument("-i", "--input-file", required=True, help="h")
parser.add_argument("-m", "--mac", required=False, help="h")
parser.add_argument("-n", default=3)
parser.add_argument("pos")
parser.add_argument("-c", action="count")
args = parser.parse_args()  # @parsed
x = 1  # @after
