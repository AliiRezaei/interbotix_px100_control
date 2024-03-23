import argparse
import sys

try:
    parser = argparse.ArgumentParser()
    parser.add_argument("k", help="k = ", type=float)
    args = parser.parse_args()

    print(args.k)

except:
    e = sys.exc_info()[0]
    print(e)

