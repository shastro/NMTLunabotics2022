#!/usr/bin/env python3

import sys

import argparse as ap



def calc_map(start, end, nbits):
    maxinput = 2**(nbits) - 1
    slope = (end-start)/maxinput
    return (start, slope)

def main():

    parser = ap.ArgumentParser()
    parser.add_argument("start", type=float, help="Start of desired value range (float)")
    parser.add_argument("end", type=float, help="End of desired value range (float)")
    parser.add_argument("nbits", type=int, help="Number of bits used to represent value range (int)")

    args = parser.parse_args()

    start, slope = calc_map(args.start, args.end, args.nbits)

    ss = f"<Value slope=\"{slope}\" intercept=\"{start}\" />"
    print(ss)

if __name__ == "__main__":
    main()
