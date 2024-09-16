#!/usr/bin/env python

import argparse
import numpy as np

def gen(gcode_file, points_file):
    with open(points_file, "r") as fp:
        with open(gcode_file, "w") as f:        
            f.write("G90\n")
            f.write("G71\n")
            f.write("G55\n")
            for line in fp:
                x, y, z = line.split(",")
                x = float(x)
                y = float(y)
                z = float(z)
                f.write("G0 X{:.3f} Y{:.3f} Z{:.3f}\n".format(x, y, z))
                f.write("M1\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--gcode-file", default="test.txt", help="Gcode file to be generated")
    parser.add_argument("--points-file", default="points.txt", help="XYZ points input")

    args = parser.parse_args()


    gen(args.gcode_file, args.points_file)