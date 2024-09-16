#!/usr/bin/env python

import argparse
import numpy as np

def gen(file, x, y, z):
    with open(file, "w") as f:        
        for k in range(len(z)):
            for i in range(len(x)):
                for j in range(len(y)):                   
                    f.write("{:.2f}, {:.2f}, {:.2f}\n".format(x[i], y[j], z[k]))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--file", default="points.txt", help="XYZ points file to be generated")
    parser.add_argument("xmin", type=float, default=0.0, help="X min coordinate")
    parser.add_argument("xmax", type=float, default=0.0, help="X max coordinate")
    parser.add_argument("xnum", type=int, default=10, help="X number of points")
    parser.add_argument("ymin", type=float, default=0.0, help="Y min coordinate")
    parser.add_argument("ymax", type=float, default=0.0, help="Y max coordinate")
    parser.add_argument("ynum", type=int, default=10, help="Y number of points")
    parser.add_argument("zmin", type=float, default=0.0, help="Z min coordinate")
    parser.add_argument("zmax", type=float, default=0.0, help="Z max coordinate")
    parser.add_argument("znum", type=int, default=10, help="Z number of points")

    args = parser.parse_args()

    x = np.linspace(args.xmin, args.xmax, args.xnum)
    y = np.linspace(args.ymin, args.ymax, args.ynum)
    z = np.linspace(args.zmin, args.zmax, args.znum)

    gen(args.file, x, y, z)