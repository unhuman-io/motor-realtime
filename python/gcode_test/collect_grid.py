#!/usr/bin/env python

import argparse
import numpy as np
import motor
import curses
import time

def run(file_out, points_file, vars, navg):
    motor_manager = motor.MotorManager()
    motor_manager.get_connected_motors()
    active_motor = motor_manager.motors()[0]

    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()
    curses.curs_set(0)
    if curses.has_colors():
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN, -1)
        curses.init_pair(2, curses.COLOR_RED, -1)

    stdscr.addstr(0, 0, "Press '^C' to quit")
    stdscr.addstr(1, 0, "X")
    stdscr.addstr(1, 10, "Y")
    stdscr.addstr(1, 20, "Z")
    stdscr.nodelay(True)

    nvars = len(vars)
    for i in range(nvars):
        stdscr.addstr(4, 20*i, vars[i])

    stdscr.refresh()

    try:
        with open(points_file, "r") as fp:
            with open(file_out, "w") as f:
                f.write("x, y, z, ")
                for i in range(nvars):
                    f.write("{}, ".format(vars[i]))
                f.write("\n")

                for line in fp:
                    x, y, z = line.split(",")
                    x = float(x)
                    y = float(y)
                    z = float(z)
                    
                    stdscr.addstr(2, 0, "{:0.3f} ".format(x), curses.color_pair(1))
                    stdscr.addstr(2, 10, "{:0.3f} ".format(y), curses.color_pair(1))
                    stdscr.addstr(2, 20, "{:0.3f} ".format(z), curses.color_pair(1))

                    while stdscr.getch() == curses.ERR:
                        for i in range(nvars):
                            val = active_motor[vars[i]].get()
                            stdscr.addstr(5, 20*i, "{:20s}".format(val), curses.color_pair(2))
                        
                        stdscr.refresh()
                        time.sleep(.1)

                    xyz_str = "{:0.3f}, {:0.3f}, {:0.3f}, ".format(x, y, z)
                    f.write(xyz_str)
                    for i in range(nvars):
                        val_avg = 0
                        for j in range(navg):
                            val_avg += float(active_motor[vars[i]].get())/navg
                        f.write("{}, ".format(val_avg))
                    f.write("\n")
                    f.flush()

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
        input("Press any key to continue")

    curses.nocbreak()
    curses.echo()
    curses.curs_set(True)
    curses.endwin()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--file-out", default="data.txt", help="data out")
    parser.add_argument("--points-file", default="points.txt", help="XYZ points input")
    parser.add_argument("--vars", default=["mai_phases", "mai_scales", "mdiag"], help="variables to be read", nargs='+')
    parser.add_argument("--navg", type=int, default=500, help="number of averages")

    args = parser.parse_args()

    run(args.file_out, args.points_file, args.vars, args.navg)
