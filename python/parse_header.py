#!/usr/bin/env python

import yaml

def print_item(item,indent=0):
    for i in item:
        if(type(i) == list):
            if(type(i[0]) == str):
                print(" "*indent + f"{i[0]} {i[1]};\t// {i[2]}")
            else:
                print_item(i, indent+2)
        elif(type(i) == dict):
            print(" "*indent + "union {")
            for k,v in i.items():
                print(" "*(indent+2) + "struct {")
                print_item(v, indent+4)
                if (k == "none"):
                    print(" "*(indent+2) + "};")
                else:
                    print(" "*(indent+2) + f"}} {k};")
            print(" "*indent + "};")
        else:
            print("unknown" + i)

def print_header(y):
    print("typedef struct {")
    print_item(y["Command"], 2)
    print("} Command;")
    print()
    print("typedef struct {")
    print_item(y["Status"], 2)
    print("} Status;")
    print("")

def cli11_option(option, name, help, name_short=None):
    if not name_short:
        name_short = name
    print(f'{option}->add_option("--{name_short}", command.{name}, "{help}");')

def print_cli11(y):
    for i in y:
        if(type(i) == dict):
            for k,v in i.items():
                if (k == "none"):
                    for j in v:
                        cli11_option("set", j[1], j[2], j[1].split("_")[0])
                else:
                    print(f"// suboptions: {k}")
                    for j in v:
                        cli11_option("set", k + "." + j[1], j[2])
        else:
            cli11_option("set", i[1], i[2], i[1].split("_")[0])

def run():
    with open("motor_messages.yaml") as f:
        y = yaml.load(f, Loader=yaml.FullLoader)
        print_header(y)
        print_cli11(y["Command"])


if __name__ == "__main__":
    run()
