# Motor util examples
## Help
```console
$ motor_util -h
```
For a subcommand, e.g. `set`
```console
$ motor_util set -h
```
Note that `bash` tab completion is available for all the motor_util commands

## Listing devices
```console
$ motor_util
2 connected motors
         Dev      Name  Serial number        Version           Path
-------------------------------------------------------------------
   /dev/mtr1        wl   206033683053       v0.1-210          1-1.1
   /dev/mtr0        J1   208C366B4752       v0.1-219            1-7
```

## Selection commands
By default all motors will be connected when executing a command. Commands can 
be limited to a subset of connected motors by using the below options for 
selecting motors. In addition the order of the the selection affects the order of 
the commands or statuses read, so one can use a motor selection command to ensure 
consistent ordering.
### By devname
```console
$ motor_util -d /dev/mtr0 /dev/mtr1
2 connected motors
...
```

### By name
```console
$ motor_util -n J1 wl
2 connected motors
...
```

### By serial number
```console
$ motor_util -s 208C366B4752 206033683053
2 connected motors
...
```

### By path
Note the path is well defined by the hub structure and the ports. That is a device 
on port 1 of hub 1 that is on port 1 of the host PC will always show up as 1.1. The 
initial 1 is the usb root hub. A system may have several root hubs, each of which 
is capable of 480 Mbps USB.
```console
$ motor_util -p 1-7 1-1.1
2 connected motors
...
```

By default the motor_util command will print the list of the selected motors before 
printing out any other responses. Using the `--no-list` argument will suppress 
that output. The responses to commands below are all shown as if the command was 
run as `motor_util --no-list`, but `--no-list` was not shown below to reduced 
clutter.

## Reading status
By default at a frequency of 1000 Hz
```console
$ motor_util read
...
mcu_timestamp0, host_timestamp_received0, motor_position0, joint_position0, iq0, torque0, motor_encoder0, reserved00, reserved10, reserved20, 
 349382901, 0,  0.00000,  0.00000, -0.53306,  0.00000, 0, 0.00000, 349386072, 349369072, 
 ...
```

### Commonly used read options
- -s,--timestamp-in-seconds   Report motor timestamp as seconds since start and unwrap
- --frequency FLOAT           Read frequency in Hz
- --text TEXT=[log] ...       Read the text api for variable
- -t,--host-time-seconds      Print host read time
- --csv                       Convenience to set --no-list, --host-time-seconds, and --timestamp-in-seconds

### Reading to csv
Using the `--csv` option above provides a csv readable format that is output to the 
terminal. This can be redirected to a file, or printed and redirected to a file using 
`tee`. Using tee is also sometimes helpful just in order to provide a buffer that prevents
file writes from blocking the process.
```console
$ motor_util read --csv > data.csv
$ motor_util read --csv | tee data.csv
$ motor_util read --csv | tee data.csv > /dev/null  # write to file with buffering
```

## Set
The `set` command is used to send commands to the motors. The command will be 
duplicated and sent to any selected motor controllers, which is all connected 
motor controllers by default. Below are a selection of the modes and commands that 
are often used
```console
$ motor_util set --mode open        # phases floating
$ motor_util set --mode damped      # phase shorted
$ motor_util set --mode current --current 1     # 1 A current command
$ motor_util set --mode position --position 10  # 10 rad position command
$ motor_util set --mode velocity --velocity 5   # 5 rad/s velocity command
$ motor_util set --mode torque --torque 3       # 3 Nm torque control command
$ motor_util set --mode reset       # hard motor controller reset
```

### Set subcommands
`set` also has subcommands that are used for sending specific command packets 
to the motor controller. The two common ones are `position_tuning` and `current_tuning`. 
They both work similarly. They are used to generate simple cycling trajectories 
that are useful for tuning control gains and seeing the response. In either current 
or position they can produce a sine wave, a linear frequency chirp sine wave, a 
square wave, or a triangle wave. Some examples are:
```console
$ # position control square wave with parameters below. Bias is the center offset
$ motor_util set position_tuning --amplitude 1 --frequency 2 --bias 3 -- mode square
$ # current control chirp starting from 0 and increasing at 10 Hz/s
$ motor_util set current_tuning --amplitude 1 --frequency 10 --mode chirp
```

## API
There is also a separate communication channel over USB that is used for tuning 
and debugging. It is referred to as the "text api" as it communicates using ASCII 
text. `motor_util` can be used in an interactive `--api` mode with responses to 
prompts or it can be used from command line arguments. In `--api` mode, tab 
completion is supported and it can give the commands supported by typing `help`. 
Some api parameters can be both set and read. To set use `=` without a space as 
sown below.
```console
$ motor_util --api
```
Api parameters can als be set or read one time from the command line as follows:
```console
$ motor_util --set-api kp
$ motor_util --set-api kp=2
```
Api can also be read using `read --text`. This allows for reading one or more values. 
Also the `log` is a special api variable that stores multiple log messages and is the 
default variable when using `read --text`. Reading the log will continue to follow
and print as new messages are added to the log by the motor controller.
```console
$ motor_util read --text    # read --text log is implied
Motor encoder init failure
Output encoder init failure
drv configure failure
3v3: 3.344064
finished startup
$ motor_util read --text ia ib ic --frequency 10    # frequency is supported in text mode
-0.115585, -0.194335, 0.007279
-0.131699, -0.194335, 0.071733
...
```

## Performance
`motor_util` is not meant for performance but it can be used to both test performance 
and also may be useful to have fixed frequency measurements. The motor controller 
system is currently not meant to be synchronized with the host PC in any way. 
However, if the host PC sends out USB commands and requests statuses at a fixed 
frequency and is expedient about processing the results, the system may behave 
as a mostly synchronized system. This can be useful if one wants fixed frequency data 
for processing or to check latency in responses. I have found a number of techniques 
to be useful when trying to get the most performance from the host PC running 
`motor_util`. First it helps to prevent the host PC from going into "idle mode". 
An easy way to do this is to install and run `cyclictest`, which will prevent idle 
mode and also do some continuously running latency tests.
```console
$ sudo apt install rt-tests
$ sudo cyclictest
```
Second, running `motor_util` at high priority will allow it to usurp other 
processes and get better latencies.
```console
$ sudo chrt -f 99 motor_util read --read-write-statistics
```
Next pinning to a specific cpu core is also useful in that by default the process 
will move between cores and cause some latency during that process
```console
$ sudo taskset -c 11 chrt -f 99 motor_util read --read-write-statistics
```
It also may make sense to isolate that cpu core or your system so that other processes 
can't run on it unless specifically requested to. This is done the the Linux 
kernel command line that is used at boot. For `systemd-boot` on Pop OS form me this 
is done by editing `/boot/efi/loader/entries/Pop_OS-current.conf` to add `isolcpus`, then
maybe run `sudo bootctl update` and reboot.
```
...
options root=UUID-... ro quiet loglevel=0 systemd.show_status=false splash isolcpus=5,11
```
Above I isolated both cores of one cpu that has hyperthreading.

Next one can also pin the USB interrupt to that same core. This allows return from system calls 
between the `motor_util` process and the system to be on the same cpu.
```console
# cat /proc/interrupts
...
 130:      29356          0      23170      19089          0          0     601535          0          0          0    9228258          0  IR-PCI-MSI 327680-edge      xhci_hcd
# irqbalance --banirq 130
# echo 11 > /proc/irq/130/smp_affinity_list
```
And a final improvement in performance is possible by installing preempt rt linux 
kernel patches.

With all these settings in place one can use the `read --read-write-statistics` 
command to see the various read and write times and the period measurements in ns. 
THe final number in the output is the ~avg_hops~ count and refers to on average 
how many host PC loops behind the `host_timestamp_received` on the motor controller 
is versus the `host_timestamp` sent. The goal is to have this number be 0, meaning that 
for each host PC loop data is read then set and the response read by the next loop 
shows that the data set from the last loop has been received and processed. As frequency 
is increased the average hops will increase from 0 indicating that data is becoming 
pipelined, i.e. several the response in a read may be the response to a set from 
several host PC loops prior. For the best realtime control it's recommended to work 
at the max frequency that "avg_hops" is at or near 0. On my laptop with the modifications 
above this is about 3 kHz
```console
sudo taskset -c 11 chrt -f 99 motor_util read --read-write-statistics --frequency 3000
```
