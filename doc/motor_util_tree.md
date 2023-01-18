# motor_util help
```
Utility for communicating with motor drivers
Usage: motor_util [OPTIONS] [SUBCOMMAND]

Options:
  -h,--help                   Print this help message and exit
  -l,--list                   Verbose list connected motors
  -c,--check-messages-version Check motor messages version
  --no-list                   Do not list connected motors
  -v,--version                Print version information
  --list-names-only           Print only connected motor names
  --list-path-only            Print only connected motor paths
  --list-devpath-only         Print only connected motor devpaths
  --list-serial-number-only   Print only connected motor serial numbers
  -u,--user-space             Connect through user space usb
  -n,--names NAME ...         Connect only to NAME(S)
  --allow-simulated Needs: --names
                              Allow simulated motors if not connected
  -p,--paths PATH ...         Connect only to PATHS(S)
  -d,--devpaths DEVPATH ...   Connect only to DEVPATHS(S)
  -s,--serial_numbers SERIAL_NUMBER ...
                              Connect only to SERIAL_NUMBERS(S)
  --set-api TEXT              Send API data (to set parameters)
  --api                       Enter API mode
  --run-stats NUM_SAMPLES=100 Check firmware run timing

Subcommands:
  set                         Send data to motor(s)
  read                        Print data received from motor(s)
```

## set subcommand
```
Send data to motor(s)
Usage: motor_util set [OPTIONS] [SUBCOMMAND]

Options:
  -h,--help                   Print this help message and exit
  --host_time UINT            Host time
  --mode UINT:value in {open->0,damped->1,current->2,position->3,torque->4,impedance->5,velocity->6,state->7,current_tuning->8,position_tuning->9,voltage->10,phase_lock->11,stepper_tuning->12,stepper_velocity->13,hardware_brake->14,clear_faults->250,fault->251,sleep->253,crash->254,reset->255} OR {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,250,251,253,254,255}
                              Mode desired
  --current FLOAT             Current desired
  --position FLOAT            Position desired
  --velocity FLOAT            Velocity desired
  --torque FLOAT              Torque desired
  --torque_dot FLOAT          Torque dot desired
  --reserved FLOAT            Reserved command

Subcommands:
  state                       State control mode
  stepper_tuning              Stepper tuning mode
  position_tuning             Position tuning mode
  current_tuning              Current tuning mode
  stepper_velocity            Stepper velocity mode
  voltage                     Voltage mode
```

### subcommands of set
```
Voltage mode
Usage: motor_util set voltage [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  --voltage FLOAT             Vq voltage desired
```

```
Current tuning mode
Usage: motor_util set current_tuning [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  --amplitude FLOAT           Current tuning amplitude
  --frequency FLOAT           Current tuning frequency hz, or hz/s for chirp
  --mode UINT:value in {sine->0,square->1,triangle->2,chirp->3} OR {0,1,2,3}
                              Current tuning mode
  --bias FLOAT                Current trajectory offset
```

```
Position tuning mode
Usage: motor_util set position_tuning [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  --amplitude FLOAT           Position tuning amplitude
  --frequency FLOAT           Position tuning frequency hz, or hz/s for chirp
  --mode UINT:value in {sine->0,square->1,triangle->2,chirp->3} OR {0,1,2,3}
                              Position tuning mode
  --bias FLOAT                Position trajectory offset
```

```
Stepper tuning mode
Usage: motor_util set stepper_tuning [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  --amplitude FLOAT           Phase position tuning amplitude
  --frequency FLOAT           Phase tuning frequency hz, or hz/s for chirp
  --mode UINT:value in {sine->0,square->1,triangle->2,chirp->3} OR {0,1,2,3}
                              Phase tuning mode
  --kv FLOAT                  Motor kv (rad/s)
```

## read subcommand
```
Print data received from motor(s)
Usage: motor_util read [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  -s,--timestamp-in-seconds   Report motor timestamp as seconds since start and unwrap
  --poll                      Use poll before read
  --ppoll                     Use multipoll before read
  --aread                     Use aread before poll
  --frequency FLOAT           Read frequency in Hz
  --statistics                Print statistics rather than values
  --read-write-statistics     Perform read then write when doing statistics test
  --text TEXT=[log] ...       Read the text api for variable
  -t,--host-time-seconds      Print host read time
  --publish                   Publish joint data to shared memory
  --csv                       Convenience to set --no-list, --host-time-seconds, and --timestamp-in-seconds
  -f,--reserved-float         Interpret reserved 1 & 2 as floats rather than uint32
  -r,--reconnect              Try to reconnect by usb path
  -v,--compute_velocity       Compute velocity from motor position
  -p,--precision INT          floating point precision output
  --timestamp-frequency FLOAT Override timestamp frequency in hz
  --bits NUM_SAMPLES RANGE=[100,1]
                              Process noise and display bits, ±3σ window 100 [experimental]
```
