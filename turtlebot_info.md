# TurtleBot Info

Just a file containing some info about the TurtleBot ;)

## Links

- [TurtleBot Documentation ](https://turtlebot.github.io/turtlebot4-user-manual/)(github)
- [Hardware Specifications](https://www.ncnynl.com/archives/202208/5371.html)
    - LIDAR: `RPLIDAR A1M8`: [Offical Documentation](https://www.slamtec.com/en/Lidar/A1)
- [Quick Start](https://www.ncnynl.com/archives/202208/5372.html)
- Tutorials
    - [Network Configuration](https://www.ncnynl.com/archives/202208/5374.html)
 
## ROS

- Installation on [Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) (humble)
    - Setup Command (to use `ros2` in the cli with bash): `source /opt/ros/humble/setup.bash`
- [Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
 
## Create 3

Drives the robot --> buttons, lights, wheels

Has its own ROS nodes!

- [Documentation](https://iroboteducation.github.io/create3_docs/)

# Credetials

## Router

- ssid: `eurovision-router`
- password: `turtlebot`
- [web interface](http://192.168.31.1) (enter password in big entry field)

## Raspberry Pi

Drives Sensors --> Lidar, Default Camera

- SSH into it: `ssh ubuntu@192.168.31.5`
- Password: `turtlebot4`

# Debug Notes

## Sourcing

**always source** ROS on your PC!

In the repository: `source setup-bash.sh` --> you will see a little `(EuroVision)` in your terminal window.

To launch vscode sourced:

```bash
source setup-bash.sh
code .
```

## Setup

Do the following steps in the order below

1. power router, connect to Tsinghua LAN
2. connect to router with laptop, ensure that connected to internet (open network login page, insert credentials)
    - you can connect to a VPN
3. power up robot by putting it on access point
    - it will first show solid red (= robot error, cf. [light ring doc](https://iroboteducation.github.io/create3_docs/hw/face/))
    - eventually it will turn white and play a sound, then turn green = successfully connected to network

Investigating can be done by:

- `ros2 topic echo /battery_state` (takes a minute or so to show output)
- running diagnostics as recommended in the [docs](https://turtlebot.github.io/turtlebot4-user-manual/troubleshooting/diagnostics.html) did not work! Got error regarding namespace, not sure how to fix this (tried editing the launch file, but did not help. Passing a random namespace does something).


## LEGACY Getting it to run 

- ensure router is connected to internet (web interface)
- ensure raspberry pi has synched RTC
    - check with `timedatectl`
    - just `sudo reboot` (once connected), and it will work
- **source with script** setup-bash.sh on pc
- wait a bit, takes a few seconds

### various other things tried

- updating all packages ~> gave error for `ros-humble-turtlebot4-setup` due to something with `confy`
- source ros on pi

### running custom python code

- source the compiled files (in working directory!): `source install/local_setup.bash`
- have vscode: first source a terminal, then run code from workspace: `code .`

### more things tried

- updating firmware of Create3 wia [webserver](https://iroboteducation.github.io/create3_docs/webserver/connect/) --> now on H.2.6
- getting sneaky suspicion that it might be a networking issue...
    - doesn't matter if it is connected to the lan or not
    - maybe it is a frequency issue --> attempting to only use 2.4GHz --> router only supports 2.4GHz (printed on it), could not get phone hotspot to run (need to change wifi settings on robot?)
    - it **was the authentification** --> set it to `WAP2` ==> works reliably :D

## Battery Investigation

### Observations

- the custom code returns 0V and 0% charged
- according to the [specifications](https://iroboteducation.github.io/create3_docs/hw/electrical/) this happens if $U<12\text{V}$ --> it switches to a `/robot_power` service, no idea what that service does
    - note that the button pressing still works, but the LED ring is not always overwritten
- same output is obtained when subscribing to the topic directly i.e. `ros2 topic echo /battery_state`
- when running teleop it works for a few seconds/minutes, and then all lights vanish and the sensors stop spinning --> no power
    - the battery information is still streamed however(?) ~> maybe the create3 hardware is still somehow awake

### Hypothesis

The battery has been stored without charge for a long time and now the battery is dead. The bot can work on power, but since the battery voltage is always low, the robot is always in the weird `/robot_power` service. This leads to the "laggy" behavior.

It also explains all networking issues etc. as the create3 hardware and/or the Pi had been rebooting themselves.

### Next Steps

To check this we can either (1) replace the battery or (2) more elegantly power it directly with some power source or (3) add a unregulated battery to the [cargo bay](https://iroboteducation.github.io/create3_docs/hw/adapter/).

Continuing trying to attempt to develop is pointless as the platform does not behave stable.

### Solution

- installed spare battery (from TA)
- works!
    - when subscribing to `/battery_state` it actually gives a battery state: voltage and percentage
    - can run teleop and navigate the robot around for quite some time
    - test code is actually flawed with the message, but the LED lighting works
- note
    - is still laggy after booting --> give it some minutes
    - subscribing to topics also takes some time, also when starting an application (probably due to the Pi relay thing)
    - still look at "getting it to run" on how to get it all to run
