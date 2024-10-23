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

- ssid: `eurovision_router`
- password: `turtlebot`
- [web interface](http://192.168.31.1) (enter password in big entry field)

## Raspberry Pi

Drives Sensors --> Lidar, Default Camera

- SSH into it: `ssh ubuntu@192.168.31.5`
- Password: `turtlebot4`


## Getting it to run

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
