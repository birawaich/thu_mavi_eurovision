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

# Architecture

## SLAM and Calibration

SLAM requires calibrated cameras. Unsure if this varies from SLAM to SLAM stack, or if there exists a "golden standard" for calibration.

- ORB-SLAMv3 has a tutorial in the repo

This calibration can be done offline.

It should only be a few lines in opencv? cf. ChatGPT

## SLAM

Key idea: publish a map from sensor data

- Implementation
    - from scratch (lots of work, not gonna be faster unless done in CPP)
    - [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) (easily available and easy to deploy)
        - also what turtlebot uses with `ros2 launch turtlebot4_navigation slam.launch.py`
        - cf. githubrepo: [package dependencies](https://github.com/turtlebot/turtlebot4/blob/humble/turtlebot4_navigation/package.xml)
    - [ORB-SLAMV3](https://github.com/UZ-SLAMLab/ORB_SLAM3) (state of the art, has python bindings(?))
        - unsure how to use LiDar

- Notes
    - map can be visualized with `ros2 launch turtlebot4_viz view_robot.launch.py`

- Sources
    - [Turtlebot Tutorial (Generating Map)](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html)

## SLAM and Navigation

Key idea: using SLAM to build a map on the go and navigate using _Nav2_

- SLAM publishes to `/map` topic, provide map->odom transformation (?)
- Navigation is than handled by Nav2
- works out of the box for SLAM Toolbox, needs more work for ORB-SLAMv3

- Sources
    - Relevant Site in [Nav2 Doc](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)

## Navigation

Key idea: use existing navigator in turtlebot, relies on Nav2SimpleCommander

- launch navigation stack `ros2 launch turtlebot4_navigation nav2.launch.py`
- somehow generate goals

- Sources
    - Tutorial in [TurtlebotDoc](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html)
    - Nav2: Simple Commander on [GitHub](https://github.com/ros-navigation/navigation2/tree/galactic/nav2_simple_commander) (for light documentation)

## SLAM and Pose Planning for Navigation

Key idea: overlay map with a grid of poses, decided which ones to visit next

- SLAM publishes maps in `pgm` format and with `yaml` descriptions

More info: ask ChatGPT: how to do the above

> I would like to explore a map (which is changing) based on pgm files. how would I do this? The pgm files stem from a SLAM in ros and the output should be poses for easynavigator in the nav2 stack
[](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html)
Difficultty: include heading information of where is something interesting ↝ need to think about this once this far

# Debug Notes

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
- have vscode: first source a terminal, then run code from workspace: `code .`

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

