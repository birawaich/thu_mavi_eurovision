source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp


# modify the console
ORIGINAL_PS1="$PS1"
PS1="(EuroVision) $ORIGINAL_PS1"
export PS1

echo "Did source ROS2 and the connections for el turtoise!"