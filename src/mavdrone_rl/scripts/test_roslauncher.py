#!/usr/bin/env python
import roslaunch
import rospy

rospy.init_node('mavdrone', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/hmi/Projects/catkin_ws/MavDrone_RL_WS/src/mavros_moveit/mavros_moveit/launch/px4_mavros_moveit.launch"])
launch.start()
rospy.loginfo("started")