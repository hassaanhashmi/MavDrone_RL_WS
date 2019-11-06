#!/usr/bin/env python
import roslaunch
import rospy
import rospkg
import os


class ROSLauncher(object):
    def __init__(self, rospackage_name, launch_file_name):

        self._rospackage_name = rospackage_name
        self._launch_file_name = launch_file_name

        self.rospack = rospkg.RosPack()

        # Check Package Exists
        try:
            pkg_path = self.rospack.get_path(rospackage_name)
            rospy.logdebug("Package FOUND...")
        except rospkg.common.ResourceNotFound:
            rospy.logwarn("Package NOT FOUND...")
            
        # If the package was found then we launch
        if pkg_path:
            rospy.loginfo(
                ">>>>>>>>>>Package found in workspace-->"+str(pkg_path))
            launch_dir = os.path.join(pkg_path, "launch")
            path_launch_file_name = os.path.join(launch_dir, launch_file_name)
            self._path_launch_file_name = path_launch_file_name

            rospy.logwarn("path_launch_file_name=="+str(path_launch_file_name))

            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, [path_launch_file_name])
            self.launch.start()

            rospy.loginfo(">>>>>>>>>STARTED Roslaunch-->" +
                          str(self._launch_file_name))
        else:
            assert False, "No Package Path was found for ROS apckage ==>" + \
                str(rospackage_name)

    def restart(self):
        self.launch.shutdown()
        #a double check before starting launch file again
        rospy.logwarn("path_launch_file_name=="+str(self._path_launch_file_name))
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self._path_launch_file_name])
        self.launch.start()