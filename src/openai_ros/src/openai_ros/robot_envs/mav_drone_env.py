#!/usr/bin/env python
import enum
import rospy
import numpy
import time
import tf
import tf2_py
import tf2_ros
import mavros
import tf2_geometry_msgs
import openai_ros.robot_gazebo_env
from actionlib import simple_action_server
from actionlib import simple_action_client
from mavros_msgs.srv import CommandTOL 
from mavros_msgs.msg._State import State
from mavros_msgs.srv._SetMode import SetMode
from mavros_msgs.srv._CommandBool import CommandBool
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Quaternion
from openai_ros.openai_ros_common import ROSLauncher
from openai_ros import robot_gazebo_env




""" class ControlMode(enum.Enum):
    POSITION = 49929
    VELOCITY = 43298
 """

class MavDroneEnv(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all MavROS environments.
    """
    def __init__(self, ros_ws_abspath):
        """Initializes a new MavROS environment. \\
        To check the ROS topics, unpause the paused simulation \\
        or reset the controllers if simulation is running.

        Sensors for RL observation space (by topic list): \\
        
        * /mavros/local_position/pose                   
        * /mavros/local_position/velocity_body          
        }


        Actuations for RL action space (by topic list):
        * /cmd_vel: Move the Drone Around when you have taken off.

        Args:
        """


        rospy.logdebug("Start MavDroneEnv INIT...")
        # Variables that we give through the constructor.

        # Internal Vars
        self.controllers_list = ['my_robot_controller1, my_robot_controller2 , ..., my_robot_controllerX']

        self.robot_name_space = ""

        #reset_controls_bool = True or False
        
        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        
        super(MavDroneEnv, self).__init__(controllers_list=self.controllers_list,
                                             robot_name_space=self.robot_name_space,
                                             reset_controls=False,
                                             start_init_physics_parameters=True,
                                             reset_world_or_sim="WORLD")
        self.gazebo.unpauseSim()

        self.ros_launcher = ROSLauncher(rospackage_name="mavros_moveit",
                    launch_file_name="posix_sitl_2.launch", #to be edited and finalized
                    ros_ws_abspath=ros_ws_abspath)
        rospy.sleep(30)
        self._current_pose = PoseStamped()
        self._current_state= State()
        self._check_all_sensors_ready()
       
        # We Start all the ROS related Subscribers and publishers
        rospy.Subscriber('mavros/state', State, callback=self._stateCb, queue_size=10)
        rospy.Subscriber('mavros/local_position/pose', PoseStamped , callback=self._poseCb, queue_size=10)

        # setpoint publishing rate must be faster than 2Hz. From MavROS documentation
        self._rate = rospy.Rate(20.0)

        # set control mode
        """ control_mode = rospy.get_param("control_mode", "velocity")
        if control_mode == "position":
            _control_mode = ControlMode.POSITION
        elif control_mode == "velocity":
            _control_mode = ControlMode.VELOCITY """

        # setup control_mode->publishers/servers
        """ if (_control_mode == ControlMode.POSITION):
            self._local_pose_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        elif (_control_mode == ControlMode.VELOCITY):
        self._local_vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel",TwistStamped,  queue_size=10) """
        
        self._local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel',TwistStamped,  queue_size=10)

        #self._check_all_publishers_ready()

        self._arming_client = rospy.ServiceProxy('mavros/cmd/arming',CommandBool) #mavros service for arming/disarming the robot
        self._set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode) #mavros service for setting mode. Position commands are only available in mode OFFBOARD.
        
        #self._check_all_services_ready()

        self.gazebo.pauseSim()

        rospy.logdebug("Finished MavROSEnv INIT...")


# Methods needed by the RobotGazeboEnv
    # ----------------------------
    
    def _reset_sim(self):
        self._reset_sim_before()
        self.ros_launcher.restart()
        #rospy.sleep()
        self._reset_sim_after()

    def _poseCb(self, msg):
        self._current_pose = msg

    def _stateCb(self, msg):
        self._current_state = msg
    

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers, services and other simulation systems are
        operational.
        """
        self._check_all_sensors_ready()
        self._check_all_publishers_ready()
        #self._check_all_services_ready()
        return True
    

    def _check_all_sensors_ready(self):
        rospy.logdebug("CHECK ALL SENSORS CONNECTION:")
        self._check_current_pose_ready()
        self._check_current_state_ready()
        rospy.logdebug("All Sensors CONNECTED and READY!")
    
    def _check_current_pose_ready(self):
        self._current_pose = None
        rospy.logdebug("Waiting for /mavros/local_position/pose to be READY...")
        while self._current_pose is None and not rospy.is_shutdown():
            try:
                self._current_pose = rospy.wait_for_message("mavros/local_position/pose", PoseStamped, timeout=5.0)
                rospy.logdebug("Current mavros/local_position/pose READY=>")
            except:
                rospy.logerr("Current mavros/local_position/pose not ready, retrying for getting lp_pose")
        return self._current_pose
    
    def _check_current_state_ready(self):
        rospy.logdebug("Waiting for /mavros/local_position/velocity_body to be READY...")
        while self._current_state is None and not rospy.is_shutdown():
            try:
                self._current_state = rospy.wait_for_message("mavros/state", State, timeout=5.0)
                rospy.logdebug("Current mavros/state READY=>")
            except:
                rospy.logerr("Current mavros/state not ready yet, retrying for getting current_state")
        return self._current_state


    def _check_all_publishers_ready(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rospy.logdebug("CHECK ALL PUBLISHERS CONNECTION:")
        """ if (_control_mode == ControlMode.POSITION):
            self._check_local_pose_pub_connection()
        elif (_control_mode == ControlMode.VELOCITY):
            self._check_local_vel_pub_connection() """
        self._check_local_vel_pub_connection()
        rospy.logdebug("All Publishers CONNECTED and READY!")
        
    def _check_local_vel_pub_connection(self):

        while self._local_vel_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("Waiting for susbribers to _local_vel_pub...")
            try:
                self._rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_local_vel_pub Publisher Connected")
    
    def _check_local_pose_pub_connection(self):

        while self._local_pose_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("Waiting for susbribers to _local_pose_pub...")
            try:
                self._rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_local_pose_pub Publisher Connected")



    # Methods that the TrainingEnvironment will need to define here as virtual
    # because they will be used in RobotGazeboEnv GrandParentClass and defined in the
    # TrainingEnvironment.
    # ----------------------------
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()
     
    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()
        
    # Methods that the TrainingEnvironment will need.
    # ----------------------------

    #def _get_proxy(self, service, type):
    #    return rospy.ServiceProxy(mavros.get_topic('cmd', service), type)

    def ExecuteTakeoff(self, alt = 4, lat = 0, long = 0, min_p = 0, yw = 0):
        self.gazebo.unpauseSim()
        # armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        # armService(True)
        rospy.wait_for_service('/mavros/cmd/takeoff')
        # takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        # ret=takeoffService(altitude=alt, latitude=lat, longitude=long, min_pitch=min_p, yaw=yw)

        try:
            takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
            ret=takeoffService(altitude=alt, latitude=lat, longitude=long, min_pitch=min_p, yaw=yw)
            while not ret.success:
                print("stuck here!!!")
                #rospy.wait_for_service('/mavros/cmd/arming')
                try:
                    armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
                    armService(False)
                    rospy.sleep(0.005)
                    armService(True)
                    #ret=takeoffService(altitude=alt, latitude=lat, longitude=long, min_pitch=min_p, yaw=yw)
                except rospy.ServiceException, e:
                    print "Service takeoff call failed: %s"%e
                takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
                ret=takeoffService(altitude=alt, latitude=lat, longitude=long, min_pitch=min_p, yaw=yw)
                   
           #self.wait_for_height( altitude_confirm = altitude,smaller_than = False,epsilon = 0.05,update_rate=20)

        except rospy.ServiceException as ex:
           fault(ex)
        if not ret.success:
           fault("Request failed. Check mavros logs. ACK:", ret.result)
    
    def wait_for_height(self, altitude_confirm, smaller_than, epsilon = 0.05, update_rate=20):
        """
        Checks if current height is smaller or bigger than a value
        :param: smaller_than: If True, we will wait until value is smaller than the one given
        """
        rate = rospy.Rate(update_rate)
        start_wait_time = rospy.get_rostime().to_sec()
        end_wait_time = 0.0

        rospy.logdebug("epsilon>>" + str(epsilon))

        while not rospy.is_shutdown():
            curr_pose = self._check_current_pose_ready()

            current_height = curr_pose.pose.position.z

            if smaller_than:
                takeoff_height_achieved = current_height <= altitude_confirm - epsilon
                rospy.logwarn("SMALLER THAN HEIGHT...current_height=" +
                              str(current_height)+"<="+str(altitude_confirm))
            else:
                takeoff_height_achieved = current_height >= altitude_confirm + epsilon
                rospy.logwarn("BIGGER THAN HEIGHT...current_height=" +
                              str(current_height)+">="+str(altitude_confirm))

            if takeoff_height_achieved:
                rospy.logwarn("Reached Height!")
                end_wait_time = rospy.get_rostime().to_sec()
                break
            rospy.logwarn("Height Not there yet, keep waiting...")
            rate.sleep()

    def ExecuteAction(self, vel_msg, epsilon=0.05, update_rate=20):
        """
        It will move the base based on the linear and angular speeds given.
        It will wait untill those twists are achieved reading from the odometry topic.
        :param vel_msg: velocity message with angular velocity about x and y axis kept at zero
        :param epsilon: Acceptable difference between the speed asked and the odometry readings
        :param update_rate: Rate at which we check the odometry.
        :return:
        """
        rospy.logdebug("MavROS Base Twist Cmd>>" + str(vel_msg))
        self._check_local_vel_pub_connection()
        self._local_vel_pub.publish(vel_msg)
        """
        self.wait_until_twist_achieved(cmd_vel_value,
                                        epsilon,
                                        update_rate)
        
        self.wait_time_for_execute_movement()
        """
    
    """ def setMavMode(self, mode):
        if (self._current_state.mode != mode):
            offb_set_mode = SetMode()
            offb_set_mode._request_class().custom_mode = mode
            if (self._set_mode_client(custom_mode=mode) and offb_set_mode._response_class().mode_sent):
                rospy.loginfo("Mode enabled.")
                return True
            else:
                rospy.loginfo("Mode could not be enabled. Cannot execute actions.")
                return False """

    def setMavMode(self, mode, timeout):
        """mode: PX4 mode string, timeout(int): seconds"""
        rospy.loginfo("setting FCU mode: {0}".format(mode))
        old_mode = self._current_state.mode
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        mode_set = False
        for i in xrange(timeout * loop_freq):
            if self._current_state.mode == mode:
                mode_set = True
                rospy.loginfo("set mode success | seconds: {0} of {1}".format(i / loop_freq, timeout))
                break
            else:
                try:
                    res = self._set_mode_client(0, mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            rate.sleep()

        """ def setArmRequest(self, arm):
        if (self._current_state.armed != arm):
            arm_cmd = CommandBool()
            arm_cmd._request_class().value = arm
            if (self._arming_client(arm_cmd._request_class().value) and arm_cmd._response_class().success):
                while (not self._current_state.armed): # Wait for arming to be complete
                    self._rate.sleep()
                return True
            else:
                rospy.logwarn("Vehicle arm/disarm request failed. Cannot execute actions.")
                return False """

    def setArmRequest(self, arm, timeout):
        #ArmRequest routine 1
        """ 
        while (self._current_state.armed != arm):
            try:
                armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
                armService(arm)
            except:
                rospy.logwarn("Vehicle arm/disarm request failed. Cannot execute actions.")
            rospy.sleep(0.04)
        
        #ArmRequest routine 2
        #arm: True to arm or False to disarm, timeout(int): seconds
        rospy.loginfo("setting FCU arm: {0}".format(arm))
        old_arm = self._current_state.armed
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        arm_set = False
        for i in xrange(timeout * loop_freq):
            if self._current_state.armed == arm:
                arm_set = True
                rospy.loginfo("set arm success | seconds: {0} of {1}".format(i / loop_freq, timeout))
                break
            else:
                try:
                    res = self._arming_client(arm)
                    print("res is currently", res)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e) 

            rate.sleep()
        """
        #ArmRequest routine 3
        if (self._current_state.armed != arm):
            arm_cmd = CommandBool()
            arm_cmd._request_class().value = arm
            if (self._arming_client(arm_cmd._request_class().value) and arm_cmd._response_class().success):
                while (not self._current_state.armed): # Wait for arming to be complete
                    self._rate.sleep()
                return True
            else:
                rospy.logwarn("Vehicle arm/disarm request failed. Cannot execute actions.")
                return False

    def get_current_pose(self):
        return self._current_pose

    def get_current_state(self):
        #print("The current state is :", self._current_state)
        return self._current_state


    """ def _check_all_services_ready(self):
        
        #Checks that all the services are working
        #:return:
        
        rospy.logdebug("CHECK ALL SERVICES CONNECTION:")
        self._check_arming_done()
        self._check_set_mode_done()
        rospy.logdebug("All Services CONNECTED and READY!")

    def _check_arming_done(self):
        self._arming_client = None
        while self._arming_client == None and not rospy.is_shutdown():
            rospy.wait_for_service('/mavros/cmd/arming')
            try:
                self._arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
                self._arming_client(True)
                rospy.logdebug("mavros/cmd/arming DONE=>")
            except rospy.ServiceException, e:
                rospy.logerr("mavros/cmd/arming not done yet, retrying for arming_client...")

    def _check_set_mode_done(self):
        self._set_mode_client = None
        while self._set_mode_client == None and not rospy.is_shutdown():
            try:
                self._set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                #http://wiki.ros.org/mavros/CustomModes for custom modes
                isModeChanged = self._set_mode_client(custom_mode='OFFBOARD') #return true or false
            except rospy.ServiceException, e:
                rospy.logerr("mavros/set_mode not done yet, retrying for _set_mode_client...") """