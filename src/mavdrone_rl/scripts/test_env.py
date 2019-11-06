#!/usr/bin/env python

import gym
import numpy
import time
#import qlearn
from gym import wrappers
# ROS packages required
import rospy
import rospkg
import roslaunch
from openai_ros.start_gym_env import Start_OpenAI_ROS_Environment
from openai_ros.roslauncher import ROSLauncher


if __name__ == '__main__':

    
    rospy.init_node('mavdrone_test_env', anonymous=True, log_level=rospy.WARN)
    rospy.logwarn("Initial state reached!!!")
    # Init OpenAI_ROS ENV
    task_and_robot_environment_name = rospy.get_param('mav_drone/task_and_robot_environment_name')
    
    env = Start_OpenAI_ROS_Environment(task_and_robot_environment_name)
    rospy.loginfo("Gym environment done")
    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('mavdrone_rl')
    outdir = pkg_path + '/training_results'
    env = wrappers.Monitor(env, outdir, force=True)
    rospy.loginfo("Monitor Wrapper started")

    last_time_steps = numpy.ndarray(0)
    nepisodes = rospy.get_param("/mav_drone/nepisodes")
    nsteps = rospy.get_param("/mav_drone/nsteps")
    start_time = time.time()
    highest_reward = 0

    # Starts the main training loop: the one about the episodes to do
    for x in range(nepisodes):
        rospy.logdebug("############### START EPISODE=>" + str(x))
        observation = env.reset()
        while True:
            pass
        state = ''.join(map(str, observation))
        rospy.logwarn(state)
        cumulated_reward = 0
        done = False
        for i in range(nsteps):
            rospy.logdebug("Episode: ", x,"step: ",i)
            time.sleep(0.05)
    env_to_wrap.close()