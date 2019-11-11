<h3><span style="color:red">This repository is under construction!</span></h3>

The required workspace structure (package level) is given below:

```
MavDrone_RL_WS
└── src
  └── mavdrone_rl
  └── openai_ros
  └── Firmware
  └── mavros
  └── mavros_moveit
```
After cloning the repository, follow the following steps to setup the workspace before building it:

```
cd MavDrone_RL_WS/src
vcs-import < ws_repo_dep.yaml
cd Firmware/Tools/sitl_gazebo
git pull origin master
git remote set_url origin https://github.com/saifullah3396/sitl_gazebo.git
git pull origin master
```
Build the Firmware for iris drone and gazebo.
Now build the workspace and add the following lines in ```~/.bashrc```

```(subl)
export MAV_GYM_WS= [Path to MavDrone_RL_WS dir]
source $MAV_GYM_WS/devel/setup.bash
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$MAV_GYM_WS/src/Firmware/build/px4_sitl_default/build_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$MAV_GYM_WS/src/Firmware/Tools/sitl_gazebo/models
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$MAV_GYM_WS/src/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$MAV_GYM_WS/src/Firmware/Tools/sitl_gazebo
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:MAV_GYM_WS/src/Firmware/build/px4_sitl_default/build_gazebo
```


Demo version:
A test script is available at ```scripts/test_env.py```
The Launch file in ```Launch``` folder is used to launch ```test_env.py``` with parameters found in ```config/mavdrone_test.yaml```

