<h3><span style="color:red">This repository is under construction!</span></h3>

The required repo structure (package level) is given below:

```
MavDrone_RL_WS
├── src
  └── mavdrone_rl
  └── openai_ros
  └── Firmware
  └── mavros
  └── mavros_moveit
```
After cloning the repository, follow the following steps to setup the workspace before building it:

```
cd MavDrone_RL_WS/src
git clone https://github.com/irl-at-ncai/Firmware.git && cd Firmware
cd Tools/sitl_gazebo
git pull origin master
git remote set_url origin https://github.com/saifullah3396/sitl_gazebo.git
git pull origin master
```
Build the Firmware for iris drone and gazebo. Afterwards clone mavros and mavros_moveit:
```
cd ../../../
git clone https://github.com/mavlink/mavros.git
git clone https://github.com/hassaanhashmi/mavros_moveit.git
cd ..
```
Now build the workspace with ```catkin_make```

