# lite6_labauto
This package works as bridge of other packages and Ufactory official packages.

## Install & Build
- install ROS
- setup workspace
  ```
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws
  wstool init src
  catkin build
  ```
- build pkgs
  ```
  git clone https://github.com/asanolab/lite6_labauto.git
  git clone https://github.com/asanolab/pipetty_interface.git
  wstool merge -t . lite6_labauto/install/lite6_labauto.noetic.rosinstall
  wstool update
  rosdep install -y -r --from-paths . --ignore-src
  cd lite6_labauto
  catkin bt

  echo "source ~/ipc_ws/devel/setup.bash" >> ~/.bashrc
  source ~/ipc_ws/devel/setup.bash
  ```
- install python modules
  ```
  python3 -m pip install pyserial
  ```

- set udev rules for devices
  ```
  sudo cp config/80-lite6-labauto.rules /etc/udev/rules.d/.
  sudo udevadm control --reload
  ```

## Initialization
### Prepare Lite6, gripper and camera
```
roslaunch lite6_labauto lite6_labaute.launch  # launch robot and camera at the same time
# robot_ip:=192.168.0.166 show_rviz:=true custom_gripper:=true camera:=true
```

### Initialize all Serial Ports (pH sensor, pipetty, pipette tip disposal motor)
```
roslaunch lite6_labauto all_serial_nodes.launch
```

### Initialize or shut down Lite 6
```
rosrun lite6_labauto robot_init.py
rosrun lite6_labauto robot_disable.py
```


## ROS msg and srv
### msg
**LabwareOBB.msg**
```
std_msgs/Header header
geometry_msgs/Pose pose
float32 x_length
float32 y_width
float32 z_height
```

**MovePose.msg**
```
# For pose in move_line function
# [x,y,z,r,p,y] in unit [mm,mm,mm,rad,rad,rad]

float64[6] pose
```

### srv
**BeakerMani.srv**
```
# input 
bool shake
---
# output
bool success
string message
```

**pHMeasure.srv**
```
# input

float32 timeout_s
---
# output
bool success
float32 ph
string message
```

**PipetteDo.srv**
```
# input 
string liquid # "HCl" or "water"
float32 volume_ul # Volume to aspirate
int32 tip_id
---
# output
bool success
string message
```

**TweezersDraw.srv**
```
# input 
bool draw
---
# output
bool success
string message
```

## Gripper
- Gripper was made using an open-source project for UFactory Lite6, [OpenParallelGripper](https://github.com/hygradme/OpenParallelGripper).
  - [XL330_version](https://github.com/hygradme/OpenParallelGripper/blob/main/XL330_version/README.md) is used.
  - This gripper can be controlled by position.
  - Firmware of Arduino is placed in sketchbook folder
  - Gripper tip parts are available here. [Google Drive](https://drive.google.com/open?id=1WP06H0lVdvfj6VCkU2ATvlK_sCpn3dsM&usp=drive_fs)
  - Modified the gripper jaws to adapt to different objects, as shown in the figure, ![Modified Gripper](assets/gripper.png "gripper").
