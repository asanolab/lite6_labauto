# lite6_labauto
This package provides a customized version of UFactory Lite6 for laboratory automation.  
![Image](https://github.com/user-attachments/assets/d9c33df8-a048-40a5-a189-fb4538dc64d3)

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
### Prepare robot
```
roslaunch lite6_labauto lite6_labauto.launch  # launch robot, gripper and camera at the same time
# robot_ip:=192.168.0.166 show_rviz:=true custom_gripper:=true camera:=true
```

### Initialize or shut down Lite 6
```
rosrun lite6_labauto robot_init.py
rosrun lite6_labauto robot_disable.py
```

## Gripper
- Gripper was made using an open-source project for UFactory Lite6, [OpenParallelGripper](https://github.com/hygradme/OpenParallelGripper).
  - [XL330_version](https://github.com/hygradme/OpenParallelGripper/blob/main/XL330_version/README.md) is used.
  - This gripper can be controlled by position.
  - Firmware of Arduino is placed in sketchbook folder
  - Gripper tip parts are available here. [Google Drive](https://drive.google.com/open?id=1WP06H0lVdvfj6VCkU2ATvlK_sCpn3dsM&usp=drive_fs)
  - Modified the gripper jaws to adapt to different objects, as shown in the figure.  
![Image](https://github.com/user-attachments/assets/caa11f8e-b77a-432a-8f00-71bb4d26cede)


## ROS msg
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