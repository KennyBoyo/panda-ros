# Franka Controllers

**For more information of the franka software setup, please refer to `Setup` > `SETUP.md`**

**For more information of the prerequisites of controller setup, please refer to `Setup` > `CONTROLLER.md`**

**For more information of the robot parameter reading, please refer to `Setup` > `TRANSMAT.md`**

**For more information of the two panda setup, please refer to `Setup` > `DUALCONTROLLER.md`**

## To compile the file after modified the code:
```
cd catkin_ws 
source /opt/ros/melodic/setup.sh 
rosdep install --from-paths src --ignore-src --rosdistro melodic -y --skip-keys libfranka 
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/opt/ros/melodic/lib/libfranka/build 
source devel/setup.sh 
```

## To source directories:
Open the bashrc file:
```
gedit .bashrc
```
Source the commands: 
```
source /opt/ros/melodic/setup.sh
source ~/catkin_ws/devel/setup.bash
source ~/ws_moveit/devel/setup.bash
source ~/catkin_ws/devel/setup.bash
```

## joint impedance controller
```
roslaunch franka_panda_controller_swc joint_impedance_controller.launch robot_ip:=172.16.0.2 load_gripper:=true
```
## joint position controller
```
roslaunch franka_panda_controller_swc joint_position_controller.launch robot_ip:=172.16.0.2 load_gripper:=true
```
## cartesian impedance controller
```
roslaunch franka_panda_controller_swc cartesian_impedance_controller.launch robot_ip:=172.16.0.2 load_gripper:=true
```
## dual cartisan impedance controller 
```
roslaunch franka_panda_controller_swc dual_arm_cartesian_impedance_controller.launch
```
---
#### Plot velocities and distance of dual cartisan impedance controller porject
```
rqt_plot /panda_dual/dual_arm_cartesian_impedance_controller/velocity/distance
rqt_plot /panda_dual/dual_arm_cartesian_impedance_controller/velocity/left_vel
rqt_plot /panda_dual/dual_arm_cartesian_impedance_controller/velocity/right_vel
```
---
#### To record the data
```
rosbag record -O subset /panda_dual/dual_arm_cartesian_impedance_controller/velocity
```


## Vidoe
#### two moving bodies collisions
![gif3](https://user-images.githubusercontent.com/79889031/134857667-35bdc2cd-7474-45cf-90a9-bdc0b7ecb067.gif)

#### Semi-moving bodies collisions
![gif2](https://user-images.githubusercontent.com/79889031/134857638-457e5273-5fb8-4a32-b67c-e3f175efa242.gif)

#### One static and one moving bodies collision
![gif4](https://user-images.githubusercontent.com/79889031/134857694-a2a8a860-8b62-45c8-9ab6-1e84b38841dc.gif)

#### Test  
![gif1](https://user-images.githubusercontent.com/79889031/134857330-82e714e4-4e8f-47c6-b29d-77b71d0762ef.gif)


## Origin
https://github.com/iyawx/franka_panda_controller_swc

Shih-Wen Chen  
Email: iyawx@livemail.tw 
