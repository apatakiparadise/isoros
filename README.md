# Franka Controllers

**Based on controllers orignially created by Shih-Wen Chen. Refer to: github.com/iyawx/franka_panda_controller_swc**



## To compile the file after code has been modified:
```
cd catkin_ws_josh 
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
source ~/catkin_ws_josh/devel/setup.bash
source ~/ws_moveit/devel/setup.bash
source ~/catkin_ws_josh/devel/setup.bash
```


## cartesian impedance controller
```
roslaunch franka_panda_controller_swc cartesian_impedance_controller_NR.launch robot_ip:=172.16.0.2 load_gripper:=true
```
