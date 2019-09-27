# extrinsic_calibration
D415 Extrinsic Calibration with ROS

# Installation

1. From catkin_ws/src directory, clone the repo
```bash
git clone https://github.com/hjsuh94/extrinsic_calibration.git
```
2. Make 
```
../
catkin_make
```

# How to Use 

1. Run D415 node in ROS by 
```bash
roslaunch realsense2_camera rs_camera.launch
```
2. Make sure the camera topics are streaming by 
```bash
rostopic echo /camera/color/camera_info
```
(this can also be checked in rviz) 

3. Run the program  
```bash
rosrun extrinsic_calibration calibration_pub.py
```

