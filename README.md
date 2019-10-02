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
rosrun extrinsic_calibration calibration.py 50
```
where 50 is the number of frames to average from. 

4. Publish transforms based on calibrated extrinsics 
```bash
rosrun extrinsic_calibration publish_calibration.py
```

to publish the transform between the camera frames and the board frame, and visualize the board frame in rviz. 

# Results

Results are printed in command line, but also saved the directory where the program is run. TODO: fix this! 
Transformation given is FROM the camera optical frame TO the board frame. (i.e. what is the pose of my board frame seen from the camera frame?) 

calibration_image.jpg  : Useful to see what the board frame actually is.  
calibration_log.csv    : Log of 100 detected transforms. 
calibration_result.csv : first 3 rows are rotation matrix, 
