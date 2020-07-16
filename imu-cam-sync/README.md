# Prerequisite Setup #

## apt packages ##
sudo apt-get install libi2c-dev i2c-tools  
sudo apt-get install v4l-utils  
sudo apt-get install ros-melodic-imu-tools  

## GPIO ##
Follow https://github.com/pjueon/JetsonGPIO  

## Install OpenCV 3 ##
Follow https://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html  

After you clone the opencv git repository, switch to an opencv 3 branch with:  
git checkout -b 3.4.10  

Use the following cmake command:  
cmake -D CMAKE_BUILD_TYPE=Release -D WITH_V4L=ON -D CMAKE_INSTALL_PREFIX=/usr/local ..  

# ROS setup #
In ~/catkin_ws/src, place/clone:
- https://github.com/ros-perception/vision_opencv, git checkout -b melodic
- MPU6050 IMU ROS package
- cv_camera package
- imu-camera-sync package

catkin build

# Run imu-camera sensor #
cd ~/catkin_ws
source devel/setup.bash

roslaunch imu-camera-sync sync.launch

