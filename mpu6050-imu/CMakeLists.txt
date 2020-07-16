cmake_minimum_required(VERSION 3.0.2)
project(mpu6050-imu)

find_package(catkin REQUIRED COMPONENTS
  mavros_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  src/MPU6050_I2C
)

## Declare a C++ library
# add_library(MPU src/MPU.cpp)
# add_library(MPU6xx0 src/MPU6xx0.cpp)
# add_library(MPU6050 src/MPU6050.cpp)
# add_library(I2CDevI2c src/I2CDevI2C.cpp)

 add_library(
	 MPU6050_I2C 
	 src/MPU6050_I2C/I2CDevI2C.cpp 
	 src/MPU6050_I2C/MPU.cpp 
	 src/MPU6050_I2C/MPU6xx0.cpp 
	 src/MPU6050_I2C/MPU6050.cpp
	 src/MPU6050_I2C/CrossPlatformI2C.h
	 src/MPU6050_I2C/CrossPlatformI2C_Core.h
	 src/MPU6050_I2C/CrossPlatformSPI.h
	 src/MPU6050_I2C/MPU.h
	 src/MPU6050_I2C/MPU6xx0.h
	 src/MPU6050_I2C/MPU6050.h
	 src/MPU6050_I2C/timing.cpp
	 src/MPU6050_I2C/timing.h
) 

# add_executable(${PROJECT_NAME}_node src/mpu6050_imu_node.cpp)
add_executable(mpu6050_imu_node src/mpu6050_imu_node.cpp)


target_link_libraries(mpu6050_imu_node ${catkin_LIBRARIES} MPU6050_I2C i2c JetsonGPIO)


install(TARGETS mpu6050_imu_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

