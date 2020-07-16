#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/TimeReference.h"
#include "MPU6050.h"
#include <JetsonGPIO.h>

const int IMU_FREQ = 200;//hertz
const int CAM_FPS = 20;
const int TRIGGER_PIN = 31;

int main(int argc, char** argv)
{
	//setup gpio
	GPIO::setmode(GPIO::BOARD);
	GPIO::setup(TRIGGER_PIN, GPIO::OUT, GPIO::LOW);
	
	//setup imu
	MPU6050 imu(MPUIMU::AFS_2G, MPUIMU::GFS_250DPS);
	switch (imu.begin(0))
	{
		case MPUIMU::ERROR_IMU_ID:
			ROS_ERROR("Bad device id");
			return 1;
		case MPUIMU::ERROR_SELFTEST:
			ROS_ERROR("Failed self-test");
			return 1;
		default:
			ROS_INFO("MPU6050 onlne!");
			break;
	}

	//ROS node setup
	ros::init(argc, argv, "imu");
	ros::NodeHandle node;
	ros::Publisher imu_pub = node.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
	ros::Publisher trigger_pub = node.advertise<sensor_msgs::TimeReference>("imu/trigger_time", 1000);
	ros::Rate rate(IMU_FREQ);

	//camera trigger math
	int counter_reset = IMU_FREQ / CAM_FPS;
	int freq_counter = 0;
	int trigger_count = 0;

	//ROS_INFO("TEST_TRIGGER");
	//GPIO::output(TRIGGER_PIN, GPIO::HIGH);
	//GPIO::output(TRIGGER_PIN, GPIO::LOW);


	//wait for camera to start
	//TEMPORARY SOLUTION
	ros::Duration(3).sleep();

	while (ros::ok())
	{
		ros::Time current_time = ros::Time::now();

		sensor_msgs::Imu msg;
		msg.header.stamp = current_time;
		msg.header.frame_id = '0';

		static float ax, ay, az;
		static float gx, gy, gz;
		if (imu.checkNewData())
		{
			imu.readAccelerometer(ax, ay, az);
			imu.readGyrometer(gx, gy, gz);
		}

		msg.angular_velocity.x = gx;
		msg.angular_velocity.y = gy;
		msg.angular_velocity.z = gz;

		msg.linear_acceleration.x = ax;
		msg.linear_acceleration.y = ay;
		msg.linear_acceleration.z = az;

		//camera trigger
		if (freq_counter == counter_reset)// || first_publish))
		{
			
			sensor_msgs::TimeReference trigger_msg;
			ros::Time time_ref(0,0);
			trigger_msg.header.frame_id = '0';
			trigger_msg.header.stamp = current_time;
			trigger_msg.time_ref = time_ref;
			trigger_pub.publish(trigger_msg);

	       		ROS_INFO("CAMERA TRIGGER %10u", trigger_count);
			GPIO::output(TRIGGER_PIN, GPIO::HIGH);
			GPIO::output(TRIGGER_PIN, GPIO::LOW);	

			freq_counter = 0;
			trigger_count++;
		}

		imu_pub.publish(msg);

		++freq_counter;
		
		ros::spinOnce();
		rate.sleep();

	}

	GPIO::cleanup();

	return 0;


}
