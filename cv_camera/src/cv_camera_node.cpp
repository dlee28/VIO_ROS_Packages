// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "cv_camera/driver.h"
#include <JetsonGPIO.h>

const int TRIGGER_PIN = 31;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cv_camera");
  ros::NodeHandle private_node("~");
  cv_camera::Driver driver(private_node, private_node);

  GPIO::setmode(GPIO::BOARD);
  GPIO::setup(TRIGGER_PIN, GPIO::OUT, GPIO::LOW);

  try
  {
    driver.setup();

    bool init = false;
    GPIO::output(TRIGGER_PIN, GPIO::HIGH);
    GPIO::output(TRIGGER_PIN, GPIO::LOW);
    while (!init)
    {
      init = driver.proceedInit();
    }

    while (ros::ok())
    {
      driver.proceed();
      ros::spinOnce();
    }
  }
  catch (cv_camera::DeviceError &e)
  {
    ROS_ERROR_STREAM("cv camera open failed: " << e.what());
    return 1;
  }

  return 0;
}
