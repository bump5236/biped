#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <math.h>


void callback(const sensor_msgs::Imu& data){
  printf("accl_x=%f\n", data.linear_acceleration.x);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("imu/data", 10, callback);
  ros::Rate loop_rate(1000);

  ros::spin();
  return 0;
}
