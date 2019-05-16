#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include <math.h>

//global領域で宣言
ros::Subscriber sub;

void callback(const sensor_msgs::Imu& data){
  printf("accl_x=%f\n", data.linear_acceleration.x);
}

//メインループを使わず一定間隔の処理
void timer_callback(const ros::TimerEvent&){
  std_msgs::String msg;
  msg.data = "hello world!";
  ROS_INFO("%s", msg.data.c_str());
}

int main(int argc, char **argv){
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  sub = n.subscribe("imu/data", 10, callback);
  ros::Timer timer = n.createTimer(ros::Duration(0.1), timer_callback);
  // ros::Rate loop_rate(1000);
  ros::spin();
  return 0;
}
