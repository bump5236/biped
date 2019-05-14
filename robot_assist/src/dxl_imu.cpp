#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <math.h>
#include <tf/transform_broadcaster.h>
#include "dynamixel_sdk/dynamixel_sdk.h"

#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64               // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132

#define ADDR_MODE                       11
#define ADDR_GOAL_TORQUE                102

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          4                   // Dynamixel ID: 1
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      10             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      2000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     50                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

// 変数定義
float accl_x, accl_y, accl_z, gyro_x, gyro_y, gyro_z;
float quat_x, quat_y, quat_z, quat_w;
double roll, pitch, yaw;
int counter = 0, callback_counter = 0;
clock_t st_clock, ed_clock;

// IMUデータ取得
void callback(const sensor_msgs::Imu& data) {

  // Linear acceleration
  accl_x = data.linear_acceleration.x;
  accl_y = data.linear_acceleration.y;
  accl_z = data.linear_acceleration.z;

  // Angular velocity
  gyro_x = data.angular_velocity.x;
  gyro_y = data.angular_velocity.y;
  gyro_z = data.angular_velocity.z;

  // Orientation (not provided)
  quat_x = data.orientation.x;
  quat_y = data.orientation.y;
  quat_z = data.orientation.z;
  quat_w = data.orientation.w;

  tf::Quaternion quat(quat_x, quat_y, quat_z, quat_w);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  callback_counter += 1;
  printf("%d\n", callback_counter);
  // printf(
  //   "accl_x:%f\naccl_y:%f\naccl_z:%f\ngyro_x:%f\ngyro_y:%f\ngyro_z:%f\nroll:%f\npitch:%f\nyaw:%f\n",
  //   accl_x,
  //   accl_y,
  //   accl_z,
  //   gyro_x,
  //   gyro_y,
  //   gyro_z,
  //   roll,
  //   pitch,
  //   yaw
  // );
}

int getch() {
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("imu/data", 1, callback);     //subscribe[1]はバッファサイズ
  ros::Rate r(100);        // 100Hz

  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  uint8_t dxl_error = 0;                          // Dynamixel error
  int32_t dxl_present_position = 0;               // Present position
  // Open port
  if (portHandler->openPort()) {
    printf("Succeeded to open the port!\n");
  }
  else {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE)) {
    printf("Succeeded to change the baudrate!\n");
  }
  else {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }
  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_error));
  }
  else {
    printf("Dynamixel has been successfully connected \n");
  }
  // // MODE
  // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MODE,3, &dxl_error);
  // if (dxl_comm_result != COMM_SUCCESS) {
  //   printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  // }
  // else if (dxl_error != 0) {
  //   printf("%s\n", packetHandler->getTxRxResult(dxl_error));
  // }
  // else {
  //   printf("Dynamixel has been successfully connected \n");
  // }

  while(false == ros::isShuttingDown()) {
    // printf("Press any key to continue! (or press ESC to quit!)\n");
    // if (getch() == ESC_ASCII_VALUE)
    //   break;
    counter += 1;
    printf("%d\n", counter);
    ros::spinOnce();
    st_clock = clock();
    int dxl_goal_position = roll*4096/(2*M_PI);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_position, &dxl_error);
    ed_clock = clock();
    printf("write_time:%f\n",(double)(ed_clock - st_clock)/CLOCKS_PER_SEC);
    // sleep(1);
    if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_error));
    }
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
    printf("[ID:%03d] Roll:%03f, GoalPos:%03d  PresPos:%03d\n", DXL_ID, roll, dxl_goal_position, dxl_present_position);

    // do {
    //   // Read present position
    //   dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
    //   if (dxl_comm_result != COMM_SUCCESS) {
    //     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    //   }
    //   else if (dxl_error != 0) {
    //     printf("%s\n", packetHandler->getTxRxResult(dxl_error));
    //   }
    //
    //   printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID, dxl_goal_position, dxl_present_position);
    //
    // }while((abs(dxl_goal_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

    r.sleep();
  }

  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_error));
  }

  // Close port
  portHandler->closePort();
  printf("Succeeded Closeport!\n");

  return 0;
}
