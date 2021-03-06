#include "ros/ros.h"
#include <math.h>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "std_msgs/Int16.h"

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
#define ADDR_TORQUE_ENABLE              64               // Control table address is different in Dynamixel model
#define ADDR_GOAL_POSITION              116
#define ADDR_PRESENT_POSITION           132

#define ADDR_OPERATING_MODE             11
#define ADDR_GOAL_CURRENT               102
#define ADDR_CURRENT_LIMIT              38
#define ADDR_PRESENT_CURRENT            126

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
#define DXL_OPERATING_MODE              0
#define DXL_CURRENT_LIMIT               100
#define ESC_ASCII_VALUE                 0x1b

// 変数定義
float accl_x, accl_y, accl_z, gyro_x, gyro_y, gyro_z;
float quat_x, quat_y, quat_z, quat_w;
double roll, pitch, yaw;
int counter = 0;
clock_t st_clock, ed_clock;

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

int16_t dxl_goal_current = 0;
ros::Subscriber sub;

void callback(const std_msgs::Int16& msg) {
  dxl_goal_current = msg.data;
  printf("call\n");
}

int16_t dxl_old_current = 0;
uint16_t dxl_present_current;               // Present position
int dxl_comm_result = COMM_TX_FAIL;             // Communication result
uint8_t dxl_error = 0;
uint8_t dxl_mode;                          // Dynamixel error
uint16_t dxl_Max_current;
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);


void timer_callback(const ros::TimerEvent&){
  printf("time_call\n");
  if (dxl_old_current != dxl_goal_current) {
    printf("Torque Change\n");
    st_clock = clock();
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_CURRENT, dxl_goal_current, &dxl_error);
    ed_clock = clock();

    printf("write_time:%f\n",(double)(ed_clock - st_clock)/CLOCKS_PER_SEC);
    if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_error));
    }
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_CURRENT, &dxl_present_current, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_error));
    }

    printf("[ID:%03d] GoalTorq:%03d  PresTorq:%03d\n", DXL_ID, dxl_goal_current, dxl_present_current);
    dxl_old_current = dxl_goal_current;
  }
}

int main(int argc, char **argv) {
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

  // MODE
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, DXL_OPERATING_MODE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_error));
  }
  else {
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, &dxl_mode, &dxl_error);
    printf("Operatind Mode is %d\n", dxl_mode);
  }

  // Setting DXL_CURRENT_LIMIT
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_CURRENT_LIMIT, DXL_CURRENT_LIMIT, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_error));
  }
  else {
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_CURRENT_LIMIT, &dxl_Max_current, &dxl_error);
    printf("Limit of current is %d\n", dxl_Max_current);
  }

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_error));
  }
  else {
    printf("Dynamixel has been successfully connected \n");
  }

  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_CURRENT, 0, &dxl_error);

  ros::init(argc, argv, "torq_listener");
  ros::NodeHandle n;
  sub = n.subscribe("dxl_sub", 1, callback);     //subscribe[1]はバッファサイズ
  // ros::Rate r(100);        // 100Hz
  ros::Timer timer = n.createTimer(ros::Duration(0.1), timer_callback);

  counter += 1;
  printf("%d\n", counter);
  ros::spin();

  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
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
