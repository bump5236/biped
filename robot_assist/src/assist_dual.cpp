#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <math.h>
#include <tf/transform_broadcaster.h>
#include "dynamixel_sdk/dynamixel_sdk.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

// Control table address
#define ADDR_TORQUE_ENABLE              64
#define ADDR_GOAL_POSITION              116
#define ADDR_PRESENT_POSITION           132

#define ADDR_OPERATING_MODE             11
#define ADDR_GOAL_CURRENT               102
#define ADDR_CURRENT_LIMIT              38
// Protocol version
#define PROTOCOL_VERSION                2.0

// Default setting
#define DXL_ID                          4
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"
#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0
#define DXL_MOVING_STATUS_THRESHOLD     50
#define DXL_OPERATING_MODE              5
#define DXL_CURRENT_LIMIT               100
#define ESC_ASCII_VALUE                 0x1b

/*--- 構造体宣言 ----*/
struct data_parameter{
  double time;
  double rec_mode;
  double udp_time;
  double fz;
  double ffz;

  double accl_x;
  double accl_y;
  double accl_z;
  double gyro_x;
  double gyro_y;
  double gyro_z;
  double quat_x;
  double quat_y;
  double quat_z;
  double quat_w;

  double roll;
  double pitch;
  double yaw;

  int32_t tgt_ang1;
  int32_t ang1;
  uint16_t tgt_torq1;
  uint16_t torq1;

  int32_t tgt_ang2;
  int32_t ang2;
  uint16_t tgt_torq2;
  uint16_t torq2;

  int act1;
  int act2;
};

/*--- 関数宣言 ---*/
void get_memory(void);
void free_memory(void);
double get_time_sec(int mode);
// double ang_vc(double t, int run_mode, int motor_number);  //motorの処理
int get_date(int mode);
char kbhit(void);
void callback(const sensor_msgs::Imu& data);

struct data_parameter *contrl_data;

/*--- 変数定義 ---*/
int counter = 0, callback_counter = 0;

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("imu/data", 1, callback);     //subscribe[1]はバッファサイズ
  ros::Rate r(100);        // 100Hz

  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error, dxl_mode;
  int32_t dxl_present_position = 0;
  uint16_t dxl_Max_current;

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

  while(false == ros::isShuttingDown()) {
    // printf("Press any key to continue! (or press ESC to quit!)\n");
    // if (getch() == ESC_ASCII_VALUE)
    //   break;
    counter += 1;
    printf("%d\n", counter);
    ros::spinOnce();
    st_clock = clock();
    int dxl_goal_position = roll*4096/(2*M_PI);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position, &dxl_error);
    ed_clock = clock();
    printf("write_time:%f\n",(double)(ed_clock - st_clock)/CLOCKS_PER_SEC);
    // sleep(1);
    if (dxl_comm_result != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_error));
    }
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
    printf("[ID:%03d] Roll:%03f, GoalPos:%03d  PresPos:%03d\n", DXL_ID, roll, dxl_goal_position, dxl_present_position);

    // do {
    //   // Read present position
    //   dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
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


// -------------------------------------------------------------------------------------
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

void get_memory(void){
  if ((contrl_data = (struct data_parameter*)calloc(LOOP_COUNT+100, sizeof(struct data_parameter)))  == NULL) {
    perror("calloc control_data");
    exit(1);
  }
}

void free_memory(void){
  free(contrl_data);
}

double get_time_sec(int mode){
  static unsigned long long tsc0=0;
  unsigned long long tsc1;
  #define rdtsc(x) __asm__ __volatile__ ("rdtsc" : "=A" (x))
  if (mode!=0 && tsc0!=0)
  {
    rdtsc(tsc1);
    return (double)(0.000001*(int)((tsc1 - tsc0)*CPU_USEC_PER_CLOCK+0.5));
  } else {
    rdtsc(tsc0);
    return 0;
  }
}

char kbhit(void){
    struct termios oldt, newt;
    char ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        ch = getchar();
        return ch;
    }
    return 0;
}

int get_date(int mode){
  struct tm *date;
  time_t now;
  int year,month,day;
  int hour,minute,second;

  time(&now);
  date = localtime(&now);

  year   = date->tm_year + 1900;
  month  = date->tm_mon +1;
  day    = date->tm_mday;
  hour   = date->tm_hour;
  minute = date->tm_min;
  second = date->tm_sec;
  if(mode == 1) return year;
  if(mode == 2) return month;
  if(mode == 3) return day;
  if(mode == 4) return hour;
  if(mode == 5) return minute;
  if(mode == 6) return second;
  return 0;
}
