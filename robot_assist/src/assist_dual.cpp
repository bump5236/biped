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
#define ADDR_OPERATING_MODE             11

#define ADDR_CURRENT_LIMIT              38
#define ADDR_GOAL_CURRENT               102
#define ADDR_PRESENT_CURRENT            126

#define ADDR_GOAL_POSITION              116
#define ADDR_PRESENT_POSITION           132

// Protocol version
#define PROTOCOL_VERSION                2.0

// Default setting
#define DXL_ID1                         1
#define DXL_ID2                         2
#define BAUDRATE                        57600
#define DEVICENAME1                     "/dev/ttyUSB0"
#define DEVICENAME2                     "/dev/ttyUSB1"
#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0
#define DXL_MOVING_STATUS_THRESHOLD     50

#define DXL_TORQ_MODE                   0
#define DXL_EXPOS_MODE                  5
#define DXL_CURRENT_LIMIT               600
#define ESC_ASCII_VALUE                 0x1b

const long unsigned int  SAMPLING_TIME_USEC = 1000;                   // sampling time [us]
const double             TS      =0.001*0.001*SAMPLING_TIME_USEC;     // sampling time [s]
const long int           LOOP_COUNT = 1*3600*1000000/SAMPLING_TIME_USEC;  // performance time 120[s]

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

  double roll;
  double pitch;
  double yaw;

  int32_t tgt_ang1;
  int32_t ang1;
  double tgt_torq1;
  double torq1;

  int32_t tgt_ang2;
  int32_t ang2;
  double tgt_torq2;
  double torq2;

  int act1;
  int act2;
};


class IMU_data{
public:
  double accl_x;
  double accl_y;
  double accl_z;

  double gyro_x;
  double gyro_y;
  double gyro_z;

  double roll;
  double pitch;
  double yaw;
};


/*--- 関数宣言 ---*/
void get_memory(void);
void free_memory(void);
double get_time_sec(int mode);
// double ang_vc(double t, int run_mode, int motor_number);  //motorの処理
int get_date(int mode);
char kbhit(void);
int getch();
void callback(const sensor_msgs::Imu& data);
void errorProc(int result, uint8 error);


/*--- 変数定義 ---*/
int counter = 0, callback_counter = 0;

// initialize
struct data_parameter *ctrl_data;
IMU_data imu_data;



int main(int argc, char **argv) {


  get_memory();

  int i = 0;
  double a = 0.0401;
  double b = 0.4392;
  double c = 0.0001;

  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error, dxl_mode;
  uint16_t C1, C2;
  uint16_t aC1, aC2;





  // ROSsetting
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("imu/data", 1, callback);  //subscribe[1]はバッファサイズ
  ros::Rate r(100);  // 100Hz

  // DXLsetting
  dynamixel::PortHandler *portHandler1 = dynamixel::PortHandler::getPortHandler(DEVICENAME1);
  dynamixel::PortHandler *portHandler2 = dynamixel::PortHandler::getPortHandler(DEVICENAME2);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);


  // Open port1, port2
  if (portHandler1->openPort()) {
    printf("Succeeded to open the port1!\n");
  }
  else {
    printf("Failed to open the port1!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }
  if (portHandler2->openPort()) {
    printf("Succeeded to open the port2!\n");
  }
  else {
    printf("Failed to open the port2!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port1, port2 baudrate
  if (portHandler1->setBaudRate(BAUDRATE)) {
    printf("Succeeded to change the baudrate1!\n");
  }
  else {
    printf("Failed to change the baudrate1!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }
  if (portHandler2->setBaudRate(BAUDRATE)) {
    printf("Succeeded to change the baudrate2!\n");
  }
  else {
    printf("Failed to change the baudrate2!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // ID1----------------------------------------------

  // MODE
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler1, DXL_ID1, ADDR_OPERATING_MODE, DXL_OPERATING_MODE, &dxl_error);
  errorProc(dxl_comm_result, dxl_error);
  // Setting DXL_CURRENT_LIMIT
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler1, DXL_ID1, ADDR_CURRENT_LIMIT, DXL_CURRENT_LIMIT, &dxl_error);
  errorProc(dxl_comm_result, dxl_error);
  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler1, DXL_ID1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  errorProc(dxl_comm_result, dxl_error);
  printf("Succeeded ready ID:%d \n", DXL_ID1);

  // ID2----------------------------------------------

  // MODE
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, DXL_ID2, ADDR_OPERATING_MODE, DXL_OPERATING_MODE, &dxl_error);
  errorProc(dxl_comm_result, dxl_error);
  // Setting DXL_CURRENT_LIMIT
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler2, DXL_ID2, ADDR_CURRENT_LIMIT, DXL_CURRENT_LIMIT, &dxl_error);
  errorProc(dxl_comm_result, dxl_error);
  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, DXL_ID2, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  errorProc(dxl_comm_result, dxl_error);
  printf("Succeeded ready ID:%d \n", DXL_ID2);


  // ---------------------------------------------------------------------------------

  while(false == ros::isShuttingDown()) {
    printf("Press any key to continue! (or press ESC to quit!)\n");

    /*位置制御でだんだん降ろす*/

    if(getch() == ESC_ASCII_VALUE) break;

    get_time_sec(0);

    for (i = 0; i < LOOP_COUNT ;i++){

      ctrl_data[i].time = get_time_sec(1);

      // IMUデータ取得----------------------
      ros::spinOnce();

      ctrl_data[i].accl_x = imu_data.accl_x;
      ctrl_data[i].accl_y = imu_data.accl_y;
      ctrl_data[i].accl_z = imu_data.accl_z;

      ctrl_data[i].gyro_x = imu_data.gyro_x;
      ctrl_data[i].gyro_y = imu_data.gyro_y;
      ctrl_data[i].gyro_z = imu_data.gyro_z;

      ctrl_data[i].roll   = imu_data.roll*360/(2*M_PI);
      ctrl_data[i].pitch  = imu_data.pitch*360/(2*M_PI);
      ctrl_data[i].yaw    = imu_data.yaw*360/(2*M_PI);


      // Current setting-------------------
      ctrl_data[i].tgt_torq1 = ctrl_data[i].roll*0.01;
      ctrl_data[i].tgt_torq2 = ctrl_data[i].roll*0.02;

      C1 = (a*ctrl_data[i].tgt_torq1^2 + b*ctrl_data[i].tgt_torq1 - c)*1000/2.69;
      C2 = (a*ctrl_data[i].tgt_torq2^2 + b*ctrl_data[i].tgt_torq2 - c)*1000/2.69;


      // torq------------------------------
      dxl_comm_result = packetHandler->write2ByteTxRx(portHandler1, DXL_ID1, ADDR_GOAL_CURRENT, C1, &dxl_error);
      errorProc(dxl_comm_result, dxl_error);

      dxl_comm_result = packetHandler->write2ByteTxRx(portHandler2, DXL_ID2, ADDR_GOAL_CURRENT, C2, &dxl_error);
      errorProc(dxl_comm_result, dxl_error);


      // read------------------------------
      dxl_comm_result = packetHandler->read2ByteTxRx(portHandler1, DXL_ID1, ADDR_PRESENT_CURRENT, &aC1, &dxl_error);
      errorProc(dxl_comm_result, dxl_error);
      ctrl_data[i].torq1 = (- b + sqrt(b^2-4*a*(c-aC1*2.69/1000)))/2*a

      dxl_comm_result = packetHandler->read2ByteTxRx(portHandler2, DXL_ID2, ADDR_PRESENT_CURRENT, &aC2, &dxl_error);
      errorProc(dxl_comm_result, dxl_error);
      ctrl_data[i].torq2 = (- b + sqrt(b^2-4*a*(c-aC2*2.69/1000)))/2*a


      printf("time=%03.4f Roll:%03f Tgt_torq1:%03f Torq1:%03f Tgt_torq2:%03f Torq2:%03f\n",
              ctrl_data[i].time,
              ctrl_data[i].roll,
              ctrl_data[i].tgt_torq1,
              ctrl_data[i].torq1,
              ctrl_data[i].tgt_torq2,
              ctrl_data[i].torq2);


      // -----------------------------------

      /*エラー処理*/

      // -----------------------------------

      stop_count=i;

      switch(kbhit()){
        case 'q':
          mode = 99;
          break;
        }
      if(mode == 99)  break;

      r.sleep();
    }

    /*引き上げたら終わり*/
    break
  }


  // End Process--------------------------------------

  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler1, DXL_ID1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  errorProc(dxl_comm_result, dxl_error);
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, DXL_ID2, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  errorProc(dxl_comm_result, dxl_error);

  // Close port
  portHandler1->closePort();
  printf("Succeeded Closeport1!\n");
  portHandler2->closePort();
  printf("Succeeded Closeport2!\n");

  return 0;
}


// -------------------------------------------------------------------------------------
void callback(const sensor_msgs::Imu& data) {

  double quat_x, quat_y, quat_z, quat_w;

  // Linear acceleration
  imu_data.accl_x = data.linear_acceleration.x;
  imu_data.accl_y = data.linear_acceleration.y;
  imu_data.accl_z = data.linear_acceleration.z;

  // Angular velocity
  imu_data.gyro_x = data.angular_velocity.x;
  imu_data.gyro_y = data.angular_velocity.y;
  imu_data.gyro_z = data.angular_velocity.z;

  // Orientation (not provided)
  quat_x = data.orientation.x;
  quat_y = data.orientation.y;
  quat_z = data.orientation.z;
  quat_w = data.orientation.w;

  tf::Quaternion quat(quat_x, quat_y, quat_z, quat_w);
  tf::Matrix3x3(quat).getRPY(imu_data.roll, imu_data.pitch, imu_data.yaw);

  callback_counter += 1;
  printf("%d\n", callback_counter);
  // printf(
  //   "accl_x:%f\naccl_y:%f\naccl_z:%f\ngyro_x:%f\ngyro_y:%f\ngyro_z:%f\nroll:%f\npitch:%f\nyaw:%f\n",
  //   imu_data.accl_x,
  //   imu_data.accl_y,
  //   imu_data.accl_z,
  //   imu_data.gyro_x,
  //   imu_data.gyro_y,
  //   imu_data.gyro_z,
  //   imu_data.roll,
  //   imu_data.pitch,
  //   imu_data.yaw
  // );
}

void get_memory(void){
  if ((ctrl_data = (struct data_parameter*)calloc(LOOP_COUNT+100, sizeof(struct data_parameter)))  == NULL) {
    perror("calloc ctrl_data");
    exit(1);
  }
}

void free_memory(void){
  free(ctrl_data);
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

int getch(){
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
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

void errorProc(int result, uint8_t error){
 if (result != COMM_SUCCESS) {
   printf("%s\n", packetHandler->getTxRxResult(result));
 }
 else if (error != 0) {
   printf("%s\n", packetHandler->getTxRxResult(error));
 }
}
