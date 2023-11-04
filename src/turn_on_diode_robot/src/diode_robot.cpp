#include "diode_robot.h"
#include "Quaternion_Solution.h"
sensor_msgs::Imu Mpu6050;//Instantiate an IMU object //实例化IMU对象 

/**************************************
Date: January 28, 2021
Function: The main function, ROS initialization, creates the Robot_control object through the Turn_on_robot class and automatically calls the constructor initialization
功能: 主函数，ROS初始化，通过turn_on_robot类创建Robot_control对象并自动调用构造函数初始化
***************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "diode_robot"); //ROS initializes and sets the node name //ROS初始化 并设置节点名称 
  turn_on_robot Robot_Control; //Instantiate an object //实例化一个对象
  ros::spinOnce(); 
  return 0;  
} 

typedef union
{
    float f;
    unsigned char c[4];
} float2uchar;
/**************************************
Date: January 28, 2021
Function: The speed topic subscription Callback function, according to the subscribed instructions through the serial port command control of the lower computer
功能: 速度话题订阅回调函数Callback，根据订阅的指令通过串口发指令控制下位机
***************************************/
void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::Twist &twist_aux)   // 0为0x7D 1～2为0 3～6为x速度下位机需要的是厘米每秒 7～8为crc16校验 
                                                                              // 9～12为y速度 13～16为z角速度 17～18为crc16校验 19为0x8c
{
  short  transition;  //intermediate variable //中间变量
  float2uchar need_return_vel_x;
  float2uchar need_return_vel_y;
  float2uchar need_return_vel_z;
  // Send_Data.tx[0]=FRAME_HEADER; //frame head 0x7B //帧头0X7B
  // Send_Data.tx[1] = 0; //set aside //预留位
  // Send_Data.tx[2] = 0; //set aside //预留位
  Send_Data.tx[0]=0x7D; //frame head 0x7B //帧头0X7B
  Send_Data.tx[1] = 0; //set aside //预留位
  Send_Data.tx[2] = 0; //set aside //预留位
  //The target velocity of the X-axis of the robot
  //机器人x轴的目标线速度
  //transition=0;
  // transition = twist_aux.linear.x*1000; //将浮点数放大一千倍，简化传输
  // Send_Data.tx[4] = transition;     //取数据的低8位
  // Send_Data.tx[3] = transition>>8;  //取数据的高8位
  need_return_vel_x.f = twist_aux.linear.x*100;
  Send_Data.tx[3] = need_return_vel_x.c[0];
  Send_Data.tx[4] = need_return_vel_x.c[1];
  Send_Data.tx[5] = need_return_vel_x.c[2];
  Send_Data.tx[6] = need_return_vel_x.c[3];

  need_return_vel_y.f = twist_aux.linear.y*100;
  Append_CRC16_Check_Sum(Send_Data.tx , 9);
  Send_Data.tx[9] = need_return_vel_y.c[0];
  Send_Data.tx[10] = need_return_vel_y.c[1];
  Send_Data.tx[11] = need_return_vel_y.c[2];
  Send_Data.tx[12] = need_return_vel_y.c[3];

  need_return_vel_z.f = twist_aux.angular.z;
  Send_Data.tx[13] = need_return_vel_z.c[0];
  Send_Data.tx[14] = need_return_vel_z.c[1];
  Send_Data.tx[15] = need_return_vel_z.c[2];
  Send_Data.tx[16] = need_return_vel_z.c[3];
  Append_CRC16_Check_Sum(Send_Data.tx , 19);
  // Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //For the BCC check bits, see the Check_Sum function //BCC校验位，规则参见Check_Sum函数
  Send_Data.tx[19]=0x8c; //frame tail 0x7D //帧尾0X7D
  try
  {
    int bytes = 0;
    bytes=Stm32_Serial.write(Send_Data.tx,20); //Sends data to the downloader via serial port //通过串口向下位机发送数据 
    if(bytes != 0 && need_return_vel_x.f!=0){
      // cout<<"bytes:"<<bytes<<endl;
      // cout<<"velback_x:"<<need_return_vel_x.f<<endl;
      // cout<<"velback_y:"<<need_return_vel_y.f<<endl;
      // cout<<"velback_z:"<<need_return_vel_z.f<<endl;
      // cout<<need_return_vel_x.f<<endl;
      // printf("0X%x ",Send_Data.tx[19]);
    }
  }
  catch (serial::IOException& e)   
  {
    ROS_ERROR_STREAM("Unable to send data through serial port"); //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
  }
}

/**************************************
Date: January 28, 2021
Function: Serial port communication check function, packet n has a byte, the NTH -1 byte is the check bit, the NTH byte bit frame end.Bit XOR results from byte 1 to byte n-2 are compared with byte n-1, which is a BCC check
Input parameter: Count_Number: Check the first few bytes of the packet
功能: 串口通讯校验函数，数据包n有个字节，第n-1个字节为校验位，第n个字节位帧尾。第1个字节到第n-2个字节数据按位异或的结果与第n-1个字节对比，即为BCC校验
输入参数： Count_Number：数据包前几个字节加入校验   mode：对发送数据还是接收数据进行校验
***************************************/
unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number,unsigned char mode)
{
  unsigned char check_sum=0,k;
  
  if(mode==0) //Receive data mode //接收数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^Receive_Data.rx[k]; //By bit or by bit //按位异或
     }
  }
  if(mode==1) //Send data mode //发送数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^Send_Data.tx[k]; //By bit or by bit //按位异或
     }
  }
  return check_sum; //Returns the bitwise XOR result //返回按位异或结果
}
/**************************************
Date: May 11, 2023
功能: CRC发送校验中间函数
输入参数： 
***************************************/
uint16_t CRC_INIT = 0xffff;
const unsigned char CRC8_INIT = 0xff;
uint16_t turn_on_robot::Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
	uint8_t chData;

	if (pchMessage == NULL)
	{
		return 0xFFFF;
	}

	while (dwLength--)
	{
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
	}

	return wCRC;
}
unsigned char turn_on_robot::Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8)
{
	unsigned char ucIndex;

	while (dwLength--)
	{
		ucIndex = ucCRC8 ^ (*pchMessage++);
		ucCRC8 = CRC8_TAB[ucIndex];
	}

	return (ucCRC8);
}
/**************************************
Date: May 11, 2023
功能: CRC发送校验
输入参数： 
***************************************/
void turn_on_robot::Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
	uint16_t wCRC = 0;

	if ((pchMessage == NULL) || (dwLength <= 2))
	{
		return;
	}

	wCRC = Get_CRC16_Check_Sum((uint8_t *)pchMessage, dwLength - 2, CRC_INIT);
	pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00ff);
	pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00ff);
}
void turn_on_robot::Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
	unsigned char ucCRC = 0;

	if ((pchMessage == 0) || (dwLength <= 2)) return;

	ucCRC = Get_CRC8_Check_Sum((unsigned char *)pchMessage, dwLength - 1, CRC8_INIT);
	pchMessage[dwLength - 1] = ucCRC;
}
/**************************************
Date: May 11, 2023
功能: CRC接收校验
输入参数： 
***************************************/
unsigned int turn_on_robot::Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
	unsigned char ucExpected = 0;

	if ((pchMessage == 0) || (dwLength <= 2)) return 0;

	ucExpected = Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT);
	return (ucExpected == pchMessage[dwLength - 1]);
}
uint32_t turn_on_robot::Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
	uint16_t wExpected = 0;

	if ((pchMessage == NULL) || (dwLength <= 2))
	{
		return 0;
	}

	wExpected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC_INIT);
	return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/**************************************
Date: January 28, 2021
Function: Constructor, executed only once, for initialization
功能: 构造函数, 只执行一次，用于初始化
***************************************/
turn_on_robot::turn_on_robot():Sampling_Time(0)
{
  //Clear the data
  //清空数据
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data)); 
  memset(&Send_Data, 0, sizeof(Send_Data));
  memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));

  ros::NodeHandle private_nh("~"); //Create a node handle //创建节点句柄
  //The private_nh.param() entry parameter corresponds to the initial value of the name of the parameter variable on the parameter server
  //private_nh.param()入口参数分别对应：参数服务器上的名称  参数变量名  初始值
  private_nh.param<std::string>("usart_port_name",  usart_port_name,  "/dev/ttyUSB0"); //Fixed serial port number //固定串口号
  private_nh.param<int>        ("serial_baud_rate", serial_baud_rate, 115200); //Communicate baud rate 115200 to the lower machine //和下位机通信波特率115200
  private_nh.param<std::string>("odom_frame_id",    odom_frame_id,    "odom_combined");      //The odometer topic corresponds to the parent TF coordinate //里程计话题对应父TF坐标
  private_nh.param<std::string>("robot_frame_id",   robot_frame_id,   "base_footprint"); //The odometer topic corresponds to sub-TF coordinates //里程计话题对应子TF坐标
  private_nh.param<std::string>("gyro_frame_id",    gyro_frame_id,    "gyro_link"); //IMU topics correspond to TF coordinates //IMU话题对应TF坐标

  //Odometer correction parameters
  //里程计误差修正参数
  private_nh.param<float>("odom_x_scale",    odom_x_scale,    1.0); 
  private_nh.param<float>("odom_y_scale",    odom_y_scale,    1.0); 
  private_nh.param<float>("odom_z_scale_positive",    odom_z_scale_positive,    1.0); 
  private_nh.param<float>("odom_z_scale_negative",    odom_z_scale_negative,    1.0); 

  //Set the velocity control command callback function
  //速度控制命令订阅回调函数设置
  Cmd_Vel_Sub     = n.subscribe("cmd_vel",     100, &turn_on_robot::Cmd_Vel_Callback, this); 

  ROS_INFO_STREAM("Data ready"); //Prompt message //提示信息
  
  try
  { 
    //Attempts to initialize and open the serial port //尝试初始化与开启串口
    Stm32_Serial.setPort(usart_port_name); //Select the serial port number to enable //选择要开启的串口号
    Stm32_Serial.setBaudrate(serial_baud_rate); //Set the baud rate //设置波特率
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000); //Timeout //超时等待
    Stm32_Serial.setTimeout(_time);
    ROS_INFO_STREAM("Stm32_open_success");
    Stm32_Serial.open(); //Open the serial port //开启串口
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("diode_robot can not open Stm32_serial port,Please check the serial port cable!: " << e.what()); //If opening the serial port fails, an error message is printed //如果开启串口失败，打印错误信息
  }
  if(Stm32_Serial.isOpen())
  {
    ROS_INFO_STREAM("diode_robot Stm32_serial port opened"); //Serial port opened successfully //串口开启成功提示
  }
}
/**************************************
Date: January 28, 2021
Function: Destructor, executed only once and called by the system when an object ends its life cycle
功能: 析构函数，只执行一次，当对象结束其生命周期时系统会调用这个函数
***************************************/
turn_on_robot::~turn_on_robot()
{
  //Sends the stop motion command to the lower machine before the turn_on_robot object ends
  //对象turn_on_robot结束前向下位机发送停止运动命令
  Send_Data.tx[0]=FRAME_HEADER;
  Send_Data.tx[1] = 0;  
  Send_Data.tx[2] = 0; 

  //The target velocity of the X-axis of the robot //机器人X轴的目标线速度 
  Send_Data.tx[4] = 0;     
  Send_Data.tx[3] = 0;  

  //The target velocity of the Y-axis of the robot //机器人Y轴的目标线速度 
  Send_Data.tx[6] = 0;
  Send_Data.tx[5] = 0;  

  //The target velocity of the Z-axis of the robot //机器人Z轴的目标角速度 
  Send_Data.tx[8] = 0;  
  Send_Data.tx[7] = 0;    
  Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //Check the bits for the Check_Sum function //校验位，规则参见Check_Sum函数
  Send_Data.tx[10]=FRAME_TAIL; 
  try
  {
    Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //Send data to the serial port //向串口发数据  
  }
  catch (serial::IOException& e)   
  {
    ROS_ERROR_STREAM("Unable to send data through serial port"); //If sending data fails, an error message is printed //如果发送数据失败,打印错误信息
  }
  Stm32_Serial.close(); //Close the serial port //关闭串口  
  ROS_INFO_STREAM("Shutting down"); //Prompt message //提示信息
}
