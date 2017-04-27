
////////////////////////////////////////////////////////////////////////////////
// 定义SJA1000 CAN 收发各节点数据时使用的ID号。
//
////////////////////////////////////////////////////////////////////////////////

//定义 C8051F040 CAN 用于接收的消息号 
// (消息号范围1~16,大于16，需修改CAN1中断服务程序)
////////////////////////////////////////////////////////////////////////////////
//用于接收的消息号
////////////////////////////////////////////////////////////////////////////////
#define RX_MSGNUM_ZKB		 1   //  接收主控板动作指令数据及灯控指令数据
////////////////////////////////////////////////////////////////////////////////

//用于发送的消息号
////////////////////////////////////////////////////////////////////////////////
#define TX_MSGNUM_ZKB		8	// 向主控板发送	
#define TIMEVALUE_ALARM1	200	// 左右翻板故障时间预值



//左右定位长度最大值，作为无限运行值，需要区分左右
#define		UNLIMITED_DISTANCE_LEFT			2000000000
#define		UNLIMITED_DISTANCE_RIGHT		-2000000000
//
#define		UNLIMITED_SPEED			10000
#define		UNLIMITED_ADDTIME		2000
#define		UNLIMITED_SUBTIME		2000
//






/***********************

  HL10(LED9)为信号检测3，和4指示;
  HL9(LED8)为CAN通讯故障指示;
  HL8(LED7)为CAN通讯接受数据指示;
  HL7(LED6)为接收测距仪数据指示
  HL6(LED5)为步进脉冲2指示;  现改为X,Y轴运行到位后的指示，仅在Z轴板上实现。
  HL5(LED4)为步进脉冲1指示;
  HL4(LED3)为 触发限位2指示 
  HL3(LED2)为 触发限位1指示 
  HL2(LED1)为程序运行指示;
  HL1(LED0)电源故障指示;
************************/
sbit  LED1=P2^4;                  //状态指示灯
sbit  LED2=P2^6;                  //状态指示灯
sbit  LED3=P1^0;                  //状态指示灯
sbit  LED4=P1^2;                  //状态指示灯
sbit  LED5=P1^4;                  //状态指示灯
sbit  LED6=P1^6;                  //状态指示灯
sbit  LED7=P3^2;                  //状态指示灯
sbit  LED8=P3^4;                  //状态指示灯
sbit  LED9=P3^6;                  //状态指示灯
//
sbit  motor1_dir=P0^5;              //步进电机1   高电平：前进；低电平：后退
sbit  motor1_ENB=P0^6;              //步进电机1   高电平：前进；低电平：后退
//
sbit  DC_motor1=P5^3;	//左闸门下行信号    高电平停止；低电平运行
sbit  DC_motor2=P5^1;	//左闸门上行信号    高电平停止；低电平运行
sbit  DC_motor3=P5^7;	//右闸门下行信号    高电平停止；低电平运行
sbit  DC_motor4=P5^5;	//右闸门上行信号    高电平停止；低电平运行

//
sbit RE1=P4^1;
sbit DE1=P4^3;
//
sbit  MONITOR_INPUT1=P2^2;	//右闸门下限传感器信号
sbit  MONITOR_INPUT2=P0^3;	//左闸门下限传感器信号
sbit  MONITOR_INPUT3=P0^2;	//左闸门上限传感器信号
sbit  MONITOR_INPUT4=P2^0;	//右闸门上限传感器信号

//-----------------------------------------------------------------------------
// 全局常量
//-----------------------------------------------------------------------------

#define YES 1
#define NO  0
#define SYSCLK       11059200     // Internal oscillator frequency in Hz

//#define SAVEBUFSIZE  5   //flash

//串口接收缓冲器大小

#define RECBUF1SIZE 19


//串口发送缓冲器大小
//#define TRANSBUF1SIZE 4

//激光测距初始基本值
/*#define Base_JiGuangCeJu_Value 132

#define Current_Shaft_Modulus1 2.13
#define Current_Shaft_Modulus2 2.13
#define Current_Shaft_Modulus3 375
#define Current_Shaft_Modulus4 5*/

//-----------------------------------------------------------------------------
// 类型定义
//-----------------------------------------------------------------------------

//串口0    
/*typedef struct    
{
	unsigned char  head ;  
	unsigned char  data1;
	unsigned char  sum;
}TRANSMIT1;            

typedef union 
{ 
	TRANSMIT1 trans;
	unsigned char buf[TRANSBUF1SIZE];
}TRANSBUF1;*/
typedef struct 
{
	unsigned char  head ;  
	unsigned char  data1;
	unsigned char  data2;
	unsigned char  data3;
	unsigned char  data4;
	unsigned char  data5;
	unsigned char  data6;
	unsigned char  data7;
	unsigned char  data8;
	unsigned char  data9;
	unsigned char  data10;
	unsigned char  data11;
	unsigned char  data12;
	unsigned char  data13;
	unsigned char  data14;
	unsigned char  data15;
	unsigned char  data16;
	unsigned char  data17;
	unsigned char  data18;	
}RECEIVE1;            

typedef union 
{ 
	RECEIVE1 rec;
	unsigned char buf[RECBUF1SIZE];
}RECBUF1;



//整数提取高低位字节
/*typedef union 
{ 
	int value;
	unsigned char buf[2];
}INTUNION;*/

typedef union 
{ 
	unsigned int value;
	unsigned char buf[2];
}UINTUNION;

/*typedef union 
{ 
	long int value;
	unsigned char buf[4];
}LONGUNION;*/





/*********************************************************************
//CAN数据传输格式（发送接收同一格式）
/*********************************************************************/
typedef struct                    
{
	unsigned char  address1;    // 1
	unsigned char  command;      // 2
	unsigned char  index;      // 3
	unsigned char  data1;      // 4
	unsigned char  data2;      // 5
	unsigned char  data3;      // 6
	unsigned char  data4;      // 7
	unsigned char  checkout;   // 8
}CANTRANSCHAR3;
// CAN1 buffer definition
//-----------------CAN 总线接受缓冲区-------------
/*typedef struct                     //包头包尾数据、归零及自检指令
{
	unsigned char  command;       // 1
	unsigned char  index;         // 2
	unsigned char  shake_hand1;   // 3
	unsigned char  shake_hand2;   // 4
	unsigned char  box_code;      // 5
	unsigned char  shaft_code;    // 6
	unsigned char  bakeup1;       // 7
	unsigned char  checkout;      // 8
}CANRECCHAR1;*/



/*typedef struct                     //电机轴数据
{
	unsigned char  command;     // 1
	unsigned char  shaft_code;  // 2
	unsigned char  box_code;   // 3
	unsigned char  bakeup0;   // 4
	unsigned char  bakeup1;   // 5
	unsigned char  add_speed;   // 6
	unsigned char  max_speed;   // 7
	unsigned char  checkout;    // 8
}CANRECCHAR2;*/

/*typedef struct                     //强制运行指令
{
	unsigned char  command;       // 1
	unsigned char  shaft_code;    // 2
	unsigned char  shake_hand;    // 3
	unsigned char  action_mode;   // 4
	unsigned char  bakeup1;       // 5
	unsigned char  bakeup2;       // 6
	unsigned char  bakeup3;       // 7
	unsigned char  checkout;      // 8
}CANRECCHAR3;*/

/*typedef struct                     //电机动作完毕后反馈OK数据
{
	unsigned char  command;      // 1
	unsigned char  index;        // 2
	unsigned char  shaft_code;   // 3
	unsigned char  resoult;      // 4
	unsigned char  bakeup0;      // 5   
	unsigned char  bakeup1;      // 6
	unsigned char  bakeup2;      // 7
	unsigned char  checkout;     // 8
}CANRECCHAR4;*/

typedef union 
{ 
	//CANRECCHAR1 total_ctl_buf;
	//CANRECCHAR2 motor_ctl_buf;
	//CANRECCHAR3 motor_force_buf;
	//CANRECCHAR4 motor_feedbake_buf;
	unsigned char buf[8];
}CANRECDATBUF;

/*typedef struct                     //电机动作完毕后反馈OK数据
{
	unsigned char  command;    // 1
	unsigned char  index;      // 2
	unsigned char  shaft_code;   // 3
	unsigned char  resoult;      // 4
	unsigned char  bakeup0;      // 5   
	unsigned char  bakeup1;      // 6
	unsigned char  bakeup2;      // 7
	unsigned char  checkout;     // 8
}CANTRANSCHAR1;*/

/*typedef struct                     //激光测距仪检测完毕后反馈数据
{
	unsigned char  command;    // 1
	unsigned char  index;      // 2
	unsigned char  address1;   // 3
	unsigned char  address2;   // 4
	unsigned int   result;     // 5
	unsigned char  left_right; // 7
	unsigned char  checkout;   // 8
}CANTRANSCHAR2;*/



/*typedef struct                     //补药的数量反馈数据
{
	unsigned char  command;    // 1
	unsigned char  index;      // 2
	unsigned char  address1;   // 3
	unsigned char  address2;   // 4
	unsigned char  amount;     // 5
	unsigned char  bakeup1;     // 6
	unsigned char  bakeup2;     // 7
	unsigned char  checkout;    // 8
}CANTRANSCHAR4;*/

//-----------------------------------------------------------------------------
// 联合体 组合上述三种结构体以便使用
//-----------------------------------------------------------------------------

typedef union 
{ 
	//CANTRANSCHAR1 motor_feedbake_buf;
	//CANTRANSCHAR2 jg_feedbake_buf;
	CANTRANSCHAR3 normal_buf;
	//CANTRANSCHAR4 buyao_feedbake_buf;
	unsigned char buf[8];
}CANTRANSDATBUF;

/*typedef struct 
{ 
	unsigned int north_angle;    //寻北仪显示的角度
	unsigned int north_zero;     //寻北仪确定的零点
	unsigned char num;
}SAVEBUF;  */          //save to flash

/*typedef union 
{ 
	SAVEBUF para;
	unsigned char buf[SAVEBUFSIZE];
}FLASH;*/
// ----------------CAN2 buffer definition-----与电磁铁驱动控制板联系

/*typedef struct                     //电磁铁驱动数据
{
	unsigned char  command;    // 1
	unsigned char  index1;      // 2
	unsigned char  address1;   // 3
	unsigned char  address2;   // 4
	unsigned char  off_time;   // 5
	unsigned char  on_time;    // 6
	unsigned char  aim_amount; // 7
	unsigned char  checkout;   // 8
}CAN2TRANS1;*/

/*typedef struct                     //点灯数据
{
	unsigned char  command;    // 1
	unsigned char  index2;      // 2
	unsigned char  address1;   // 3
	unsigned char  address2;   // 4
	unsigned char  statement;  // 5
	unsigned char  blank1;     // 6
	unsigned char  blank2;     // 7
	unsigned char  checkout;   // 8
}CAN2TRANS2;*/

/*typedef struct                     //底层驱动板电磁铁故障动作次数反馈及故障代码数据
{
	unsigned char  command;    // 1
	unsigned char  index1;     // 2
	unsigned char  address1;   // 3
	unsigned char  address2;   // 4
	unsigned char  bite;       // 5
	unsigned char  blank1;     // 6
	unsigned char  amount;     // 7
	unsigned char  checkout;   // 8
}CAN2REC1;*/

/*typedef struct                     //底层驱动板电磁铁OK反馈数据
{
	unsigned char  command;    // 1
	unsigned char  index1;     // 2
	unsigned char  address1;   // 3
	unsigned char  address2;   // 4
	unsigned char  bite;       // 5
	unsigned char  Result_OK_Sign;// 6
	unsigned char  Action_OK_Sign;// 7
	unsigned char  checkout;   // 8
}CAN2REC2;*/
//-----------------------------------------------------------------------------
// 联合体 组合上述三种结构体以便使用
//-----------------------------------------------------------------------------
/*typedef union 
{ 
	//CAN2TRANS1 act_buf;
	unsigned char buf[8];
}CAN2TRANSBUF1;

typedef union 
{ 
	//CAN2TRANS2 light_buf;
	unsigned char buf[8];
}CAN2TRANSBUF2;

typedef union 
{ 
	//CAN2REC1 amount_buf; 
	//CAN2REC2 bite_buf; 		 
	unsigned char buf[8];
}CAN2RECBUF1 ;*/

typedef struct 
{
	
	 unsigned char   AUTOsystem_command;
	
	unsigned char    AUTOsystem_alarm1;
	unsigned char    AUTOsystem_alarm2;
}systemcrtl; 
typedef union  
{
  long displace;
  unsigned char buf[4];	
} Servomotordisplace;
typedef union  
{
  unsigned long speed;
  unsigned char buf[4];	
} Servomotorspeed;

typedef union  
{
  unsigned long addtime;
  unsigned char buf[4];	
} Servomotoraddtime;
typedef union  
{
  unsigned long subtime;
  unsigned char buf[4];	
} Servomotorsubtime;







extern xdata unsigned  char Servopara[107];
/*Servopara[]数据定义
//前进0.5m的立即值数据
**CANINDEX*16+[0-3]：Servomotor_displace
**CANINDEX*16+[4-7]：Servomotor_speed
**CANINDEX*16+[8-11]：Servomotor_addtime
**CANINDEX*16+[12-15]：Servomotor_subtime
//前进1m的立即值数据
**CANINDEX*16+[16-19]：Servomotor_displace
**CANINDEX*16+[20-23]：Servomotor_speed
**CANINDEX*16+[24-27]：Servomotor_addtime
**CANINDEX*16+[28-31]：Servomotor_subtime
//步进电机数据
**CANINDEX*9+[80-83]：Shaft1_aim_pulse
**CANINDEX*9+[84-85]：Shaft1_Speed_max
**CANINDEX*9+[86-87]：Shaft1_Add_Speed
**CANINDEX*9+[88]：Shaft1_ADD_Speed_Time_Base
*/

















