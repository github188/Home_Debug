
////////////////////////////////////////////////////////////////////////////////
// 定义SJA1000 CAN 收发各节点数据时使用的ID号。
//
////////////////////////////////////////////////////////////////////////////////

//定义 C8051F040 CAN 用于接收的消息号
// (消息号范围1~16,大于16，需修改CAN1中断服务程序)
////////////////////////////////////////////////////////////////////////////////
//用于接收的消息号
////////////////////////////////////////////////////////////////////////////////
#define		Delay_Times	20000		//延时循环次数
#define RX_MSGNUM_ZKB		 1   //  接收主控板动作指令数据及灯控指令数据	

//#define RX_MSGNUM_ZKB_Union	 2   // 接收主控板动作指令的始发包或结束包数据(公共通道)
//#define RX_MSGNUM_X_Shaft    3   // 接收X轴发送过来的数据
//#define RX_MSGNUM_Y_Shaft    4   // 接收Y轴发送过来的数据
//#define RX_MSGNUM_Z_Shaft    5   // 接收Z轴发送过来的数据
//#define RX_MSGNUM_H_Shaft    6   // 接收H轴发送过来的数据
//#define RX_MSGNUM_ZKB_1      7   //接收主控板动作指令的始发包或结束包数据(公共通道)



//#define CAN2TXID_Uion	         18	// 底层驱动板接收共用数据的地址
//#define Driver_Board_Amount	 96	    // 底层驱动板总个数
//#define Jijia_Amount	 6	        // 机架的总个数
//#define BYTE_Amount	 12	            // 判断动作是否结束的字节总个数



////////////////////////////////////////////////////////////////////////////////

//用于发送的消息号
////////////////////////////////////////////////////////////////////////////////
#define TX_MSGNUM_ZKB		2	// 向主控板发送	
//#define TX_MSGNUM_Y	    	9	// 向Y轴发送

//------共用通道的地址------------------
//#define Address_Union		418  // 接收主控板动作指令的始发包或结束包数据地址


/***********************

  HL10(LED9)为信号检测3，和4指示;
  HL9(LED8)为CAN通讯故障指示;
  HL8(LED7)为CAN通讯接受数据指示;
  HL7(LED6)为接收测距仪数据指示
  HL6(LED5)为步进脉冲2指示;  现改为X,Y
  轴运行到位后的指示，仅在Z轴板上实现。
  HL5(LED4)为步进脉冲1指示;
  HL4(LED3)为 触发限位2指示
  HL3(LED2)为 触发限位1指示
  HL2(LED1)为程序运行指示;
  HL1(LED0)电源故障指示;
************************/



#define		Max_displace	2850000		//伺服最大位移量
#define		Max_speedvalue	300000	        //伺服最大速速度*100
#define		Zero_backdisplace	500	//伺服
#define		GAOKONG_TIMER			600	//高空输送线延时定时
//
sbit  LED1 = P2 ^ 4;              //状态指示灯
sbit  LED2 = P2 ^ 6;              //状态指示灯
sbit  LED3 = P1 ^ 0;              //状态指示灯
sbit  LED4 = P1 ^ 2;              //状态指示灯
sbit  LED5 = P1 ^ 4;              //状态指示灯
sbit  LED6 = P1 ^ 6;              //状态指示灯
sbit  LED7 = P3 ^ 2;              //状态指示灯
sbit  LED8 = P3 ^ 4;              //状态指示灯
sbit  LED9 = P3 ^ 6;              //状态指示灯
//
sbit	motor1_dir = P0 ^ 5;          //步进电机1   高电平：前进；低电平：后退
sbit	motor1_ENB = P0 ^ 6;          //步进电机1   高电平：前进；低电平：后退
//
sbit	RE1 = P4 ^ 1;
sbit	DE1 = P4 ^ 3;
//
sbit	Motor2 = P5 ^ 1; //高空输送线
sbit	Motor1_Power = P5 ^ 3; //倾倒输送线正转
sbit	Motor1_Dir = P5 ^ 5; //倾倒输送线反转——引脚待定、记得初始化
//
sbit	MONITOR_INPUT1 = P0 ^ 2; //高空输送线传感器
sbit	MONITOR_INPUT2 = P0 ^ 3; //提升输送线传感器
//sbit	MONITOR_INPUT3 = P2 ^ 0;
//sbit	MONITOR_INPUT4 = P2 ^ 2;
//-----------------------------------------------------------------------------
// 全局常量
//-----------------------------------------------------------------------------

#define YES 1
#define NO  0
#define SYSCLK       11059200     // Internal oscillator frequency in Hz

//#define SAVEBUFSIZE  5   //flash

//串口接收缓冲器大小

//#define RECBUF1SIZE 30


//串口发送缓冲器大小
#define TRANSBUF1SIZE 4

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
}RECEIVE1;*/

/*typedef union
{
	//RECEIVE1 rec;
	unsigned char buf[RECBUF1SIZE];
}RECBUF1;*/



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
} UINTUNION;



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
} CANTRANSCHAR3;
// CAN1 buffer definition
//-----------------CAN 总线接受缓冲区-------------
typedef struct                     //包头包尾数据、归零及自检指令
{
    unsigned char  command;       // 1
    unsigned char  index;         // 2
    unsigned char  shake_hand1;   // 3
    unsigned char  shake_hand2;   // 4
    unsigned char  box_code;      // 5
    unsigned char  shaft_code;    // 6
    unsigned char  bakeup1;       // 7
    unsigned char  checkout;      // 8
} CANRECCHAR1;
typedef union
{
    CANRECCHAR1 total_ctl_buf;
    //CANRECCHAR2 motor_ctl_buf;
    //CANRECCHAR3 motor_force_buf;
    //CANRECCHAR4 motor_feedbake_buf;
    unsigned char buf[8];
} CANRECDATBUF;

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
} CANTRANSDATBUF;






//-----------------------------------------------------------------------------
// 联合体 组合上述三种结构体以便使用
//-----------------------------------------------------------------------------


typedef struct
{
    unsigned char   AUTOsystem_command;
    unsigned char    AUTOsystem_alarm1;
    //unsigned long    AUTOsystem_alarm2;
} systemcrtl;

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





//-------------------------------------Yaoping.c--------------------------------------
// 全局变量
//-----------------------------------------------------------------------------
extern bit Receive_command_finished;
extern bit Receive485_command_finished;
extern bit Time_FLAG;
extern bit Time_FLAG1;
extern bit AUTO_FLAG;
extern bit Step_FLAG;
extern bit CANINFOR_FLAG;
extern bit ONCE_FLAG;
extern xdata volatile unsigned char Runmode;
//定义定时器的软件计数器
extern xdata unsigned char  T0Counter5;  //
extern xdata unsigned char  T0Counter2;
extern xdata volatile unsigned char  T0Counter3;  //
extern xdata unsigned char  T0Counter4;  //
extern xdata unsigned int  T0Counter6;  //
extern xdata unsigned char  T0Counter8;
extern xdata unsigned int   T0Counter9;
extern xdata unsigned int   T0Counter10;  //高空输送线定时器
extern xdata unsigned char  GAOKONG_COUT; //高空输送线计数器
extern xdata unsigned char  YAOPINGTISH_COUT; //药品提升计数器
extern xdata unsigned char  CANcomand;    //CAN命令字
extern  xdata volatile unsigned char  nCANcomand;     //防止CAN中断修改运行中的命令
extern xdata unsigned char  AUTOCANcomand;
extern xdata unsigned char  AUTOCMD_STATE;
extern xdata unsigned char  AUTOCMD_TIME;
extern xdata volatile unsigned char  AUTOCMD_CHECK;
extern xdata unsigned char  temppage;
extern xdata unsigned char BPSSET;
extern xdata unsigned char IDSET;
extern xdata unsigned char Shaft1_Run_mode;
extern xdata unsigned long Shaft1_aim_pulse;
extern xdata unsigned char Sampleperiod;
extern xdata unsigned int ShowTime1;
//xdata FLASH Flash;            //定义Flash 缓冲区
extern xdata CANTRANSDATBUF CANTXBUF_ZKB;
extern xdata unsigned  int Pulse1;
extern xdata unsigned char Shaft1_Base_Speed_Value;
extern xdata long int Shaft1_Velocity_Subsection_Parameter;
extern xdata long int motor1_out;                        //步进电机1实际的转速
extern xdata unsigned char Shaft1_ADD_Speed_Time_Base;
extern xdata unsigned char Shaft1_SUB_Speed_Time_Base;
extern xdata unsigned  long Pulse1_Cnt;
extern xdata unsigned int  Shaft1_Add_Speed;
extern xdata unsigned int  Shaft1_Speed_max;
extern xdata long int Vel1;
extern xdata unsigned long speed_value;
extern xdata unsigned char buf[30];
extern xdata unsigned char Rec1[30];
extern xdata unsigned  char RecPointer1;
//xdata  RECBUF1 RxBuf1;
extern xdata unsigned  long Servoparadisplace;
extern xdata unsigned  long Servoparaspeed;
extern xdata unsigned  long Servoparaaddtime;
extern xdata unsigned  long Servoparasubtime;
extern xdata unsigned  long ShowTime;
extern xdata unsigned  char Servopara[107];
/*Servopara[]数据定义
**CANINDEX*16+[0-3]：Servomotor_displace
**CANINDEX*16+[4-7]：Servomotor_speed
**CANINDEX*16+[8-11]：Servomotor_addtime
**CANINDEX*16+[12-15]：Servomotor_subtime

**CANINDEX*9+[80-83]：Shaft1_aim_pulse
**CANINDEX*9+[84-85]：Shaft1_Speed_max
**CANINDEX*9+[86-87]：Shaft1_Add_Speed
**CANINDEX*9+[88]：Shaft1_ADD_Speed_Time_Base
*/
extern xdata unsigned  char CANTRASTEMINFOR[20];
/*CANTRASTEMINFOR[8]标志位：
**bit0:伺服运行标志
**bit1:步进电机运行标志
**bit2:高空输送线传感器信号
**bit3:提升输送线传感器信号
**bit6:伺服回原点运行标志
*/
extern xdata unsigned  char repair_flag;
extern xdata unsigned  char Servomotor_original;
extern xdata volatile systemcrtl system_crtl;
extern xdata Servomotordisplace  Servomotor_displace;
extern xdata Servomotorspeed     Servomotor_speed;
extern xdata Servomotoraddtime   Servomotor_addtime;
extern xdata Servomotorsubtime   Servomotor_subtime;
extern code unsigned int Speedvalue[2000];

/*****************************************Yaoping.c*******************************/
extern void Watchdog_Init (void);
extern void Initial(void);
extern void PORT_Init (void);
extern void OSCILLATOR_Init (void);
extern void Timer01_Init(void);
extern void init_para(void);
extern void Board_Parameter_Setup(void);
extern void init_INT0 (void);
extern void init_INT1 (void);
extern void Motor1_CTL(void);     //T2输出，控制电机1;
extern void Shaft1_CTL(void);
extern void ramp1_1(long int max);
extern void init_T2 (void);
extern void Uart0Send(unsigned char *buf, unsigned char bufsize );
extern void delay1( unsigned int us);
extern void Send_to_Motordriver_CTL(unsigned char command );
extern unsigned int crc_chk(unsigned char *puchMsg, unsigned char length);
extern void delay_ms(unsigned int ms);
extern void send_to_motor(void);
extern void RunLEDDIS(void);
extern void SteppermotorCTRL(void);
extern void ServomotorCRTL(unsigned char comand);
extern void Dataacquisition();
extern void SystemControl(void);
extern void ServomotorDataacquisition(unsigned char addressID);
extern void Returntopoint(void);
extern void UnactiveCANtransfer(void);
extern void write_to_flash(void);
extern void read_from_flash(void);
extern void TRANFORSATATIONINFOR(void);
extern void CANsetpraback(void);
extern void CurrentCollection(void);
extern void ServomotorINP(unsigned char mode);
extern void CANsteminforback(void);
extern void AUTORUNMODE0(unsigned char comand);
extern void AUTORUNMODE1(unsigned char comand);
extern unsigned char AUTOTaskassignment0(void);
extern unsigned char AUTOTaskassignment1(void);
extern void DCmotorCTRL(void);
extern void ActiveCANtransfer(void);
extern void init_machine(void);
//extern void Deviationremoval(void);


//--------------------------------------CAN1.C-------------------------------------
// 函数原型
//-----------------------------------------------------------------------------
extern void clear_msg_objects (void);
extern void init_msg_object_TX (char MsgNum, unsigned int id);
extern void init_msg_object_RX (char MsgNum, unsigned int id);
extern void start_CAN (void);
extern void can1_transmit(char MsgNum, unsigned char *buf);
extern void can1_receive (char MsgNum, unsigned char *buf);
extern void init_can1_1 (void);

//--------------------------------------CAN1.C-------------------------------------
// 全局变量
//-----------------------------------------------------------------------------
extern xdata CANRECDATBUF	CANRXBUF_ZKB;
extern xdata volatile unsigned char CANaddress1;
extern xdata unsigned char CANINDEX;
extern xdata unsigned char CAN1FaultCounter;






