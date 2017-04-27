#include <c8051f040.h>                 // SFR declaration

//-----------------------------------------------------------------------------
// 全局常量
//-----------------------------------------------------------------------------
#define YES 1
#define OK 1
#define NO  0

#define K  112.676

#define		ZTS_CANID		2	//左提升板CAN ID
#define		YTS_CANID		4	//右提升板CAN ID
#define		DM_CANID		3	//地面板CAN ID


typedef unsigned int Uint;
typedef unsigned char Uchar;

#define SYSCLK     11059200     // Internal oscillator frequency in Hz
#define Bite_Delay_Time 500

#define TRANSBUF0SIZE	8
#define RECBUF0SIZE  8
#define TRANSBUF1SIZE  8
#define RECBUF1SIZE  8

//定义 C8051F040 CAN 用于接收的消息号
// (消息号范围1~16,大于16，需修改CAN1中断服务程序)
#define MSGNUMA   1
#define MSGNUMB   2

#define CAN1RXID_SWJ		 5		//16 上位机接受数据的地址
#define CAN1TXID_SWJ		 80		// 上位机发送数据的地址

#define CAN2TXID_Uion	     518	// 底层驱动板接收共用数据的地址

#define BYTE_Amount	     4	        // 判断动作是否结束的字节总个数

#define MAXZREO_TIME	    300	    // 等待原点最大时间

#define Driver_Board_Amount	 32	    // 底层驱动板总个数 
#define Floor_Number         10	    // 机架的层数
#define Jijia_Amount	6	        // 机架的总个数

sbit RS485_EN = P3 ^ 0;

sbit OUTPUT1 = P1 ^ 3;
sbit OUTPUT2 = P1 ^ 2;
sbit OUTPUT3 = P1 ^ 0;
sbit OUTPUT4 = P1 ^ 1;

sbit Monitor1 = P0 ^ 6;
sbit Monitor2 = P0 ^ 7;
sbit Monitor3 = P1 ^ 4;
sbit Monitor4 = P1 ^ 5;

sbit K1 = P5 ^ 1;
sbit K2 = P5 ^ 2;
sbit K3 = P5 ^ 3;
sbit K4 = P5 ^ 4;

sbit FM_SDA = P5 ^ 5;
sbit FM_SCL = P5 ^ 6;
sbit FM_WP = P5 ^ 7;

sbit LCD_CS = P2 ^ 4; //串口时为CS
sbit LCD_SID = P2 ^ 5; //串口为SID
sbit LCD_SCLK = P2 ^ 6; //串口为时钟LCD_SCLK
sbit LCD_RST = P2 ^ 7; //串口屏复位

sbit GM_MS = P4 ^ 0;
sbit STADD0 = P4 ^ 1;
sbit STADD1 = P4 ^ 2;
sbit SRADD0 = P4 ^ 3;
sbit SRADD1 = P4 ^ 4;
sbit GM_RST = P5 ^ 0;

sbit NET_IO = P3 ^ 1;
sbit NET_CFG = P3 ^ 2;
sbit NET_RST = P3 ^ 3;



//------------------------------------数据类型定义--------------------------------------
//-----------------------------------------------------------------------------
typedef struct
{
    unsigned char	DCT_Input_Command[4];            // 32个底层驱动板输入命令状态
    unsigned char	DCT_OK_Result[4];                // 32个底层驱动板动作OK结果状态
    unsigned char	DCT_OK_Action[4];                // 32个底层驱动板动作结束状态
    unsigned char   DCT_Bite[Driver_Board_Amount];   // 判断32个底层驱动板故障代码用
} SYSDAT;

//-----------------------------------------------------------------------------
// 类型定义
//-----------------------------------------------------------------------------

//----------------整数提取高低位字节
typedef union
{
    int value;
    unsigned char buf[2];
} INTUNION;

typedef union
{
    unsigned int value;
    unsigned char buf[2];
} UINTUNION;

typedef union
{
    long int value;
    unsigned char buf[4];
} LONGUNION;


// ----------------CAN1 buffer definition  ----与上位机联系

typedef struct                     //电磁铁驱动数据
{
    unsigned char  command;    // 1
    unsigned char  index1;     // 2
    unsigned char  address1;   // 3
    unsigned char  address2;   // 4
    unsigned char  off_time;   // 5
    unsigned char  on_time;    // 6
    unsigned char  aim_amount; // 7
    unsigned char  checkout;   // 8
} CAN1REC1;

typedef struct                     //向上位机反馈电磁铁故障动作次数数据及相应的故障代码
{
    unsigned char  command;    // 1
    unsigned char  index1;     // 2
    unsigned char  address1;   // 3
    unsigned char  address2;   // 4
    unsigned char  bite;       // 5
    unsigned char  blank1;     // 6
    unsigned char  amount;     // 7
    unsigned char  checkout;   // 8
} CAN1TRANS1;

typedef struct                     //向上位机反馈电磁铁动作OK信号数据
{
    unsigned char  command;     // 1
    unsigned char  index1;      // 2
    unsigned char  blank1;      // 3
    unsigned char  blank2;      // 4
    unsigned char  ALL_Result_OK_Sign; // 5
    unsigned char  ALL_Action_OK_Sign; // 6
    unsigned char  blank3;      // 7
    unsigned char  checkout;    // 8
} CAN1TRANS2;

typedef struct                     //向上位机反馈药框动作响应
{
    unsigned char  command;    // 1
    unsigned char  index1;     // 2
    unsigned char  order1;     // 3
    unsigned char  order2;     // 4
    unsigned char  blank1;     // 5
    unsigned char  blank2;     // 6
    unsigned char  blank3;     // 7
    unsigned char  checkout;   // 8
} CAN1TRANS3;

//-----------------------------------------------------------------------------
// 联合体 组合上述三种结构体以便使用
//-----------------------------------------------------------------------------
typedef union
{
    CAN1REC1 act_buf;
    unsigned char buf[8];
} CAN1RECBUF1;

typedef union
{
    CAN1TRANS1 amount_buf;
    CAN1TRANS2 OK_buf;
    unsigned char buf[8];
} CAN1TRANSBUF1;


// ----------------CAN2 buffer definition-----与电磁铁驱动控制板联系

typedef struct                     //电磁铁驱动数据
{
    unsigned char  command;    // 1
    unsigned char  index1;      // 2
    unsigned char  address1;   // 3
    unsigned char  address2;   // 4
    unsigned char  off_time;   // 5
    unsigned char  on_time;    // 6
    unsigned char  aim_amount; // 7
    unsigned char  checkout;   // 8
} CAN2TRANS1;

typedef struct                     //点灯数据
{
    unsigned char  command;    // 1
    unsigned char  index2;      // 2
    unsigned char  address1;   // 3
    unsigned char  address2;   // 4
    unsigned char  statement;  // 5
    unsigned char  blank1;     // 6
    unsigned char  blank2;     // 7
    unsigned char  checkout;   // 8
} CAN2TRANS2;

typedef struct                     //底层驱动板电磁铁故障动作次数反馈及故障代码数据
{
    unsigned char  command;    // 1
    unsigned char  index1;     // 2
    unsigned char  address1;   // 3
    unsigned char  address2;   // 4
    unsigned char  bite;       // 5
    unsigned char  blank1;     // 6
    unsigned char  amount;     // 7
    unsigned char  checkout;   // 8
} CAN2REC1;

typedef struct                     //底层驱动板电磁铁OK反馈数据
{
    unsigned char  command;    // 1
    unsigned char  index1;     // 2
    unsigned char  address1;   // 3
    unsigned char  address2;   // 4
    unsigned char  bite;       // 5
    unsigned char  Result_OK_Sign;// 6
    unsigned char  Action_OK_Sign;// 7
    unsigned char  checkout;   // 8
} CAN2REC2;
//-----------------------------------------------------------------------------
// 联合体 组合上述三种结构体以便使用
//-----------------------------------------------------------------------------
typedef union
{
    CAN2TRANS1 act_buf;
    unsigned char buf[8];
} CAN2TRANSBUF1;

typedef union
{
    CAN2TRANS2 light_buf;
    unsigned char buf[8];
} CAN2TRANSBUF2;

typedef union
{
    CAN2REC1 amount_buf;
    CAN2REC2 bite_buf;
    unsigned char buf[8];
} CAN2RECBUF1 ;

// -----------------CAN3 buffer definition----与传送带系统联系
typedef struct
{
    unsigned char  command;       // 1
    unsigned char  index;         // 2
    unsigned char  shake_hand1;   // 3
    unsigned char  shake_hand2;   // 4
    unsigned char  bakeup1;      // 5
    unsigned char  bakeup2;       // 6
    unsigned char  bakeup3;    // 7
    unsigned char  checkout;      // 8
} CAN3REC1;

typedef union
{
    CAN3REC1 rec;
    unsigned char buf[8];
} CAN3RECBUF1;


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
} CAN3TRANS1;

typedef union
{
    CAN3TRANS1 trans;
    unsigned char buf[8];
} CAN3TRANSBUF1;

//-------------------普通串口定义部分-------------------------------
//主控板发送数据信息给上位机
typedef struct                               //向上位机反馈电磁铁故障动作次数数据及相应的故障代码
{
    unsigned char  command;    // 1
    unsigned char  index1;     // 2
    unsigned char  address1;   // 3
    unsigned char  address2;   // 4
    unsigned char  bite;       // 5
    unsigned char  blank1;     // 6
    unsigned char  amount;     // 7
    unsigned char  checkout;   // 8
} TRANSMIT01;

typedef struct                               //向上位机反馈电磁铁故障动作次数数据及相应的故障代码
{
    unsigned char  command;     // 1
    unsigned char  index1;      // 2
    unsigned char  blank1;      // 3
    unsigned char  blank2;      // 4
    unsigned char  ALL_Result_OK_Sign; // 5
    unsigned char  ALL_Action_OK_Sign; // 6
    unsigned char  blank3;      // 7
    unsigned char  checkout;    // 8
} TRANSMIT02;
typedef union
{
    TRANSMIT01 trans1;
    TRANSMIT02 trans2;
    unsigned char buf[TRANSBUF0SIZE];
} TRANSBUF0;

typedef struct                          //接收上位机的控制指令
{
    unsigned char  command;    // 1
    unsigned char  index1;      // 2
    unsigned char  address1;   // 3
    unsigned char  address2;   // 4
    unsigned char  off_time;   // 5
    unsigned char  on_time;    // 6
    unsigned char  aim_amount; // 7
    unsigned char  checkout;   // 8
} RECEIVE0;
typedef union
{
    RECEIVE0 recei1;
    unsigned char buf[RECBUF0SIZE];
} RECBUF0;

//-------------------网络（串口）定义部分-------------------------------
//主控板发送数据信息给上位机
typedef struct                     //向上位机反馈电磁铁故障动作次数数据及相应的故障代码
{
    unsigned char  command;    // 1
    unsigned char  index1;     // 2
    unsigned char  address1;   // 3
    unsigned char  address2;   // 4
    unsigned char  bite;       // 5
    unsigned char  blank1;     // 6
    unsigned char  amount;     // 7
    unsigned char  checkout;   // 8
} TRANSMIT11;

typedef struct                      //向上位机反馈电磁铁故障动作次数数据及相应的故障代码
{
    unsigned char  command;     // 1
    unsigned char  index1;      // 2
    unsigned char  blank1;      // 3
    unsigned char  blank2;      // 4
    unsigned char  ALL_Result_OK_Sign; // 5
    unsigned char  ALL_Action_OK_Sign; // 6
    unsigned char  blank3;      // 7
    unsigned char  checkout;    // 8
} TRANSMIT12;

typedef struct                      //通用型数据格式
{
    unsigned char  blank0;      // 1
    unsigned char  blank1;      // 2
    unsigned char  blank2;      // 3
    unsigned char  blank3;      // 4
    unsigned char  blank4;      // 5
    unsigned char  blank5;      // 6
    unsigned char  blank6;      // 7
    unsigned char  checkout;    // 8
} TRANSMIT13;

typedef union
{
    TRANSMIT11 amount_buf;
    TRANSMIT12 OK_buf;
    TRANSMIT13 trans1;
    unsigned char buf[TRANSBUF1SIZE];
} TRANSBUF1;

typedef struct                     //接收上位机的控制指令
{
    unsigned char  command;    // 1
    unsigned char  index1;     // 2
    unsigned char  address1;   // 3
    unsigned char  address2;   // 4
    unsigned char  off_time;   // 5
    unsigned char  on_time;    // 6
    unsigned char  aim_amount; // 7
    unsigned char  checkout;   // 8
} RECEIVE1;
typedef union
{
    RECEIVE1 act_buf;
    unsigned char buf[RECBUF1SIZE];
} RECBUF1;

typedef union
{
    long displace;
    unsigned char buf[4];
} Servomotordisplace;
//------------------






//-----------------------------------zk.c--------------------------------------
// 函数原型
//-----------------------------------------------------------------------------
extern void TIME0_ISR (void);
extern int  get_ad_value(unsigned char channel);
extern signed char get_temp(void);
extern void config (void);
extern void initialize(void);

extern void init_AD(void);
extern void LedBlink(unsigned char num, unsigned char state, unsigned char blink );

extern void init_para();
extern void init_index_para();
extern void delay1( unsigned int us);

extern void CAN_AUTO_RESET(void);
extern void can_rx_from_DCT_amount_feedback(unsigned char *buf);
extern void can_rx_from_DCT_bite_feedback(unsigned char *buf);
extern void can_rx_from_SWJ_Action_Command(void);
extern void can_rx_from_SWJ_Zijian_Command(void);
extern void Comunication_BITE_to_SWJ(void);

extern bit Get_DCT_Input_State(unsigned char  number) ; 	        // 获取底层驱动板控制指令的状态
extern void Set_DCT_Input_State(unsigned char  number, bit state); // 设置或复位底层驱动板的输入状态
extern void Set_DCT_OK_Result(unsigned char  number, bit state);   // 设置或复位底层驱动板的动作OK状态
extern void Set_DCT_OK_Action(unsigned char  number, bit state);   // 设置或复位底层驱动板的动作结束状态

extern void CAN2_Data_Collect_Process_Programme(void);                 //底层发药板数据反馈子程序
extern void Medica_Auto_Transportation_Programme(void);                //药品运输系统自动操作
extern void Send_CO_Begin_Programme(void);
//
extern void delay1(unsigned int us);
extern void delay_ms(unsigned int ms);
//向上位机发送CO程序段
extern void Lan_backinfor(unsigned char m_pra1, unsigned char m_pra2);
extern void Lan_backalarm(unsigned char ID, unsigned char MUM, unsigned char m_alarm1);
//
extern void Can_CHECKinfor(unsigned char ID, unsigned char command); //used 3
extern void ElectromagnetCRTL(void);//used 0
extern void YaolanStepRun(void);//used 0
extern void Yaolantisheng(void);//used 0
//
extern void Yaopingtishengleft(void);//used 1
extern void Yaopingtisheng(void);//used 1
extern void ConveyingLineMovement(void);//used 1
extern void ConveyingrightLineMovement(void);//used 1
extern void Susongxiangtuiyao(void);//used 1
extern void Susongxiangrighttuiyao(void);//used 1
//added start
extern void FZSSXRightFZ(unsigned char signal);//used 1
extern void FZSSXLeftFZ(unsigned char signal);//used 1
void ConveyingLongSlow_Start(unsigned char dir);
void ConveyingLongSlow_Stop();
//added end
/*CAN3发送地址与对象参照
*1：
*2：右侧提升机构的电机驱动板CAN ID
*3：地面输送线的电机驱动板CAN ID
*4：左侧提升机构的电机驱动板CAN ID
*/
//
extern void init_GM8123(void);
extern void init_uart();
//extern void Uart0Send(void);
extern void Uart1Send(void);
extern void Uart0Send(unsigned char addr, unsigned char *buf, unsigned char bufsize, bit  sumflag);
//extern void Uart1Send(unsigned char addr, unsigned char *buf, unsigned char bufsize, bit  sumflag);

//-----------------------------------zk.c--------------------------------------
// 全局变量
//-----------------------------------------------------------------------------
//定义定时器的软件计数器
extern unsigned char  T0Counter1;  //运行指示灯
extern unsigned char  T0Counter2;  //CAN 发送超时计数器
extern unsigned int   T0Counter3;  //BITE反馈给上位机
extern unsigned char  T0Counter4;  //对OK信息的处理和上报

extern xdata unsigned char  T0Counter12;  //串口控制器使用1
extern xdata unsigned char  T0Counter13;  //串口控制器使用2
extern xdata unsigned int   T0Counter14;  //电磁铁发药结束后，延时用
extern xdata unsigned int   T0Counter15;  //电磁铁发药结束后，延时用

extern unsigned char Group_index_DCT;      //电磁铁动作批次号也即数据序列
extern unsigned char Board_Address_Number;  //自检用

extern unsigned char ALL_Action_OK_Sign ;
extern unsigned char ALL_Action_OK_Sign_last;
extern unsigned char ALL_Result_OK_Sign ;

extern unsigned char OK_Sign_TX_Amount;   //0k信号的发送次数
extern unsigned char temppage;

extern unsigned char Zijian_Sign ;     //自检标志，0xaa退出自检模式，0X55进入自检模式

//extern xdata SYSDAT  SysData;
extern xdata  unsigned char  databuf[8];

extern xdata unsigned char LED_Address;
extern xdata unsigned char LED_BUF;

extern bit Command_Finish_Sign ;   //指令结束和开始标志
extern bit Command_Begin_Sign ;    //传送带启动指令
extern bit Servo_Begin_Sign ;      //伺服触发标志
extern bit  Answer_ok;
extern unsigned char Board_ID;

extern unsigned int  Proce_Count;            //自动操作，动作到位后的等待时间
extern unsigned char Procession1;            //自动操作步骤代码
extern unsigned char Run_modeFayaoL;
extern unsigned char Run_modeFayaoR;
extern unsigned char Run_mode;

extern unsigned char SNSORSTATUS1;
/*SNSORSTATUS1各位定义：
**bit0:左闸门原点建立标志
**bit1:左提升机构原点标志
**bit4:输送线定位运行结束标志
**bit5:输送线推药结束标志
*/
extern unsigned char SNSORSTATUS2;
/*SNSORSTATUS2各位定义：
**bit0:右闸门原点建立标志
**bit1:右提升机构原点标志
**bit4:输送线定位运行结束标志
**bit5:输送线推药结束标志
*/
extern unsigned char MAINRUNSTATUSL;
/*MAINRUNSTATUSL各位定义：
**bit0:输送线左定位运行状态位
**bit4:输送线左推药状态位
**bit5:左提升机构动作状态位
*/
extern unsigned char MAINRUNSTATUSL1;
/*MAINRUNSTATUSL1各位定义：
**bit0:
**bit1:
*/
extern unsigned char RUNSTATUS1;
extern unsigned char RUNSTATUS2;



extern unsigned char Runbiaoji1;
extern unsigned char Runbiaoji2;
extern bit Tansflag;

extern unsigned char Runbiaoji3;
extern unsigned char Runbiaoji4;
extern unsigned char FAYAOinput_pra[4];
extern unsigned char communication_step;
extern bit  ONCEFLAG;
extern bit  Alarm_stepflagL;  //故障点标志位
extern bit  Alarm_stepflagR;  //故障点标志位
extern bit  ALARM_ONCEFLAGL;
extern bit  ALARM_ONCEFLAGR;
extern bit  Servo_left_Begin_Sign;
extern bit  Servo_right_Begin_Sign;
extern bit  FaYao_Prosetion_Sign;
extern bit  Active_Data_Begin_Sign_last;
extern bit  Active_Data_Begin_Sign;

extern xdata Servomotordisplace Servodisplace;
extern xdata  float m_floatServodisplace;
extern xdata  unsigned int m_intServodisplace;
extern xdata  unsigned char Run_REAFY1;
extern xdata  unsigned char Run_REAFY2;
extern xdata  unsigned char Run_ALARM1;
extern xdata  unsigned char Run_ALARM2;
extern xdata  unsigned char Mode_ALARM1;
extern xdata  unsigned char Mode_ALARM2;
extern xdata  unsigned char Mode_ALARM3;
extern xdata  unsigned char Mode_ALARM4;
extern xdata  unsigned char Mode_adress1;
extern xdata  unsigned char Mode_adress2;
extern xdata  unsigned char Mode_adress3;
extern xdata  unsigned char Mode_adress4;
extern xdata  unsigned char Basket_stat;
extern xdata  unsigned char strat_flag;
extern xdata  unsigned char CAN_index;
extern xdata  unsigned int  AUTO_countimerL;
extern xdata  unsigned int  AUTO_countimerR;

//------------------------------------zk.c-------------------------------------
// 外部全局变量
//-----------------------------------------------------------------------------
//extern xdata CAN1RECBUF1  CAN1RXbuffer1;
//extern xdata CAN1TRANSBUF1   CAN1TXbuffer1;

extern xdata CAN2RECBUF1  CAN2RXbuffer1;
extern xdata CAN2RECBUF1  CAN2RXbuffer2;
extern xdata CAN2RECBUF1  CAN2RXbuffer3;

extern xdata CAN2TRANSBUF1   CAN2TXbuffer1;
extern xdata CAN2TRANSBUF2   CAN2TXbuffer2;

extern xdata CAN3TRANSBUF1  CAN3TXbuffer1;
extern xdata CAN3RECBUF1  CAN3RXbuffer1;

//extern bit Can1NewDataA;
extern bit Can2NewData1;
extern bit Can2NewData2;
extern bit Can2NewData3;
extern bit Can3NewData1;

//extern unsigned char CAN1FaultCounter;
extern unsigned char CAN2FaultCounter;
extern unsigned char CAN3FaultCounter;





//-----------------------------------CAN1.C---------------------------------------
// 函数原型
//-----------------------------------------------------------------------------
extern void clear_msg_objects (void);
extern void init_msg_object_TX (char MsgNum, unsigned int id);
extern void init_msg_object_RX (char MsgNum, unsigned int id);
extern void start_CAN (void);
extern void can1_transmit(char MsgNum, unsigned char *buf);
extern void can1_receive (char MsgNum, unsigned char *buf);
extern void init_can1 (void);


//-----------------------------------CAN2.C---------------------------------------
// 函数原型
//-----------------------------------------------------------------------------
extern void init_can2 (void);
extern void INT0_ISR (void)  ;
extern void can2_transmit(unsigned int id, unsigned char *buf);
extern void can2_receive (unsigned char *buf);


//-----------------------------------CAN3.C---------------------------------------
// 函数原型
//-----------------------------------------------------------------------------
extern void init_can3 (void);
extern void INT1_ISR (void)  ;
extern void can3_transmit(unsigned int id, unsigned char *buf);
extern void can3_receive (unsigned char *buf);


////---------------------------------uart.c-----------------------------------------
//// 全局变量
////-----------------------------------------------------------------------------
//extern unsigned char RecPointer0;
//extern bit RecFlag0;

//extern unsigned char RecPointer1;
//extern bit RecFlag1;

//// 定义串口接收缓冲
//extern xdata  RECBUF0 RxBuf0;
//extern xdata  RECBUF1 RxBuf1;

//extern xdata TRANSBUF0 TxBuf0;

////定义接收缓冲器
//extern xdata unsigned char Rec0[RECBUF0SIZE];
//extern xdata unsigned char Rec1[RECBUF1SIZE];

//extern bit TI0Flag;  //发送结束标志
//extern bit TI1Flag;  //发送结束标志

//extern unsigned char CommandByte1;


////---------------------------------UART0.c-----------------------------------------
//// 全局变量
////-----------------------------------------------------------------------------
extern TRANSBUF1 TransBuf0;
extern RECBUF1  RecBuf0;
extern bit UART0_Refresh;

////---------------------------------UART0.c-----------------------------------------
//// 全局变量
////-----------------------------------------------------------------------------
extern TRANSBUF1 TransBuf1;
extern RECBUF1  RecBuf1;
extern bit UART1_Refresh;









