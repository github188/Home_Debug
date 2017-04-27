#define		UINT		unsigned int
#define		R_MaxLen		53    //最大接收字节数
#define		T_MaxLen		92//41	//最大发送字节数
#define		Delay_Times	20000		//延时循环次数
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
sbit  LED1=P1^5;                  //状态指示灯
sbit  LED2=P1^6;                  //状态指示灯
sbit  LED3=P1^7;                  //状态指示灯

sbit  LED4=P2^0;                  //状态指示灯
sbit  LED5=P2^1;                  //状态指示灯
sbit  LED6=P2^2;                  //状态指示灯
sbit  LED7=P2^3;                  //状态指示灯
sbit  LED8=P2^4;                  //状态指示灯



   
sbit RE1=P0^3;
sbit DE1=P0^4;
             
sbit  MONITOR_INPUT1=P3^0;
sbit  MONITOR_INPUT2=P3^1;  
sbit  MONITOR_INPUT3=P3^2;
sbit  MONITOR_INPUT4=P3^3;
sbit  MONITOR_INPUT5=P3^4;
sbit  MONITOR_INPUT6=P3^5;
sbit  MONITOR_INPUT7=P3^6;
sbit  MONITOR_INPUT8=P6^0;
sbit  MONITOR_INPUT9=P6^1;
sbit  MONITOR_INPUT10=P6^2;
sbit  MONITOR_INPUT11=P6^3;
sbit  MONITOR_INPUT12=P6^4;
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
typedef struct    
{
	unsigned char  head ;  
	unsigned char  data1;
	unsigned char  sum;
}TRANSMIT1;            

typedef union 
{ 
	TRANSMIT1 trans;
	unsigned char buf[TRANSBUF1SIZE];
}TRANSBUF1;
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







 