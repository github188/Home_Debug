//------------------------------------------------------------------------------
// CAN1.c
//------------------------------------------------------------------------------
#include <c8051f040.h>                          // SFR declarations
#include "Yaolansample.h"
// CAN Protocol Register Index for CAN0ADR, from TABLE 18.1 of the C8051F040
// datasheet
////////////////////////////////////////////////////////////////////////////////
#define CANCTRL            0x00                 //Control Register
#define CANSTAT            0x01                 //Status register
#define ERRCNT             0x02                 //Error Counter Register
#define BITREG             0x03                 //Bit Timing Register
#define INTREG             0x04                 //Interrupt Low Byte Register
#define CANTSTR            0x05                 //Test register
#define BRPEXT             0x06                 //BRP Extension         Register
////////////////////////////////////////////////////////////////////////////////
//IF1 Interface Registers
////////////////////////////////////////////////////////////////////////////////
#define IF1CMDRQST         0x08                 //IF1 Command Rest      Register
#define IF1CMDMSK          0x09                 //IF1 Command Mask      Register
#define IF1MSK1            0x0A                 //IF1 Mask1             Register
#define IF1MSK2            0x0B                 //IF1 Mask2             Register
#define IF1ARB1            0x0C                 //IF1 Arbitration 1     Register
#define IF1ARB2            0x0D                 //IF1 Arbitration 2     Register
#define IF1MSGC            0x0E                 //IF1 Message Control   Register
#define IF1DATA1           0x0F                 //IF1 Data A1           Register
#define IF1DATA2           0x10                 //IF1 Data A2           Register
#define IF1DATB1           0x11                 //IF1 Data B1           Register
#define IF1DATB2           0x12                 //IF1 Data B2           Register
////////////////////////////////////////////////////////////////////////////////
//IF2 Interface Registers
////////////////////////////////////////////////////////////////////////////////
#define IF2CMDRQST         0x20                 //IF2 Command Rest      Register
#define IF2CMDMSK          0x21                 //IF2 Command Mask      Register
#define IF2MSK1            0x22                 //IF2 Mask1             Register
#define IF2MSK2            0x23                 //IF2 Mask2             Register
#define IF2ARB1            0x24                 //IF2 Arbitration 1     Register
#define IF2ARB2            0x25                 //IF2 Arbitration 2     Register
#define IF2MSGC            0x26                 //IF2 Message Control   Register
#define IF2DATA1           0x27                 //IF2 Data A1           Register
#define IF2DATA2           0x28                 //IF2 Data A2           Register
#define IF2DATB1           0x29                 //IF2 Data B1           Register
#define IF2DATB2           0x2A                 //IF2 Data B2           Register
////////////////////////////////////////////////////////////////////////////////
//Message Handler Registers
////////////////////////////////////////////////////////////////////////////////
#define TRANSREQ1          0x40                 //Transmission Rest1 Register
#define TRANSREQ2          0x41                 //Transmission Rest2 Register
#define NEWDAT1            0x48                 //New Data 1            Register
#define NEWDAT2            0x49                 //New Data 2            Register
#define INTPEND1           0x50                 //Interrupt Pending 1   Register
#define INTPEND2           0x51                 //Interrupt Pending 2   Register
#define MSGVAL1            0x58                 //Message Valid 1       Register
#define MSGVAL2            0x59                 //Message Valid 2       Register
//-----------------------------------------------------------------------------
// C8051F040的SFR定义
//-----------------------------------------------------------------------------
sfr16 CAN0DAT = 0xD8;

//-----------------------------------------------------------------------------
// 全局变量
//-----------------------------------------------------------------------------
xdata  CANRECDATBUF	CANRXBUF_ZKB;
xdata  CANRECDATBUF	CANRXBUF_X;
xdata  CANRECDATBUF	CANRXBUF_Y;
xdata  CANRECDATBUF	CANRXBUF_Z;
xdata  CANRECDATBUF	CANRXBUF_H;
bit CANRefresh_ZKB=NO; 
bit CANRefresh_X=NO; 
bit CANRefresh_Y=NO; 
bit CANRefresh_Z=NO; 
bit CANRefresh_H=NO; 



xdata unsigned char CANaddress1=0;
xdata unsigned char CANINDEX=0;






//-----------------------------------------------------------------------------
// 外部全局变量
//-----------------------------------------------------------------------------
extern idata unsigned char Runmode;
extern xdata unsigned  char station[5];
extern xdata unsigned  char Servopara[107];
extern xdata unsigned  char CANTRASTEMINFOR[44];
extern xdata unsigned char T0Counter2;
extern xdata unsigned int  T0Counter3;  
extern xdata unsigned char IDSET,BPSSET;
extern xdata unsigned char Shaft1_Run_mode;
extern xdata unsigned long  Shaft1_aim_pulse; 
extern xdata unsigned long  Pulse1_Cnt;
extern bit Receive485_command_finished; 
extern bit CANINFOR_FLAG;
//extern bit Step_FLAG;
extern xdata unsigned  char Servoparastation;
extern xdata unsigned  long Servoparadisplace;
extern xdata unsigned  long Servoparaspeed;
extern xdata unsigned  long Servoparaaddtime;
extern xdata unsigned  long Servoparasubtime;
extern xdata systemcrtl system_crtl;
extern xdata unsigned char  nCANcomand;    //CAN命令字
extern xdata unsigned int Timerdelay1;
extern xdata unsigned int Timerdelay2;
extern void Send_to_Motordriver_CTL(unsigned char node_id,unsigned char command );
extern void Send_to_Motordriver_Position_CTL(unsigned char node_id,unsigned char Mode,
                      long int Position_Value,long int Speed_Value,long int Add_Speed_Time);
extern void send_to_motor(void);
extern void send_to_motor2(void);
extern void delay_ms(unsigned int ms);
extern void write_to_flash(void);

//-----------------------------------------------------------------------------
// 函数原型
//-----------------------------------------------------------------------------
void clear_msg_objects (void);
void init_msg_object_TX (char MsgNum,unsigned int id);
void init_msg_object_RX (char MsgNum,unsigned int id);
void start_CAN (void);
void can1_transmit(char MsgNum,unsigned char *buf);
void can1_receive (char MsgNum,unsigned char *buf);
void init_can1_1 (void);  
//外部函数
////////////////////////////////////////////////////////////////////////////////
void init_can1_1 (void) 
{
////////////////////////////////////////////////////////////////////////////////
// Configure CAN communications
//
// IF1 used for procedures calles by main program
// IF2 used for interrupt service procedure can1_receive
//
////////////////////////////////////////////////////////////////////////////////
char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page
  SFRPAGE  = CONFIG_PAGE;        //Port SFR's on Configuration page
  XBR3     |= 0x80;     // Configure CAN TX pin (CTX) as push-pull digital output
  EA=0;
  // Clear CAN RAM
  clear_msg_objects();
	// Initialize message object to transmit data
  init_msg_object_TX(TX_MSGNUM_ZKB,(IDSET + 64));         // 发出本地数据
	// Initialize message object to receive data
  init_msg_object_RX(RX_MSGNUM_ZKB,IDSET);                // 接收主控板动作指令数据
  init_msg_object_RX(RX_MSGNUM_Y_Shaft,98);               // 接收Y轴动作OK反馈指令数据
  init_msg_object_RX(RX_MSGNUM_ZKB_Union,Address_Union);  // 接收主控板始发包或结束包数据
  init_msg_object_RX(RX_MSGNUM_ZKB_1,35);                 // 接收主控板动作指令数据
  // Enable CAN interrupts in CIP-51
  EIE2 |= 0x20;
  // 设置CAN中断优先级高
  EIP2|= 0x20;
  //Function call to start CAN
  start_CAN();
  //Global enable 8051 interrupts
  EA = 1;
  SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
}



//Clear Message Objects
void clear_msg_objects (void)
{
  unsigned char i;
  char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page
  SFRPAGE  = CAN0_PAGE;
  CAN0ADR  = IF1CMDMSK;    // Point to Command Mask Register 1
  CAN0DATL = 0xFF;         // Set direction to WRITE all IF registers to Msg Obj
  for (i=1;i<33;i++)
  {
      CAN0ADR = IF1CMDRQST; // Write blank (reset) IF registers to each msg obj
      CAN0DATL = i;
  }
  SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
}

//Initialize Message Object for RX
void init_msg_object_RX (char MsgNum,unsigned int id)
{
  unsigned int temp;
  char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page
  SFRPAGE  = CAN0_PAGE;
  CAN0ADR  = IF2CMDMSK;  // Point to Command Mask 1
  CAN0DAT  = 0x00BB;     // Set to WRITE, and alter all Msg Obj except ID MASK
                        
  CAN0ADR  = IF2ARB1;    // Point to arbitration1 register
  CAN0DAT  = 0x0000;     // Set arbitration1 ID to "0"
  temp=id<<2;
  temp&=0x1fff;
  temp|=0x8000;
  CAN0DAT  = temp;     // Arb2 high byte:Set MsgVal bit, no extended ID,
                         // Dir = RECEIVE
  CAN0DAT  = 0x1488;//0x488;    // Msg Cntrl: set RXIE,
                       // remote frame function disabled,
					   //接收产生中断
  CAN0ADR  = IF2CMDRQST; // Point to Command Request reg.
  CAN0DATL = MsgNum;     // Select Msg Obj passed into function parameter list
                         // --initiates write to Msg Obj
  // 3-6 CAN clock cycles to move IF register contents to the Msg Obj in CAN RAM
  SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
}

//Initialize Message Object for TX
void init_msg_object_TX (char MsgNum,unsigned int id)
{
  unsigned int temp;
  char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page
  SFRPAGE = CAN0_PAGE;
  CAN0ADR = IF1CMDMSK;  // Point to Command Mask 1
  CAN0DAT = 0x00B3;     // Set to WRITE, & alter all Msg Obj except ID MASK bits
  CAN0ADR = IF1ARB1;    // Point to arbitration1 register
  CAN0DAT = 0x0000;     // Set arbitration1 ID to highest priority
  temp=id<<2;
  temp&=0x1fff;
  temp|=0xa000;
  CAN0DAT = temp;     // Autoincrement to Arb2 high byte:
                        // Set MsgVal bit, no extended ID, Dir = WRITE
  CAN0DAT = 0x1088;//0x0088;     // Msg Cntrl: DLC = 8, 
                        //remote frame function not enabled,
						//发送不产生中断
  CAN0ADR = IF1CMDRQST; // Point to Command Request reg.
  CAN0DAT = MsgNum;     // Select Msg Obj passed into function parameter list
                        // --initiates write to Msg Obj
  // 3-6 CAN clock cycles to move IF reg contents to the Msg Obj in CAN RAM.
  SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
}

void start_CAN (void)
{
  /* Calculation of the CAN bit timing :

  System clock        f_sys = 11.0592 MHz.
  System clock period t_sys = 1/f_sys = 90.422454 ns.
  CAN time quantum       tq = t_sys (at BRP = 0)

  Desired bit rate is 1 MBit/s, desired bit time is 1000 ns.
  Actual bit time = 11 tq = 994.642ns 
  Actual bit rate is 1.00539 MBit/s = Desired bit rate+0.5381%

  CAN bus length = 10 m, with 5 ns/m signal delay time.
  Propagation delay time : 2*(transceiver loop delay + bus line delay) = 400 ns
  (maximum loop delay between CAN nodes)

  Prop_Seg = 5 tq = 452 ns ( >= 400 ns).
  Sync_Seg = 1 tq

  Phase_seg1 + Phase_Seg2 = (11-6) tq = 5 tq
  Phase_seg1 <= Phase_Seg2,  =>  Phase_seg1 = 2 tq and Phase_Seg2 = 3 tq
  SJW = (min(Phase_Seg1, 4) tq = 2 tq

  TSEG1 = (Prop_Seg + Phase_Seg1 - 1) = 6
  TSEG2 = (Phase_Seg2 - 1)            = 2
  SJW_p = (SJW - 1)                   = 1

  Bit Timing Register = BRP + SJW_p*0x0040 = TSEG1*0x0100 + TSEG2*0x1000 = 2640

  Clock tolerance df :

  A: df < min(Phase_Seg1, Phase_Seg2) / (2 * (13*bit_time - Phase_Seg2))
  B: df < SJW / (20 * bit_time)

  A: df < 2/(2*(13*11-3)) = 1/(141-3) = 1/138 = 0.7246%
  B: df < 2/(20*11)                   = 1/110 = 0.9091%

  Actual clock tolerance is 0.7246% - 0.5381% = 0.1865% (no problem for quartz)
  
  注：SJW越长，抗噪能力越强，且Prop_Seg+Phase_Seg1越长，抗噪能力越强。SJW不能大于
  Phase_Seg1和Phase_Seg2中如何一个值。
  详见c8051f040.pdf
  */
  char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page
  SFRPAGE  = CAN0_PAGE;
  CAN0CN  |= 0x41;       // Configuration Change Enable CCE and INIT
  CAN0ADR  = BITREG   ;  // Point to Bit Timing register

  if(BPSSET==3)
	{
		CAN0DAT  = 0x2640; // (at BRP = 0) bps=1.00539MHz
	}
  else if(BPSSET==2)
	{
		CAN0DAT  = 0x2641; // (at BRP = 1) bps=502693Hz
	}
  else if(BPSSET==1)
	{
		CAN0DAT  = 0x2643; // (at BRP = 3) bps=251347Hz
	}
  else
	{
		CAN0DAT  = 0x2647; // (at BRP = 7) bps=125673Hz
	}
 
  CAN0CN =0x0A; //模块中断开启//错误中断开启
                 // Clear CCE and INIT bits, starts CAN state machine
  SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
}


void can1_transmit(char MsgNum,unsigned char *buf )
{
  unsigned char i;
  char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page
  SFRPAGE  = CAN0_PAGE;  // IF1 already set up for TX

  T0Counter2=0;
  CAN0ADR = TRANSREQ1;
  while ((CAN0DAT & (0x0001<<(MsgNum-1)) ) != 0) //消息号1~16没有发送完，等待
  {
       if(T0Counter2>=3)
	   {
	       return; //延时30ms  超时退出
       }
  }
  EA=0;
  CAN0ADR  = IF1CMDMSK;  // Point to Command Mask 1
  CAN0DAT  = 0x0087;     // Config to WRITE to CAN RAM, write data bytes,
                         // set TXrqst/NewDat, Clr IntPnd
  CAN0ADR  = IF1DATA1;   // Point to 1st byte of Data Field
  for(i=0 ; i<8 ; i+=2)
  {
     CAN0DATH = buf[i+1];
     CAN0DATL = buf[i];
  }
  CAN0ADR  = IF1CMDRQST; // Point to Command Request Reg.
  CAN0DATL = MsgNum;     // Move new data for TX to Msg Obj "MsgNum"
  EA=1;
  SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
}


// Receive Data from the IF2 buffer
void can1_receive (char MsgNum,unsigned char *buf)
{
  unsigned char i;
  char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page
  SFRPAGE  = CAN0_PAGE; 
  CAN0ADR  = IF2CMDMSK;  // Point to Command Mask 1
  CAN0DATL  = 0x0F;   
  EA=0;
  CAN0ADR  = IF2CMDRQST;// Point to Command Request Reg.
  CAN0DATL = MsgNum;    // Move new data for RX from Msg Obj "MsgNum"
                        // Move new data to a
 
  CAN0ADR  = IF2DATA1;  // Point to 1st byte of Data Field
  for(i=0 ; i<8 ; i+=2)
  {
     buf[i+1]=CAN0DATH ;
     buf[i]=CAN0DATL ;
  }
  EA=1;
  SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
}

////////////////////////////////////////////////////////////////////////////////
//Interrupt Service Routine
////////////////////////////////////////////////////////////////////////////////
void ISRname (void) interrupt 19  
{
  unsigned char status;
  unsigned int  intstate;
  unsigned char temp;
  temp=SFRPAGE;
  SFRPAGE  = CAN0_PAGE; 
  status = CAN0STA;
   if ((status&0x10) != 0)
    {                            // RxOk is set, interrupt caused by reception
      CAN0STA = (CAN0STA&0xEF)|0x07;         // Reset RxOk, set LEC to NoChange
      /* read message number from CAN INTREG */
      CAN0ADR  = INTPEND1;
	  intstate=CAN0DAT;
	  if( intstate&(0x0001<<(RX_MSGNUM_ZKB-1)) )        // 接收主控板动作指令
	   {
				
		can1_receive (RX_MSGNUM_ZKB,CANRXBUF_ZKB.buf);
		switch(CANRXBUF_ZKB.buf[1])
		{
			
			
			
			case 0x20:
			CANaddress1=CANRXBUF_ZKB.buf[0];
			nCANcomand=CANRXBUF_ZKB.buf[1];
                        CANINDEX=CANRXBUF_ZKB.buf[2];
                        Servopara[28]=CANRXBUF_ZKB.buf[5];
                        Servopara[29]=CANRXBUF_ZKB.buf[6];
                        Timerdelay1=Servopara[28];
	                Timerdelay1=Timerdelay1<<8;
	                Timerdelay1=Timerdelay1+Servopara[29];
                        Runmode=0;//手动运行模式
                        CANINFOR_FLAG=YES;
			break;
			case 0x21:
			CANaddress1=CANRXBUF_ZKB.buf[0];
			nCANcomand=CANRXBUF_ZKB.buf[1];
                        CANINDEX=CANRXBUF_ZKB.buf[2];
                        Servopara[30]=CANRXBUF_ZKB.buf[5];
                        Servopara[31]=CANRXBUF_ZKB.buf[6];
                        Timerdelay2=Servopara[30];
	                Timerdelay2=Timerdelay1<<8;
	                Timerdelay2=Timerdelay1+Servopara[31];
                        Runmode=0;//手动运行模式
                        CANINFOR_FLAG=YES;
			break;
			case 0x22:
			CANaddress1=CANRXBUF_ZKB.buf[0];
			nCANcomand=CANRXBUF_ZKB.buf[1];
                        CANINDEX=CANRXBUF_ZKB.buf[2];
                        Servopara[CANINDEX*9+0]=CANRXBUF_ZKB.buf[3];
                        Servopara[CANINDEX*9+1]=CANRXBUF_ZKB.buf[4];
                        Servopara[CANINDEX*9+2]=CANRXBUF_ZKB.buf[5];
                        Servopara[CANINDEX*9+3]=CANRXBUF_ZKB.buf[6];
                        Runmode=0;//手动运行模式
                        CANINFOR_FLAG=YES;
			break;
			case 0x23:
			CANaddress1=CANRXBUF_ZKB.buf[0];
			nCANcomand=CANRXBUF_ZKB.buf[1];
                        CANINDEX=CANRXBUF_ZKB.buf[2];
                        Servopara[CANINDEX*9+4]=CANRXBUF_ZKB.buf[5];
                        Servopara[CANINDEX*9+5]=CANRXBUF_ZKB.buf[6];
                        Runmode=0;//手动运行模式
                        CANINFOR_FLAG=YES;
			break;
			case 0x24:
			CANaddress1=CANRXBUF_ZKB.buf[0];
			nCANcomand=CANRXBUF_ZKB.buf[1];
                        CANINDEX=CANRXBUF_ZKB.buf[2];
                        Servopara[CANINDEX*9+6]=CANRXBUF_ZKB.buf[5];
                        Servopara[CANINDEX*9+7]=CANRXBUF_ZKB.buf[6];
                        Runmode=0;//手动运行模式
                        CANINFOR_FLAG=YES;
			break;
			case 0x25:
			CANaddress1=CANRXBUF_ZKB.buf[0];
			nCANcomand=CANRXBUF_ZKB.buf[1];
                        CANINDEX=CANRXBUF_ZKB.buf[2];
                        Servopara[CANINDEX*9+8]=CANRXBUF_ZKB.buf[6];
                        Runmode=0;//手动运行模式
                        CANINFOR_FLAG=YES;
			break;
			
			
			case 0x30://读取所有工位信息
			CANaddress1=CANRXBUF_ZKB.buf[0];
			nCANcomand=CANRXBUF_ZKB.buf[1];
                        CANINDEX=CANRXBUF_ZKB.buf[2];
                        Runmode=0;//手动运行模式
			break;
			
			
			case 0x10://0X66
			CANaddress1=CANRXBUF_ZKB.buf[0];
			nCANcomand=CANRXBUF_ZKB.buf[1];
                        CANINDEX=CANRXBUF_ZKB.buf[2];
                        Runmode=0;//手动运行模式
			break;
			
			case 0x11://0X55
			CANaddress1=CANRXBUF_ZKB.buf[0];
			nCANcomand=CANRXBUF_ZKB.buf[1];
                        CANINDEX=CANRXBUF_ZKB.buf[2];
                        Runmode=0;//手动运行模式
			break;
			case 0x12://0X55
			CANaddress1=CANRXBUF_ZKB.buf[0];
			nCANcomand=CANRXBUF_ZKB.buf[1];
                        CANINDEX=CANRXBUF_ZKB.buf[2];
                        Runmode=0;//手动运行模式
			break;
			
			case 0x13://0X66
			CANaddress1=CANRXBUF_ZKB.buf[0];
			nCANcomand=CANRXBUF_ZKB.buf[1];
                        CANINDEX=CANRXBUF_ZKB.buf[2];
                        Runmode=0;//手动运行模式
			break;
			
			case 0x14://0X55
			CANaddress1=CANRXBUF_ZKB.buf[0];
			nCANcomand=CANRXBUF_ZKB.buf[1];
                        CANINDEX=CANRXBUF_ZKB.buf[2];
                        Runmode=0;//手动运行模式
			break;
			case 0x15://0X55
			CANaddress1=CANRXBUF_ZKB.buf[0];
			nCANcomand=CANRXBUF_ZKB.buf[1];
                        CANINDEX=CANRXBUF_ZKB.buf[2];
                        Runmode=0;//手动运行模式
			break;
			case 0x16://测试
			CANaddress1=CANRXBUF_ZKB.buf[0];
			nCANcomand=CANRXBUF_ZKB.buf[1];
                        CANINDEX=CANRXBUF_ZKB.buf[2];
                        Runmode=0;//手动运行模式
			break;
			case 0x17://0X66
			CANaddress1=CANRXBUF_ZKB.buf[0];
			nCANcomand=CANRXBUF_ZKB.buf[1];
                        CANINDEX=CANRXBUF_ZKB.buf[2];
                        Runmode=0;//手动运行模式
			break;
			





			case 0xD2://0X55
			CANaddress1=CANRXBUF_ZKB.buf[0];
			nCANcomand=CANRXBUF_ZKB.buf[1];
                        CANINDEX=CANRXBUF_ZKB.buf[2];
                        Runmode=0;//手动运行模式
			break;
                        case 0xD3:
			CANaddress1=CANRXBUF_ZKB.buf[0];
			nCANcomand=CANRXBUF_ZKB.buf[1];
                        CANINDEX=CANRXBUF_ZKB.buf[2];
                        Runmode=0;//手动运行模式
			break;
			case 0xD4://原点复归
			CANaddress1=CANRXBUF_ZKB.buf[0];
			nCANcomand=CANRXBUF_ZKB.buf[1];
                        CANINDEX=CANRXBUF_ZKB.buf[2];
                        Runmode=0;//手动运行模式
			break;
			
			case 0xFE:
			if(system_crtl.AUTOsystem_command!=0)//误操作退出
		        {
			CANTRASTEMINFOR[40]=CANTRASTEMINFOR[40]|0x01;
		        }
		        else
		        {
			CANaddress1=CANRXBUF_ZKB.buf[0];
			nCANcomand=CANRXBUF_ZKB.buf[1];
                        CANINDEX=CANRXBUF_ZKB.buf[2];
                        system_crtl.AUTOsystem_command=1;
                        CANTRASTEMINFOR[36]=CANTRASTEMINFOR[36]&0X08;//
		        CANTRASTEMINFOR[37]=0x00;
		        CANTRASTEMINFOR[38]=0x00;
                        CANTRASTEMINFOR[39]=0x00;
                        T0Counter3=51;//尽快的定时发送清除状态类表
                        Runmode=1;//自动运行
                        }
			break;
			
			case 0xF0:
			CANaddress1=CANRXBUF_ZKB.buf[0];
			nCANcomand=CANRXBUF_ZKB.buf[1];
                        CANINDEX=CANRXBUF_ZKB.buf[2];
                        Runmode=1;//自动运行模式
			break;
			case 0xF1:
			CANaddress1=CANRXBUF_ZKB.buf[0];
			nCANcomand=CANRXBUF_ZKB.buf[1];
                        CANINDEX=CANRXBUF_ZKB.buf[2];
                        Runmode=1;//自动运行模式
			break;
			case 0xF2:
			CANaddress1=CANRXBUF_ZKB.buf[0];
			nCANcomand=CANRXBUF_ZKB.buf[1];
                        CANINDEX=CANRXBUF_ZKB.buf[2];
                        Runmode=1;//自动运行模式
			break;
			default:
			break;
		}	
	   }
	   
	  if( intstate&(0x0001<<(RX_MSGNUM_ZKB_1-1)) )        // 接收主控板动作指令
	   {
			{
				can1_receive (RX_MSGNUM_ZKB_1,CANRXBUF_ZKB.buf);
				CANRefresh_ZKB=YES;
			}
	   }

	  if( intstate&(0x0001<<(RX_MSGNUM_ZKB_Union-1)) )  // 接收主控板公共通道数据
	  {
			{
				can1_receive (RX_MSGNUM_ZKB_Union,CANRXBUF_ZKB.buf);
				CANRefresh_ZKB=YES;
			}
	  }

	  if( intstate&(0x0001<<(RX_MSGNUM_Y_Shaft-1)) )    // 接收Y轴通道数据
	  {
			{
				can1_receive (RX_MSGNUM_Y_Shaft,CANRXBUF_Y.buf);
				CANRefresh_Y=YES;
			}
	  }

	  LED8=YES;   //故障指示灭 
    }
    if ((status&0x08) != 0)
    {                        // TxOk is set, interrupt caused by transmision
      CAN0STA = (CAN0STA&0xF7)|0x07;        // Reset TxOk, set LEC to NoChange
    }
    if (((status&0x07) != 0)&&((status&0x07) != 7))
    {                           // Error interrupt, LEC changed
      /* error handling ? */
      CAN0STA = CAN0STA|0x07;              // Set LEC to NoChange
      if( (status&0x80)!=0)     //总线关闭
	  {
	    CAN0CN  &= 0xFE;        //恢复正常模式
	  } 

	  LED8=NO ;   //故障指示
    }
    SFRPAGE=temp;
}

