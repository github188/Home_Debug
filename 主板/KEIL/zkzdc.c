/*  
带有网络接口的发药系统主控板程序  

  2016.02.14 将网络主板升级为装机用程序，除去不必要的程序段
  在以前主控板程序的基础上进行升级
  2016.02.15 增加药品输送及其他辅助控制程序段 
  网络通讯也已经重新测试完毕
  2016.02.24 CAN2，CAN3出现故障，但原先的老版本程序，能运行。现已更换成老版本。 
  
*/
//-----------------------------------------------------------------------------
// 包含文件
//-----------------------------------------------------------------------------
#include <c8051f040.h>                 // SFR declaration
#include <zk.h> 
#include <math.h>

//-----------------------------------------------------------------------------
// C8051F040的SFR定义
//-----------------------------------------------------------------------------
sfr16 ADC0 = 0xbe;  // ADC0 data

//-----------------------------------------------------------------------------
// 函数原型
//-----------------------------------------------------------------------------
void TIME0_ISR (void);
int  get_ad_value(unsigned char channel);
signed char get_temp(void);
void config (void);
void initialize(void);

void init_AD(void);
void LedBlink(unsigned char num, unsigned char state,unsigned char blink ); 

void init_para();
void init_index_para();
void delay1( unsigned int us);

void CAN_AUTO_RESET(void);
void can_rx_from_DCT_amount_feedback(unsigned char *buf);
void can_rx_from_DCT_bite_feedback(unsigned char *buf);
void can_rx_from_SWJ_Action_Command(void);
void can_rx_from_SWJ_Zijian_Command(void);
void Comunication_BITE_to_SWJ(void);

bit Get_DCT_Input_State(unsigned char  number) ; 	        // 获取底层驱动板控制指令的状态
void Set_DCT_Input_State(unsigned char  number, bit state); // 设置或复位底层驱动板的输入状态
void Set_DCT_OK_Result(unsigned char  number, bit state);   // 设置或复位底层驱动板的动作OK状态
void Set_DCT_OK_Action(unsigned char  number, bit state);   // 设置或复位底层驱动板的动作结束状态

void CAN2_Data_Collect_Process_Programme(void);             // 底层发药板数据反馈子程序
void Medica_Auto_Transportation_Programme(void);            // 药品运输系统自动操作
void ConveyingLineMovement(void);
void ElectromagnetCRTL(void);
void Susongxiangtuiyao(void);
void Yaolantisheng(void);
void YaolanStepRun(void);
void ConveyingrightLineMovement(void);
void Susongxiangrighttuiyao(void);
void Yaopingtisheng(void);
void Lan_backinfor(unsigned char m_pra1,unsigned char m_pra2);
void delay1(unsigned int us);
void delay_ms(unsigned int ms);
void Can_CHECKinfor(unsigned char ID,unsigned char command);
void Lan_backalarm(unsigned char ID,unsigned char MUM,unsigned char m_alarm1);
//-----------------------------------------------------------------------------
//外部函数原型
//-----------------------------------------------------------------------------
extern void init_can1 (void); 
extern void init_can2 (void); 
extern void init_can3 (void); 

extern void can1_transmit(char MsgNum,unsigned char *buf);
extern void can2_transmit(unsigned int id,unsigned char *buf);
extern void can3_transmit(unsigned int id,unsigned char *buf);

extern void init_GM8123(void);
extern void Uart0Send(unsigned char addr, unsigned char *buf, unsigned char bufsize, bit  sumflag);
extern void Uart1Send(void);

//-----------------------------------------------------------------------------
// 外部全局变量
//-----------------------------------------------------------------------------
extern xdata CAN1RECBUF1  CAN1RXbuffer1;
xdata CAN1TRANSBUF1   CAN1TXbuffer1;

extern xdata CAN2RECBUF1  CAN2RXbuffer1;
extern xdata CAN2RECBUF1  CAN2RXbuffer2;
extern xdata CAN2RECBUF1  CAN2RXbuffer3;



xdata CAN2TRANSBUF1   CAN2TXbuffer1;
xdata CAN2TRANSBUF2   CAN2TXbuffer2;


xdata CAN3TRANSBUF1  CAN3TXbuffer1;
extern xdata CAN3RECBUF1  CAN3RXbuffer1;


extern bit Can1NewDataA;
extern bit Can2NewData1;
extern bit Can2NewData2;
extern bit Can2NewData3;
extern bit Can3NewData1;

extern unsigned char CAN1FaultCounter;
extern unsigned char CAN2FaultCounter;
extern unsigned char CAN3FaultCounter;

extern bit UART0_Refresh;
extern bit UART1_Refresh;
extern TRANSBUF0 TransBuf0;
extern TRANSBUF1 TransBuf1;
extern RECBUF0  RecBuf0;
extern RECBUF1  RecBuf1;

//-----------------------------------------------------------------------------
// 全局变量
//-----------------------------------------------------------------------------
//定义定时器的软件计数器
unsigned char  T0Counter1=0;  //运行指示灯
unsigned char  T0Counter2=0;  //CAN 发送超时计数器
unsigned int   T0Counter3=0;  //BITE反馈给上位机
unsigned char  T0Counter4=0;  //对OK信息的处理和上报

xdata unsigned char  T0Counter12=0;  //串口控制器使用1
xdata unsigned char  T0Counter13=0;  //串口控制器使用2
unsigned char Group_index_DCT=0;   //电磁铁动作批次号也即数据序列 

unsigned char ALL_Action_OK_Sign =0;
unsigned char ALL_Action_OK_Sign_last=0;
unsigned char ALL_Result_OK_Sign =0;

unsigned char OK_Sign_TX_Amount=0;   //0k信号的发送次数
unsigned char temppage=0;

unsigned char Zijian_Sign = 0xaa;     //自检标志，0xaa退出自检模式，0X55进入自检模式
unsigned char Box_code=0;             //缓存架格子编码

xdata SYSDAT  SysData;
xdata  unsigned char  databuf[8]; 

xdata unsigned char LED_Address _at_ 0xFB00 ;
xdata unsigned char LED_BUF=0;

bit Command_Finish_Sign = NO;   //指令结束和开始标志
bit Command_Begin_Sign = NO;    //传送带启动指令

bit  Answer_ok=NO;      
unsigned char Board_ID=0x00;

unsigned int  Proce_Count=0x00;            //自动操作，动作到位后的等待时间
unsigned char Procession1=0x00;            //自动操作步骤代码
unsigned char Run_modeFatao=0;
unsigned char Run_mode=0;
unsigned char SNSORSTATUS1=0;
unsigned char SNSORSTATUS2=0;
unsigned char RUNSTATUS1=0;
unsigned char RUNSTATUS2=0;

unsigned char MAINRUNSTATUS=0;
unsigned int MAINRUNSTATUS1=0;

unsigned char Runbiaoji1=0;
unsigned char Runbiaoji2=0;
bit Tansflag=0;

unsigned char Runbiaoji3=0;
unsigned char Runbiaoji4=0;
unsigned char FAYAOinput_pra[4];
unsigned char communication_step=0;
bit  ONCEFLAG=NO;   
xdata Servomotordisplace Servodisplace;
xdata  float m_floatServodisplace=0.0;
xdata  unsigned int m_intServodisplace=0;
xdata  unsigned char Run_REAFY1=0;
xdata  unsigned char Run_REAFY2=0;
xdata  unsigned char Run_ALARM1=1;
xdata  unsigned char Run_ALARM2=1;
xdata  unsigned char Mode_ALARM1=0;
xdata  unsigned char Mode_ALARM2=0;
xdata  unsigned char Mode_ALARM3=0;
xdata  unsigned char Mode_ALARM4=0;
xdata  unsigned char Mode_adress1=0;
xdata  unsigned char Mode_adress2=0;
xdata  unsigned char Mode_adress3=0;
xdata  unsigned char Mode_adress4=0;
xdata  unsigned char Basket_stat=0;
xdata  unsigned char strat_flag=0;
xdata  unsigned char CAN_index=0;
//////////////////////////////////////////////////////////////////////////////////
// 读取底层驱动板输入信号的状态
// 电磁铁驱动板编号 number ：0～Driver_Board_Amount-1
//////////////////////////////////////////////////////////////////////////////////
bit Get_DCT_Input_State(unsigned char  number)  
{
	return (bit)( SysData.DCT_Input_Command[number/8] & (0x01<<(number%8)) ) ;
} 
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//////////////////////////////////////////////////////////////////////////////////
// 设置电磁铁驱动板输入的状态，也即输入指令的状态标志，为1时，表示此命令有效，为0时，表示此命令无效 
// 电磁铁驱动板编号端口号 number ：0～Driver_Board_Amount-1 ;
// 状态   设置为1； 设置为0；
//////////////////////////////////////////////////////////////////////////////////
void Set_DCT_Input_State(unsigned char  number, bit state) 
{
	if(state==1)
		SysData.DCT_Input_Command[number/8] |= (0x01<<(number%8));
	else
		SysData.DCT_Input_Command[number/8] &= ~(0x01<<(number%8));
}

//-----------------------------------------------------------------------------
//////////////////////////////////////////////////////////////////////////////////
// 设置电磁铁驱动板动作次数OK的状态，为1表示此槽位该批次动作次数OK.
// 电磁铁驱动板编号端口号 number ：0～Driver_Board_Amount-1 ;
// 状态   设置为1； 设置为0；
//////////////////////////////////////////////////////////////////////////////////
void Set_DCT_OK_Result(unsigned char  number, bit state) 
{
	if(state==1)
		SysData.DCT_OK_Result[number/8] |= (0x01<<(number%8));
	else
		SysData.DCT_OK_Result[number/8] &= ~(0x01<<(number%8));
}

//-----------------------------------------------------------------------------
//////////////////////////////////////////////////////////////////////////////////
// 设置电磁铁动作结束的状态，为1表示此槽位该批次动作结束
// 端口号 number ：0～Driver_Board_Amount-1 ;
// 状态   设置为1； 设置为0；
//////////////////////////////////////////////////////////////////////////////////
void Set_DCT_OK_Action(unsigned char  number, bit state) 
{
	if(state==1)
		SysData.DCT_OK_Action[number/8] |= (0x01<<(number%8));
	else
		SysData.DCT_OK_Action[number/8] &= ~(0x01<<(number%8));
}

//////////////////////////////
//定时0中断,模式1,16位定时计数器, 时钟12分频 ,高优先级
//T0=65536-1000us*11.0592/4=0xF533
//////////////////////////////
void TIME0_ISR (void) interrupt 1  
{
	//unsigned char i ;
	unsigned char tempsfr ;
	tempsfr = SFRPAGE ;
	SFRPAGE = TIMER01_PAGE;
	// TH0=0x94;    //Reset 10ms interrupt  //时钟采用4分频
	// TL0=0x00 ;
	// TH0=0xF5;    //Reset 1ms interrupt //时钟采用4分频
	// TL0=0x33 ;
	TH0=0xDC;    //Reset 10ms interrupt  //时钟采用12分频
	TL0=0x00 ;
	
	WDTCN = 0xA5;  //看门狗复位
	
	T0Counter1++;
	T0Counter2++;
	T0Counter3++;
	T0Counter4++;
	T0Counter12++;
	T0Counter13++;
	
	LED_Address= LED_BUF;      //LED输出控制
	SFRPAGE = tempsfr ;   
}


//--------------------------------------------------------------
void CAN_AUTO_RESET()
{
	if(CAN1FaultCounter>10)
	{
		init_can1() ;
		CAN1FaultCounter=0 ;
	}
	
	if(CAN2FaultCounter>10)    
	{
		init_can2() ;
		CAN2FaultCounter=0;
	}
	
	if(CAN3FaultCounter>10)  
	{
		init_can3() ;
		CAN3FaultCounter=0;
	}
}

//-------------------------------------------
//当底层驱动板发药的次数不相符时，主控板就能接收到底层驱动板发的该槽位的反馈数据，
//且及时将此信息反馈给上位机
//其中第5个字节就是该槽位的故障代码
//-------------------------------------------
void can_rx_from_DCT_amount_feedback(unsigned char *buf)
{
	// unsigned char i,sum;
	TransBuf1.amount_buf.command= buf[0];
	TransBuf1.amount_buf.index1=  buf[1];
	TransBuf1.amount_buf.address1= buf[2];
	TransBuf1.amount_buf.address2= buf[3];
	TransBuf1.amount_buf.bite=  buf[4];    
	TransBuf1.amount_buf.blank1=  buf[5];  
	TransBuf1.amount_buf.amount=  buf[6];  
	Uart1Send();
}

//接受底层驱动板送过来的BITE信息及动作OK信息
//-----------------------------------------------------------------
//都按照标准的一个机架16排计算，若有的机架，不够16排，通过上位机在界面上调整。
void can_rx_from_DCT_bite_feedback(unsigned char *buf)
{
	unsigned char Number_amout_1;
	
	//接收到动作OK指令
	Number_amout_1 = (buf[2]&0x0F)
		+((((buf[2]&0xE0)>>5)&0x07)-1)*16;
	
	if( Command_Finish_Sign == YES )                 //只有在指令结束后，才判断
	{
		if( ( buf[5]==0xAA )&& ( Get_DCT_Input_State(Number_amout_1)==YES ) )     //此驱动板的确接受到过控制输入指令  
		{
			Set_DCT_OK_Result(Number_amout_1, OK);       //表示此排都已经动作结束并次数OK
		}
		
		if( (buf[6]==0xBB) && ( Get_DCT_Input_State(Number_amout_1)==YES ) )      //此驱动板的确接受到过控制输入指令 
		{
			Set_DCT_OK_Action(Number_amout_1, OK);        //表示此排都已经动作结束
		}
	}
	SysData.DCT_Bite[Number_amout_1]=0;
	
}

//---------------------- 处理从上位机发过来的数据指令--------------------
void can_rx_from_SWJ_Zijian_Command(void)    //自检指令
{
	unsigned char i,sum; 
	
	//------------------向上位机回馈-------------------------
	//注释
	/* TransBuf1.amount_buf.command= 0x14;
	TransBuf1.amount_buf.address1= 0xbf;
	TransBuf1.amount_buf.address2= RecBuf1.act_buf.address2;
	TransBuf1.amount_buf.bite= 0x00;
	TransBuf1.amount_buf.blank1= 0x00;
	TransBuf1.amount_buf.amount= 0x00;
	Uart1Send();                                 //命令回馈*/
	
		  
	if(   (RecBuf1.act_buf.address1==0xbf)       //接受到自检指令
		&&(RecBuf1.act_buf.address2==0x55)
		)                                             
		  {
		Zijian_Sign=0x55;
		  }
	else if(   (RecBuf1.act_buf.address1==0xbf)   //接受结束自检指令
		&&(RecBuf1.act_buf.address2==0xaa)
		) 
		  {
		
		Zijian_Sign=0xaa;   
		  }
	else 
		Zijian_Sign=0xaa;  
		  
		  //----------------向底层驱动板发送自检指令---------------------------------
	CAN2TXbuffer1.act_buf.command=RecBuf1.act_buf.command;
	CAN2TXbuffer1.act_buf.index1=RecBuf1.act_buf.index1;
	CAN2TXbuffer1.act_buf.address1=RecBuf1.act_buf.address1;
	CAN2TXbuffer1.act_buf.address2=RecBuf1.act_buf.address2;
	CAN2TXbuffer1.act_buf.off_time=0x00;
	CAN2TXbuffer1.act_buf.on_time=0x00;
	CAN2TXbuffer1.act_buf.aim_amount=0x00;
		  
		  sum=0;
		  i=0;
		  do
		  {
			  sum+=CAN2TXbuffer1.buf[i];
			  i++;
		  }while(i<7);
          CAN2TXbuffer1.act_buf.checkout=sum;
          can2_transmit(02,CAN2TXbuffer1.buf); 
}

//---------------------- 处理从上位机发过来的数据指令--------------------
void can_rx_from_SWJ_Action_Command(void)
{
	unsigned char i,sum,Number_amout_2; 
	
    if( (RecBuf1.act_buf.address1==0xbe)
		&&(RecBuf1.act_buf.address2==0x66)
		)                         //向上位机发送始发包回馈信号
    {
		init_index_para();    //相关参数初始化
		Command_Begin_Sign = YES;
		Command_Finish_Sign = NO;
		Box_code=RecBuf1.act_buf.on_time;    //缓存架格子的编码
		
		//------------------向上位机回馈-------------------------
		Group_index_DCT= RecBuf1.act_buf.index1;
		TransBuf1.amount_buf.command= 0x80;
		TransBuf1.amount_buf.index1= RecBuf1.act_buf.index1; 
		TransBuf1.amount_buf.address1= 0xaf;
		TransBuf1.amount_buf.address2= 0x55;
		TransBuf1.amount_buf.bite= 0x00;
		TransBuf1.amount_buf.blank1= 0x00;
		TransBuf1.amount_buf.amount= 0x00;
		Uart1Send();
		//----------------向底层驱动板发送准备好指令---------------------------------
		CAN2TXbuffer1.act_buf.command=RecBuf1.act_buf.command;
		CAN2TXbuffer1.act_buf.index1=RecBuf1.act_buf.index1;
		CAN2TXbuffer1.act_buf.address1=0xbe;
		CAN2TXbuffer1.act_buf.address2=0x66;
		CAN2TXbuffer1.act_buf.off_time=0x00;
		CAN2TXbuffer1.act_buf.on_time=0x00;
		CAN2TXbuffer1.act_buf.aim_amount=0x00;
		
		sum=0;
		i=0;
		do
		{
			sum+=CAN2TXbuffer1.buf[i];
			i++;
		}while(i<7);
		CAN2TXbuffer1.act_buf.checkout=sum;
		can2_transmit(CAN2TXID_Uion,CAN2TXbuffer1.buf); 
	}
	
    else if( (RecBuf1.act_buf.address1==0xbe)
		&&(RecBuf1.act_buf.address2==0x88)
		)                        
    {
		Command_Finish_Sign = YES;        //设立指令结束标志
		// ----------------向上位机结束包发送回馈信号----------------------  
		Group_index_DCT= RecBuf1.act_buf.index1;     
		TransBuf1.amount_buf.command= 0x80;
		TransBuf1.amount_buf.index1= RecBuf1.act_buf.index1; 
		TransBuf1.amount_buf.address1= 0xaf;
		TransBuf1.amount_buf.address2= 0x44;
		TransBuf1.amount_buf.bite= 0x00;
		TransBuf1.amount_buf.blank1= 0x00;
		TransBuf1.amount_buf.amount= 0x00;
		Uart1Send();
		//---------------向底层驱动板发送指令结束命令----------------------
		CAN2TXbuffer1.act_buf.command=RecBuf1.act_buf.command;
		CAN2TXbuffer1.act_buf.index1=RecBuf1.act_buf.index1;
		CAN2TXbuffer1.act_buf.address1=0xbe;
		CAN2TXbuffer1.act_buf.address2=0x88;
		CAN2TXbuffer1.act_buf.off_time=0x00;
		CAN2TXbuffer1.act_buf.on_time=0x00;
		CAN2TXbuffer1.act_buf.aim_amount=0x00;
		
		sum=0;
		i=0;
		do
		{
			sum+=CAN2TXbuffer1.buf[i];
			i++;
		}while(i<7);
		CAN2TXbuffer1.act_buf.checkout=sum;
		can2_transmit(CAN2TXID_Uion,CAN2TXbuffer1.buf); 
	}
	
    else                         //向底层驱动板发送动作指令
	{
		Number_amout_2 = (RecBuf1.act_buf.address1&0x0F)
			+((((RecBuf1.act_buf.address1&0xE0)>>5)&0x07)-1)*16;
		
		CAN2TXbuffer1.act_buf.command=RecBuf1.act_buf.command;
		CAN2TXbuffer1.act_buf.index1=RecBuf1.act_buf.index1;
		CAN2TXbuffer1.act_buf.address1=RecBuf1.act_buf.address1;
		CAN2TXbuffer1.act_buf.address2=RecBuf1.act_buf.address2;
		CAN2TXbuffer1.act_buf.off_time=RecBuf1.act_buf.off_time;
		CAN2TXbuffer1.act_buf.on_time=RecBuf1.act_buf.on_time;
		CAN2TXbuffer1.act_buf.aim_amount=RecBuf1.act_buf.aim_amount;
		
		sum=0;
		i=0;
		do
		{
			sum+=CAN2TXbuffer1.buf[i];
			i++;
		}while(i<7);
		
		CAN2TXbuffer1.act_buf.checkout=sum;
		can2_transmit(CAN2TXbuffer1.act_buf.address1,CAN2TXbuffer1.buf); 
		
		Set_DCT_Input_State(Number_amout_2, YES);  
    }   
}

//---------------------------------------------------------------------
//-----向上位机汇报底层驱动板的通讯及故障信息--------------------------
//-----不论驱动板工作是否正常，都上报----------
//都按照标准的一个机架16排计算，若有的机架，不够16排，通过上位机在界面上调整。
void Comunication_BITE_to_SWJ(void)
{
	static unsigned char  Number_amout=0; 
	unsigned char i,sum,bite_value,address_value;
	
	if( Number_amout>=Driver_Board_Amount ) Number_amout=0;    
	address_value = (Number_amout/16)+1;
	address_value = ((address_value<<5)&0xE0)+ (Number_amout%16);
	
	if(SysData.DCT_Bite[Number_amout]>=200)               //若2秒还没接受到此驱动板的应答，认为其有故障
		bite_value=0x01;
	else    bite_value=0x00;
	
	TransBuf1.amount_buf.command= 0xc0;
	TransBuf1.amount_buf.index1=  Group_index_DCT;
	TransBuf1.amount_buf.address1= address_value;         //机架号(从1开始)&&印制板的排号(从0开始)
	TransBuf1.amount_buf.address2= 0x00;        
	TransBuf1.amount_buf.bite=  bite_value;    
	TransBuf1.amount_buf.blank1=  0x00; 
	TransBuf1.amount_buf.amount=  0x00; 
	Uart1Send();
	
	Number_amout++;
}

void CAN2_Data_Collect_Process_Programme(void)
{
	unsigned char i,sum; 
	//-------------------------------------------------------------------------
	//通过CAN2网络接受1号机柜底层主控板送过来的反馈数据1
	//分为两种类型数据，一种是故障电磁铁的实际动作次数，一种是故障状态或正常状态信息
	//--------------------------------------------------------------------------
	if(Can2NewData1==YES)  
	{
		sum=0;
		i=0;
		do
		{
			sum+=CAN2RXbuffer1.buf[i];
			i++;
		}while(i<7);
		
		if( ( sum==CAN2RXbuffer1.amount_buf.checkout )
			&&( CAN2RXbuffer1.amount_buf.command&0xE0)==0xA0 )    //电磁铁动作次数计数反馈指令
		{	
			can_rx_from_DCT_amount_feedback(CAN2RXbuffer1.buf);
		}
		
		if( ( sum==CAN2RXbuffer1.amount_buf.checkout )
			&&( CAN2RXbuffer1.bite_buf.command&0xE0)==0xE0 )      //电磁铁BITE及OK信号反馈指令
		{	
			can_rx_from_DCT_bite_feedback(CAN2RXbuffer1.buf);
		}
		
		Can2NewData1=NO;
	}
	//-------------------------------------------------------------------
	//通过CAN2网络接受2号机柜底层主控板送过来的反馈数据1
	//分为两种类型数据，一种是故障电磁铁的实际动作次数，一种是故障状态或正常状态信息
	//-------------------------------------------------------------------
	if(Can2NewData2==YES)  
	{
		sum=0;
		i=0;
		do
		{
			sum+=CAN2RXbuffer2.buf[i];
			i++;
		}while(i<7);
		
		if( ( sum==CAN2RXbuffer2.amount_buf.checkout )
			&&( CAN2RXbuffer2.amount_buf.command&0xE0)==0xA0 )    //电磁铁动作次数计数反馈指令
		{	
			can_rx_from_DCT_amount_feedback(CAN2RXbuffer2.buf);
		}
		
		if( ( sum==CAN2RXbuffer2.amount_buf.checkout )
			&&( CAN2RXbuffer2.bite_buf.command&0xE0)==0xE0 )      //电磁铁BITE及OK信号反馈指令
		{	
			can_rx_from_DCT_bite_feedback(CAN2RXbuffer2.buf);
		}
		
		Can2NewData2=NO;
	}
	//------------------------------------------------------------------------------
	//通过CAN2网络接受3号机柜底层主控板送过来的反馈数据1
	//分为两种类型数据，一种是故障电磁铁的实际动作次数，一种是故障状态或正常状态信息
	//------------------------------------------------------------------------------
	if(Can2NewData3==YES)  
	{
		sum=0;
		i=0;
		do
		{
			sum+=CAN2RXbuffer3.buf[i];
			i++;
		}while(i<7);
		
		if( ( sum==CAN2RXbuffer3.amount_buf.checkout )
			&&( CAN2RXbuffer3.amount_buf.command&0xE0)==0xA0 )    //电磁铁错误动作次数计数反馈指令
		{	
			can_rx_from_DCT_amount_feedback(CAN2RXbuffer3.buf);
		}
		
		if( ( sum==CAN2RXbuffer3.amount_buf.checkout )
			&&( CAN2RXbuffer3.bite_buf.command&0xE0)==0xE0 )      //电磁铁BITE及OK信号反馈指令
		{	
			can_rx_from_DCT_bite_feedback(CAN2RXbuffer3.buf);
		}
		
		Can2NewData3=NO;
	}
}

//-------------------------------药品自动输送子程序--------------------------------
void init_machine(void)
{
	/*Can_CHECKinfor(1,0XFC);
	delay_ms(500);
	Can_CHECKinfor(2,0XFC);
	delay_ms(500);
	if((Can3NewData1==YES)&&(Mode_adress2==1))  
	{
		Can3NewData1=NO;
		Mode_adress2=0;
		if((CAN3RXbuffer1.buf[1]==0xFC)||(CAN3RXbuffer1.buf[1]==0xC3))//输送线
		{
			if((CAN3RXbuffer1.buf[4]&0x01)!=0x01)	//药蓝已经存在
			{
				Basket_stat=1;				
			}
			if((CAN3RXbuffer1.buf[4]&0x02)!=0x02)	
			{
				Lan_backalarm(2,1,52);//
				Mode_ALARM2=1;
			}
			
			
		}	
	}*/
	Can_CHECKinfor(2,0XFC);
	delay_ms(500);
	if((Can3NewData1==YES)&&(Mode_adress4==1))  
	{
		if((CAN3RXbuffer1.buf[1]==0xFC)||(CAN3RXbuffer1.buf[1]==0xC3))//药品提升初始化准备就绪判断
		{
			//注意报警清零条件
			/*if((CAN3RXbuffer1.buf[3]&0x08)!=0x08)//带翻斗报警位检测
			{
				Lan_backalarm(4,2,1);//药品倾倒原位传感器报警
				Mode_ALARM4=1;
			}*/
			if(CAN3RXbuffer1.buf[5]!=0x00)//自动运行中报警
			{
				//Lan_backalarm(4,2,2);//药品倾倒原位传感器报警
				Mode_ALARM2=1;
			}
			
			if(CAN3RXbuffer1.buf[6]!=0x00)//报警位建立报警
			{
				//Lan_backalarm(4,2,3);//药品倾倒原位传感器报警
				Mode_ALARM2=1;
			}
		
			
			//CAN3RXbuffer1.buf[5]=
		}
		Can3NewData1=NO;
		Mode_adress2=0;
	}
	Can_CHECKinfor(3,0XFC);
	delay_ms(500);
	if((Can3NewData1==YES)&&(Mode_adress3==1))  
	{
		Can3NewData1=NO;
		Mode_adress3=0;
		if((CAN3RXbuffer1.buf[1]==0xFC)||(CAN3RXbuffer1.buf[1]==0xC3))//输送线
		{
			if((CAN3RXbuffer1.buf[4]&0x01)!=0x01)	
			{
				Lan_backalarm(3,0,51);//左翻板传感器不再原位报警
				Mode_ALARM3=1;				
			}
			if((CAN3RXbuffer1.buf[4]&0x04)!=0x04)	
			{
				Lan_backalarm(3,0,52);//右翻板传感器不再原位报警
				Mode_ALARM3=1;
			}
			
			
		}	
	}
	Can_CHECKinfor(4,0XFC);
	delay_ms(500);
	if((Can3NewData1==YES)&&(Mode_adress4==1))  
	{
		if((CAN3RXbuffer1.buf[1]==0xFC)||(CAN3RXbuffer1.buf[1]==0xC3))//药品提升初始化准备就绪判断
		{
			//注意报警清零条件
			/*if((CAN3RXbuffer1.buf[3]&0x08)!=0x08)//带翻斗报警位检测
			{
				Lan_backalarm(4,2,1);//药品倾倒原位传感器报警
				Mode_ALARM4=1;
			}*/
			if(CAN3RXbuffer1.buf[5]!=0x00)//自动运行中报警
			{
				//Lan_backalarm(4,2,2);//药品倾倒原位传感器报警
				Mode_ALARM4=1;
			}
			
			if(CAN3RXbuffer1.buf[6]!=0x00)//报警位建立报警
			{
				//Lan_backalarm(4,2,3);//药品倾倒原位传感器报警
				Mode_ALARM4=1;
			}
		
			
			//CAN3RXbuffer1.buf[5]=
		}
		Can3NewData1=NO;
		Mode_adress4=0;
	}
}
//-------------------------------药品自动输送子程序--------------------------------
void  Medica_Auto_Transportation_Programme(void)
{
	if(((SNSORSTATUS1&0x01)==0x01)&&((SNSORSTATUS2&0x01)==0x01))
	{
		if(ONCEFLAG==0)
		{
			ONCEFLAG=1;
			//delay_ms(500);
			if(Run_ALARM1==0)
			{
				Lan_backinfor(0x08,0x80);//左准备好
			}
			if(Run_ALARM2==0)
			{
				//delay_ms(500);
				Lan_backinfor(0x09,0x80);//右准备好
			}
		}	
	}
	///////////////////
	//电磁铁动作推药蓝条件运行
	///////////////////
	/*if(Run_ALARM1==0)
	{
		if(((SNSORSTATUS1&0x10)==0x10)&&((MAINRUNSTATUS&0X04)==0X00))//电磁铁动作//原位判断(SNSORSTATUS1&0x10)==0x10)
		{
			MAINRUNSTATUS=MAINRUNSTATUS|0X04;
			RUNSTATUS1=RUNSTATUS1|0x01;
			ElectromagnetCRTL();
			
		}
		if(((SNSORSTATUS1&0x10)==0x10)&&((MAINRUNSTATUS&0X08)==0X00)&&((RUNSTATUS1&0X01)==0X00)&&((MAINRUNSTATUS&0X04)==0X04))//推药蓝//原位判断(SNSORSTATUS1&0x10)==0x10)
		{
			RUNSTATUS1=RUNSTATUS1|0x04;
			MAINRUNSTATUS=MAINRUNSTATUS|0X08;
			YaolanStepRun();
		}
		///////////////////
		//输送线先行等待原点建
		///////////////////
		if(Run_modeFatao==0x01)
		{
			//SNSORSTATUS2右翻遍建立标志SNSORSTATUS1左翻板建立标志MAINRUNSTATUS禁止运行标志
			if(((SNSORSTATUS2&0x01)==0x01)&&((SNSORSTATUS1&0x01)==0x01)&&((MAINRUNSTATUS&0X01)==0X00))
			{
				SNSORSTATUS1=SNSORSTATUS1&0XFE;//失效左翻板标志
				//Runbiaoji3=0;
				//Runbiaoji4=0;
				MAINRUNSTATUS=MAINRUNSTATUS|0X01;
				RUNSTATUS1=RUNSTATUS1|0X02;//输送线左定位运行标志
				RUNSTATUS1=RUNSTATUS1|0x10;
				ConveyingLineMovement();//输送线左定位运行
			}
			if((SNSORSTATUS1&0X10)==0x10)  //药蓝提升原点建立
			{
				Run_modeFatao=0;
				Run_REAFY1=0x01;
			}
		}
		//////////////////////////////////////
		if(Run_REAFY1==0x01)//左正常运行
		{
			//RUNSTATUS1&0x04步进电机推药蓝完成查询MAINRUNSTATUS&0X10禁止运行标志
			//RUNSTATUS1&0X02定位运行标志（承上启下作用）MAINRUNSTATUS&0X08推药蓝运行标志（承上启下作用）
			if(((RUNSTATUS1&0x04)==0x00)&&((MAINRUNSTATUS&0X10)==0X00)&&((RUNSTATUS1&0X02)==0X02)&&((MAINRUNSTATUS&0X08)==0X08))//输送线左推药
			{
				RUNSTATUS1=RUNSTATUS1|0x08;
				RUNSTATUS1=RUNSTATUS1&0XFD;//清除定位运行标志
				MAINRUNSTATUS=MAINRUNSTATUS|0X10;
				Susongxiangtuiyao();
			}
			
			//RUNSTATUS1&0X08输送线推药结束MAINRUNSTATUS&0X20禁止运行标志(MAINRUNSTATUS&0X10（承上启下作用）
			if(((RUNSTATUS1&0X08)==0X00)&&((MAINRUNSTATUS&0X20)==0X00)&&((MAINRUNSTATUS&0X10)==0X10))//药蓝提升
			{
				SNSORSTATUS1=SNSORSTATUS1&0XEF;  //输送线推完药蓝提升原点标志失效
				SNSORSTATUS1=SNSORSTATUS1|0X01;  //翻板上翻状态建立
				MAINRUNSTATUS=MAINRUNSTATUS&0XFE;//定位运行放开
				SNSORSTATUS1=SNSORSTATUS1&0xBF; //失效推药结束标志
				MAINRUNSTATUS=MAINRUNSTATUS|0X20;
				//Runbiaoji3=1;
				ONCEFLAG=0;
				Yaolantisheng();	
			}
			//(Runbiaoji3==0)提升完成(MAINRUNSTATUS&0X20)（承上启下作用）
			if((Runbiaoji3==1)&&((MAINRUNSTATUS&0X20)==0X20))//药蓝提升回归原位
				//if(Runbiaoji3==1)//药蓝提升回归原位
			{
				Run_REAFY1==0x00;
				Runbiaoji3=0;
				SNSORSTATUS1=SNSORSTATUS1|0X10;//药蓝提升原点建立
				MAINRUNSTATUS=MAINRUNSTATUS&0XFB;//电磁铁动作放开
				MAINRUNSTATUS=MAINRUNSTATUS&0XF7;//推药蓝动作放开
				MAINRUNSTATUS=MAINRUNSTATUS&0XEF;//输送线推药放开
				MAINRUNSTATUS=MAINRUNSTATUS&0XDF;//药蓝放开提升
			}
			
		} 
	}
	else
	{
		Run_REAFY1==0x00;
		Runbiaoji3=0;
		SNSORSTATUS1=0x01;
		SNSORSTATUS1=SNSORSTATUS1|0X10;//药蓝提升原点建立
		MAINRUNSTATUS=MAINRUNSTATUS&0XFE;//定位运行放开
		MAINRUNSTATUS=MAINRUNSTATUS&0XFB;//电磁铁动作放开
		MAINRUNSTATUS=MAINRUNSTATUS&0XF7;//推药蓝动作放开
		MAINRUNSTATUS=MAINRUNSTATUS&0XEF;//输送线推药放开
		MAINRUNSTATUS=MAINRUNSTATUS&0XDF;//药蓝放开提升
	}
	*/
	/////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////
	
	
	///////////////////
	//输送线先行等待原点建
	///////////////////
	if(Run_ALARM1==0)
	{
		if(Run_modeFatao==0x01)
		{	
			//SNSORSTATUS2右翻遍建立标志SNSORSTATUS1左翻板建立标志MAINRUNSTATUS1禁止运行标志
			if(((SNSORSTATUS1&0x01)==0x01)&&((SNSORSTATUS2&0x01)==0x01)&&((MAINRUNSTATUS&0X01)==0X00))//输送线右定位运行
			{ 
				SNSORSTATUS1=SNSORSTATUS1&0xFE;//失效左翻板标志
				MAINRUNSTATUS=MAINRUNSTATUS|0X01;
				ConveyingrightLineMovement();	
			}
			if((SNSORSTATUS1&0X02)==0x02)  //药品提升原点建立
			{
				Run_modeFatao=0;
				Run_REAFY1=0x01;
			}
		}
		////////////////////////////
		/////////////////////////////
		if(Run_REAFY1==0x01)//左正常运行
		{
			// (SNSORSTATUS2&0x02)药品提升原点建立//定位运行结束
			if(((SNSORSTATUS1&0x02)==0x02)&&((SNSORSTATUS1&0x10)==0x10)&&((MAINRUNSTATUS&0X10)==0X00))//输送线右推药//(SNSORSTATUS2&0x02)==0x02提升机构原点
			{ 
				SNSORSTATUS2=SNSORSTATUS2&0xEF;
				MAINRUNSTATUS1=MAINRUNSTATUS1|0X10;
				Susongxiangrighttuiyao();	
			}
			if(((SNSORSTATUS2&0x02)==0x02)&&((SNSORSTATUS2&0x20)==0x20)&&((MAINRUNSTATUS1&0X20)==0X00))//输送线药品提升
			{
				SNSORSTATUS2=SNSORSTATUS2&0xDF;
				SNSORSTATUS2=SNSORSTATUS2|0x01;//建立右翻板标志
				SNSORSTATUS2=SNSORSTATUS2&0xFD;//失效药品提升原点标志
				MAINRUNSTATUS1=MAINRUNSTATUS1|0X20;
				ONCEFLAG=0;
				MAINRUNSTATUS1=MAINRUNSTATUS1&0XFE;//右输送线定位运行放开
				Yaopingtisheng();
			}
			if(Runbiaoji4==1)
			{
				Runbiaoji4=0;
				Run_REAFY2==0x00;
				SNSORSTATUS2=SNSORSTATUS2|0x02;//建立药品提升原点标志
				
				MAINRUNSTATUS1=MAINRUNSTATUS1&0XEF;//右输送线推药运行
				MAINRUNSTATUS1=MAINRUNSTATUS1&0XDF;//药品提升运行
			}
			
		}
	}
	else
	{
		Runbiaoji4=0;
		Run_REAFY2==0x00;
		SNSORSTATUS2=0x01;
		SNSORSTATUS2=SNSORSTATUS2|0x02;//建立药品提升原点标志
		MAINRUNSTATUS1=MAINRUNSTATUS1&0XFE;//右输送线定位运行放开
		MAINRUNSTATUS1=MAINRUNSTATUS1&0XEF;//右输送线推药运行
		MAINRUNSTATUS1=MAINRUNSTATUS1&0XDF;//药品提升运行
		
	} 	
	
	
	
	
	
	
	
	
	
	///////////////////
	//输送线先行等待原点建
	///////////////////
	if(Run_ALARM2==0)
	{
		if(Run_modeFatao==0x02)
		{	
			//SNSORSTATUS2右翻遍建立标志SNSORSTATUS1左翻板建立标志MAINRUNSTATUS1禁止运行标志
			if(((SNSORSTATUS1&0x01)==0x01)&&((SNSORSTATUS2&0x01)==0x01)&&((MAINRUNSTATUS1&0X01)==0X00))//输送线右定位运行
			{ 
				SNSORSTATUS2=SNSORSTATUS2&0xFE;//失效右翻板标志
				MAINRUNSTATUS1=MAINRUNSTATUS1|0X01;
				ConveyingrightLineMovement();	
			}
			if((SNSORSTATUS2&0X02)==0x02)  //药品提升原点建立
			{
				Run_modeFatao=0;
				Run_REAFY2=0x02;
			}
		}
		////////////////////////////
		/////////////////////////////
		if(Run_REAFY2==0x02)//右正常运行
		{
			// (SNSORSTATUS2&0x02)药品提升原点建立//定位运行结束
			if(((SNSORSTATUS2&0x02)==0x02)&&((SNSORSTATUS2&0x10)==0x10)&&((MAINRUNSTATUS1&0X10)==0X00))//输送线右推药//(SNSORSTATUS2&0x02)==0x02提升机构原点
			{ 
				SNSORSTATUS2=SNSORSTATUS2&0xEF;
				MAINRUNSTATUS1=MAINRUNSTATUS1|0X10;
				Susongxiangrighttuiyao();	
			}
			if(((SNSORSTATUS2&0x02)==0x02)&&((SNSORSTATUS2&0x20)==0x20)&&((MAINRUNSTATUS1&0X20)==0X00))//输送线药品提升
			{
				SNSORSTATUS2=SNSORSTATUS2&0xDF;
				SNSORSTATUS2=SNSORSTATUS2|0x01;//建立右翻板标志
				SNSORSTATUS2=SNSORSTATUS2&0xFD;//失效药品提升原点标志
				MAINRUNSTATUS1=MAINRUNSTATUS1|0X20;
				ONCEFLAG=0;
				MAINRUNSTATUS1=MAINRUNSTATUS1&0XFE;//右输送线定位运行放开
				Yaopingtisheng();
			}
			if(Runbiaoji4==1)
			{
				Runbiaoji4=0;
				Run_REAFY2==0x00;
				SNSORSTATUS2=SNSORSTATUS2|0x02;//建立药品提升原点标志
				
				MAINRUNSTATUS1=MAINRUNSTATUS1&0XEF;//右输送线推药运行
				MAINRUNSTATUS1=MAINRUNSTATUS1&0XDF;//药品提升运行
			}
			
		}
	}
	else
	{
		Runbiaoji4=0;
		Run_REAFY2==0x00;
		SNSORSTATUS2=0x01;
		SNSORSTATUS2=SNSORSTATUS2|0x02;//建立药品提升原点标志
		MAINRUNSTATUS1=MAINRUNSTATUS1&0XFE;//右输送线定位运行放开
		MAINRUNSTATUS1=MAINRUNSTATUS1&0XEF;//右输送线推药运行
		MAINRUNSTATUS1=MAINRUNSTATUS1&0XDF;//药品提升运行
		
	} 	
} //  药品自动输送过程结束
//////////////////////////
//初始化变量
//////////////////////////
void init_para()
{
	unsigned char i;
	
	for(i=0;i<8;i++) 
	{
		RecBuf1.buf[i]=0;
		CAN2RXbuffer1.buf[i]=0;
		CAN3RXbuffer1.buf[i]=0;
	}
	for(i=0;i<Driver_Board_Amount;i++) 
	{
		SysData.DCT_Bite[i]=0;          //统计底层驱动板的通讯及供电情况的BITE
	}
	
	for(i=0;i<BYTE_Amount;i++) 
	{
		SysData.DCT_Input_Command[i]=0;
		SysData.DCT_OK_Result[i]=0;
		SysData.DCT_OK_Action[i]=0;
	}
	
	Procession1=0;      
	Proce_Count=0; 
	
	OUTPUT1=1;    //光耦输出初始化
	OUTPUT2=1;
	OUTPUT3=1;
	OUTPUT4=1;
	
	RS485_EN=NO;     //默认是RS485接收数据
	NET_CFG=YES;     //默认网络插座为工作状态
	NET_RST=YES;     //高电平工作，低电平复位
	
	LED_BUF=0xFF;
	SFRPAGE = CONFIG_PAGE ; 
	Board_ID= (P5 & 0x1E)>>1;
	
	
	
	//发药
	for(i=0;i<4;i++) 
	{
		FAYAOinput_pra[i]=0;
	}
	//默认零位已经完成，靠报警位是否可以运行
	SNSORSTATUS1=0x01|0x02|0x10;
	SNSORSTATUS2=0x01|0x02;
	MAINRUNSTATUS=0;
        MAINRUNSTATUS1=0;
	communication_step=0;
	Run_ALARM1=0;//初始化为报警状态
	Run_ALARM2=1;//初始化为报警状态
	Basket_stat=0;
	CAN_index=0;
	//Mode_adress=0;
	
	
	
	
}
//----------一个新的批次到来后的参数初始化-----------
void init_index_para()
{
	unsigned char i;
	
	ALL_Action_OK_Sign=0;
	ALL_Result_OK_Sign=0;
	OK_Sign_TX_Amount=0;
	Answer_ok=NO;
	
	for(i=0;i<BYTE_Amount;i++) 
	{
		SysData.DCT_Input_Command[i]=0;
		SysData.DCT_OK_Result[i]=0;
		SysData.DCT_OK_Action[i]=0;
	}
}

//////////////////////////
//初始化硬件配置
//////////////////////////
void initialize()
{
	config();      //配置寄存器
	init_AD();    //初始化A/D
	init_can1();   //初始化c8051f040自带CAN
	init_can2();   //初始化SJA1000_1
	init_can3();   //初始化SJA1000_2
	// init_GM8123(); //初始化串口控制器
	init_para();
}

//////////////////////////
//12位A/D采样
//输入：channel:通道号
//返回：0～4096 对应 0～2.43V
//////////////////////////
int get_ad_value(unsigned char channel)  
{
	SFRPAGE = 0x00;
	AMX0SL = channel; //channel select
	AD0INT = 0; //清除转换结束标记
	AD0BUSY = 1; // 开始转换
	while (AD0INT == 0); // 等待转换结束
	return(ADC0); // 读ADC0数据  
}

//////////////////////////
//读芯片温度
//返回：温度值（单位：度）
//////////////////////////
signed char get_temp()  
{
    return( (signed char)((get_ad_value(0x0F)*16-20928)/77) );
}

//////////////////////////
//初始化A/D
//////////////////////////
void  init_AD()
{
    SFRPAGE = 0x00;
	REF0CN = 0x03;	// Reference Control Register
    ADC0CN |=0xC0;  // 使能ADC
    ADC0CF = (SYSCLK/2500000) << 3;   // ADC conversion clock = 2.5MHz
    REF0CN|=0x04;//使能温度传感器
}

//---------------------------------------------------------------------------
//state=0: 灭 ;  state=1: 亮 。blink为YES取反操作，为NO按state状态操作
//---------------------------------------------------------------------------
void LedBlink(unsigned char num, unsigned char state,unsigned char blink )  
{
    static unsigned char ledbuf1=0x00;
    if( blink==NO )
	{
		if(state==0) 
		{
			ledbuf1 = ledbuf1 &( ~(0x01<<(num-1)) );    
		} 
		else         
		{
			ledbuf1 = ledbuf1 | (0x01<<(num-1)) ;    
		}
    }
	else
	{
		ledbuf1 = ledbuf1 ^(0x01<<(num-1)) ;    
	}
	LED_BUF=~ledbuf1;
}





//---------------------- --------------------自动发药程序

void Can_CHECKinfor(unsigned char ID,unsigned char command)
{
	unsigned char i,sum; 
	//----------------向底层驱动板发送准备好指令---------------------------------
	CAN3TXbuffer1.trans.address1=0X01;
	CAN3TXbuffer1.trans.command=command;
	CAN3TXbuffer1.trans.index=0x01;
	CAN3TXbuffer1.trans.data1=0X00;
	CAN3TXbuffer1.trans.data2=0X00;
	CAN3TXbuffer1.trans.data3=0X00;
	CAN3TXbuffer1.trans.data4=0X00;
		  sum=0;
		  i=0;
		  do
		  {
			  sum+=CAN3TXbuffer1.buf[i];
			  i++;
		  }while(i<7);
          CAN3TXbuffer1.trans.checkout=sum;
          can3_transmit(ID,CAN3TXbuffer1.buf);   
          Command_Finish_Sign = YES;        //设立指令结束标志
}


void Lan_backalarm(unsigned char ID,unsigned char MUM,unsigned char m_alarm1)
{
	unsigned char i=0;
	unsigned char sum=0;
	TransBuf1.amount_buf.command=22;
	TransBuf1.amount_buf.index1= 22; 
	TransBuf1.amount_buf.address1=ID;
	TransBuf1.amount_buf.address2=MUM;
	TransBuf1.amount_buf.bite= 0x00;
	TransBuf1.amount_buf.blank1=m_alarm1;
	TransBuf1.amount_buf.amount= 0x00;
	sum=0;
	for(i=0;i<7;i++)
	{
		sum=sum+TransBuf1.buf[i];
	}
	TransBuf1.amount_buf.checkout=sum;
	Uart1Send();
}
void Lan_backinfor(unsigned char m_pra1,unsigned char m_pra2)
{
	unsigned char i=0;
	unsigned char sum=0;
	TransBuf1.amount_buf.command= m_pra1;
	TransBuf1.amount_buf.index1= 0x00; 
	TransBuf1.amount_buf.address1= 0xAF;
	TransBuf1.amount_buf.address2= m_pra2;
	TransBuf1.amount_buf.bite= 0x00;
	TransBuf1.amount_buf.blank1= 0x00;
	TransBuf1.amount_buf.amount= 0x00;
	TransBuf1.amount_buf.amount= 0x00;
	
	sum=0;
	for(i=0;i<7;i++)
	{
		sum=sum+TransBuf1.buf[i];
	}
	TransBuf1.amount_buf.checkout=sum;
	Uart1Send();
}
//---------------------- --------------------自动发药程序
void ConveyingLineMovement(void)
{
	unsigned char i,sum; 
	//----------------向底层驱动板发送准备好指令---------------------------------
	CAN3TXbuffer1.trans.address1=0X01;
	CAN3TXbuffer1.trans.command=0xFE;
	CAN3TXbuffer1.trans.index=0x01;
	CAN3TXbuffer1.trans.data1=FAYAOinput_pra[0];
	CAN3TXbuffer1.trans.data2=FAYAOinput_pra[1];
	CAN3TXbuffer1.trans.data3=FAYAOinput_pra[2];
	CAN3TXbuffer1.trans.data4=FAYAOinput_pra[3];
		  sum=0;
		  i=0;
		  do
		  {
			  sum+=CAN3TXbuffer1.buf[i];
			  i++;
		  }while(i<7);
          CAN3TXbuffer1.trans.checkout=sum;
          can3_transmit(3,CAN3TXbuffer1.buf);   
          Command_Finish_Sign = YES;        //设立指令结束标志
}
void ElectromagnetCRTL(void)
{
	unsigned char i,sum; 
	
	//----------------向底层驱动板发送准备好指令---------------------------------
	CAN3TXbuffer1.trans.address1=0X01;
	CAN3TXbuffer1.trans.command=0xFE;
	CAN3TXbuffer1.trans.index=0x01;
	CAN3TXbuffer1.trans.data1=0X00;
	CAN3TXbuffer1.trans.data2=0X00;
	CAN3TXbuffer1.trans.data3=0X00;
	CAN3TXbuffer1.trans.data4=0X00;
		  sum=0;
		  i=0;
		  do
		  {
			  sum+=CAN3TXbuffer1.buf[i];
			  i++;
		  }while(i<7);
          CAN3TXbuffer1.trans.checkout=sum;
          can3_transmit(1,CAN3TXbuffer1.buf); 
		  
		  Command_Finish_Sign = YES;        //设立指令结束标志
}
void YaolanStepRun(void)
{
	unsigned char i,sum; 
	
	//----------------向底层驱动板发送准备好指令---------------------------------
	CAN3TXbuffer1.trans.address1=0X01;
	CAN3TXbuffer1.trans.command=0xFD;
	CAN3TXbuffer1.trans.index=0x01;
	CAN3TXbuffer1.trans.data1=0X00;
	CAN3TXbuffer1.trans.data2=0X00;
	CAN3TXbuffer1.trans.data3=0X00;
	CAN3TXbuffer1.trans.data4=0X00;
		  sum=0;
		  i=0;
		  do
		  {
			  sum+=CAN3TXbuffer1.buf[i];
			  i++;
		  }while(i<7);
          CAN3TXbuffer1.trans.checkout=sum;
          can3_transmit(1,CAN3TXbuffer1.buf); 
		  
		  Command_Finish_Sign = YES;        //设立指令结束标志
}
void Susongxiangtuiyao(void)
{
	unsigned char i,sum; 
	
	//----------------向底层驱动板发送准备好指令---------------------------------
	CAN3TXbuffer1.trans.address1=0X01;
	CAN3TXbuffer1.trans.command=0xFD;
	CAN3TXbuffer1.trans.index=0x01;
	CAN3TXbuffer1.trans.data1=0X00;
	CAN3TXbuffer1.trans.data2=0X00;
	CAN3TXbuffer1.trans.data3=0X00;
	CAN3TXbuffer1.trans.data4=0X00;
		  sum=0;
		  i=0;
		  do
		  {
			  sum+=CAN3TXbuffer1.buf[i];
			  i++;
		  }while(i<7);
          CAN3TXbuffer1.trans.checkout=sum;
          can3_transmit(3,CAN3TXbuffer1.buf); 
		  
		  Command_Finish_Sign = YES;        //设立指令结束标志
}
void Yaolantisheng(void)
{
	unsigned char i,sum; 
	
	//----------------向底层驱动板发送准备好指令---------------------------------
	CAN3TXbuffer1.trans.address1=0X01;
	CAN3TXbuffer1.trans.command=0xFE;
	CAN3TXbuffer1.trans.index=0x00;
	CAN3TXbuffer1.trans.data1=0X00;
	CAN3TXbuffer1.trans.data2=0X00;
	CAN3TXbuffer1.trans.data3=0X00;
	CAN3TXbuffer1.trans.data4=0X00;
		  sum=0;
		  i=0;
		  do
		  {
			  sum+=CAN3TXbuffer1.buf[i];
			  i++;
		  }while(i<7);
          CAN3TXbuffer1.trans.checkout=sum;
          can3_transmit(2,CAN3TXbuffer1.buf); 
		  
		  Command_Finish_Sign = YES;        //设立指令结束标志
}
void ConveyingrightLineMovement(void)
{
	unsigned char i,sum; 
	
	//----------------向底层驱动板发送准备好指令---------------------------------
	CAN3TXbuffer1.trans.address1=0X01;
	CAN3TXbuffer1.trans.command=0xFE;
	CAN3TXbuffer1.trans.index=0x02;
	CAN3TXbuffer1.trans.data1=FAYAOinput_pra[0];
	CAN3TXbuffer1.trans.data2=FAYAOinput_pra[1];
	CAN3TXbuffer1.trans.data3=FAYAOinput_pra[2];
	CAN3TXbuffer1.trans.data4=FAYAOinput_pra[3];
		  sum=0;
		  i=0;
		  do
		  {
			  sum+=CAN3TXbuffer1.buf[i];
			  i++;
		  }while(i<7);
          CAN3TXbuffer1.trans.checkout=sum;
          can3_transmit(3,CAN3TXbuffer1.buf); 
		  
		  Command_Finish_Sign = YES;        //设立指令结束标志
}
void Yaopingtisheng(void)
{
	unsigned char i,sum; 
	
	//----------------向底层驱动板发送准备好指令---------------------------------
	CAN3TXbuffer1.trans.address1=0X01;
	CAN3TXbuffer1.trans.command=0xFE;
	CAN3TXbuffer1.trans.index=CAN_index;
	CAN3TXbuffer1.trans.data1=0X00;
	CAN3TXbuffer1.trans.data2=0X00;
	CAN3TXbuffer1.trans.data3=0X00;
	CAN3TXbuffer1.trans.data4=0X00;
		  sum=0;
		  i=0;
		  do
		  {
			  sum+=CAN3TXbuffer1.buf[i];
			  i++;
		  }while(i<7);
          CAN3TXbuffer1.trans.checkout=sum;
          can3_transmit(4,CAN3TXbuffer1.buf); 
		  
		  Command_Finish_Sign = YES;        //设立指令结束标志
}
void Susongxiangrighttuiyao(void)
{
	unsigned char i,sum; 
	
	//----------------向底层驱动板发送准备好指令---------------------------------
	CAN3TXbuffer1.trans.address1=0X01;
	CAN3TXbuffer1.trans.command=0xFD;
	CAN3TXbuffer1.trans.index=0x02;
	CAN3TXbuffer1.trans.data1=0X00;
	CAN3TXbuffer1.trans.data2=0X00;
	CAN3TXbuffer1.trans.data3=0X00;
	CAN3TXbuffer1.trans.data4=0X00;
		  sum=0;
		  i=0;
		  do
		  {
			  sum+=CAN3TXbuffer1.buf[i];
			  i++;
		  }while(i<7);
          CAN3TXbuffer1.trans.checkout=sum;
          can3_transmit(3,CAN3TXbuffer1.buf); 
		  
		  Command_Finish_Sign = YES;        //设立指令结束标志
}
//-----------------------------------------------------------------------------
//自定义延时
//延时时间约为(us N10)毫秒
//-----------------------------------------------------------------------------
void delay1( unsigned int us)
{
	unsigned int i=us;
	while(i--) ;
}
/**************************************************************************/
void delay_ms(unsigned int ms)
{
	unsigned int us=1000;
	while(ms--)
	{
		WDTCN = 0xA5;  
		delay1(us);
	}
}






//-----------------------------------------------------------------------
//////////////////////////
// 主程序
//////////////////////////
void main(void) 
{
	unsigned char i,sum,Action_OK_Num; 
	initialize(); 
	delay_ms(4000);
	init_machine();
	while(1)
	{          
		
		if(T0Counter1>= 15)    
		{
			T0Counter1=0 ;
			LedBlink(1, 1,1 );      //程序运行指示灯
			
			
		}
		/*****************************************************************
		一、CAN1口或网络口通讯
		1.通过CAN1通讯接受上位机送过来的指令性数据
		2.通过网络通讯接受上位机送过来的指令性数据
		3.主要为1，自检指令，2.发药电磁铁动作指令，3.传送带闸板选择指令
		*****************************************************************/
		//-------------------------------------------------------------------------
		//                        接受上位机的控制指令
		//--------------------------------------------------------------------------
		if( UART1_Refresh==YES )  
		{
			sum=0;
			i=0;
			do
			{
				sum+=RecBuf1.buf[i];
				i++;
			}while(i<7);
			
			if( sum==RecBuf1.act_buf.checkout )
			{
				if( (RecBuf1.act_buf.command&0x1C)==0x04 )        //自检指令
				{
					can_rx_from_SWJ_Zijian_Command();
				}
				else  if( (RecBuf1.act_buf.command&0xE0)==0x20 )  //发药指令，其中包含左右翻板的选择
				{
					can_rx_from_SWJ_Action_Command();
				}
				
				else if((RecBuf1.buf[0]==0x01)&&(RecBuf1.buf[2]==0xBE)&&(RecBuf1.buf[3]==0x66) )     //  发药起始包
				{
					Lan_backinfor(0x01,0x55);
					communication_step=1;
				}
				else if((RecBuf1.buf[0]==0x03)&&(RecBuf1.buf[1]==0x03)&&(communication_step==1) )     //  发药数据包
				{
					//Lan_backinfor(0x66);
					switch(RecBuf1.buf[4])
					{
					case 1:
						Run_mode=1;
						break;
					case 2:
						Run_mode=2;
						break;
					default:
						break;
					}
					if(Run_mode==2)
					{
					switch(RecBuf1.buf[5])
					{
						case 1:
						CAN_index=0;
						break;
						case 2:
						CAN_index=1;
						break;
						case 3:
						CAN_index=2;
						break;
						case 4:
						CAN_index=3;
						break;
						case 5:
						CAN_index=4;
						break;
						default:
						CAN_index=4;
						break;
						
					}
					}
					Servodisplace.buf[0]=0;
					Servodisplace.buf[1]=0;
					Servodisplace.buf[2]=0;
					Servodisplace.buf[3]=0;
					
					m_intServodisplace=RecBuf1.buf[2];
					m_intServodisplace=m_intServodisplace<<8;
					m_intServodisplace=m_intServodisplace+RecBuf1.buf[3];
					m_floatServodisplace=(float)m_intServodisplace*K+0.5;
					Servodisplace.displace=(long)m_floatServodisplace;
					
					switch(Run_mode)
					{
					case 1:
					        Servodisplace.displace=0-Servodisplace.displace;
						FAYAOinput_pra[0]=Servodisplace.buf[0];
						FAYAOinput_pra[1]=Servodisplace.buf[1];
						FAYAOinput_pra[2]=Servodisplace.buf[2];
						FAYAOinput_pra[3]=Servodisplace.buf[3];
						break;
					case 2:
						
						FAYAOinput_pra[0]=Servodisplace.buf[0];
						FAYAOinput_pra[1]=Servodisplace.buf[1];
						FAYAOinput_pra[2]=Servodisplace.buf[2];
						FAYAOinput_pra[3]=Servodisplace.buf[3];
						break;
					default:
						break;
					}
					communication_step=2;
				}
				else if((RecBuf1.buf[0]==0x01)&&(RecBuf1.buf[1]==0x04)&&(RecBuf1.buf[2]==0xBE)&&(RecBuf1.buf[3]==0x88)&&(communication_step==2))     //  发药结束包
				{
					Lan_backinfor(0x01,0x44);
					communication_step=0;
					switch(Run_mode)
					{
					case 1:
						Run_modeFatao=1;
						break;
					case 2:
						Run_modeFatao=2;
						break;
					default:
						Run_modeFatao=0;
						break;
					}
					Run_mode=0;
					
				}
				else 
				{
					
				}
			}
			
			UART1_Refresh=NO;
		}
		
		//-------------------------------------------------------------------------
		//  向上位机汇报底层驱动板的通讯及供电工作状况
		//--------------------------------------------------------------------------
		
		if(T0Counter3>= 20)                  //每隔200ms向上位机发送通讯BITE信息
		{
			T0Counter3=0 ;
			
			if( Zijian_Sign ==0x55 )
			{
				Comunication_BITE_to_SWJ();    //向上位机发送底层驱动板的通讯BITE数据信息
			}
			else
			{
				
			} 
		}
		
		//-------------------------------------------------------------------------
		//                        向上位机汇报底层驱动板发药的BITE状态
		//--------------------------------------------------------------------------
		
		if(T0Counter4>= 10)   //100ms回路控制，对OK信息的处理和上报
		{
			T0Counter4=0;
			
			//-------------------底层驱动板送过来的动作OK信息汇总处理----------------------
			//在指令结束后，才启动判断流程
			//在动作指令下发给底层驱动板后,感觉没必要，所以剔除了
			if( Command_Finish_Sign == YES )         //现支持6个药架，共96个排
			{
				Action_OK_Num=0;
				for(i=0;i<BYTE_Amount;i++)	
				{
					if( ( SysData.DCT_Input_Command[i] == SysData.DCT_OK_Result[i] ) )
						Action_OK_Num=Action_OK_Num+1;
					else   Action_OK_Num=Action_OK_Num+0;
				}
				if( Action_OK_Num==BYTE_Amount )	 ALL_Result_OK_Sign = 0xAA;
				else                                 ALL_Result_OK_Sign = 0x00;
				
				//-------------------底层驱动板送过来的动作结束信息汇总处理----------------------
				Action_OK_Num=0;
				for(i=0;i<BYTE_Amount;i++)	
				{
					if( ( SysData.DCT_Input_Command[i] == SysData.DCT_OK_Action[i] ) )
						Action_OK_Num=Action_OK_Num+1;
					else   Action_OK_Num=Action_OK_Num+0;
				}
				if( Action_OK_Num==BYTE_Amount )	 ALL_Action_OK_Sign = 0xBB;
				else                                 ALL_Action_OK_Sign = 0x00;
			}
			//-------当动作都结束后，向上位机上报，且只上报3次结果信息------------
			//暂时屏蔽
			/*if( (ALL_Action_OK_Sign == 0xBB)&&( OK_Sign_TX_Amount<3 ) )
			{
			TransBuf1.OK_buf.command= 0xE0;
			TransBuf1.OK_buf.index1= Group_index_DCT; 
			TransBuf1.OK_buf.blank1= 0x00;
			TransBuf1.OK_buf.blank2= 0x00;
			TransBuf1.OK_buf.ALL_Result_OK_Sign= ALL_Result_OK_Sign;
			TransBuf1.OK_buf.ALL_Action_OK_Sign= ALL_Action_OK_Sign;
			TransBuf1.OK_buf.blank3= 0x00;
			Uart1Send();
			OK_Sign_TX_Amount++; 
		}*/
		}    //100ms定时程序结束
		
			 /*****************************************************************
			 二、CAN2口通讯
			 1.通过CAN1通讯接受上位机送过来的指令性数据-------------
			 2.通过网络通讯接受上位机送过来的指令性数据-------------
			 3.主要为1，自检指令，2.发药电磁铁动作指令，3.传送带闸板选择指令
		*****************************************************************/
		// CAN2_Data_Collect_Process_Programme();
		
		/*****************************************************************
		三、CAN3通道用于驱动控制药品传输机构，包括传送带、药框管理、药框提升等   
		并适时采集上报机构状态信息
		*****************************************************************/ 
		/*--------------------------------------------------------------------
		1、采用CAN3口向药品输送机构发送工作指令
		----------------------------------------------------------------------*/
		// Medica_Auto_Transportation_Programme();   //药品自动输送
		/*--------------------------------------------------------------------
		2、采用CAN3口接收药品输送机构反馈回来的状态信息及故障信息
		----------------------------------------------------------------------*/
		if(Can3NewData1==YES)  
		{
			sum=0;
			i=0;
			do
			{
				sum+=CAN3RXbuffer1.buf[i];
				i++;
			}while(i<7);
			
			if( ( sum==CAN3RXbuffer1.rec.checkout )
				&&( CAN3RXbuffer1.rec.command==0x18 )
				)                                                  
			{	
				Answer_ok=YES;
			}
			
			
			if(Mode_adress3==1)
			{
				Mode_adress3=0;
				if((CAN3RXbuffer1.buf[1]==0xFC)||(CAN3RXbuffer1.buf[1]==0xC4))//药品提升初始化准备就绪判断
				{
					if(CAN3RXbuffer1.buf[1]==0xC4){ONCEFLAG=0;}
					
					if((CAN3RXbuffer1.buf[4]&0x01)!=0x01)
					{
						Lan_backalarm(3,0,51);//左翻板传感器不再原位报警
						Mode_ALARM3=1;
						
					}
				        if((CAN3RXbuffer1.buf[4]&0x04)!=0x04)
					{
						Lan_backalarm(3,0,52);//右翻板传感器不再原位报警
						Mode_ALARM3=1;
					}
					if(((CAN3RXbuffer1.buf[4]&0x01)==0x01)&&((CAN3RXbuffer1.buf[4]&0x04)==0x04))
					{
						Mode_ALARM3=0;
						
					}
					
				}
				if(CAN3RXbuffer1.buf[1]==0xE0)
				{
					Lan_backalarm(3,0,53);//输送线左下翻报错
					Mode_ALARM3=1;
				}
				if(CAN3RXbuffer1.buf[1]==0xE1)
				{
					Lan_backalarm(3,0,54);//输送线左上翻报错
					Mode_ALARM3=1;
				}
			}
			if(Mode_adress4==1)
			{
				if(CAN3RXbuffer1.buf[1]==0xC4)
				{
					Lan_backalarm(4,0,9);//
					Mode_ALARM4=1;
				}
				if((CAN3RXbuffer1.buf[1]==0xFC)||(CAN3RXbuffer1.buf[1]==0xC3))//药品提升初始化准备就绪判断
				{
					//注意报警清零条件
					if(CAN3RXbuffer1.buf[1]==0xC3){ONCEFLAG=0;}
					if((CAN3RXbuffer1.buf[3]&0x08)!=0x08)
					{
						Lan_backalarm(4,2,1);//药品倾倒原位传感器报警
						Mode_ALARM4=1;
					}
				        if(CAN3RXbuffer1.buf[5]!=0x00)//自动运行中报警
					{
						Mode_ALARM4=1;
					}
					
				        if(CAN3RXbuffer1.buf[6]!=0x00)//报警位建立报警
					{
						Mode_ALARM4=1;
					}
					if(((CAN3RXbuffer1.buf[3]&0x08)==0x08)&&(CAN3RXbuffer1.buf[5]==0x00)&&(CAN3RXbuffer1.buf[6]==0x00))
					{
						Mode_ALARM4=0;
					}
					
					//CAN3RXbuffer1.buf[5]=
				}
				if(CAN3RXbuffer1.buf[1]==0xE0)
				{
					Mode_ALARM4=1;
				}
				
				Mode_adress4=0;
			}
			Can3NewData1=NO;
		}
		/*if(Run_modeFatao==1)
		{
		//Run_modeFatao=0;
		Medica_Auto_Transportation_Programme();
		}*/
		if((Mode_ALARM4==1)||(Mode_ALARM3==1))
		{
			Run_ALARM2=1;
		}
		else 
		{
			Run_ALARM2=0;//维修后故障清除报警自动清除
		}
		if((Mode_ALARM1==1)||(Mode_ALARM2==1)||(Mode_ALARM3==1))
		{
		      Run_ALARM1=1;
		}
		else
		{
		      Run_ALARM1=0;
		}
		Medica_Auto_Transportation_Programme();	
		
		/*****************************************************************
		三个CAN通道初始化
		*****************************************************************/   
		CAN_AUTO_RESET();     //CAN接口自动复位程序  
   }
   
}
