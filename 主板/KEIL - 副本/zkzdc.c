/*  
��������ӿڵķ�ҩϵͳ���ذ����  

  2016.02.14 ��������������Ϊװ���ó��򣬳�ȥ����Ҫ�ĳ����
  ����ǰ���ذ����Ļ����Ͻ�������
  2016.02.15 ����ҩƷ���ͼ������������Ƴ���� 
  ����ͨѶҲ�Ѿ����²������
  2016.02.24 CAN2��CAN3���ֹ��ϣ���ԭ�ȵ��ϰ汾���������С����Ѹ������ϰ汾�� 
  
*/
//-----------------------------------------------------------------------------
// �����ļ�
//-----------------------------------------------------------------------------
#include <c8051f040.h>                 // SFR declaration
#include <zk.h> 
#include <math.h>

//-----------------------------------------------------------------------------
// C8051F040��SFR����
//-----------------------------------------------------------------------------
sfr16 ADC0 = 0xbe;  // ADC0 data

//-----------------------------------------------------------------------------
// ����ԭ��
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

bit Get_DCT_Input_State(unsigned char  number) ; 	        // ��ȡ�ײ����������ָ���״̬
void Set_DCT_Input_State(unsigned char  number, bit state); // ���û�λ�ײ������������״̬
void Set_DCT_OK_Result(unsigned char  number, bit state);   // ���û�λ�ײ�������Ķ���OK״̬
void Set_DCT_OK_Action(unsigned char  number, bit state);   // ���û�λ�ײ�������Ķ�������״̬

void CAN2_Data_Collect_Process_Programme(void);             // �ײ㷢ҩ�����ݷ����ӳ���
void Medica_Auto_Transportation_Programme(void);            // ҩƷ����ϵͳ�Զ�����
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
//�ⲿ����ԭ��
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
// �ⲿȫ�ֱ���
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
// ȫ�ֱ���
//-----------------------------------------------------------------------------
//���嶨ʱ�������������
unsigned char  T0Counter1=0;  //����ָʾ��
unsigned char  T0Counter2=0;  //CAN ���ͳ�ʱ������
unsigned int   T0Counter3=0;  //BITE��������λ��
unsigned char  T0Counter4=0;  //��OK��Ϣ�Ĵ�����ϱ�

xdata unsigned char  T0Counter12=0;  //���ڿ�����ʹ��1
xdata unsigned char  T0Counter13=0;  //���ڿ�����ʹ��2
unsigned char Group_index_DCT=0;   //������������κ�Ҳ���������� 

unsigned char ALL_Action_OK_Sign =0;
unsigned char ALL_Action_OK_Sign_last=0;
unsigned char ALL_Result_OK_Sign =0;

unsigned char OK_Sign_TX_Amount=0;   //0k�źŵķ��ʹ���
unsigned char temppage=0;

unsigned char Zijian_Sign = 0xaa;     //�Լ��־��0xaa�˳��Լ�ģʽ��0X55�����Լ�ģʽ
unsigned char Box_code=0;             //����ܸ��ӱ���

xdata SYSDAT  SysData;
xdata  unsigned char  databuf[8]; 

xdata unsigned char LED_Address _at_ 0xFB00 ;
xdata unsigned char LED_BUF=0;

bit Command_Finish_Sign = NO;   //ָ������Ϳ�ʼ��־
bit Command_Begin_Sign = NO;    //���ʹ�����ָ��

bit  Answer_ok=NO;      
unsigned char Board_ID=0x00;

unsigned int  Proce_Count=0x00;            //�Զ�������������λ��ĵȴ�ʱ��
unsigned char Procession1=0x00;            //�Զ������������
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
// ��ȡ�ײ������������źŵ�״̬
// ������������� number ��0��Driver_Board_Amount-1
//////////////////////////////////////////////////////////////////////////////////
bit Get_DCT_Input_State(unsigned char  number)  
{
	return (bit)( SysData.DCT_Input_Command[number/8] & (0x01<<(number%8)) ) ;
} 
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//////////////////////////////////////////////////////////////////////////////////
// ���õ���������������״̬��Ҳ������ָ���״̬��־��Ϊ1ʱ����ʾ��������Ч��Ϊ0ʱ����ʾ��������Ч 
// ������������Ŷ˿ں� number ��0��Driver_Board_Amount-1 ;
// ״̬   ����Ϊ1�� ����Ϊ0��
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
// ���õ���������嶯������OK��״̬��Ϊ1��ʾ�˲�λ�����ζ�������OK.
// ������������Ŷ˿ں� number ��0��Driver_Board_Amount-1 ;
// ״̬   ����Ϊ1�� ����Ϊ0��
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
// ���õ��������������״̬��Ϊ1��ʾ�˲�λ�����ζ�������
// �˿ں� number ��0��Driver_Board_Amount-1 ;
// ״̬   ����Ϊ1�� ����Ϊ0��
//////////////////////////////////////////////////////////////////////////////////
void Set_DCT_OK_Action(unsigned char  number, bit state) 
{
	if(state==1)
		SysData.DCT_OK_Action[number/8] |= (0x01<<(number%8));
	else
		SysData.DCT_OK_Action[number/8] &= ~(0x01<<(number%8));
}

//////////////////////////////
//��ʱ0�ж�,ģʽ1,16λ��ʱ������, ʱ��12��Ƶ ,�����ȼ�
//T0=65536-1000us*11.0592/4=0xF533
//////////////////////////////
void TIME0_ISR (void) interrupt 1  
{
	//unsigned char i ;
	unsigned char tempsfr ;
	tempsfr = SFRPAGE ;
	SFRPAGE = TIMER01_PAGE;
	// TH0=0x94;    //Reset 10ms interrupt  //ʱ�Ӳ���4��Ƶ
	// TL0=0x00 ;
	// TH0=0xF5;    //Reset 1ms interrupt //ʱ�Ӳ���4��Ƶ
	// TL0=0x33 ;
	TH0=0xDC;    //Reset 10ms interrupt  //ʱ�Ӳ���12��Ƶ
	TL0=0x00 ;
	
	WDTCN = 0xA5;  //���Ź���λ
	
	T0Counter1++;
	T0Counter2++;
	T0Counter3++;
	T0Counter4++;
	T0Counter12++;
	T0Counter13++;
	
	LED_Address= LED_BUF;      //LED�������
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
//���ײ������巢ҩ�Ĵ��������ʱ�����ذ���ܽ��յ��ײ������巢�ĸò�λ�ķ������ݣ�
//�Ҽ�ʱ������Ϣ��������λ��
//���е�5���ֽھ��Ǹò�λ�Ĺ��ϴ���
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

//���ܵײ��������͹�����BITE��Ϣ������OK��Ϣ
//-----------------------------------------------------------------
//�����ձ�׼��һ������16�ż��㣬���еĻ��ܣ�����16�ţ�ͨ����λ���ڽ����ϵ�����
void can_rx_from_DCT_bite_feedback(unsigned char *buf)
{
	unsigned char Number_amout_1;
	
	//���յ�����OKָ��
	Number_amout_1 = (buf[2]&0x0F)
		+((((buf[2]&0xE0)>>5)&0x07)-1)*16;
	
	if( Command_Finish_Sign == YES )                 //ֻ����ָ������󣬲��ж�
	{
		if( ( buf[5]==0xAA )&& ( Get_DCT_Input_State(Number_amout_1)==YES ) )     //���������ȷ���ܵ�����������ָ��  
		{
			Set_DCT_OK_Result(Number_amout_1, OK);       //��ʾ���Ŷ��Ѿ���������������OK
		}
		
		if( (buf[6]==0xBB) && ( Get_DCT_Input_State(Number_amout_1)==YES ) )      //���������ȷ���ܵ�����������ָ�� 
		{
			Set_DCT_OK_Action(Number_amout_1, OK);        //��ʾ���Ŷ��Ѿ���������
		}
	}
	SysData.DCT_Bite[Number_amout_1]=0;
	
}

//---------------------- �������λ��������������ָ��--------------------
void can_rx_from_SWJ_Zijian_Command(void)    //�Լ�ָ��
{
	unsigned char i,sum; 
	
	//------------------����λ������-------------------------
	//ע��
	/* TransBuf1.amount_buf.command= 0x14;
	TransBuf1.amount_buf.address1= 0xbf;
	TransBuf1.amount_buf.address2= RecBuf1.act_buf.address2;
	TransBuf1.amount_buf.bite= 0x00;
	TransBuf1.amount_buf.blank1= 0x00;
	TransBuf1.amount_buf.amount= 0x00;
	Uart1Send();                                 //�������*/
	
		  
	if(   (RecBuf1.act_buf.address1==0xbf)       //���ܵ��Լ�ָ��
		&&(RecBuf1.act_buf.address2==0x55)
		)                                             
		  {
		Zijian_Sign=0x55;
		  }
	else if(   (RecBuf1.act_buf.address1==0xbf)   //���ܽ����Լ�ָ��
		&&(RecBuf1.act_buf.address2==0xaa)
		) 
		  {
		
		Zijian_Sign=0xaa;   
		  }
	else 
		Zijian_Sign=0xaa;  
		  
		  //----------------��ײ������巢���Լ�ָ��---------------------------------
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

//---------------------- �������λ��������������ָ��--------------------
void can_rx_from_SWJ_Action_Command(void)
{
	unsigned char i,sum,Number_amout_2; 
	
    if( (RecBuf1.act_buf.address1==0xbe)
		&&(RecBuf1.act_buf.address2==0x66)
		)                         //����λ������ʼ���������ź�
    {
		init_index_para();    //��ز�����ʼ��
		Command_Begin_Sign = YES;
		Command_Finish_Sign = NO;
		Box_code=RecBuf1.act_buf.on_time;    //����ܸ��ӵı���
		
		//------------------����λ������-------------------------
		Group_index_DCT= RecBuf1.act_buf.index1;
		TransBuf1.amount_buf.command= 0x80;
		TransBuf1.amount_buf.index1= RecBuf1.act_buf.index1; 
		TransBuf1.amount_buf.address1= 0xaf;
		TransBuf1.amount_buf.address2= 0x55;
		TransBuf1.amount_buf.bite= 0x00;
		TransBuf1.amount_buf.blank1= 0x00;
		TransBuf1.amount_buf.amount= 0x00;
		Uart1Send();
		//----------------��ײ������巢��׼����ָ��---------------------------------
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
		Command_Finish_Sign = YES;        //����ָ�������־
		// ----------------����λ�����������ͻ����ź�----------------------  
		Group_index_DCT= RecBuf1.act_buf.index1;     
		TransBuf1.amount_buf.command= 0x80;
		TransBuf1.amount_buf.index1= RecBuf1.act_buf.index1; 
		TransBuf1.amount_buf.address1= 0xaf;
		TransBuf1.amount_buf.address2= 0x44;
		TransBuf1.amount_buf.bite= 0x00;
		TransBuf1.amount_buf.blank1= 0x00;
		TransBuf1.amount_buf.amount= 0x00;
		Uart1Send();
		//---------------��ײ������巢��ָ���������----------------------
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
	
    else                         //��ײ������巢�Ͷ���ָ��
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
//-----����λ���㱨�ײ��������ͨѶ��������Ϣ--------------------------
//-----���������幤���Ƿ����������ϱ�----------
//�����ձ�׼��һ������16�ż��㣬���еĻ��ܣ�����16�ţ�ͨ����λ���ڽ����ϵ�����
void Comunication_BITE_to_SWJ(void)
{
	static unsigned char  Number_amout=0; 
	unsigned char i,sum,bite_value,address_value;
	
	if( Number_amout>=Driver_Board_Amount ) Number_amout=0;    
	address_value = (Number_amout/16)+1;
	address_value = ((address_value<<5)&0xE0)+ (Number_amout%16);
	
	if(SysData.DCT_Bite[Number_amout]>=200)               //��2�뻹û���ܵ����������Ӧ����Ϊ���й���
		bite_value=0x01;
	else    bite_value=0x00;
	
	TransBuf1.amount_buf.command= 0xc0;
	TransBuf1.amount_buf.index1=  Group_index_DCT;
	TransBuf1.amount_buf.address1= address_value;         //���ܺ�(��1��ʼ)&&ӡ�ư���ź�(��0��ʼ)
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
	//ͨ��CAN2�������1�Ż���ײ����ذ��͹����ķ�������1
	//��Ϊ�����������ݣ�һ���ǹ��ϵ������ʵ�ʶ���������һ���ǹ���״̬������״̬��Ϣ
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
			&&( CAN2RXbuffer1.amount_buf.command&0xE0)==0xA0 )    //���������������������ָ��
		{	
			can_rx_from_DCT_amount_feedback(CAN2RXbuffer1.buf);
		}
		
		if( ( sum==CAN2RXbuffer1.amount_buf.checkout )
			&&( CAN2RXbuffer1.bite_buf.command&0xE0)==0xE0 )      //�����BITE��OK�źŷ���ָ��
		{	
			can_rx_from_DCT_bite_feedback(CAN2RXbuffer1.buf);
		}
		
		Can2NewData1=NO;
	}
	//-------------------------------------------------------------------
	//ͨ��CAN2�������2�Ż���ײ����ذ��͹����ķ�������1
	//��Ϊ�����������ݣ�һ���ǹ��ϵ������ʵ�ʶ���������һ���ǹ���״̬������״̬��Ϣ
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
			&&( CAN2RXbuffer2.amount_buf.command&0xE0)==0xA0 )    //���������������������ָ��
		{	
			can_rx_from_DCT_amount_feedback(CAN2RXbuffer2.buf);
		}
		
		if( ( sum==CAN2RXbuffer2.amount_buf.checkout )
			&&( CAN2RXbuffer2.bite_buf.command&0xE0)==0xE0 )      //�����BITE��OK�źŷ���ָ��
		{	
			can_rx_from_DCT_bite_feedback(CAN2RXbuffer2.buf);
		}
		
		Can2NewData2=NO;
	}
	//------------------------------------------------------------------------------
	//ͨ��CAN2�������3�Ż���ײ����ذ��͹����ķ�������1
	//��Ϊ�����������ݣ�һ���ǹ��ϵ������ʵ�ʶ���������һ���ǹ���״̬������״̬��Ϣ
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
			&&( CAN2RXbuffer3.amount_buf.command&0xE0)==0xA0 )    //�����������������������ָ��
		{	
			can_rx_from_DCT_amount_feedback(CAN2RXbuffer3.buf);
		}
		
		if( ( sum==CAN2RXbuffer3.amount_buf.checkout )
			&&( CAN2RXbuffer3.bite_buf.command&0xE0)==0xE0 )      //�����BITE��OK�źŷ���ָ��
		{	
			can_rx_from_DCT_bite_feedback(CAN2RXbuffer3.buf);
		}
		
		Can2NewData3=NO;
	}
}

//-------------------------------ҩƷ�Զ������ӳ���--------------------------------
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
		if((CAN3RXbuffer1.buf[1]==0xFC)||(CAN3RXbuffer1.buf[1]==0xC3))//������
		{
			if((CAN3RXbuffer1.buf[4]&0x01)!=0x01)	//ҩ���Ѿ�����
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
		if((CAN3RXbuffer1.buf[1]==0xFC)||(CAN3RXbuffer1.buf[1]==0xC3))//ҩƷ������ʼ��׼�������ж�
		{
			//ע�ⱨ����������
			/*if((CAN3RXbuffer1.buf[3]&0x08)!=0x08)//����������λ���
			{
				Lan_backalarm(4,2,1);//ҩƷ�㵹ԭλ����������
				Mode_ALARM4=1;
			}*/
			if(CAN3RXbuffer1.buf[5]!=0x00)//�Զ������б���
			{
				//Lan_backalarm(4,2,2);//ҩƷ�㵹ԭλ����������
				Mode_ALARM2=1;
			}
			
			if(CAN3RXbuffer1.buf[6]!=0x00)//����λ��������
			{
				//Lan_backalarm(4,2,3);//ҩƷ�㵹ԭλ����������
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
		if((CAN3RXbuffer1.buf[1]==0xFC)||(CAN3RXbuffer1.buf[1]==0xC3))//������
		{
			if((CAN3RXbuffer1.buf[4]&0x01)!=0x01)	
			{
				Lan_backalarm(3,0,51);//�󷭰崫��������ԭλ����
				Mode_ALARM3=1;				
			}
			if((CAN3RXbuffer1.buf[4]&0x04)!=0x04)	
			{
				Lan_backalarm(3,0,52);//�ҷ��崫��������ԭλ����
				Mode_ALARM3=1;
			}
			
			
		}	
	}
	Can_CHECKinfor(4,0XFC);
	delay_ms(500);
	if((Can3NewData1==YES)&&(Mode_adress4==1))  
	{
		if((CAN3RXbuffer1.buf[1]==0xFC)||(CAN3RXbuffer1.buf[1]==0xC3))//ҩƷ������ʼ��׼�������ж�
		{
			//ע�ⱨ����������
			/*if((CAN3RXbuffer1.buf[3]&0x08)!=0x08)//����������λ���
			{
				Lan_backalarm(4,2,1);//ҩƷ�㵹ԭλ����������
				Mode_ALARM4=1;
			}*/
			if(CAN3RXbuffer1.buf[5]!=0x00)//�Զ������б���
			{
				//Lan_backalarm(4,2,2);//ҩƷ�㵹ԭλ����������
				Mode_ALARM4=1;
			}
			
			if(CAN3RXbuffer1.buf[6]!=0x00)//����λ��������
			{
				//Lan_backalarm(4,2,3);//ҩƷ�㵹ԭλ����������
				Mode_ALARM4=1;
			}
		
			
			//CAN3RXbuffer1.buf[5]=
		}
		Can3NewData1=NO;
		Mode_adress4=0;
	}
}
//-------------------------------ҩƷ�Զ������ӳ���--------------------------------
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
				Lan_backinfor(0x08,0x80);//��׼����
			}
			if(Run_ALARM2==0)
			{
				//delay_ms(500);
				Lan_backinfor(0x09,0x80);//��׼����
			}
		}	
	}
	///////////////////
	//�����������ҩ����������
	///////////////////
	/*if(Run_ALARM1==0)
	{
		if(((SNSORSTATUS1&0x10)==0x10)&&((MAINRUNSTATUS&0X04)==0X00))//���������//ԭλ�ж�(SNSORSTATUS1&0x10)==0x10)
		{
			MAINRUNSTATUS=MAINRUNSTATUS|0X04;
			RUNSTATUS1=RUNSTATUS1|0x01;
			ElectromagnetCRTL();
			
		}
		if(((SNSORSTATUS1&0x10)==0x10)&&((MAINRUNSTATUS&0X08)==0X00)&&((RUNSTATUS1&0X01)==0X00)&&((MAINRUNSTATUS&0X04)==0X04))//��ҩ��//ԭλ�ж�(SNSORSTATUS1&0x10)==0x10)
		{
			RUNSTATUS1=RUNSTATUS1|0x04;
			MAINRUNSTATUS=MAINRUNSTATUS|0X08;
			YaolanStepRun();
		}
		///////////////////
		//���������еȴ�ԭ�㽨
		///////////////////
		if(Run_modeFatao==0x01)
		{
			//SNSORSTATUS2�ҷ��齨����־SNSORSTATUS1�󷭰彨����־MAINRUNSTATUS��ֹ���б�־
			if(((SNSORSTATUS2&0x01)==0x01)&&((SNSORSTATUS1&0x01)==0x01)&&((MAINRUNSTATUS&0X01)==0X00))
			{
				SNSORSTATUS1=SNSORSTATUS1&0XFE;//ʧЧ�󷭰��־
				//Runbiaoji3=0;
				//Runbiaoji4=0;
				MAINRUNSTATUS=MAINRUNSTATUS|0X01;
				RUNSTATUS1=RUNSTATUS1|0X02;//��������λ���б�־
				RUNSTATUS1=RUNSTATUS1|0x10;
				ConveyingLineMovement();//��������λ����
			}
			if((SNSORSTATUS1&0X10)==0x10)  //ҩ������ԭ�㽨��
			{
				Run_modeFatao=0;
				Run_REAFY1=0x01;
			}
		}
		//////////////////////////////////////
		if(Run_REAFY1==0x01)//����������
		{
			//RUNSTATUS1&0x04���������ҩ����ɲ�ѯMAINRUNSTATUS&0X10��ֹ���б�־
			//RUNSTATUS1&0X02��λ���б�־�������������ã�MAINRUNSTATUS&0X08��ҩ�����б�־�������������ã�
			if(((RUNSTATUS1&0x04)==0x00)&&((MAINRUNSTATUS&0X10)==0X00)&&((RUNSTATUS1&0X02)==0X02)&&((MAINRUNSTATUS&0X08)==0X08))//����������ҩ
			{
				RUNSTATUS1=RUNSTATUS1|0x08;
				RUNSTATUS1=RUNSTATUS1&0XFD;//�����λ���б�־
				MAINRUNSTATUS=MAINRUNSTATUS|0X10;
				Susongxiangtuiyao();
			}
			
			//RUNSTATUS1&0X08��������ҩ����MAINRUNSTATUS&0X20��ֹ���б�־(MAINRUNSTATUS&0X10�������������ã�
			if(((RUNSTATUS1&0X08)==0X00)&&((MAINRUNSTATUS&0X20)==0X00)&&((MAINRUNSTATUS&0X10)==0X10))//ҩ������
			{
				SNSORSTATUS1=SNSORSTATUS1&0XEF;  //����������ҩ������ԭ���־ʧЧ
				SNSORSTATUS1=SNSORSTATUS1|0X01;  //�����Ϸ�״̬����
				MAINRUNSTATUS=MAINRUNSTATUS&0XFE;//��λ���зſ�
				SNSORSTATUS1=SNSORSTATUS1&0xBF; //ʧЧ��ҩ������־
				MAINRUNSTATUS=MAINRUNSTATUS|0X20;
				//Runbiaoji3=1;
				ONCEFLAG=0;
				Yaolantisheng();	
			}
			//(Runbiaoji3==0)�������(MAINRUNSTATUS&0X20)�������������ã�
			if((Runbiaoji3==1)&&((MAINRUNSTATUS&0X20)==0X20))//ҩ�������ع�ԭλ
				//if(Runbiaoji3==1)//ҩ�������ع�ԭλ
			{
				Run_REAFY1==0x00;
				Runbiaoji3=0;
				SNSORSTATUS1=SNSORSTATUS1|0X10;//ҩ������ԭ�㽨��
				MAINRUNSTATUS=MAINRUNSTATUS&0XFB;//����������ſ�
				MAINRUNSTATUS=MAINRUNSTATUS&0XF7;//��ҩ�������ſ�
				MAINRUNSTATUS=MAINRUNSTATUS&0XEF;//��������ҩ�ſ�
				MAINRUNSTATUS=MAINRUNSTATUS&0XDF;//ҩ���ſ�����
			}
			
		} 
	}
	else
	{
		Run_REAFY1==0x00;
		Runbiaoji3=0;
		SNSORSTATUS1=0x01;
		SNSORSTATUS1=SNSORSTATUS1|0X10;//ҩ������ԭ�㽨��
		MAINRUNSTATUS=MAINRUNSTATUS&0XFE;//��λ���зſ�
		MAINRUNSTATUS=MAINRUNSTATUS&0XFB;//����������ſ�
		MAINRUNSTATUS=MAINRUNSTATUS&0XF7;//��ҩ�������ſ�
		MAINRUNSTATUS=MAINRUNSTATUS&0XEF;//��������ҩ�ſ�
		MAINRUNSTATUS=MAINRUNSTATUS&0XDF;//ҩ���ſ�����
	}
	*/
	/////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////
	
	
	///////////////////
	//���������еȴ�ԭ�㽨
	///////////////////
	if(Run_ALARM1==0)
	{
		if(Run_modeFatao==0x01)
		{	
			//SNSORSTATUS2�ҷ��齨����־SNSORSTATUS1�󷭰彨����־MAINRUNSTATUS1��ֹ���б�־
			if(((SNSORSTATUS1&0x01)==0x01)&&((SNSORSTATUS2&0x01)==0x01)&&((MAINRUNSTATUS&0X01)==0X00))//�������Ҷ�λ����
			{ 
				SNSORSTATUS1=SNSORSTATUS1&0xFE;//ʧЧ�󷭰��־
				MAINRUNSTATUS=MAINRUNSTATUS|0X01;
				ConveyingrightLineMovement();	
			}
			if((SNSORSTATUS1&0X02)==0x02)  //ҩƷ����ԭ�㽨��
			{
				Run_modeFatao=0;
				Run_REAFY1=0x01;
			}
		}
		////////////////////////////
		/////////////////////////////
		if(Run_REAFY1==0x01)//����������
		{
			// (SNSORSTATUS2&0x02)ҩƷ����ԭ�㽨��//��λ���н���
			if(((SNSORSTATUS1&0x02)==0x02)&&((SNSORSTATUS1&0x10)==0x10)&&((MAINRUNSTATUS&0X10)==0X00))//����������ҩ//(SNSORSTATUS2&0x02)==0x02��������ԭ��
			{ 
				SNSORSTATUS2=SNSORSTATUS2&0xEF;
				MAINRUNSTATUS1=MAINRUNSTATUS1|0X10;
				Susongxiangrighttuiyao();	
			}
			if(((SNSORSTATUS2&0x02)==0x02)&&((SNSORSTATUS2&0x20)==0x20)&&((MAINRUNSTATUS1&0X20)==0X00))//������ҩƷ����
			{
				SNSORSTATUS2=SNSORSTATUS2&0xDF;
				SNSORSTATUS2=SNSORSTATUS2|0x01;//�����ҷ����־
				SNSORSTATUS2=SNSORSTATUS2&0xFD;//ʧЧҩƷ����ԭ���־
				MAINRUNSTATUS1=MAINRUNSTATUS1|0X20;
				ONCEFLAG=0;
				MAINRUNSTATUS1=MAINRUNSTATUS1&0XFE;//�������߶�λ���зſ�
				Yaopingtisheng();
			}
			if(Runbiaoji4==1)
			{
				Runbiaoji4=0;
				Run_REAFY2==0x00;
				SNSORSTATUS2=SNSORSTATUS2|0x02;//����ҩƷ����ԭ���־
				
				MAINRUNSTATUS1=MAINRUNSTATUS1&0XEF;//����������ҩ����
				MAINRUNSTATUS1=MAINRUNSTATUS1&0XDF;//ҩƷ��������
			}
			
		}
	}
	else
	{
		Runbiaoji4=0;
		Run_REAFY2==0x00;
		SNSORSTATUS2=0x01;
		SNSORSTATUS2=SNSORSTATUS2|0x02;//����ҩƷ����ԭ���־
		MAINRUNSTATUS1=MAINRUNSTATUS1&0XFE;//�������߶�λ���зſ�
		MAINRUNSTATUS1=MAINRUNSTATUS1&0XEF;//����������ҩ����
		MAINRUNSTATUS1=MAINRUNSTATUS1&0XDF;//ҩƷ��������
		
	} 	
	
	
	
	
	
	
	
	
	
	///////////////////
	//���������еȴ�ԭ�㽨
	///////////////////
	if(Run_ALARM2==0)
	{
		if(Run_modeFatao==0x02)
		{	
			//SNSORSTATUS2�ҷ��齨����־SNSORSTATUS1�󷭰彨����־MAINRUNSTATUS1��ֹ���б�־
			if(((SNSORSTATUS1&0x01)==0x01)&&((SNSORSTATUS2&0x01)==0x01)&&((MAINRUNSTATUS1&0X01)==0X00))//�������Ҷ�λ����
			{ 
				SNSORSTATUS2=SNSORSTATUS2&0xFE;//ʧЧ�ҷ����־
				MAINRUNSTATUS1=MAINRUNSTATUS1|0X01;
				ConveyingrightLineMovement();	
			}
			if((SNSORSTATUS2&0X02)==0x02)  //ҩƷ����ԭ�㽨��
			{
				Run_modeFatao=0;
				Run_REAFY2=0x02;
			}
		}
		////////////////////////////
		/////////////////////////////
		if(Run_REAFY2==0x02)//����������
		{
			// (SNSORSTATUS2&0x02)ҩƷ����ԭ�㽨��//��λ���н���
			if(((SNSORSTATUS2&0x02)==0x02)&&((SNSORSTATUS2&0x10)==0x10)&&((MAINRUNSTATUS1&0X10)==0X00))//����������ҩ//(SNSORSTATUS2&0x02)==0x02��������ԭ��
			{ 
				SNSORSTATUS2=SNSORSTATUS2&0xEF;
				MAINRUNSTATUS1=MAINRUNSTATUS1|0X10;
				Susongxiangrighttuiyao();	
			}
			if(((SNSORSTATUS2&0x02)==0x02)&&((SNSORSTATUS2&0x20)==0x20)&&((MAINRUNSTATUS1&0X20)==0X00))//������ҩƷ����
			{
				SNSORSTATUS2=SNSORSTATUS2&0xDF;
				SNSORSTATUS2=SNSORSTATUS2|0x01;//�����ҷ����־
				SNSORSTATUS2=SNSORSTATUS2&0xFD;//ʧЧҩƷ����ԭ���־
				MAINRUNSTATUS1=MAINRUNSTATUS1|0X20;
				ONCEFLAG=0;
				MAINRUNSTATUS1=MAINRUNSTATUS1&0XFE;//�������߶�λ���зſ�
				Yaopingtisheng();
			}
			if(Runbiaoji4==1)
			{
				Runbiaoji4=0;
				Run_REAFY2==0x00;
				SNSORSTATUS2=SNSORSTATUS2|0x02;//����ҩƷ����ԭ���־
				
				MAINRUNSTATUS1=MAINRUNSTATUS1&0XEF;//����������ҩ����
				MAINRUNSTATUS1=MAINRUNSTATUS1&0XDF;//ҩƷ��������
			}
			
		}
	}
	else
	{
		Runbiaoji4=0;
		Run_REAFY2==0x00;
		SNSORSTATUS2=0x01;
		SNSORSTATUS2=SNSORSTATUS2|0x02;//����ҩƷ����ԭ���־
		MAINRUNSTATUS1=MAINRUNSTATUS1&0XFE;//�������߶�λ���зſ�
		MAINRUNSTATUS1=MAINRUNSTATUS1&0XEF;//����������ҩ����
		MAINRUNSTATUS1=MAINRUNSTATUS1&0XDF;//ҩƷ��������
		
	} 	
} //  ҩƷ�Զ����͹��̽���
//////////////////////////
//��ʼ������
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
		SysData.DCT_Bite[i]=0;          //ͳ�Ƶײ��������ͨѶ�����������BITE
	}
	
	for(i=0;i<BYTE_Amount;i++) 
	{
		SysData.DCT_Input_Command[i]=0;
		SysData.DCT_OK_Result[i]=0;
		SysData.DCT_OK_Action[i]=0;
	}
	
	Procession1=0;      
	Proce_Count=0; 
	
	OUTPUT1=1;    //���������ʼ��
	OUTPUT2=1;
	OUTPUT3=1;
	OUTPUT4=1;
	
	RS485_EN=NO;     //Ĭ����RS485��������
	NET_CFG=YES;     //Ĭ���������Ϊ����״̬
	NET_RST=YES;     //�ߵ�ƽ�������͵�ƽ��λ
	
	LED_BUF=0xFF;
	SFRPAGE = CONFIG_PAGE ; 
	Board_ID= (P5 & 0x1E)>>1;
	
	
	
	//��ҩ
	for(i=0;i<4;i++) 
	{
		FAYAOinput_pra[i]=0;
	}
	//Ĭ����λ�Ѿ���ɣ�������λ�Ƿ��������
	SNSORSTATUS1=0x01|0x02|0x10;
	SNSORSTATUS2=0x01|0x02;
	MAINRUNSTATUS=0;
        MAINRUNSTATUS1=0;
	communication_step=0;
	Run_ALARM1=0;//��ʼ��Ϊ����״̬
	Run_ALARM2=1;//��ʼ��Ϊ����״̬
	Basket_stat=0;
	CAN_index=0;
	//Mode_adress=0;
	
	
	
	
}
//----------һ���µ����ε�����Ĳ�����ʼ��-----------
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
//��ʼ��Ӳ������
//////////////////////////
void initialize()
{
	config();      //���üĴ���
	init_AD();    //��ʼ��A/D
	init_can1();   //��ʼ��c8051f040�Դ�CAN
	init_can2();   //��ʼ��SJA1000_1
	init_can3();   //��ʼ��SJA1000_2
	// init_GM8123(); //��ʼ�����ڿ�����
	init_para();
}

//////////////////////////
//12λA/D����
//���룺channel:ͨ����
//���أ�0��4096 ��Ӧ 0��2.43V
//////////////////////////
int get_ad_value(unsigned char channel)  
{
	SFRPAGE = 0x00;
	AMX0SL = channel; //channel select
	AD0INT = 0; //���ת���������
	AD0BUSY = 1; // ��ʼת��
	while (AD0INT == 0); // �ȴ�ת������
	return(ADC0); // ��ADC0����  
}

//////////////////////////
//��оƬ�¶�
//���أ��¶�ֵ����λ���ȣ�
//////////////////////////
signed char get_temp()  
{
    return( (signed char)((get_ad_value(0x0F)*16-20928)/77) );
}

//////////////////////////
//��ʼ��A/D
//////////////////////////
void  init_AD()
{
    SFRPAGE = 0x00;
	REF0CN = 0x03;	// Reference Control Register
    ADC0CN |=0xC0;  // ʹ��ADC
    ADC0CF = (SYSCLK/2500000) << 3;   // ADC conversion clock = 2.5MHz
    REF0CN|=0x04;//ʹ���¶ȴ�����
}

//---------------------------------------------------------------------------
//state=0: �� ;  state=1: �� ��blinkΪYESȡ��������ΪNO��state״̬����
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





//---------------------- --------------------�Զ���ҩ����

void Can_CHECKinfor(unsigned char ID,unsigned char command)
{
	unsigned char i,sum; 
	//----------------��ײ������巢��׼����ָ��---------------------------------
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
          Command_Finish_Sign = YES;        //����ָ�������־
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
//---------------------- --------------------�Զ���ҩ����
void ConveyingLineMovement(void)
{
	unsigned char i,sum; 
	//----------------��ײ������巢��׼����ָ��---------------------------------
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
          Command_Finish_Sign = YES;        //����ָ�������־
}
void ElectromagnetCRTL(void)
{
	unsigned char i,sum; 
	
	//----------------��ײ������巢��׼����ָ��---------------------------------
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
		  
		  Command_Finish_Sign = YES;        //����ָ�������־
}
void YaolanStepRun(void)
{
	unsigned char i,sum; 
	
	//----------------��ײ������巢��׼����ָ��---------------------------------
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
		  
		  Command_Finish_Sign = YES;        //����ָ�������־
}
void Susongxiangtuiyao(void)
{
	unsigned char i,sum; 
	
	//----------------��ײ������巢��׼����ָ��---------------------------------
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
		  
		  Command_Finish_Sign = YES;        //����ָ�������־
}
void Yaolantisheng(void)
{
	unsigned char i,sum; 
	
	//----------------��ײ������巢��׼����ָ��---------------------------------
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
		  
		  Command_Finish_Sign = YES;        //����ָ�������־
}
void ConveyingrightLineMovement(void)
{
	unsigned char i,sum; 
	
	//----------------��ײ������巢��׼����ָ��---------------------------------
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
		  
		  Command_Finish_Sign = YES;        //����ָ�������־
}
void Yaopingtisheng(void)
{
	unsigned char i,sum; 
	
	//----------------��ײ������巢��׼����ָ��---------------------------------
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
		  
		  Command_Finish_Sign = YES;        //����ָ�������־
}
void Susongxiangrighttuiyao(void)
{
	unsigned char i,sum; 
	
	//----------------��ײ������巢��׼����ָ��---------------------------------
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
		  
		  Command_Finish_Sign = YES;        //����ָ�������־
}
//-----------------------------------------------------------------------------
//�Զ�����ʱ
//��ʱʱ��ԼΪ(us N10)����
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
// ������
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
			LedBlink(1, 1,1 );      //��������ָʾ��
			
			
		}
		/*****************************************************************
		һ��CAN1�ڻ������ͨѶ
		1.ͨ��CAN1ͨѶ������λ���͹�����ָ��������
		2.ͨ������ͨѶ������λ���͹�����ָ��������
		3.��ҪΪ1���Լ�ָ�2.��ҩ���������ָ�3.���ʹ�բ��ѡ��ָ��
		*****************************************************************/
		//-------------------------------------------------------------------------
		//                        ������λ���Ŀ���ָ��
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
				if( (RecBuf1.act_buf.command&0x1C)==0x04 )        //�Լ�ָ��
				{
					can_rx_from_SWJ_Zijian_Command();
				}
				else  if( (RecBuf1.act_buf.command&0xE0)==0x20 )  //��ҩָ����а������ҷ����ѡ��
				{
					can_rx_from_SWJ_Action_Command();
				}
				
				else if((RecBuf1.buf[0]==0x01)&&(RecBuf1.buf[2]==0xBE)&&(RecBuf1.buf[3]==0x66) )     //  ��ҩ��ʼ��
				{
					Lan_backinfor(0x01,0x55);
					communication_step=1;
				}
				else if((RecBuf1.buf[0]==0x03)&&(RecBuf1.buf[1]==0x03)&&(communication_step==1) )     //  ��ҩ���ݰ�
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
				else if((RecBuf1.buf[0]==0x01)&&(RecBuf1.buf[1]==0x04)&&(RecBuf1.buf[2]==0xBE)&&(RecBuf1.buf[3]==0x88)&&(communication_step==2))     //  ��ҩ������
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
		//  ����λ���㱨�ײ��������ͨѶ�����繤��״��
		//--------------------------------------------------------------------------
		
		if(T0Counter3>= 20)                  //ÿ��200ms����λ������ͨѶBITE��Ϣ
		{
			T0Counter3=0 ;
			
			if( Zijian_Sign ==0x55 )
			{
				Comunication_BITE_to_SWJ();    //����λ�����͵ײ��������ͨѶBITE������Ϣ
			}
			else
			{
				
			} 
		}
		
		//-------------------------------------------------------------------------
		//                        ����λ���㱨�ײ������巢ҩ��BITE״̬
		//--------------------------------------------------------------------------
		
		if(T0Counter4>= 10)   //100ms��·���ƣ���OK��Ϣ�Ĵ�����ϱ�
		{
			T0Counter4=0;
			
			//-------------------�ײ��������͹����Ķ���OK��Ϣ���ܴ���----------------------
			//��ָ������󣬲������ж�����
			//�ڶ���ָ���·����ײ��������,�о�û��Ҫ�������޳���
			if( Command_Finish_Sign == YES )         //��֧��6��ҩ�ܣ���96����
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
				
				//-------------------�ײ��������͹����Ķ���������Ϣ���ܴ���----------------------
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
			//-------������������������λ���ϱ�����ֻ�ϱ�3�ν����Ϣ------------
			//��ʱ����
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
		}    //100ms��ʱ�������
		
			 /*****************************************************************
			 ����CAN2��ͨѶ
			 1.ͨ��CAN1ͨѶ������λ���͹�����ָ��������-------------
			 2.ͨ������ͨѶ������λ���͹�����ָ��������-------------
			 3.��ҪΪ1���Լ�ָ�2.��ҩ���������ָ�3.���ʹ�բ��ѡ��ָ��
		*****************************************************************/
		// CAN2_Data_Collect_Process_Programme();
		
		/*****************************************************************
		����CAN3ͨ��������������ҩƷ����������������ʹ���ҩ�����ҩ��������   
		����ʱ�ɼ��ϱ�����״̬��Ϣ
		*****************************************************************/ 
		/*--------------------------------------------------------------------
		1������CAN3����ҩƷ���ͻ������͹���ָ��
		----------------------------------------------------------------------*/
		// Medica_Auto_Transportation_Programme();   //ҩƷ�Զ�����
		/*--------------------------------------------------------------------
		2������CAN3�ڽ���ҩƷ���ͻ�������������״̬��Ϣ��������Ϣ
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
				if((CAN3RXbuffer1.buf[1]==0xFC)||(CAN3RXbuffer1.buf[1]==0xC4))//ҩƷ������ʼ��׼�������ж�
				{
					if(CAN3RXbuffer1.buf[1]==0xC4){ONCEFLAG=0;}
					
					if((CAN3RXbuffer1.buf[4]&0x01)!=0x01)
					{
						Lan_backalarm(3,0,51);//�󷭰崫��������ԭλ����
						Mode_ALARM3=1;
						
					}
				        if((CAN3RXbuffer1.buf[4]&0x04)!=0x04)
					{
						Lan_backalarm(3,0,52);//�ҷ��崫��������ԭλ����
						Mode_ALARM3=1;
					}
					if(((CAN3RXbuffer1.buf[4]&0x01)==0x01)&&((CAN3RXbuffer1.buf[4]&0x04)==0x04))
					{
						Mode_ALARM3=0;
						
					}
					
				}
				if(CAN3RXbuffer1.buf[1]==0xE0)
				{
					Lan_backalarm(3,0,53);//���������·�����
					Mode_ALARM3=1;
				}
				if(CAN3RXbuffer1.buf[1]==0xE1)
				{
					Lan_backalarm(3,0,54);//���������Ϸ�����
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
				if((CAN3RXbuffer1.buf[1]==0xFC)||(CAN3RXbuffer1.buf[1]==0xC3))//ҩƷ������ʼ��׼�������ж�
				{
					//ע�ⱨ����������
					if(CAN3RXbuffer1.buf[1]==0xC3){ONCEFLAG=0;}
					if((CAN3RXbuffer1.buf[3]&0x08)!=0x08)
					{
						Lan_backalarm(4,2,1);//ҩƷ�㵹ԭλ����������
						Mode_ALARM4=1;
					}
				        if(CAN3RXbuffer1.buf[5]!=0x00)//�Զ������б���
					{
						Mode_ALARM4=1;
					}
					
				        if(CAN3RXbuffer1.buf[6]!=0x00)//����λ��������
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
			Run_ALARM2=0;//ά�޺������������Զ����
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
		����CANͨ����ʼ��
		*****************************************************************/   
		CAN_AUTO_RESET();     //CAN�ӿ��Զ���λ����  
   }
   
}
