#include <zk.h> 

//-----------------------------------------------------------------------------
// C8051F040��SFR����
//-----------------------------------------------------------------------------

//����SJA1000T�Ĵ�����ַ
xdata unsigned char CAN_MODE _at_ 0xFE00 ;
xdata unsigned char CAN_CMR _at_ 0xFE01 ;
xdata unsigned char CAN_SR _at_ 0xFE02 ;
xdata unsigned char CAN_IR _at_ 0xFE03 ;
xdata unsigned char CAN_IREN _at_ 0xFE04;
xdata unsigned char CAN_AMR _at_ 0xFE05 ;
xdata unsigned char CAN_BTR0 _at_ 0xFE06 ;
xdata unsigned char CAN_BTR1 _at_ 0xFE07 ;
xdata unsigned char CAN_OCR _at_ 0xFE08 ;
xdata unsigned char CAN_CDR _at_ 0xFE1F;


xdata unsigned char FIFO16 _at_ 0xFE10 ;        //FIFO���� �� ���ñ�׼֡
xdata unsigned char FIFO17 _at_ 0xFE11 ;     
xdata unsigned char FIFO18 _at_ 0xFE12 ;
xdata unsigned char FIFO19 _at_ 0xFE13 ;
xdata unsigned char FIFO20 _at_ 0xFE14 ;
xdata unsigned char FIFO21 _at_ 0xFE15 ;
xdata unsigned char FIFO22 _at_ 0xFE16 ;
xdata unsigned char FIFO23 _at_ 0xFE17 ;
xdata unsigned char FIFO24 _at_ 0xFE18 ;
xdata unsigned char FIFO25 _at_ 0xFE19 ;
xdata unsigned char FIFO26 _at_ 0xFE1A ;

xdata unsigned char RBSA _at_ 0xFE1E ;
xdata unsigned char ERRLIMIT _at_ 0xFE0D;
xdata unsigned char RXERR _at_ 0xFE0E;
xdata unsigned char TXERR _at_ 0xFE0F ;
xdata unsigned char ALC _at_ 0xFE0B;
xdata unsigned char ECC _at_ 0xFE0C;


//-----------------------------------------------------------------------------
// ȫ�ֱ���
//-----------------------------------------------------------------------------
xdata CAN2RECBUF1    CAN2RXbuffer1;
xdata CAN2RECBUF1    CAN2RXbuffer2;
xdata CAN2RECBUF1    CAN2RXbuffer3;
xdata CAN2TRANSBUF1   CAN2TXbuffer1;
xdata CAN2TRANSBUF2   CAN2TXbuffer2;
bit Can2NewData1=NO;
bit Can2NewData2=NO;
bit Can2NewData3=NO;
unsigned char CAN2FaultCounter=0;


void init_can2 (void)
{ 
  SFRPAGE  = CONFIG_PAGE;        //Port SFR's on Configuration page
  XBR1  |= 0x04;                 //����INT0�ܽ�
  SFRPAGE  = LEGACY_PAGE; 
  EA=0;
  EX0 = 1; //ʹ��INT0
  PX0 = 0;//�����ȼ�
  IT0=0; //�͵�ƽ�����ж�3
  
  CAN_MODE=0x01;    //0X09; ��λģʽ
  CAN_CDR=0X88 ;    //ѡ��PeliCAN ģʽ   ��ʱ�������ֹ(0x80 ʱ�����)
  CAN_IREN=0x0D;    //��������жϡ������жϡ����󱨾��ж�ʹ��
 
  //���������˲������ڹ����޹�ID��ֻ��ͨ���˲���ID���������ж�
  //����AMR
  FIFO20=0xFF;     //�������μĴ�������8λ����ַ���������˲� , ���Ϊ1���ACRΪ�޹�,
  FIFO21=0xFF;     //�������μĴ������θ�8λ��RTR���������˲�,����λ�޹�
  FIFO22=0xFF;     //�������μĴ������ε�8λ����Ϊ"1"���޹�
  FIFO23=0xFF;     //�������μĴ�������8λ��  ��Ϊ"1"���޹�
  //����ACR
  FIFO16=0x00;     //������Ĵ�������8λ��
  FIFO17=0x00;     //������Ĵ������θ�8λ��
  FIFO18=0x00;     //������Ĵ������ε�8λ��
  FIFO19=0x00;     //������Ĵ�������8λ��

  //���յ�ַ��0000,000*,***   //��FIFO20=0x01; FIFO16=0x01��0x00 ʱ
  //���յ�ַ��****,****,***   //��FIFO20=0xFF; FIFO16=**ʱ 
  //���յ�ַ��0000,0001,***   //��FIFO20=0x00; FIFO16=0x01ʱ 


 // TSEG1 = (Prop_Seg + Phase_Seg1 - 1) = 6 , 7
 //  TSEG2 = (Phase_Seg2 - 1)           = 2
 // SJW_p = (SJW - 1)                   = 1
  //CAN_BTR0=0x40;    //  ������=502693Hz
 // CAN_BTR0=0x41;    //  ������=502693Hz/2
//  CAN_BTR0=0x43;    //  ������=502693Hz/4
 // CAN_BTR0=0x44;    //  ������=502693Hz/5

//  CAN_BTR0=0x40;    //  ������=500000Hz
 // CAN_BTR0=0x41;    //  ������=500000Hz/2
    CAN_BTR0=0x43;    //  ������=500000Hz/4
 // CAN_BTR0=0x44;    //  ������=500000Hz/5


  CAN_BTR1 = 0xA7;  //ͨѶ������ʱ��0xA7,ͨѶ������ʱ��0x27
                     //0x26(11.0592MHz),  0x27(12MHz)  
  CAN_OCR = 0xda;
  RBSA = 0;
  ECC = 0 ;
  CAN_MODE=0x08;  //���õ��˲����շ�ʽ�����ع���״̬
  EA=1;
}

void can2_transmit(unsigned int id,unsigned char *buf)
{ 
   UINTUNION  temp;
   unsigned char r;
   T0Counter2=0;
   
   do{
       r = CAN_SR ;
       if(T0Counter2>=2)
	   {    
	       CAN2FaultCounter++; 
	       return; //��ʱ20ms  ��ʱ�˳�
       }
   }while(((r&0x10)==0x10)||((r&0x04)==0x00));//||((r&0x08)==0x00));
	// ��Ҫ���뷢�����״̬�жϣ�������ֹ���ʱ�������Իָ�
   EA=0;
   FIFO16=0x08;  //֡��Ϣ�� ��׼֡��8λ����

   temp.value=id<<5;

   FIFO17 = temp.buf[0];   //ID1
   FIFO18 = (temp.buf[1]&0xE0)|0x08 ;  //ID2  

   FIFO19=buf[0];
   FIFO20=buf[1];
   FIFO21=buf[2];
   FIFO22=buf[3];
   FIFO23=buf[4];
   FIFO24=buf[5];
   FIFO25=buf[6];
   FIFO26=buf[7];
   CAN_CMR=0x01;
   EA=1;  
   CAN2FaultCounter=0;
}

void can2_receive (unsigned char *buf)
{
	  buf[0]=FIFO19;
      buf[1]=FIFO20;
      buf[2]=FIFO21;
      buf[3]=FIFO22;
      buf[4]=FIFO23;
      buf[5]=FIFO24;
      buf[6]=FIFO25;
      buf[7]=FIFO26;	
}

//CAN2������SJA1000T�������ⲿ�ж�INT0 �������ȼ� 
void INT0_ISR (void) interrupt 0  
{   
   unsigned char temp;
   unsigned char ir_state;
   UINTUNION   address;
   ir_state=CAN_IR;

   if( (ir_state&0x01)==0x01 )   //�����ж�
   {
      address.buf[0] = FIFO17 ;
      address.buf[1] = FIFO18 ;
      address.value=( (address.value)>>5)&0x07FF;
      if( ((address.value&0xf0)==0x30)&&((FIFO16&0x40)!=0x40))   //����1�Ż������������������
	  { 
          can2_receive (CAN2RXbuffer1.buf); 
          Can2NewData1=YES;
      }

      if( ((address.value&0xf0)==0x50)&&((FIFO16&0x40)!=0x40))   //����2�Ż������������������
	  { 
          can2_receive (CAN2RXbuffer2.buf); 
          Can2NewData2=YES;
      }

      if( ((address.value&0xf0)==0x70)&&((FIFO16&0x40)!=0x40))   //����3�Ż������������������
	  { 
          can2_receive (CAN2RXbuffer3.buf); 
          Can2NewData3=YES;
      }
      LedBlink(5, 1,1 );          //����ָʾ��
      LedBlink(6, 0,0 );          //����ָʾ�� 	 
  	 
      CAN_CMR=0x04;              //�ͷŽ��ջ�����
      CAN2FaultCounter=0;
	}
    
	if((ir_state&0x08)!=0x0) //��������ж�
	{
       CAN_CMR=0x0C;  //�������������ͷŽ��ջ�����	    
	}
	if((ir_state&0x04)!=0x0)   //���󱨾��ж�
    {
       if((CAN_SR&0x80)!=0x0)
	   {
	      CAN_MODE=0x08 ;   
	   }   
	   LedBlink(6, 1,0 );      //����ָʾ
    }
	temp=ALC;   //�ͷ��ٲö�ʧ��׽�Ĵ���
	temp=ECC;   //�ͷŴ�����벶׽�Ĵ���
    IE0=0;      //INT0�жϸ�λ
}

