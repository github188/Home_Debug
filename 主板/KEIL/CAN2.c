#include <zk.h> 

//-----------------------------------------------------------------------------
// C8051F040的SFR定义
//-----------------------------------------------------------------------------

//定义SJA1000T寄存器地址
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


xdata unsigned char FIFO16 _at_ 0xFE10 ;        //FIFO窗口 ， 采用标准帧
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
// 全局变量
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
  XBR1  |= 0x04;                 //配置INT0管脚
  SFRPAGE  = LEGACY_PAGE; 
  EA=0;
  EX0 = 1; //使能INT0
  PX0 = 0;//低优先级
  IT0=0; //低电平触发中断3
  
  CAN_MODE=0x01;    //0X09; 复位模式
  CAN_CDR=0X88 ;    //选择PeliCAN 模式   ，时钟输出禁止(0x80 时钟输出)
  CAN_IREN=0x0D;    //数据溢出中断、接收中断、错误报警中断使能
 
  //配置验收滤波，用于过虑无关ID，只对通过滤波的ID产生接收中断
  //定义AMR
  FIFO20=0xFF;     //验收屏蔽寄存器（高8位）地址参与验收滤波 , 如果为1则对ACR为无关,
  FIFO21=0xFF;     //验收屏蔽寄存器（次高8位）RTR参与验收滤波,其余位无关
  FIFO22=0xFF;     //验收屏蔽寄存器（次低8位）若为"1"表无关
  FIFO23=0xFF;     //验收屏蔽寄存器（低8位）  若为"1"表无关
  //定义ACR
  FIFO16=0x00;     //验收码寄存器（高8位）
  FIFO17=0x00;     //验收码寄存器（次高8位）
  FIFO18=0x00;     //验收码寄存器（次低8位）
  FIFO19=0x00;     //验收码寄存器（低8位）

  //接收地址：0000,000*,***   //当FIFO20=0x01; FIFO16=0x01或0x00 时
  //接收地址：****,****,***   //当FIFO20=0xFF; FIFO16=**时 
  //接收地址：0000,0001,***   //当FIFO20=0x00; FIFO16=0x01时 


 // TSEG1 = (Prop_Seg + Phase_Seg1 - 1) = 6 , 7
 //  TSEG2 = (Phase_Seg2 - 1)           = 2
 // SJW_p = (SJW - 1)                   = 1
  //CAN_BTR0=0x40;    //  波特率=502693Hz
 // CAN_BTR0=0x41;    //  波特率=502693Hz/2
//  CAN_BTR0=0x43;    //  波特率=502693Hz/4
 // CAN_BTR0=0x44;    //  波特率=502693Hz/5

//  CAN_BTR0=0x40;    //  波特率=500000Hz
 // CAN_BTR0=0x41;    //  波特率=500000Hz/2
    CAN_BTR0=0x43;    //  波特率=500000Hz/4
 // CAN_BTR0=0x44;    //  波特率=500000Hz/5


  CAN_BTR1 = 0xA7;  //通讯低速率时用0xA7,通讯高速率时用0x27
                     //0x26(11.0592MHz),  0x27(12MHz)  
  CAN_OCR = 0xda;
  RBSA = 0;
  ECC = 0 ;
  CAN_MODE=0x08;  //设置单滤波接收方式，返回工作状态
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
	       return; //延时20ms  超时退出
       }
   }while(((r&0x10)==0x10)||((r&0x04)==0x00));//||((r&0x08)==0x00));
	// 不要加入发送完毕状态判断，否则出现故障时，不能自恢复
   EA=0;
   FIFO16=0x08;  //帧信息： 标准帧，8位数据

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

//CAN2控制器SJA1000T产生的外部中断INT0 ，高优先级 
void INT0_ISR (void) interrupt 0  
{   
   unsigned char temp;
   unsigned char ir_state;
   UINTUNION   address;
   ir_state=CAN_IR;

   if( (ir_state&0x01)==0x01 )   //接收中断
   {
      address.buf[0] = FIFO17 ;
      address.buf[1] = FIFO18 ;
      address.value=( (address.value)>>5)&0x07FF;
      if( ((address.value&0xf0)==0x30)&&((FIFO16&0x40)!=0x40))   //接收1号机柜电磁铁驱动板的数据
	  { 
          can2_receive (CAN2RXbuffer1.buf); 
          Can2NewData1=YES;
      }

      if( ((address.value&0xf0)==0x50)&&((FIFO16&0x40)!=0x40))   //接收2号机柜电磁铁驱动板的数据
	  { 
          can2_receive (CAN2RXbuffer2.buf); 
          Can2NewData2=YES;
      }

      if( ((address.value&0xf0)==0x70)&&((FIFO16&0x40)!=0x40))   //接收3号机柜电磁铁驱动板的数据
	  { 
          can2_receive (CAN2RXbuffer3.buf); 
          Can2NewData3=YES;
      }
      LedBlink(5, 1,1 );          //数据指示灯
      LedBlink(6, 0,0 );          //故障指示灭 	 
  	 
      CAN_CMR=0x04;              //释放接收缓冲器
      CAN2FaultCounter=0;
	}
    
	if((ir_state&0x08)!=0x0) //数据溢出中断
	{
       CAN_CMR=0x0C;  //清除数据溢出；释放接收缓冲器	    
	}
	if((ir_state&0x04)!=0x0)   //错误报警中断
    {
       if((CAN_SR&0x80)!=0x0)
	   {
	      CAN_MODE=0x08 ;   
	   }   
	   LedBlink(6, 1,0 );      //故障指示
    }
	temp=ALC;   //释放仲裁丢失捕捉寄存器
	temp=ECC;   //释放错误代码捕捉寄存器
    IE0=0;      //INT0中断复位
}

