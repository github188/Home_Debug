//-----------------------------------------------------------------------------
// 包含文件
//-----------------------------------------------------------------------------
#include <zk.h> 

//-----------------------------------------------------------------------------
// C8051F040的SFR定义
//-----------------------------------------------------------------------------
//sbit RE1=P3^0;
//sbit DE1=P3^1;


//-----------------------------------------------------------------------------
// 全局变量
//-----------------------------------------------------------------------------
unsigned char RecPointer0=0;
bit RecFlag0=NO;

unsigned char RecPointer1=0;
bit RecFlag1=NO;

// 定义串口接收缓冲
xdata  RECBUF0 RxBuf0;
xdata  RECBUF1 RxBuf1;

xdata TRANSBUF0 TxBuf0;

//定义接收缓冲器
xdata unsigned char Rec0[RECBUF0SIZE];
xdata unsigned char Rec1[RECBUF1SIZE];

bit TI0Flag = NO;  //发送结束标志
bit TI1Flag = NO;  //发送结束标志

unsigned char CommandByte1 =0 ;  


//UART0中断,使用定时器2作为时钟，16位自动重载模式,优先级高 
// 波特率： 9600 bps
void UART0_ISR (void) interrupt 4      
{
     unsigned char n;
     unsigned char i;
     unsigned char sum;
     SFRPAGE = UART0_PAGE;
     if(RI0==1) 
     {    		     
	       RI0=0 ;
		   n=SBUF0 ;
		   if( (RecFlag0==NO) && (n==0xC1) )
		   { 
		       RecFlag0=YES;
		   }
		   if (RecFlag0==YES)
		   {
		     Rec0[RecPointer0++]=n;
		     if (RecPointer0==RECBUF0SIZE)  
			 {
  		       sum=0;    
               i=0;
               do
               {
			     sum+=Rec0[i];
                 i++;
               }while(i<RECBUF0SIZE-1);
			   if( sum==Rec0[RECBUF0SIZE-1])
			   {
			            for(i=0 ; i<RECBUF0SIZE ;i++)  
				        {
				           RxBuf0.buf[i]=Rec0[i];     
                        } 
                        LedBlink(12, 1,1);  
               }
			   RecPointer0=0; 
			   RecFlag0=NO;	
             }
           }
	 }
	 else if (TI0==1)
	 { 
	     
	      TI0=0;
		  TI0Flag=YES;
     } 
}


void Uart0Send(unsigned char *buf ,
               unsigned char bufsize, bit  sumflag  )
{
  unsigned char i=0;
  if(sumflag==YES)
  {
     buf[bufsize-1] =0;
     do    //和校验
     {
        buf[bufsize-1] +=buf[i];
        i++;
     }while(i<bufsize-1);
  } 
  i=0;
  SFRPAGE = UART0_PAGE;
  do
  { 
     SBUF0=buf[i];
     while(TI0Flag==NO);
     TI0Flag = NO;
     i++;
  }while(i<bufsize);
}




//UART1中断,使用定时器1作为时钟，模式2,8位自动重载,优先级高
void UART1_ISR (void) interrupt 20   
{
     unsigned char n;
     unsigned char i;
	 unsigned char sum;
     SFRPAGE = UART1_PAGE;
     if(RI1==1) 
     {   
		   RI1=0;
		   n=SBUF1;
		   if (RecFlag1==NO && n==0xAA)
		   { RecFlag1=YES;
		   }
		   if (RecFlag1==YES)
		   {
		     Rec1[RecPointer1++]=n;

			 if (RecPointer1==RECBUF1SIZE)  
			 {
  		       sum=0;
               i=0;
               do
               {
			     sum+=Rec1[i];
                 i++;
               }while(i<RECBUF1SIZE-1);
			   if( sum==Rec1[RECBUF1SIZE-1] )  //数据正确
			   {
			       for(i=0 ; i<RECBUF1SIZE ;i++)  
				   {  
				      RxBuf1.buf[i]=Rec1[i];
                   } 
				   LedBlink(13, 1,1);  
               }
			   RecPointer1=0; 
		       RecFlag1=NO;	
             }
           }
     }
	 else if (TI1==1)
	 {		
	      TI1=0;
		  TI1Flag=YES;
     }  
}

/*
//mode=1为RS422,mode=0为RS485
void Uart1Send(unsigned char *buf ,
               unsigned char bufsize, bit  sumflag ,bit mode  )  
{
  unsigned char i=0;
  if(mode==1)
  {
     RE1=0;  //串口1配置成RS422模式
     DE1=1;
  }
  else
  {
     RE1=1;   //串口1配置成RS485模式,发送
     DE1=1;
  }
  if(sumflag==YES)  // //和校验
  {
     buf[bufsize-1] =0;
     do   
     {
        buf[bufsize-1] +=buf[i];
        i++;
     }while(i<bufsize-1);
  } 
  SFRPAGE = CONFIG_PAGE ; 

  i=0;
  delay1(2);
  SFRPAGE = UART1_PAGE;
  do
  { 
     SBUF1=buf[i];
     while(TI1Flag==NO);
     TI1Flag = NO;
     i++;
  }while(i<bufsize);
  if(mode==0)  //串口1配置成RS485模式,发送完接收
  {
     SFRPAGE = CONFIG_PAGE ; 
     RE1=0;
     DE1=0;
  }
}
*/

void init_uart()//开 UART0 和 UART1中断及优先级
{
    SFRPAGE = 0x00;
    IE |= 0x10;          //Interrupt Enable   //串口0
    IP |= 0x10;          //Interrupt Priority
    EIE2 |= 0x40;        //Extended Interrupt Enable   //串口1
    EIP2 |= 0x40;        //Extended Interrupt Priority 
   
   //串口0波特率配置  //当前9600bps
    RCAP2L = 0xB8;  // Timer 2 Reload Register Low Byte   //0xFFB8 : 9600 bps  0xFEE0 : 2400bps
    RCAP2H = 0xFF;  // Timer 2 Reload Register High Byte   //0xFFDC :19200bps   0xFFEE : 38400bps



//注意:修改波特率时注意定时器0采用的时钟分频,
//     应修改相应的定时器0重置值.否则定时中断时间不对
//     定时器0和定时器1共用时钟预分频器

   //串口1波特率配置   //当前9600bps
   // CKCON = 0x00;   // Clock Control Register //时钟采用12分频
  //  TH1 = 0x40;     // Timer 1 High Byte      //2400bps

  //  CKCON = 0x00;   // Clock Control Register
  //  TH1 = 0xA0;     // Timer 1 High Byte      //4800bps

    CKCON = 0x01;   // Clock Control Register//时钟采用4分频
    TH1 = 0x70;     // Timer 1 High Byte      //9600bps

  //  CKCON = 0x01;   // Clock Control Register
  //  TH1 = 0xB8;     // Timer 1 High Byte      //19200bps

  //  CKCON = 0x10;   // Clock Control Register
  //  TH1 = 0x70;     // Timer 1 High Byte      //38400bps

   // SFRPAGE = CONFIG_PAGE ;   //串口1配置成RS422模式
   // RE1=0;
   // DE1=1;
}


