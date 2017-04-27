
//------------------------------------------------------------------------------
// UART0.c            一拖三扩展串口   (USB RS485  RS232)
// 扩展后的 1号是 RS485 ,2号是 RS232 ,3号是 USB
//------------------------------------------------------------------------------
#include <zk.h> 

TRANSBUF1 TransBuf0;
RECBUF1  RecBuf0;

unsigned char xdata RecPointer0=0;
extern xdata unsigned char Rec0[RECBUF0SIZE];
unsigned char CommandByte =0; 
bit RecFlag0=NO;
bit TI0Flag=NO;
bit UART0_Refresh=NO;
extern xdata unsigned char T0Counter12;  //串口控制器使用1
extern xdata unsigned char T0Counter13;  //串口控制器使用2

void UART0_ISR (void); 
void Uart0Send(unsigned char addr, unsigned char *buf, unsigned char bufsize, bit  sumflag);
extern void LedBlink(unsigned char num, unsigned char state,unsigned char blink ); 

void init_GM8123(void)
{
 unsigned char control_data;
 CommandByte =0; 
 SFRPAGE = CONFIG_PAGE;
 GM_RST=0;       				  	//复位
 T0Counter12=0;
 while( T0Counter12<=2);			//延时20ms
 GM_RST=1;
 T0Counter12=0;
 while( T0Counter12<=20);			//延时200ms

 //8123_1单通道工作模式
 SFRPAGE = CONFIG_PAGE;
 STADD0=1;
 STADD1=1;
 SRADD0=1;
 SRADD1=1;
 GM_MS=1;

/*
 //8123多通道工作模式 
 SFRPAGE = CONFIG_PAGE; 
 STADD0=0 ;   			   	    //写命令字
 STADD1=0 ;
 SRADD0=1;
 SRADD1=1;
 GM_MS=0 ;
 SFRPAGE =UART0_PAGE;
 REN0 = 0;  					// 停止接收
 TMR2CN &= ~0x04; 				// 停止 Timer 2 	
 SCON0 |=  0xC0 ; 				//9位可变波特率
 RCAP2L = 0x70;   				// Timer 2 Reload Register Low Byte ,4800
 RCAP2H = 0xFF;   				// Timer 2 Reload Register High Byte
 REN0=1;          				//开始接收
 TMR2CN |= 0x04;  				//启动 Timer 2 
 control_data=0xF1 ; 			//母串口9600bps;子串口2400bps  
 SBUF0=control_data;
 while(TI0Flag==NO);
 TI0Flag = NO;
 T0Counter12=0;
 while( T0Counter12<=1);   	    //延时10ms
 SFRPAGE = CONFIG_PAGE;   		//读命令字
 GM_MS=1;
 T0Counter12=0;
 while( T0Counter12<=2);
 
 T0Counter13=0;
 while(CommandByte!=control_data)
 {
    SFRPAGE = CONFIG_PAGE; 
    GM_MS=0 ;
    SFRPAGE =UART0_PAGE;
    SBUF0=control_data;
    while(TI0Flag==NO);
    TI0Flag = NO;
    T0Counter12=0;
    while( T0Counter12<=2);
    SFRPAGE = CONFIG_PAGE; 
    GM_MS=1;
    T0Counter12=0;
    while( T0Counter12<=2);
    
	if(T0Counter13>=20)
	{
	   return;  			// 延时200ms  超时退出
    }
 }
 SFRPAGE = CONFIG_PAGE;
 GM_MS=0 ;
 SFRPAGE =UART0_PAGE;
 REN0 = 0;   					//停止接收
 TMR2CN &= ~0x04; 				//停止 Timer 2 	
 SCON0 &=  ~0xC0 ; 
 SCON0 |=  0x40 ; 				//8位可变波特率
 RCAP2L = 0xB8;   				//Timer 2 Reload Register Low Byte ,9600
 RCAP2H = 0xFF;   				//Timer 2 Reload Register High Byte
 REN0=1;
 TMR2CN |= 0x04;  				//启动 Timer 2 

 SFRPAGE = CONFIG_PAGE;
 STADD0=1;   					//写发送地址
 STADD1=0;
*/
}

//-----------------------------------------------------------------------------
// 中断服务程序
// UART0中断
// 使用定时器2作为时钟，优先级高
//-----------------------------------------------------------------------------
void UART0_ISR (void) interrupt 4      
{
     unsigned char n;
     unsigned char i;
     unsigned char sum;
	 unsigned char add;
	 unsigned char temp;

     SFRPAGE = UART0_PAGE;
     if(RI0==1) 
     {  
	    RI0=0;
		SFRPAGE = CONFIG_PAGE;   
		add = SRADD1;
        temp= SRADD0; 
		add=add*2+temp ;
        SFRPAGE = UART0_PAGE;
		switch(add) 
		{
		   case 0:
		   { 
		      CommandByte=SBUF0; 
			  break;
		   }   
		//---------------------------------------   
		//接收通道1
           case 1: 
		   {  
		     n=SBUF0;
		     if (RecFlag0==NO && n==0x55)
		      { RecFlag0=YES;
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
			    if( sum==Rec0[RECBUF0SIZE-1] )
			     {
			       for(i=0 ; i<RECBUF0SIZE ;i++)  
				   {
				      RecBuf0.buf[i]=Rec0[i];
                   } 
                  UART0_Refresh=YES;
                  LedBlink(2, 1,1 );     //串口运行指示灯
                 }
			     RecPointer0=0; 
			     RecFlag0=NO;	
               }
             }
           } break;
		//---------------------------------------   
		//接收通道2
           case 2: 
		   {  
		     n=SBUF0;
		     if (RecFlag0==NO && n==0x55)
		      { RecFlag0=YES;
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
			    if( sum==Rec0[RECBUF0SIZE-1] )
			     {
			       for(i=0 ; i<RECBUF0SIZE ;i++)  
				   {
				      RecBuf0.buf[i]=Rec0[i];
                   } 
                  UART0_Refresh=YES;
                  LedBlink(2, 1,1 );     //串口运行指示灯
                 }
			     RecPointer0=0; 
			     RecFlag0=NO;	
               }
             }
		   } break;

		//---------------------------------------   
		//接收通道2
           case 3: 
		   {  
		     n=SBUF0;
		     if (RecFlag0==NO && n==0x55)
		      { RecFlag0=YES;
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
			    if( sum==Rec0[RECBUF0SIZE-1] )
			     {
			       for(i=0 ; i<RECBUF0SIZE ;i++)  
				   {
				      RecBuf0.buf[i]=Rec0[i];
                   } 
                  UART0_Refresh=YES;
                  LedBlink(2, 1,1 );     //串口运行指示灯
                 }
			     RecPointer0=0; 
			     RecFlag0=NO;	
               }
             }
		   } break;
		   default: break;
		}
	}
	else if (TI0==1)
	{
		TI0=0;
		TI0Flag=YES;
	} 
}
//-----------------------------------------------------------------------------
// UART0发送程序
// 使用定时器2作为时钟，优先级高
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// 串口1组数据发送控制
// 通道1～3,分时轮流工作(多通道)方式
//-----------------------------------------------------------------------------
void Uart0Send(unsigned char addr, unsigned char *buf, unsigned char bufsize, bit  sumflag)
{
  unsigned char i=1;

  RS485_EN=YES;   //默认是RS485接收数据
  if(sumflag==YES)
  {
     i=1;
     buf[bufsize-1] =0;
     do
     {
        buf[bufsize-1]+=buf[i];	   //TxBuf1.buf[i];
        i++;
     }while(i<bufsize-1);          //累加和效验
  }
  i=0;
  do
  {
/*
   SFRPAGE = CONFIG_PAGE;
   STADD0=(bit)(addr&0x01) ;       //写发送地址
   STADD1=(bit)(addr&0x02) ;
*/
   SFRPAGE = UART0_PAGE; 
   SBUF0=buf[i];
   while(TI0Flag==NO);
   TI0Flag = NO;
   i++;
/*
   SFRPAGE = CONFIG_PAGE;
   STADD0=0;  			  	       //写无效发送地址
   STADD1=0 ;
   SFRPAGE = UART0_PAGE;   
   SBUF0=0;
   while(TI0Flag==NO);			   //发送无效数据 
   TI0Flag = NO;
   SBUF0=0;
   while(TI0Flag==NO);
   TI0Flag = NO;
   SBUF0=0;
   while(TI0Flag==NO);
   TI0Flag = NO;
*/
  }while(i<bufsize);

  RS485_EN=NO;   //默认是RS485接收数据
}

