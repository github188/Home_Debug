#include <c8051f040.h>                 // SFR declaration
#include <math.h>
#include "Yaolansample.h"
#define CMD_CHK_MAX 20

//定义定时器的软件计数器
xdata unsigned char m_comand = 0;
xdata unsigned char SN1 = 0x01;
xdata unsigned int  T0Counter5 = 0;
xdata volatile unsigned char Sensordate1 = 0;
xdata volatile unsigned char Sensordate2 = 0;
bit R_FLAG = NO;
unsigned char xdata  Send_Msg[T_MaxLen];

/***************************************************************************/
void Watchdog_Init (void);
void Initial(void);
void PORT_Init (void);
void OSCILLATOR_Init (void);
void Timer01_Init(void);
void init_para(void);
void init_INT0 (void);
void init_INT1 (void);
void Send_Packet(void);
void delay1( unsigned int us);
unsigned int crc_chk(unsigned char *puchMsg, unsigned char length);
void delay_ms(unsigned int ms);
void RunLEDDIS(void);
void write_to_flash(void);
void read_from_flash(void);
/**************************************************************************/

void delay_ms(unsigned int ms)
{
    unsigned int us = 1000;
    while(ms--)
    {
        delay1(us);
    }
}
/**************************************************************************/
void Watchdog_Init (void)
{
    WDTCN &= ~0x80;                     // WDTCN.7 must be logic 0 when setting
    // the interval.
    WDTCN |= 0x07;                      // Set the WDTCN[2-0] to 110b
}
//------------------------------------------------------
// 名    称：Initial
// 功    能：初始化模块
//------------------------------------------------------
void Initial(void)
{
    OSCILLATOR_Init ();
    Watchdog_Init ();
    PORT_Init ();
    Timer01_Init();
    //init_INT0();
    //init_INT1();
    init_para();
    //E2PROM初始化
    WDTCN = 0xA5;  //看门狗复位
    EA = 1;	//开中断
}
void Timer01_Init(void)
{
    char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page
    SFRPAGE = TIMER01_PAGE;             // Set SFR page
    TMOD = (TMOD & 0x00) | 0x21; //T0工作于方式1，T1工作于方式2
    PCON = 0x00;		//电源和SMOD控制字
    TH1  = 0xfd;		//9600 bit band:TH1=0xfd;1200 bit:TH1=0xe8
    TL1  = 0xfd;		//T1常数设置bt=11059200/(32*12*(256-TH1))
    TF1  = 0;		//定时器1溢出标志位清零
    TR1  = 1;		//T1开始计数 bt=28800/(256-TH1)
    TH0  = 0xDC;      //Reset 10ms interrupt
    TL0  = 00;
    TF0  = 0;
    TR0  = 1;
    ET0 = 1;
    SCON0 = 0x50;		//串行通讯方式一：起始位(L)、8位数据位、无校验位、停止位(H)
//    SCON0 = 0xD0;       // Serial Port Control Register
    //SCON = 0x40;		//串行通讯方式一：起始位(L)、8位数据位、无校验位、停止位(H)
    ES0 = 1;	//串行中断允许
    SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
}
/////////////////////////////
//定时0中断,模式1,16位定时计数器, 时钟4分频 ,高优先级
//T0=65536-1000us*11.0592/4=0xF533      时钟4分频下的1毫秒
//T0=65536-1000us*11.0592/12=0xFC66     时钟12分频下的1毫秒
//T0=65536-10000us*11.0592/12=0xDC00    时钟12分频下的10毫秒
void Timer0_ISR (void) interrupt 1
{
    char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page
    SFRPAGE = TIMER01_PAGE;
    TH0 = 0xDC;    //Reset 10ms interrupt
    TL0 = 0x00;
    WDTCN = 0xA5;                    // Reset the WDT
    SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
    SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page
    T0Counter5++;
    SFRPAGE = 0x0F;

    //
    if(MONITOR_INPUT1 == 0)
    {
        Sensordate1 = Sensordate1 | 0x01;
    }
    else
    {
        Sensordate1 = Sensordate1 & 0xFE;
    }
    //
    if(MONITOR_INPUT2 == 0)
    {
        Sensordate1 = Sensordate1 | 0x02;
    }
    else
    {
        Sensordate1 = Sensordate1 & 0xFD;
    }
    //
    if(MONITOR_INPUT3 == 0)
    {
        Sensordate1 = Sensordate1 | 0x04;
    }
    else
    {
        Sensordate1 = Sensordate1 & 0xFB;
    }
    //
    if(MONITOR_INPUT4 == 0)
    {
        Sensordate1 = Sensordate1 | 0x08;
    }
    else
    {
        Sensordate1 = Sensordate1 & 0xF7;
    }
    //
    if(MONITOR_INPUT5 == 0)
    {
        Sensordate1 = Sensordate1 | 0x10;
    }
    else
    {
        Sensordate1 = Sensordate1 & 0xEF;
    }
    //
    if(MONITOR_INPUT6 == 0)
    {
        Sensordate1 = Sensordate1 | 0x20;
    }
    else
    {
        Sensordate1 = Sensordate1 & 0xDF;
    }
    //
//    if(MONITOR_INPUT7 == 0)
//    {
//        Sensordate1 = Sensordate1 | 0x40;
//    }
//    else
//    {
//        Sensordate1 = Sensordate1 & 0xBF;
//    }
    //
    if(MONITOR_INPUT8 == 0)
    {
        Sensordate1 = Sensordate1 | 0x80;
    }
    else
    {
        Sensordate1 = Sensordate1 & 0x7F;
    }
    //
    if(MONITOR_INPUT9 == 0)
    {
        Sensordate2 = Sensordate2 | 0x01;
    }
    else
    {
        Sensordate2 = Sensordate2 & 0xFE;
    }
    //
    if(MONITOR_INPUT10 == 0)
    {
        Sensordate2 = Sensordate2 | 0x02;
    }
    else
    {
        Sensordate2 = Sensordate2 & 0xFD;
    }
    //
    if(MONITOR_INPUT11 == 0)
    {
        Sensordate2 = Sensordate2 | 0x04;
    }
    else
    {
        Sensordate2 = Sensordate2 & 0xFB;
    }
    //
    if(MONITOR_INPUT12 == 0)
    {
        Sensordate2 = Sensordate2 | 0x08;
    }
    else
    {
        Sensordate2 = Sensordate2 & 0xF7;
    }
    SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
}

//-----------------------------------------------------------------------------
// Port_Init ()
//-----------------------------------------------------------------------------

void PORT_Init (void)
{

    SFRPAGE = 0x0F;
    XBR0 = 0x04;	// XBAR0: Initial Reset Value  串口0
    XBR1 = 0x00;	// XBAR1: Initial Reset Value
    XBR2 = 0x40;	// XBAR2: Initial Reset Value  交叉总开关
    XBR3 = 0x00;    // XBAR3: Initial Reset Value
// Select Pin I/0

// NOTE: Some peripheral I/O pins can function as either inputs or
// outputs, depending on the configuration of the peripheral. By default,
// the configuration utility will configure these I/O pins as push-pull
// outputs.
    // Port configuration (1 = Push Pull Output)
    SFRPAGE = 0x0F;
    P0MDOUT = 0xFD; // 1111 1001b Output configuration for P0
    P1MDOUT = 0x00; // Output configuration for P1
    P2MDOUT = 0x00; // Output configuration for P2
    P3MDOUT = 0x00; // Output configuration for P3

    P4MDOUT = 0x00; // Output configuration for P4
    P5MDOUT = 0x00; // Output configuration for P5
    P6MDOUT = 0x00; // Output configuration for P6
    P7MDOUT = 0x00; // Output configuration for P7

    P1MDIN = 0xFF;  // Input configuration for P1
    P2MDIN = 0xFF;  // Input configuration for P2
    P3MDIN = 0xFF;  // Input configuration for P3

// View port pinout

    // The current Crossbar configuration results in the
    // following port pinout assignment:
    // Port 0
    // P0.0 = UART0 TX        (Push-Pull Output)(Digital)
    // P0.1 = UART0 RX        (Open-Drain Output/Input)(Digital)
    // P0.2 = UART1 TX        (Push-Pull Output)(Digital)
    // P0.3 = UART1 RX        (Open-Drain Output/Input)(Digital)
    // P0.4 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P0.5 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P0.6 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P0.7 = GP I/O          (Open-Drain Output/Input)(Digital)

    // Port 1
    // P1.0 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P1.1 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P1.2 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P1.3 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P1.4 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P1.5 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P1.6 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P1.7 = GP I/O          (Open-Drain Output/Input)(Digital)

    // Port 2
    // P2.0 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P2.1 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P2.2 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P2.3 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P2.4 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P2.5 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P2.6 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P2.7 = GP I/O          (Open-Drain Output/Input)(Digital)

    // Port 3
    // P3.0 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P3.1 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P3.2 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P3.3 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P3.4 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P3.5 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P3.6 = GP I/O          (Open-Drain Output/Input)(Digital)
    // P3.7 = GP I/O          (Open-Drain Output/Input)(Digital)

    SFRPAGE = 0x00;
    EMI0CF = 0x27;//27  // External Memory Configuration Register

}
//-----------------------------------------------------------------------------
// OSCILLATOR_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// This function initializes the system clock to use an external 22.1184MHz
// crystal.
//
//-----------------------------------------------------------------------------
void OSCILLATOR_Init (void)
{
    int i;                              // Software timer
    char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page
    SFRPAGE = CONFIG_PAGE;              // Set SFR page
    OSCXCN = 0x67;                      // Enable external crystal osc.
    for (i = 0; i < 256; i++);          // Wait at least 1ms

    while (!(OSCXCN & 0x80));           // Wait for crystal osc to settle

    CLKSEL = 0x01;
    // Select external crystal as SYSTEMCLOCK source

    SFRPAGE = SFRPAGE_SAVE;             // Restore SFRPAGE
}

//////////////////////////
//初始化变量
//////////////////////////
void init_para(void)
{
    /***********************************/
    char SFRPAGE_SAVE;
    SFRPAGE  = CONFIG_PAGE;
    Sensordate1 = 0;
    Sensordate2 = 0;
    SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page
    SFRPAGE = 0x0F;
    RE1 = 0;
    DE1 = 0;
    SFRPAGE = SFRPAGE_SAVE;             // Restore SFRPAGE


}
//-----------------------------------------------------------
// 名    称：Serial_Port_Interrupt
// 功    能：串口中断服务程序
//-----------------------------------------------------------
//# pragma disable
# pragma enable
void Serial_Port_Interrupt(void) interrupt 4
{
    // Save Current SFR page
    unsigned int   TEMP, K = 0;
    unsigned char   xdata Receiv_Msg[R_MaxLen];
    unsigned char   xdata TEMP1, TEMP2, len = 0, j = 0, Check_Sum = 0;
    char SFRPAGE_SAVE = SFRPAGE;

    SFRPAGE = UART0_PAGE;
    if(!RI0)		return ;		//误动作，退出
    //接收文件头
    RI0 = 0;
    Receiv_Msg[j] = SBUF0;	//接收的第一字节
    TEMP1 = Receiv_Msg[j];
    Check_Sum += Receiv_Msg[j];    
    j++;
    K = 0;

    while(!RI0)			//超时处理
    {
        K++;
        if(K > Delay_Times)
            goto END_ISR;
    }
    //BEEP=1;		//错误退出
    RI0 = 0;
    Receiv_Msg[j] = SBUF0;		//接收的第二字节
    TEMP2 = Receiv_Msg[j];
    Check_Sum += Receiv_Msg[j];
    j++;
    TEMP = ((UINT)TEMP1 << 8) + TEMP2;	//合成接收的文件头
    if(TEMP != 0xEFEF)		//查接收的文件头
        goto END_ISR;
    //接收长度字节
    K = 0;
    while(!RI0)
    {
        K++;
        if(K > Delay_Times)
            goto END_ISR;
    }
    RI0 = 0;
    Receiv_Msg[j] = SBUF0;		
    len = Receiv_Msg[j];
    Check_Sum += Receiv_Msg[j];
    j++;
    //接收数据、指令字节
    for(; j < len + 2; j++)
    {
        K = 0;
        while(!RI0)
        {
            K++;
            if(K > Delay_Times)
                goto END_ISR ;
        }
        RI0 = 0;
        Receiv_Msg[j] = SBUF0;
        Check_Sum += Receiv_Msg[j];
    }
    //接收校验和
    K = 0;
    while(!RI0)
    {
        K++;
        if(K > Delay_Times)
            goto END_ISR;
    }
    RI0 = 0;
    TEMP1 = SBUF0;
		//暂时屏蔽
//    if(TEMP1 != Check_Sum)			//校验和的校验比较
//        goto END_ISR;
    ES0 = 0;
    m_comand = Receiv_Msg[4];

    //身份验证
//    if(Receiv_Msg[3] != SN1)
//    {
//        goto END_ISR;			//身份不符。退出。
//    }
    //指令识别
    R_FLAG = 1;				//标志位置1，允许发送数据
    switch(m_comand)
    {
    case 0x01:
        LED1 = 0;
        break;
    default:
        break;
    }
END_ISR:
    RI0  = 0;
    ES0  = 1;
    SFRPAGE = SFRPAGE_SAVE;             // Restore SFRPAGE
    return;

}


void init_INT0 (void)
{
    char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page
    SFRPAGE  = CONFIG_PAGE;        //Port SFR's on Configuration page
    XBR1  |= 0x04;                 //配置INT0管脚
    SFRPAGE  = LEGACY_PAGE;
    EA = 0;
    EX0 = 1;    //使能INT0
    PX0 = 1;    //高优先级
    IT0 = 1;    //下降沿触发中断
    EA = 1;
    SFRPAGE = SFRPAGE_SAVE;             // Restore SFRPAGE
}
//对应于X轴为左边的限位开关，对应于Y轴为上边的限位开关
//外部中断INT0 ，高优先级
void INT0_ISR (void) interrupt 0
{
    unsigned char temp;
    temp = SFRPAGE;
    SFRPAGE  = LEGACY_PAGE;
    EX0 = 0 ;   //关外部中断0
    IE0 = 0;    //INT0中断复位
    EX0 = 1 ;   //开外部中断0
    SFRPAGE = temp;
}
void init_INT1 (void)
{
    char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page
    SFRPAGE  = CONFIG_PAGE; //Port SFR's on Configuration page
    XBR1  |= 0x10;          //配置INT0管脚
    SFRPAGE  = LEGACY_PAGE;
    EA = 0;
    EX1 = 1;               //使能INT1
    PX1 = 1;               //高优先级
    IT1 = 1;               //下降沿触发中断
    EA = 1;
    SFRPAGE = SFRPAGE_SAVE;             // Restore SFRPAGE
}

//外部中断INT1 ，高优先级
//对应于X轴为右边的限位开关，对应于Y轴为下边的限位开关
void INT1_ISR (void) interrupt 2
{
    unsigned char temp;
    temp = SFRPAGE;
    SFRPAGE  = LEGACY_PAGE;
    EX1 = 0 ;           //关外部中断1

    IE1 = 0;             //INT1中断复位
    EX1 = 1 ;            //开外部中断1
    SFRPAGE = temp;
}

//仅配为RS485模式  与RS422的区别就是在发送程序中，增加了发送前，关闭接受端口；发送完即打开接受端口。
//该串口发送函数里集成了CRC校验，专用于modbus发送
void Uart0Send(unsigned char *buf, unsigned char bufsize )
{
    unsigned char i = 0;
    unsigned int k = 0;
    unsigned int crc_z = 0;
    char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page

    SFRPAGE = CONFIG_PAGE;

    RE1 = 1;
    DE1 = 1;
    delay_ms(10);
    ES0 = 0;
    SFRPAGE = UART0_PAGE;
    do
    {
        SBUF0 = buf[i];
        while(!TI0);
        TI0 = 0;
        i++;
    } while(i < bufsize);
    ES0 = 1;
    SFRPAGE = CONFIG_PAGE ;
    RE1 = 0;
    DE1 = 0;
    delay_ms(10);

    SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
}

//------------------------------------------------------
// 名    称：Send_Packet
// 功    能：发送所采集的性能量数据
//------------------------------------------------------
void Send_Packet(void)
{
    unsigned char i = 0;
    unsigned int k = 0;
    unsigned int crc_z = 0;
    unsigned char   Check_Sum = 0;
    char SFRPAGE_SAVE = SFRPAGE;
   
    Send_Msg[0] = 0xAB;	//双字节字头FLAG2：EDFA
    Send_Msg[1] = 0xCD;
    Send_Msg[3] = SN1;		//产品序列号(保留位)
    Send_Msg[4] = m_comand;
    switch(Send_Msg[4])
    {
    case 0x01:
        Send_Msg[2] = 0X05;
        Send_Msg[5] = Sensordate1;
        Send_Msg[6] = Sensordate2;
        break;

    default:
        break;
    }
    //数据发送
    for(i = 0; i < Send_Msg[2] + 2; i++)
    {
        Check_Sum += Send_Msg[i];
    }
		Send_Msg[7] = Check_Sum;
		Uart0Send(Send_Msg,8);

    R_FLAG = 0;;

    SFRPAGE = SFRPAGE_SAVE;             // Restore SFRPAGE
}
//void Send_Packet(void)
//{
//    unsigned char i = 0;
//    unsigned int k = 0;
//    unsigned int crc_z = 0;
//    unsigned char   Check_Sum = 0;
//    char SFRPAGE_SAVE;

//    SFRPAGE_SAVE = SFRPAGE;
//    SFRPAGE = CONFIG_PAGE;
//    RE1 = 1;
//    DE1 = 1;
//    Send_Msg[0] = 0xAB;	//双字节字头FLAG2：EDFA
//    Send_Msg[1] = 0xCD;
//    Send_Msg[3] = SN1;		//产品序列号(保留位)
//    Send_Msg[4] = m_comand;
//    switch(Send_Msg[4])
//    {
//    case 0x01:
//        Send_Msg[2] = 0X05;
//        Send_Msg[5] = Sensordate1;
//        Send_Msg[6] = Sensordate2;
//        break;

//    default:
//        break;
//    }
//    //数据发送
//    SFRPAGE = SFRPAGE_SAVE;             // Restore SFRPAGE
//    SFRPAGE_SAVE = SFRPAGE;
//    SFRPAGE = UART0_PAGE;
//    TI0 = 0;
//    for(i = 0; i < Send_Msg[2] + 2; i++)
//    {
//        ACC = Send_Msg[i];
//        TB80 = P;
//        SBUF0 = Send_Msg[i];
//        Check_Sum += Send_Msg[i];
//        while(!TI0);
//        TI0 = 0;
//    }
////    ACC = Check_Sum;
////    TB80 = P;
//    SBUF0 = Check_Sum;	//校验和发送，即第Send_Msg[2]+2个数据
//    while(!TI0);
//    TI0 = 0;
//    SFRPAGE = SFRPAGE_SAVE;             // Restore SFRPAGE
//    SFRPAGE_SAVE = SFRPAGE;
//    SFRPAGE = CONFIG_PAGE;
//    R_FLAG = 0;;
//    RE1 = 0;
//    DE1 = 0;
//    SFRPAGE = SFRPAGE_SAVE;             // Restore SFRPAGE
//}

//-----------------------------------------------------------------------------
//自定义延时
//延时时间约为(us N10)毫秒
//-----------------------------------------------------------------------------
void delay1( unsigned int us)
{
    unsigned int i = us;
    while(i--) ;
}

void RunLEDDIS(void)//运行显示
{
    if(T0Counter5 >= 20) //200ms时间定时
    {
        T0Counter5 = 0 ;
        LED2 = !LED2;
    }
}

unsigned int crc_chk(unsigned char *puchMsg, unsigned char length)
{
    int j;
    unsigned int crc_reg = 0xFFFF;
    while(length--)
    {
        crc_reg ^= *puchMsg++;


        for(j = 0; j < 8; j++)
        {
            if(crc_reg & 0x01)
            {
                crc_reg = (crc_reg >> 1) ^ 0xA001;
            }
            else
            {
                crc_reg = crc_reg >> 1;
            }
        }
    }
    return crc_reg;
}



void SystemControl(void)//系统控制
{

    ;
}
/********************************************************
////自动运行任务安排
********************************************************/



//////////////////////////
//将定义的Flash缓冲区写入Flash
//////////////////////////
void write_to_flash(void)
{
    unsigned char xdata *pwrite ;
    unsigned char i;
    EA = 0; //关中断
    SFRPAGE = LEGACY_PAGE;
    FLSCL = FLSCL | 0x01; //允许写擦除flash
    PSCTL = PSCTL | 0x07; //允许写擦除flash
    pwrite = 0x00;
    *pwrite = 0;  //擦除flash
    PSCTL = PSCTL & 0xFD; //禁止擦除
    i = 0;
    do {
        //	*pwrite++=Servopara[i];
        i++;
    } while(i < 107);
    PSCTL = PSCTL & 0xFA; //禁止flash写
    FLSCL = FLSCL & 0xFE; //禁止写擦除flash
    EA = 1; //开中断
}

//////////////////////////
//将Flash中保存的数据读入Flash缓冲区
//////////////////////////
void read_from_flash(void)
{
    unsigned char code *pread ;
    unsigned char i;
    EA = 0;
    SFRPAGE = LEGACY_PAGE;
    PSCTL = PSCTL | 0x04; //指向flash地址0～0x7f
    pread = 0x00;
    i = 0;
    do {
        //Servopara[i]=*pread++;
        i++;
    } while(i < 107);
    PSCTL = PSCTL & 0xFB; //恢复 64k flash
    EA = 1; // 允许全局中断
}

//
unsigned char buf[11];
//////////////////////////
//         主程序
//////////////////////////
void main (void)
{
    //SFRPAGE = CONFIG_PAGE;
    Initial();
    SFRPAGE = CONFIG_PAGE;
	
    while(1)
    {
        //SystemControl();              //系统控制（控制信息来源，can\RS485\中断信号）
       
			if(R_FLAG == 1)
        {
            ES0 = 0;
            Send_Packet();
            ES0 = 1;
        }
        RunLEDDIS();                  //运行显示

    }
}
