#include <zk.h>

//-----------------------------------------------------------------------------
// C8051F040的SFR定义
//-----------------------------------------------------------------------------

//定义SJA1000T寄存器地址
xdata unsigned char CAN3_MODE _at_ 0xFD00 ;
xdata unsigned char CAN3_CMR _at_ 0xFD01 ;
xdata unsigned char CAN3_SR _at_ 0xFD02 ;
xdata unsigned char CAN3_IR _at_ 0xFD03 ;
xdata unsigned char CAN3_IREN _at_ 0xFD04;
xdata unsigned char CAN3_AMR _at_ 0xFD05 ;
xdata unsigned char CAN3_BTR0 _at_ 0xFD06 ;
xdata unsigned char CAN3_BTR1 _at_ 0xFD07 ;
xdata unsigned char CAN3_OCR _at_ 0xFD08 ;
xdata unsigned char CAN3_CDR _at_ 0xFD1F;


xdata unsigned char CAN3_FIFO16 _at_ 0xFD10 ;        //CAN3_FIFO窗口 ， 采用标准帧
xdata unsigned char CAN3_FIFO17 _at_ 0xFD11 ;
xdata unsigned char CAN3_FIFO18 _at_ 0xFD12 ;
xdata unsigned char CAN3_FIFO19 _at_ 0xFD13 ;
xdata unsigned char CAN3_FIFO20 _at_ 0xFD14 ;
xdata unsigned char CAN3_FIFO21 _at_ 0xFD15 ;
xdata unsigned char CAN3_FIFO22 _at_ 0xFD16 ;
xdata unsigned char CAN3_FIFO23 _at_ 0xFD17 ;
xdata unsigned char CAN3_FIFO24 _at_ 0xFD18 ;
xdata unsigned char CAN3_FIFO25 _at_ 0xFD19 ;
xdata unsigned char CAN3_FIFO26 _at_ 0xFD1A ;

xdata unsigned char CAN3_RBSA _at_ 0xFD1E ;
xdata unsigned char CAN3_ERRLIMIT _at_ 0xFD0D;
xdata unsigned char CAN3_RXERR _at_ 0xFD0E;
xdata unsigned char CAN3_TXERR _at_ 0xFD0F ;
xdata unsigned char CAN3_ALC _at_ 0xFD0B;
xdata unsigned char CAN3_ECC _at_ 0xFD0C;


//-----------------------------------------------------------------------------
// 全局变量
//-----------------------------------------------------------------------------
xdata CAN3RECBUF1 CAN3RXbuffer1;
xdata CAN3RECBUF1 CAN3TXbuffer1;

bit Can3NewData1 = NO;
unsigned char CAN3FaultCounter = 0;


void init_can3 (void)
{

    SFRPAGE  = CONFIG_PAGE;        //Port SFR's on Configuration page
    XBR1  |= 0x10;     //配置INT0管脚
    SFRPAGE  = LEGACY_PAGE;
    EA = 0;
    EX1 = 1; //使能INT1
    PX1 = 0;//低优先级
    IT1 = 0; //低电平触发中断

    CAN3_MODE = 0x01;   //0X09; 复位模式
    CAN3_CDR = 0X88 ;  //选择PeliCAN 模式   ，时钟输出禁止(0x80 时钟输出)
    CAN3_IREN = 0x0D;  //数据溢出中断、接收中断、错误报警中断使能

    //配置验收滤波，用于过虑无关ID，只对通过滤波的ID产生接收中断
    //定义AMR
    CAN3_FIFO20 = 0xFF;   //高8位地址参与验收滤波 , 如果为1则对ACR为无关,
    CAN3_FIFO21 = 0xEF;   //RTR参与验收滤波,其余位无关
    CAN3_FIFO22 = 0xFF;   //无关
    CAN3_FIFO23 = 0xFF;   //无关
    //定义ACR
    CAN3_FIFO16 = 0x00;
    CAN3_FIFO17 = 0x00;
    CAN3_FIFO18 = 0x00;
    CAN3_FIFO19 = 0x00;

    //接收地址：0000,000*,***   //当FIFO20=0x01; FIFO16=0x01或0x00 时
    //接收地址：****,****,***   //当FIFO20=0xFF; FIFO16=**时
    //接收地址：0000,0001,***   //当FIFO20=0x00; FIFO16=0x01时

// TSEG1 = (Prop_Seg + Phase_Seg1 - 1) = 6 , 7
//  TSEG2 = (Phase_Seg2 - 1)           = 2
// SJW_p = (SJW - 1)                   = 1
    //CAN3_BTR0=0x40;    //  波特率=502693Hz
// CAN3_BTR0=0x41;    //  波特率=502693Hz/2
//  CAN3_BTR0=0x43;    //  波特率=502693Hz/4
// CAN3_BTR0=0x44;    //  波特率=502693Hz/5

// CAN3_BTR0=0x40;    //  波特率=500000Hz
// CAN3_BTR0=0x41;    //  波特率=500000Hz/2
    CAN3_BTR0 = 0x43;    //  波特率=500000Hz/4
// CAN3_BTR0=0x44;    //  波特率=500000Hz/5

    CAN3_BTR1 = 0xA7;  //通讯低速率时用0xA7,通讯高速率时用0x27
    //0x26(11.0592MHz),  0x27(12MHz)
    CAN3_OCR = 0xda;
    CAN3_RBSA = 0;
    CAN3_ECC = 0 ;
    CAN3_MODE = 0x08; //设置单滤波接收方式，返回工作状态
    EA = 1;
}

void can3_transmit(unsigned int id, unsigned char *buf)
{
    UINTUNION  temp;
    unsigned char r;
    T0Counter2 = 0;
    do {
        r = CAN3_SR ;
        if(T0Counter2 >= 2)
        {
            CAN3FaultCounter++;
            return; //延时20ms  超时退出
        }
    } while(((r & 0x10) == 0x10) || ((r & 0x04) == 0x00)); //||((r&0x08)==0x00));
    // 不要加入发送完毕状态判断，否则出现故障时，不能自恢复
    EA = 0;
    CAN3_FIFO16 = 0x08; //帧信息： 标准帧，8位数据

    temp.value = id << 5;

    CAN3_FIFO17 = temp.buf[0];   //ID1
    CAN3_FIFO18 = (temp.buf[1] & 0xE0) | 0x08 ; //ID2

    CAN3_FIFO19 = buf[0];
    CAN3_FIFO20 = buf[1];
    CAN3_FIFO21 = buf[2];
    CAN3_FIFO22 = buf[3];
    CAN3_FIFO23 = buf[4];
    CAN3_FIFO24 = buf[5];
    CAN3_FIFO25 = buf[6];
    CAN3_FIFO26 = buf[7];

//     LedBlink(7, 1,1 );          //数据指示灯
    CAN3_CMR = 0x01;
    EA = 1;
    CAN3FaultCounter = 0;
}

void can3_receive (unsigned char *buf)
{
    buf[0] = CAN3_FIFO19;
    buf[1] = CAN3_FIFO20;
    buf[2] = CAN3_FIFO21;
    buf[3] = CAN3_FIFO22;
    buf[4] = CAN3_FIFO23;
    buf[5] = CAN3_FIFO24;
    buf[6] = CAN3_FIFO25;
    buf[7] = CAN3_FIFO26;
}

//CAN3控制器SJA1000T产生的外部中断INT1 ，高优先级
void INT1_ISR (void) interrupt 2
{
    unsigned char temp;
    unsigned char ir_state;
    UINTUNION   address;
    ir_state = CAN3_IR;
    if( (ir_state & 0x01) == 0x01 ) //接收中断
    {
        address.buf[0] = CAN3_FIFO17 ;
        address.buf[1] = CAN3_FIFO18 ;
        address.value = ( (address.value) >> 5) & 0x07FF;
        if( (address.value == 68) && ((CAN3_FIFO16 & 0x40) != 0x40)) //相同ID号 ,非远程帧
        {
            can3_receive (CAN3RXbuffer1.buf);  //
            if(CAN3RXbuffer1.buf[1] == 0xc2)
            {
                Runbiaoji4 = 1;
            }
            Mode_adress4 = 1;
            // can3buf1_process();
            Can3NewData1 = YES;
        }
        if( (address.value == 67) && ((CAN3_FIFO16 & 0x40) != 0x40)) //相同ID号 ,非远程帧
        {
            can3_receive (CAN3RXbuffer1.buf);  //
            if(CAN3RXbuffer1.buf[1] == 0xc2)
            {
                //Backinfor_flag=3;
                //Tansflag=true;
                switch(CAN3RXbuffer1.buf[2])
                {
                case 1:
                    /*Runbiaoji2=1;
                    /if((Runbiaoji1==1)&&(Runbiaoji2==1))
                    {
                    //SNSORSTATUS1=SNSORSTATUS1|0x20;
                    Runbiaoji1=0;
                    Runbiaoji2=0;
                    RUNSTATUS1=RUNSTATUS1&0xFB;
                    }*/
                    SNSORSTATUS1 = SNSORSTATUS1 | 0x10; //定位运行结束
                    break;
                case 2:
                    SNSORSTATUS2 = SNSORSTATUS2 | 0x10; //定位运行结束
                    break;
                default:
                    break;
                }
            }
            if(CAN3RXbuffer1.buf[1] == 0xc3) //推药结束
            {
                switch(CAN3RXbuffer1.buf[2])
                {
                case 1:
                    //SNSORSTATUS1=SNSORSTATUS1|0x40;
                    //RUNSTATUS1=RUNSTATUS1&0xF7;
                    SNSORSTATUS1 = SNSORSTATUS1 | 0x20;
                    break;
                case 2:
                    SNSORSTATUS2 = SNSORSTATUS2 | 0x20;
                    break;
                default:
                    break;
                }
            }
            Mode_adress3 = 1;
            Can3NewData1 = YES;
        }

        if( (address.value == 66) && ((CAN3_FIFO16 & 0x40) != 0x40)) //相同ID号 ,非远程帧
        {
            can3_receive (CAN3RXbuffer1.buf);  //
            if(CAN3RXbuffer1.buf[1] == 0xc2)
            {
                Runbiaoji3 = 1;
            }
            Mode_adress2 = 1;
            // can3buf1_process();
            Can3NewData1 = YES;
        }
        //药蓝管理和药蓝提升
        /*if( (address.value==66)&&((CAN3_FIFO16&0x40)!=0x40))   //相同ID号 ,非远程帧
        {
           can3_receive (CAN3RXbuffer1.buf);  //
           switch(CAN3RXbuffer1.buf[1])
           {
        	case 0xc2:
        	//Backinfor_flag=5;
        	Runbiaoji3=1;
        	//Tansflag=true;
        	break;
        	case 0xc3:
        	//Backinfor_flag=6;
        	//Tansflag=true;
        	break;
        	case 0xc4:
        	//Backinfor_flag=7;
        	//Tansflag=true;
        	Runbiaoji1=1;
        	if((Runbiaoji1==1)&&(Runbiaoji2==1))
        	{
        	//SNSORSTATUS1=SNSORSTATUS1|0x20;
        	Runbiaoji1=0;
        	Runbiaoji2=0;
        	RUNSTATUS1=RUNSTATUS1&0xFB;
        	}
        	break;
                default:
        	break;

          }
        }
             if( (address.value==65)&&((CAN3_FIFO16&0x40)!=0x40))   //相同ID号 ,非远程帧
          {
                can3_receive (CAN3RXbuffer1.buf);  //
                if(CAN3RXbuffer1.buf[1]==0xc2)
                {
                RUNSTATUS1=RUNSTATUS1&0xFE;
               }
               // can3buf1_process();
               Mode_adress2=1;
                Can3NewData1=YES;
             }*/
        CAN3_CMR = 0x04;            //释放接收缓冲器
        LedBlink(7, 1, 1 );         //数据指示灯
        LedBlink(8, 0, 0 );         //故障指示灭
        CAN3FaultCounter = 0;
    }

    if((ir_state & 0x08) != 0x0) //数据溢出中断
    {
        CAN3_CMR = 0x0C; //清除数据溢出；释放接收缓冲器
    }
    if((ir_state & 0x04) != 0x0) //错误报警中断
    {

        if((CAN3_SR & 0x80) != 0x0)
        {
            CAN3_MODE = 0x08 ;
        }
        LedBlink(8, 1, 0 );         //故障指示
    }
    temp = CAN3_ALC; //释放仲裁丢失捕捉寄存器
    temp = CAN3_ECC; //释放错误代码捕捉寄存器
    IE1 = 0; //INT0中断复位
}

