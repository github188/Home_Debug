
//------------------------------------------------------------------------------
// UART1.c       扩展为网络接口
//------------------------------------------------------------------------------
#include <zk.h>
//#include <system.h>



TRANSBUF1 TransBuf1;
RECBUF1  RecBuf1;

unsigned char xdata RecPointer1 = 0;
xdata unsigned char Rec1[RECBUF1SIZE];
bit RecFlag1 = NO;
bit TI1Flag = NO;
bit UART1_Refresh = NO;

void UART1_ISR (void);
void UART1Send(void);
extern void LedBlink(unsigned char num, unsigned char state, unsigned char blink );
//-----------------------------------------------------------------------------
// 中断服务程序
// UART1中断
// 使用定时器1作为时钟，优先级高
//usart1作为IPORT
//-----------------------------------------------------------------------------
void UART1_ISR (void) interrupt 20
{
    unsigned char n;
    unsigned char i;
    unsigned char sum;
    unsigned char temp;

    temp = SFRPAGE;
    SFRPAGE = UART1_PAGE;
    if(RI1 == 1)
    {
        RI1 = 0;
        n = SBUF1;
        if ((RecFlag1 == NO))	//&& (n==0x04)
        {
            RecFlag1 = YES;
        }
        if (RecFlag1 == YES)
        {
            Rec1[RecPointer1++] = n;
            if (RecPointer1 == RECBUF1SIZE)
            {
                sum = 0;
                i = 0;
                do
                {
                    sum += Rec1[i];
                    i++;
                } while(i < RECBUF1SIZE - 1);
                if( sum == Rec1[RECBUF1SIZE - 1] )
                {
                    for(i = 0 ; i < RECBUF1SIZE ; i++)
                    {
                        RecBuf1.buf[i] = Rec1[i];
                    }
                    UART1_Refresh = YES;
                    LedBlink(3, 1, 1 );         //串口运行指示灯
                }
                RecPointer1 = 0;
                RecFlag1 = NO;
            }
        }
    }
    else if (TI1 == 1)
    {
        TI1 = 0;
        TI1Flag = YES;
    }
    SFRPAGE = temp;
}

//-----------------------------------------------------------------------------
// UART1发送程序
// 使用定时器1作为时钟，优先级高
//发送前应该关中断，防止数据被意外分包处理
//-----------------------------------------------------------------------------
void UART1Send(void)
{
    unsigned char i = 0;

    TransBuf1.trans1.checkout = 0;
    do
    {
        TransBuf1.trans1.checkout += TransBuf1.buf[i];
        i++;
    } while(i < TRANSBUF1SIZE - 1);

    i = 0;
    do
    {
        SFRPAGE = UART1_PAGE;
        SBUF1 = TransBuf1.buf[i];
        while(TI1Flag == NO);
        TI1Flag = NO;
        i++;
    } while(i < TRANSBUF1SIZE);
}

