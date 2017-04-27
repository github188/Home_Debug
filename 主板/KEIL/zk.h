#include <c8051f040.h>                 // SFR declaration

//-----------------------------------------------------------------------------
// ȫ�ֳ���
//-----------------------------------------------------------------------------
#define YES 1
#define OK 1
#define NO  0

#define K  112.676

#define		ZTS_CANID		2	//��������CAN ID
#define		YTS_CANID		4	//��������CAN ID
#define		DM_CANID		3	//�����CAN ID


typedef unsigned int Uint;
typedef unsigned char Uchar;

#define SYSCLK     11059200     // Internal oscillator frequency in Hz
#define Bite_Delay_Time 500

#define TRANSBUF0SIZE	8
#define RECBUF0SIZE  8
#define TRANSBUF1SIZE  8
#define RECBUF1SIZE  8

//���� C8051F040 CAN ���ڽ��յ���Ϣ��
// (��Ϣ�ŷ�Χ1~16,����16�����޸�CAN1�жϷ������)
#define MSGNUMA   1
#define MSGNUMB   2

#define CAN1RXID_SWJ		 5		//16 ��λ���������ݵĵ�ַ
#define CAN1TXID_SWJ		 80		// ��λ���������ݵĵ�ַ

#define CAN2TXID_Uion	     518	// �ײ���������չ������ݵĵ�ַ

#define BYTE_Amount	     4	        // �ж϶����Ƿ�������ֽ��ܸ���

#define MAXZREO_TIME	    300	    // �ȴ�ԭ�����ʱ��

#define Driver_Board_Amount	 32	    // �ײ��������ܸ��� 
#define Floor_Number         10	    // ���ܵĲ���
#define Jijia_Amount	6	        // ���ܵ��ܸ���

sbit RS485_EN = P3 ^ 0;

sbit OUTPUT1 = P1 ^ 3;
sbit OUTPUT2 = P1 ^ 2;
sbit OUTPUT3 = P1 ^ 0;
sbit OUTPUT4 = P1 ^ 1;

sbit Monitor1 = P0 ^ 6;
sbit Monitor2 = P0 ^ 7;
sbit Monitor3 = P1 ^ 4;
sbit Monitor4 = P1 ^ 5;

sbit K1 = P5 ^ 1;
sbit K2 = P5 ^ 2;
sbit K3 = P5 ^ 3;
sbit K4 = P5 ^ 4;

sbit FM_SDA = P5 ^ 5;
sbit FM_SCL = P5 ^ 6;
sbit FM_WP = P5 ^ 7;

sbit LCD_CS = P2 ^ 4; //����ʱΪCS
sbit LCD_SID = P2 ^ 5; //����ΪSID
sbit LCD_SCLK = P2 ^ 6; //����Ϊʱ��LCD_SCLK
sbit LCD_RST = P2 ^ 7; //��������λ

sbit GM_MS = P4 ^ 0;
sbit STADD0 = P4 ^ 1;
sbit STADD1 = P4 ^ 2;
sbit SRADD0 = P4 ^ 3;
sbit SRADD1 = P4 ^ 4;
sbit GM_RST = P5 ^ 0;

sbit NET_IO = P3 ^ 1;
sbit NET_CFG = P3 ^ 2;
sbit NET_RST = P3 ^ 3;



//------------------------------------�������Ͷ���--------------------------------------
//-----------------------------------------------------------------------------
typedef struct
{
    unsigned char	DCT_Input_Command[4];            // 32���ײ���������������״̬
    unsigned char	DCT_OK_Result[4];                // 32���ײ������嶯��OK���״̬
    unsigned char	DCT_OK_Action[4];                // 32���ײ������嶯������״̬
    unsigned char   DCT_Bite[Driver_Board_Amount];   // �ж�32���ײ���������ϴ�����
} SYSDAT;

//-----------------------------------------------------------------------------
// ���Ͷ���
//-----------------------------------------------------------------------------

//----------------������ȡ�ߵ�λ�ֽ�
typedef union
{
    int value;
    unsigned char buf[2];
} INTUNION;

typedef union
{
    unsigned int value;
    unsigned char buf[2];
} UINTUNION;

typedef union
{
    long int value;
    unsigned char buf[4];
} LONGUNION;


// ----------------CAN1 buffer definition  ----����λ����ϵ

typedef struct                     //�������������
{
    unsigned char  command;    // 1
    unsigned char  index1;     // 2
    unsigned char  address1;   // 3
    unsigned char  address2;   // 4
    unsigned char  off_time;   // 5
    unsigned char  on_time;    // 6
    unsigned char  aim_amount; // 7
    unsigned char  checkout;   // 8
} CAN1REC1;

typedef struct                     //����λ��������������϶����������ݼ���Ӧ�Ĺ��ϴ���
{
    unsigned char  command;    // 1
    unsigned char  index1;     // 2
    unsigned char  address1;   // 3
    unsigned char  address2;   // 4
    unsigned char  bite;       // 5
    unsigned char  blank1;     // 6
    unsigned char  amount;     // 7
    unsigned char  checkout;   // 8
} CAN1TRANS1;

typedef struct                     //����λ���������������OK�ź�����
{
    unsigned char  command;     // 1
    unsigned char  index1;      // 2
    unsigned char  blank1;      // 3
    unsigned char  blank2;      // 4
    unsigned char  ALL_Result_OK_Sign; // 5
    unsigned char  ALL_Action_OK_Sign; // 6
    unsigned char  blank3;      // 7
    unsigned char  checkout;    // 8
} CAN1TRANS2;

typedef struct                     //����λ������ҩ������Ӧ
{
    unsigned char  command;    // 1
    unsigned char  index1;     // 2
    unsigned char  order1;     // 3
    unsigned char  order2;     // 4
    unsigned char  blank1;     // 5
    unsigned char  blank2;     // 6
    unsigned char  blank3;     // 7
    unsigned char  checkout;   // 8
} CAN1TRANS3;

//-----------------------------------------------------------------------------
// ������ ����������ֽṹ���Ա�ʹ��
//-----------------------------------------------------------------------------
typedef union
{
    CAN1REC1 act_buf;
    unsigned char buf[8];
} CAN1RECBUF1;

typedef union
{
    CAN1TRANS1 amount_buf;
    CAN1TRANS2 OK_buf;
    unsigned char buf[8];
} CAN1TRANSBUF1;


// ----------------CAN2 buffer definition-----�������������ư���ϵ

typedef struct                     //�������������
{
    unsigned char  command;    // 1
    unsigned char  index1;      // 2
    unsigned char  address1;   // 3
    unsigned char  address2;   // 4
    unsigned char  off_time;   // 5
    unsigned char  on_time;    // 6
    unsigned char  aim_amount; // 7
    unsigned char  checkout;   // 8
} CAN2TRANS1;

typedef struct                     //�������
{
    unsigned char  command;    // 1
    unsigned char  index2;      // 2
    unsigned char  address1;   // 3
    unsigned char  address2;   // 4
    unsigned char  statement;  // 5
    unsigned char  blank1;     // 6
    unsigned char  blank2;     // 7
    unsigned char  checkout;   // 8
} CAN2TRANS2;

typedef struct                     //�ײ��������������϶����������������ϴ�������
{
    unsigned char  command;    // 1
    unsigned char  index1;     // 2
    unsigned char  address1;   // 3
    unsigned char  address2;   // 4
    unsigned char  bite;       // 5
    unsigned char  blank1;     // 6
    unsigned char  amount;     // 7
    unsigned char  checkout;   // 8
} CAN2REC1;

typedef struct                     //�ײ�����������OK��������
{
    unsigned char  command;    // 1
    unsigned char  index1;     // 2
    unsigned char  address1;   // 3
    unsigned char  address2;   // 4
    unsigned char  bite;       // 5
    unsigned char  Result_OK_Sign;// 6
    unsigned char  Action_OK_Sign;// 7
    unsigned char  checkout;   // 8
} CAN2REC2;
//-----------------------------------------------------------------------------
// ������ ����������ֽṹ���Ա�ʹ��
//-----------------------------------------------------------------------------
typedef union
{
    CAN2TRANS1 act_buf;
    unsigned char buf[8];
} CAN2TRANSBUF1;

typedef union
{
    CAN2TRANS2 light_buf;
    unsigned char buf[8];
} CAN2TRANSBUF2;

typedef union
{
    CAN2REC1 amount_buf;
    CAN2REC2 bite_buf;
    unsigned char buf[8];
} CAN2RECBUF1 ;

// -----------------CAN3 buffer definition----�봫�ʹ�ϵͳ��ϵ
typedef struct
{
    unsigned char  command;       // 1
    unsigned char  index;         // 2
    unsigned char  shake_hand1;   // 3
    unsigned char  shake_hand2;   // 4
    unsigned char  bakeup1;      // 5
    unsigned char  bakeup2;       // 6
    unsigned char  bakeup3;    // 7
    unsigned char  checkout;      // 8
} CAN3REC1;

typedef union
{
    CAN3REC1 rec;
    unsigned char buf[8];
} CAN3RECBUF1;


typedef struct
{
    unsigned char  address1;    // 1
    unsigned char  command;      // 2
    unsigned char  index;      // 3
    unsigned char  data1;      // 4
    unsigned char  data2;      // 5
    unsigned char  data3;      // 6
    unsigned char  data4;      // 7
    unsigned char  checkout;   // 8
} CAN3TRANS1;

typedef union
{
    CAN3TRANS1 trans;
    unsigned char buf[8];
} CAN3TRANSBUF1;

//-------------------��ͨ���ڶ��岿��-------------------------------
//���ذ巢��������Ϣ����λ��
typedef struct                               //����λ��������������϶����������ݼ���Ӧ�Ĺ��ϴ���
{
    unsigned char  command;    // 1
    unsigned char  index1;     // 2
    unsigned char  address1;   // 3
    unsigned char  address2;   // 4
    unsigned char  bite;       // 5
    unsigned char  blank1;     // 6
    unsigned char  amount;     // 7
    unsigned char  checkout;   // 8
} TRANSMIT01;

typedef struct                               //����λ��������������϶����������ݼ���Ӧ�Ĺ��ϴ���
{
    unsigned char  command;     // 1
    unsigned char  index1;      // 2
    unsigned char  blank1;      // 3
    unsigned char  blank2;      // 4
    unsigned char  ALL_Result_OK_Sign; // 5
    unsigned char  ALL_Action_OK_Sign; // 6
    unsigned char  blank3;      // 7
    unsigned char  checkout;    // 8
} TRANSMIT02;
typedef union
{
    TRANSMIT01 trans1;
    TRANSMIT02 trans2;
    unsigned char buf[TRANSBUF0SIZE];
} TRANSBUF0;

typedef struct                          //������λ���Ŀ���ָ��
{
    unsigned char  command;    // 1
    unsigned char  index1;      // 2
    unsigned char  address1;   // 3
    unsigned char  address2;   // 4
    unsigned char  off_time;   // 5
    unsigned char  on_time;    // 6
    unsigned char  aim_amount; // 7
    unsigned char  checkout;   // 8
} RECEIVE0;
typedef union
{
    RECEIVE0 recei1;
    unsigned char buf[RECBUF0SIZE];
} RECBUF0;

//-------------------���磨���ڣ����岿��-------------------------------
//���ذ巢��������Ϣ����λ��
typedef struct                     //����λ��������������϶����������ݼ���Ӧ�Ĺ��ϴ���
{
    unsigned char  command;    // 1
    unsigned char  index1;     // 2
    unsigned char  address1;   // 3
    unsigned char  address2;   // 4
    unsigned char  bite;       // 5
    unsigned char  blank1;     // 6
    unsigned char  amount;     // 7
    unsigned char  checkout;   // 8
} TRANSMIT11;

typedef struct                      //����λ��������������϶����������ݼ���Ӧ�Ĺ��ϴ���
{
    unsigned char  command;     // 1
    unsigned char  index1;      // 2
    unsigned char  blank1;      // 3
    unsigned char  blank2;      // 4
    unsigned char  ALL_Result_OK_Sign; // 5
    unsigned char  ALL_Action_OK_Sign; // 6
    unsigned char  blank3;      // 7
    unsigned char  checkout;    // 8
} TRANSMIT12;

typedef struct                      //ͨ�������ݸ�ʽ
{
    unsigned char  blank0;      // 1
    unsigned char  blank1;      // 2
    unsigned char  blank2;      // 3
    unsigned char  blank3;      // 4
    unsigned char  blank4;      // 5
    unsigned char  blank5;      // 6
    unsigned char  blank6;      // 7
    unsigned char  checkout;    // 8
} TRANSMIT13;

typedef union
{
    TRANSMIT11 amount_buf;
    TRANSMIT12 OK_buf;
    TRANSMIT13 trans1;
    unsigned char buf[TRANSBUF1SIZE];
} TRANSBUF1;

typedef struct                     //������λ���Ŀ���ָ��
{
    unsigned char  command;    // 1
    unsigned char  index1;     // 2
    unsigned char  address1;   // 3
    unsigned char  address2;   // 4
    unsigned char  off_time;   // 5
    unsigned char  on_time;    // 6
    unsigned char  aim_amount; // 7
    unsigned char  checkout;   // 8
} RECEIVE1;
typedef union
{
    RECEIVE1 act_buf;
    unsigned char buf[RECBUF1SIZE];
} RECBUF1;

typedef union
{
    long displace;
    unsigned char buf[4];
} Servomotordisplace;
//------------------






//-----------------------------------zk.c--------------------------------------
// ����ԭ��
//-----------------------------------------------------------------------------
extern void TIME0_ISR (void);
extern int  get_ad_value(unsigned char channel);
extern signed char get_temp(void);
extern void config (void);
extern void initialize(void);

extern void init_AD(void);
extern void LedBlink(unsigned char num, unsigned char state, unsigned char blink );

extern void init_para();
extern void init_index_para();
extern void delay1( unsigned int us);

extern void CAN_AUTO_RESET(void);
extern void can_rx_from_DCT_amount_feedback(unsigned char *buf);
extern void can_rx_from_DCT_bite_feedback(unsigned char *buf);
extern void can_rx_from_SWJ_Action_Command(void);
extern void can_rx_from_SWJ_Zijian_Command(void);
extern void Comunication_BITE_to_SWJ(void);

extern bit Get_DCT_Input_State(unsigned char  number) ; 	        // ��ȡ�ײ����������ָ���״̬
extern void Set_DCT_Input_State(unsigned char  number, bit state); // ���û�λ�ײ������������״̬
extern void Set_DCT_OK_Result(unsigned char  number, bit state);   // ���û�λ�ײ�������Ķ���OK״̬
extern void Set_DCT_OK_Action(unsigned char  number, bit state);   // ���û�λ�ײ�������Ķ�������״̬

extern void CAN2_Data_Collect_Process_Programme(void);                 //�ײ㷢ҩ�����ݷ����ӳ���
extern void Medica_Auto_Transportation_Programme(void);                //ҩƷ����ϵͳ�Զ�����
extern void Send_CO_Begin_Programme(void);
//
extern void delay1(unsigned int us);
extern void delay_ms(unsigned int ms);
//����λ������CO�����
extern void Lan_backinfor(unsigned char m_pra1, unsigned char m_pra2);
extern void Lan_backalarm(unsigned char ID, unsigned char MUM, unsigned char m_alarm1);
//
extern void Can_CHECKinfor(unsigned char ID, unsigned char command); //used 3
extern void ElectromagnetCRTL(void);//used 0
extern void YaolanStepRun(void);//used 0
extern void Yaolantisheng(void);//used 0
//
extern void Yaopingtishengleft(void);//used 1
extern void Yaopingtisheng(void);//used 1
extern void ConveyingLineMovement(void);//used 1
extern void ConveyingrightLineMovement(void);//used 1
extern void Susongxiangtuiyao(void);//used 1
extern void Susongxiangrighttuiyao(void);//used 1
//added start
extern void FZSSXRightFZ(unsigned char signal);//used 1
extern void FZSSXLeftFZ(unsigned char signal);//used 1
void ConveyingLongSlow_Start(unsigned char dir);
void ConveyingLongSlow_Stop();
//added end
/*CAN3���͵�ַ��������
*1��
*2���Ҳ����������ĵ��������CAN ID
*3�����������ߵĵ��������CAN ID
*4��������������ĵ��������CAN ID
*/
//
extern void init_GM8123(void);
extern void init_uart();
//extern void Uart0Send(void);
extern void Uart1Send(void);
extern void Uart0Send(unsigned char addr, unsigned char *buf, unsigned char bufsize, bit  sumflag);
//extern void Uart1Send(unsigned char addr, unsigned char *buf, unsigned char bufsize, bit  sumflag);

//-----------------------------------zk.c--------------------------------------
// ȫ�ֱ���
//-----------------------------------------------------------------------------
//���嶨ʱ�������������
extern unsigned char  T0Counter1;  //����ָʾ��
extern unsigned char  T0Counter2;  //CAN ���ͳ�ʱ������
extern unsigned int   T0Counter3;  //BITE��������λ��
extern unsigned char  T0Counter4;  //��OK��Ϣ�Ĵ�����ϱ�

extern xdata unsigned char  T0Counter12;  //���ڿ�����ʹ��1
extern xdata unsigned char  T0Counter13;  //���ڿ�����ʹ��2
extern xdata unsigned int   T0Counter14;  //�������ҩ��������ʱ��
extern xdata unsigned int   T0Counter15;  //�������ҩ��������ʱ��

extern unsigned char Group_index_DCT;      //������������κ�Ҳ����������
extern unsigned char Board_Address_Number;  //�Լ���

extern unsigned char ALL_Action_OK_Sign ;
extern unsigned char ALL_Action_OK_Sign_last;
extern unsigned char ALL_Result_OK_Sign ;

extern unsigned char OK_Sign_TX_Amount;   //0k�źŵķ��ʹ���
extern unsigned char temppage;

extern unsigned char Zijian_Sign ;     //�Լ��־��0xaa�˳��Լ�ģʽ��0X55�����Լ�ģʽ

//extern xdata SYSDAT  SysData;
extern xdata  unsigned char  databuf[8];

extern xdata unsigned char LED_Address;
extern xdata unsigned char LED_BUF;

extern bit Command_Finish_Sign ;   //ָ������Ϳ�ʼ��־
extern bit Command_Begin_Sign ;    //���ʹ�����ָ��
extern bit Servo_Begin_Sign ;      //�ŷ�������־
extern bit  Answer_ok;
extern unsigned char Board_ID;

extern unsigned int  Proce_Count;            //�Զ�������������λ��ĵȴ�ʱ��
extern unsigned char Procession1;            //�Զ������������
extern unsigned char Run_modeFayaoL;
extern unsigned char Run_modeFayaoR;
extern unsigned char Run_mode;

extern unsigned char SNSORSTATUS1;
/*SNSORSTATUS1��λ���壺
**bit0:��բ��ԭ�㽨����־
**bit1:����������ԭ���־
**bit4:�����߶�λ���н�����־
**bit5:��������ҩ������־
*/
extern unsigned char SNSORSTATUS2;
/*SNSORSTATUS2��λ���壺
**bit0:��բ��ԭ�㽨����־
**bit1:����������ԭ���־
**bit4:�����߶�λ���н�����־
**bit5:��������ҩ������־
*/
extern unsigned char MAINRUNSTATUSL;
/*MAINRUNSTATUSL��λ���壺
**bit0:��������λ����״̬λ
**bit4:����������ҩ״̬λ
**bit5:��������������״̬λ
*/
extern unsigned char MAINRUNSTATUSL1;
/*MAINRUNSTATUSL1��λ���壺
**bit0:
**bit1:
*/
extern unsigned char RUNSTATUS1;
extern unsigned char RUNSTATUS2;



extern unsigned char Runbiaoji1;
extern unsigned char Runbiaoji2;
extern bit Tansflag;

extern unsigned char Runbiaoji3;
extern unsigned char Runbiaoji4;
extern unsigned char FAYAOinput_pra[4];
extern unsigned char communication_step;
extern bit  ONCEFLAG;
extern bit  Alarm_stepflagL;  //���ϵ��־λ
extern bit  Alarm_stepflagR;  //���ϵ��־λ
extern bit  ALARM_ONCEFLAGL;
extern bit  ALARM_ONCEFLAGR;
extern bit  Servo_left_Begin_Sign;
extern bit  Servo_right_Begin_Sign;
extern bit  FaYao_Prosetion_Sign;
extern bit  Active_Data_Begin_Sign_last;
extern bit  Active_Data_Begin_Sign;

extern xdata Servomotordisplace Servodisplace;
extern xdata  float m_floatServodisplace;
extern xdata  unsigned int m_intServodisplace;
extern xdata  unsigned char Run_REAFY1;
extern xdata  unsigned char Run_REAFY2;
extern xdata  unsigned char Run_ALARM1;
extern xdata  unsigned char Run_ALARM2;
extern xdata  unsigned char Mode_ALARM1;
extern xdata  unsigned char Mode_ALARM2;
extern xdata  unsigned char Mode_ALARM3;
extern xdata  unsigned char Mode_ALARM4;
extern xdata  unsigned char Mode_adress1;
extern xdata  unsigned char Mode_adress2;
extern xdata  unsigned char Mode_adress3;
extern xdata  unsigned char Mode_adress4;
extern xdata  unsigned char Basket_stat;
extern xdata  unsigned char strat_flag;
extern xdata  unsigned char CAN_index;
extern xdata  unsigned int  AUTO_countimerL;
extern xdata  unsigned int  AUTO_countimerR;

//------------------------------------zk.c-------------------------------------
// �ⲿȫ�ֱ���
//-----------------------------------------------------------------------------
//extern xdata CAN1RECBUF1  CAN1RXbuffer1;
//extern xdata CAN1TRANSBUF1   CAN1TXbuffer1;

extern xdata CAN2RECBUF1  CAN2RXbuffer1;
extern xdata CAN2RECBUF1  CAN2RXbuffer2;
extern xdata CAN2RECBUF1  CAN2RXbuffer3;

extern xdata CAN2TRANSBUF1   CAN2TXbuffer1;
extern xdata CAN2TRANSBUF2   CAN2TXbuffer2;

extern xdata CAN3TRANSBUF1  CAN3TXbuffer1;
extern xdata CAN3RECBUF1  CAN3RXbuffer1;

//extern bit Can1NewDataA;
extern bit Can2NewData1;
extern bit Can2NewData2;
extern bit Can2NewData3;
extern bit Can3NewData1;

//extern unsigned char CAN1FaultCounter;
extern unsigned char CAN2FaultCounter;
extern unsigned char CAN3FaultCounter;





//-----------------------------------CAN1.C---------------------------------------
// ����ԭ��
//-----------------------------------------------------------------------------
extern void clear_msg_objects (void);
extern void init_msg_object_TX (char MsgNum, unsigned int id);
extern void init_msg_object_RX (char MsgNum, unsigned int id);
extern void start_CAN (void);
extern void can1_transmit(char MsgNum, unsigned char *buf);
extern void can1_receive (char MsgNum, unsigned char *buf);
extern void init_can1 (void);


//-----------------------------------CAN2.C---------------------------------------
// ����ԭ��
//-----------------------------------------------------------------------------
extern void init_can2 (void);
extern void INT0_ISR (void)  ;
extern void can2_transmit(unsigned int id, unsigned char *buf);
extern void can2_receive (unsigned char *buf);


//-----------------------------------CAN3.C---------------------------------------
// ����ԭ��
//-----------------------------------------------------------------------------
extern void init_can3 (void);
extern void INT1_ISR (void)  ;
extern void can3_transmit(unsigned int id, unsigned char *buf);
extern void can3_receive (unsigned char *buf);


////---------------------------------uart.c-----------------------------------------
//// ȫ�ֱ���
////-----------------------------------------------------------------------------
//extern unsigned char RecPointer0;
//extern bit RecFlag0;

//extern unsigned char RecPointer1;
//extern bit RecFlag1;

//// ���崮�ڽ��ջ���
//extern xdata  RECBUF0 RxBuf0;
//extern xdata  RECBUF1 RxBuf1;

//extern xdata TRANSBUF0 TxBuf0;

////������ջ�����
//extern xdata unsigned char Rec0[RECBUF0SIZE];
//extern xdata unsigned char Rec1[RECBUF1SIZE];

//extern bit TI0Flag;  //���ͽ�����־
//extern bit TI1Flag;  //���ͽ�����־

//extern unsigned char CommandByte1;


////---------------------------------UART0.c-----------------------------------------
//// ȫ�ֱ���
////-----------------------------------------------------------------------------
extern TRANSBUF1 TransBuf0;
extern RECBUF1  RecBuf0;
extern bit UART0_Refresh;

////---------------------------------UART0.c-----------------------------------------
//// ȫ�ֱ���
////-----------------------------------------------------------------------------
extern TRANSBUF1 TransBuf1;
extern RECBUF1  RecBuf1;
extern bit UART1_Refresh;









