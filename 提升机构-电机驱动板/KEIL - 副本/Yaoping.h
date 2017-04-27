
////////////////////////////////////////////////////////////////////////////////
// ����SJA1000 CAN �շ����ڵ�����ʱʹ�õ�ID�š�
//
////////////////////////////////////////////////////////////////////////////////

//���� C8051F040 CAN ���ڽ��յ���Ϣ��
// (��Ϣ�ŷ�Χ1~16,����16�����޸�CAN1�жϷ������)
////////////////////////////////////////////////////////////////////////////////
//���ڽ��յ���Ϣ��
////////////////////////////////////////////////////////////////////////////////
#define		Delay_Times	20000		//��ʱѭ������
#define RX_MSGNUM_ZKB		 1   //  �������ذ嶯��ָ�����ݼ��ƿ�ָ������	

//#define RX_MSGNUM_ZKB_Union	 2   // �������ذ嶯��ָ���ʼ���������������(����ͨ��)
//#define RX_MSGNUM_X_Shaft    3   // ����X�ᷢ�͹���������
//#define RX_MSGNUM_Y_Shaft    4   // ����Y�ᷢ�͹���������
//#define RX_MSGNUM_Z_Shaft    5   // ����Z�ᷢ�͹���������
//#define RX_MSGNUM_H_Shaft    6   // ����H�ᷢ�͹���������
//#define RX_MSGNUM_ZKB_1      7   //�������ذ嶯��ָ���ʼ���������������(����ͨ��)



//#define CAN2TXID_Uion	         18	// �ײ���������չ������ݵĵ�ַ
//#define Driver_Board_Amount	 96	    // �ײ��������ܸ���
//#define Jijia_Amount	 6	        // ���ܵ��ܸ���
//#define BYTE_Amount	 12	            // �ж϶����Ƿ�������ֽ��ܸ���



////////////////////////////////////////////////////////////////////////////////

//���ڷ��͵���Ϣ��
////////////////////////////////////////////////////////////////////////////////
#define TX_MSGNUM_ZKB		2	// �����ذ巢��	
//#define TX_MSGNUM_Y	    	9	// ��Y�ᷢ��

//------����ͨ���ĵ�ַ------------------
//#define Address_Union		418  // �������ذ嶯��ָ���ʼ��������������ݵ�ַ


/***********************

  HL10(LED9)Ϊ�źż��3����4ָʾ;
  HL9(LED8)ΪCANͨѶ����ָʾ;
  HL8(LED7)ΪCANͨѶ��������ָʾ;
  HL7(LED6)Ϊ���ղ��������ָʾ
  HL6(LED5)Ϊ��������2ָʾ;  �ָ�ΪX,Y
  �����е�λ���ָʾ������Z�����ʵ�֡�
  HL5(LED4)Ϊ��������1ָʾ;
  HL4(LED3)Ϊ ������λ2ָʾ
  HL3(LED2)Ϊ ������λ1ָʾ
  HL2(LED1)Ϊ��������ָʾ;
  HL1(LED0)��Դ����ָʾ;
************************/



#define		Max_displace	2850000		//�ŷ����λ����
#define		Max_speedvalue	300000	        //�ŷ�������ٶ�*100
#define		Zero_backdisplace	500	//�ŷ�
#define		GAOKONG_TIMER			600	//�߿���������ʱ��ʱ
//
sbit  LED1 = P2 ^ 4;              //״ָ̬ʾ��
sbit  LED2 = P2 ^ 6;              //״ָ̬ʾ��
sbit  LED3 = P1 ^ 0;              //״ָ̬ʾ��
sbit  LED4 = P1 ^ 2;              //״ָ̬ʾ��
sbit  LED5 = P1 ^ 4;              //״ָ̬ʾ��
sbit  LED6 = P1 ^ 6;              //״ָ̬ʾ��
sbit  LED7 = P3 ^ 2;              //״ָ̬ʾ��
sbit  LED8 = P3 ^ 4;              //״ָ̬ʾ��
sbit  LED9 = P3 ^ 6;              //״ָ̬ʾ��
//
sbit	motor1_dir = P0 ^ 5;          //�������1   �ߵ�ƽ��ǰ�����͵�ƽ������
sbit	motor1_ENB = P0 ^ 6;          //�������1   �ߵ�ƽ��ǰ�����͵�ƽ������
//
sbit	RE1 = P4 ^ 1;
sbit	DE1 = P4 ^ 3;
//
sbit	Motor2 = P5 ^ 1; //�߿�������
sbit	Motor1_Power = P5 ^ 3; //�㵹��������ת
sbit	Motor1_Dir = P5 ^ 5; //�㵹�����߷�ת�������Ŵ������ǵó�ʼ��
//
sbit	MONITOR_INPUT1 = P0 ^ 2; //�߿������ߴ�����
sbit	MONITOR_INPUT2 = P0 ^ 3; //���������ߴ�����
//sbit	MONITOR_INPUT3 = P2 ^ 0;
//sbit	MONITOR_INPUT4 = P2 ^ 2;
//-----------------------------------------------------------------------------
// ȫ�ֳ���
//-----------------------------------------------------------------------------

#define YES 1
#define NO  0
#define SYSCLK       11059200     // Internal oscillator frequency in Hz

//#define SAVEBUFSIZE  5   //flash

//���ڽ��ջ�������С

//#define RECBUF1SIZE 30


//���ڷ��ͻ�������С
#define TRANSBUF1SIZE 4

//�������ʼ����ֵ
/*#define Base_JiGuangCeJu_Value 132

#define Current_Shaft_Modulus1 2.13
#define Current_Shaft_Modulus2 2.13
#define Current_Shaft_Modulus3 375
#define Current_Shaft_Modulus4 5*/

//-----------------------------------------------------------------------------
// ���Ͷ���
//-----------------------------------------------------------------------------

//����0

/*typedef struct
{
	unsigned char  head ;
	unsigned char  data1;
	unsigned char  data2;
	unsigned char  data3;
	unsigned char  data4;
	unsigned char  data5;
	unsigned char  data6;
	unsigned char  data7;
	unsigned char  data8;
	unsigned char  data9;
	unsigned char  data10;
	unsigned char  data11;
	unsigned char  data12;
	unsigned char  data13;
	unsigned char  data14;
	unsigned char  data15;
	unsigned char  data16;
	unsigned char  data17;
	unsigned char  data18;
}RECEIVE1;*/

/*typedef union
{
	//RECEIVE1 rec;
	unsigned char buf[RECBUF1SIZE];
}RECBUF1;*/



//������ȡ�ߵ�λ�ֽ�
/*typedef union
{
	int value;
	unsigned char buf[2];
}INTUNION;*/

typedef union
{
    unsigned int value;
    unsigned char buf[2];
} UINTUNION;



/*********************************************************************
//CAN���ݴ����ʽ�����ͽ���ͬһ��ʽ��
/*********************************************************************/
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
} CANTRANSCHAR3;
// CAN1 buffer definition
//-----------------CAN ���߽��ܻ�����-------------
typedef struct                     //��ͷ��β���ݡ����㼰�Լ�ָ��
{
    unsigned char  command;       // 1
    unsigned char  index;         // 2
    unsigned char  shake_hand1;   // 3
    unsigned char  shake_hand2;   // 4
    unsigned char  box_code;      // 5
    unsigned char  shaft_code;    // 6
    unsigned char  bakeup1;       // 7
    unsigned char  checkout;      // 8
} CANRECCHAR1;
typedef union
{
    CANRECCHAR1 total_ctl_buf;
    //CANRECCHAR2 motor_ctl_buf;
    //CANRECCHAR3 motor_force_buf;
    //CANRECCHAR4 motor_feedbake_buf;
    unsigned char buf[8];
} CANRECDATBUF;

/*typedef struct                     //���������Ϻ���OK����
{
	unsigned char  command;    // 1
	unsigned char  index;      // 2
	unsigned char  shaft_code;   // 3
	unsigned char  resoult;      // 4
	unsigned char  bakeup0;      // 5
	unsigned char  bakeup1;      // 6
	unsigned char  bakeup2;      // 7
	unsigned char  checkout;     // 8
}CANTRANSCHAR1;*/

/*typedef struct                     //�������Ǽ����Ϻ�������
{
	unsigned char  command;    // 1
	unsigned char  index;      // 2
	unsigned char  address1;   // 3
	unsigned char  address2;   // 4
	unsigned int   result;     // 5
	unsigned char  left_right; // 7
	unsigned char  checkout;   // 8
}CANTRANSCHAR2;*/



/*typedef struct                     //��ҩ��������������
{
	unsigned char  command;    // 1
	unsigned char  index;      // 2
	unsigned char  address1;   // 3
	unsigned char  address2;   // 4
	unsigned char  amount;     // 5
	unsigned char  bakeup1;     // 6
	unsigned char  bakeup2;     // 7
	unsigned char  checkout;    // 8
}CANTRANSCHAR4;*/

//-----------------------------------------------------------------------------
// ������ ����������ֽṹ���Ա�ʹ��
//-----------------------------------------------------------------------------

typedef union
{
    //CANTRANSCHAR1 motor_feedbake_buf;
    //CANTRANSCHAR2 jg_feedbake_buf;
    CANTRANSCHAR3 normal_buf;
    //CANTRANSCHAR4 buyao_feedbake_buf;
    unsigned char buf[8];
} CANTRANSDATBUF;






//-----------------------------------------------------------------------------
// ������ ����������ֽṹ���Ա�ʹ��
//-----------------------------------------------------------------------------


typedef struct
{
    unsigned char   AUTOsystem_command;
    unsigned char    AUTOsystem_alarm1;
    //unsigned long    AUTOsystem_alarm2;
} systemcrtl;

typedef union
{
    long displace;
    unsigned char buf[4];
} Servomotordisplace;
typedef union
{
    unsigned long speed;
    unsigned char buf[4];
} Servomotorspeed;

typedef union
{
    unsigned long addtime;
    unsigned char buf[4];
} Servomotoraddtime;
typedef union
{
    unsigned long subtime;
    unsigned char buf[4];
} Servomotorsubtime;





//-------------------------------------Yaoping.c--------------------------------------
// ȫ�ֱ���
//-----------------------------------------------------------------------------
extern bit Receive_command_finished;
extern bit Receive485_command_finished;
extern bit Time_FLAG;
extern bit Time_FLAG1;
extern bit AUTO_FLAG;
extern bit Step_FLAG;
extern bit CANINFOR_FLAG;
extern bit ONCE_FLAG;
extern xdata volatile unsigned char Runmode;
//���嶨ʱ�������������
extern xdata unsigned char  T0Counter5;  //
extern xdata unsigned char  T0Counter2;
extern xdata volatile unsigned char  T0Counter3;  //
extern xdata unsigned char  T0Counter4;  //
extern xdata unsigned int  T0Counter6;  //
extern xdata unsigned char  T0Counter8;
extern xdata unsigned int   T0Counter9;
extern xdata unsigned int   T0Counter10;  //�߿������߶�ʱ��
extern xdata unsigned char  GAOKONG_COUT; //�߿������߼�����
extern xdata unsigned char  YAOPINGTISH_COUT; //ҩƷ����������
extern xdata unsigned char  CANcomand;    //CAN������
extern  xdata volatile unsigned char  nCANcomand;     //��ֹCAN�ж��޸������е�����
extern xdata unsigned char  AUTOCANcomand;
extern xdata unsigned char  AUTOCMD_STATE;
extern xdata unsigned char  AUTOCMD_TIME;
extern xdata volatile unsigned char  AUTOCMD_CHECK;
extern xdata unsigned char  temppage;
extern xdata unsigned char BPSSET;
extern xdata unsigned char IDSET;
extern xdata unsigned char Shaft1_Run_mode;
extern xdata unsigned long Shaft1_aim_pulse;
extern xdata unsigned char Sampleperiod;
extern xdata unsigned int ShowTime1;
//xdata FLASH Flash;            //����Flash ������
extern xdata CANTRANSDATBUF CANTXBUF_ZKB;
extern xdata unsigned  int Pulse1;
extern xdata unsigned char Shaft1_Base_Speed_Value;
extern xdata long int Shaft1_Velocity_Subsection_Parameter;
extern xdata long int motor1_out;                        //�������1ʵ�ʵ�ת��
extern xdata unsigned char Shaft1_ADD_Speed_Time_Base;
extern xdata unsigned char Shaft1_SUB_Speed_Time_Base;
extern xdata unsigned  long Pulse1_Cnt;
extern xdata unsigned int  Shaft1_Add_Speed;
extern xdata unsigned int  Shaft1_Speed_max;
extern xdata long int Vel1;
extern xdata unsigned long speed_value;
extern xdata unsigned char buf[30];
extern xdata unsigned char Rec1[30];
extern xdata unsigned  char RecPointer1;
//xdata  RECBUF1 RxBuf1;
extern xdata unsigned  long Servoparadisplace;
extern xdata unsigned  long Servoparaspeed;
extern xdata unsigned  long Servoparaaddtime;
extern xdata unsigned  long Servoparasubtime;
extern xdata unsigned  long ShowTime;
extern xdata unsigned  char Servopara[107];
/*Servopara[]���ݶ���
**CANINDEX*16+[0-3]��Servomotor_displace
**CANINDEX*16+[4-7]��Servomotor_speed
**CANINDEX*16+[8-11]��Servomotor_addtime
**CANINDEX*16+[12-15]��Servomotor_subtime

**CANINDEX*9+[80-83]��Shaft1_aim_pulse
**CANINDEX*9+[84-85]��Shaft1_Speed_max
**CANINDEX*9+[86-87]��Shaft1_Add_Speed
**CANINDEX*9+[88]��Shaft1_ADD_Speed_Time_Base
*/
extern xdata unsigned  char CANTRASTEMINFOR[20];
/*CANTRASTEMINFOR[8]��־λ��
**bit0:�ŷ����б�־
**bit1:����������б�־
**bit2:�߿������ߴ������ź�
**bit3:���������ߴ������ź�
**bit6:�ŷ���ԭ�����б�־
*/
extern xdata unsigned  char repair_flag;
extern xdata unsigned  char Servomotor_original;
extern xdata volatile systemcrtl system_crtl;
extern xdata Servomotordisplace  Servomotor_displace;
extern xdata Servomotorspeed     Servomotor_speed;
extern xdata Servomotoraddtime   Servomotor_addtime;
extern xdata Servomotorsubtime   Servomotor_subtime;
extern code unsigned int Speedvalue[2000];

/*****************************************Yaoping.c*******************************/
extern void Watchdog_Init (void);
extern void Initial(void);
extern void PORT_Init (void);
extern void OSCILLATOR_Init (void);
extern void Timer01_Init(void);
extern void init_para(void);
extern void Board_Parameter_Setup(void);
extern void init_INT0 (void);
extern void init_INT1 (void);
extern void Motor1_CTL(void);     //T2��������Ƶ��1;
extern void Shaft1_CTL(void);
extern void ramp1_1(long int max);
extern void init_T2 (void);
extern void Uart0Send(unsigned char *buf, unsigned char bufsize );
extern void delay1( unsigned int us);
extern void Send_to_Motordriver_CTL(unsigned char command );
extern unsigned int crc_chk(unsigned char *puchMsg, unsigned char length);
extern void delay_ms(unsigned int ms);
extern void send_to_motor(void);
extern void RunLEDDIS(void);
extern void SteppermotorCTRL(void);
extern void ServomotorCRTL(unsigned char comand);
extern void Dataacquisition();
extern void SystemControl(void);
extern void ServomotorDataacquisition(unsigned char addressID);
extern void Returntopoint(void);
extern void UnactiveCANtransfer(void);
extern void write_to_flash(void);
extern void read_from_flash(void);
extern void TRANFORSATATIONINFOR(void);
extern void CANsetpraback(void);
extern void CurrentCollection(void);
extern void ServomotorINP(unsigned char mode);
extern void CANsteminforback(void);
extern void AUTORUNMODE0(unsigned char comand);
extern void AUTORUNMODE1(unsigned char comand);
extern unsigned char AUTOTaskassignment0(void);
extern unsigned char AUTOTaskassignment1(void);
extern void DCmotorCTRL(void);
extern void ActiveCANtransfer(void);
extern void init_machine(void);
//extern void Deviationremoval(void);


//--------------------------------------CAN1.C-------------------------------------
// ����ԭ��
//-----------------------------------------------------------------------------
extern void clear_msg_objects (void);
extern void init_msg_object_TX (char MsgNum, unsigned int id);
extern void init_msg_object_RX (char MsgNum, unsigned int id);
extern void start_CAN (void);
extern void can1_transmit(char MsgNum, unsigned char *buf);
extern void can1_receive (char MsgNum, unsigned char *buf);
extern void init_can1_1 (void);

//--------------------------------------CAN1.C-------------------------------------
// ȫ�ֱ���
//-----------------------------------------------------------------------------
extern xdata CANRECDATBUF	CANRXBUF_ZKB;
extern xdata volatile unsigned char CANaddress1;
extern xdata unsigned char CANINDEX;
extern xdata unsigned char CAN1FaultCounter;






