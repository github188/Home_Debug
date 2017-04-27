//---------------------------------------------------------------
// CYGNAL Integrated Products 
//
// C Code Configuration Tool: F040 INITIALIZATION/CONFIGURATION CODE
//----------------------------------------------------------------
// This file is read only. To insert the code into your  
// application, simply cut and paste or use the "Save As" 
// command in the file menu to save the file in your project 
// directory. 
//----------------------------------------------------------------

//----------------------------------------------------------------
// INCLUDES
//----------------------------------------------------------------

#include <C8051F040.h>	// Register definition file.

//------------------------------------------------------------------------------------
// Global CONSTANTS
//------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------
// Function PROTOTYPES
//------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------
// Config Routine
//------------------------------------------------------------------------------------
void config (void) {

//Local Variable Definitions
    int n = 0;

	

//----------------------------------------------------------------
// Watchdog Timer Configuration
//
// WDTCN.[7:0]: WDT Control
//   Writing 0xA5 enables and reloads the WDT.
//   Writing 0xDE followed within 4 clocks by 0xAD disables the WDT
//   Writing 0xFF locks out disable feature.
//
// WDTCN.[2:0]: WDT timer interval bits
//   NOTE! When writing interval bits, bit 7 must be a 0.
//
//  Bit 2 | Bit 1 | Bit 0
//------------------------     
//    1   |   1   |   1      Timeout interval = 1048576 x Tsysclk
//    1   |   1   |   0      Timeout interval =  262144 x Tsysclk
//    1   |   0   |   1      Timeout interval =   65636 x Tsysclk
//    1   |   0   |   0      Timeout interval =   16384 x Tsysclk
//    0   |   1   |   1      Timeout interval =    4096 x Tsysclk
//    0   |   1   |   0      Timeout interval =    1024 x Tsysclk
//    0   |   0   |   1      Timeout interval =     256 x Tsysclk
//    0   |   0   |   0      Timeout interval = 	 64 x Tsysclk
// 
//------------------------

   WDTCN = 0x07;	// Watchdog Timer Control Register
   WDTCN = 0xFF;   // Disable WDT Lockout
   WDTCN = 0xA5;
//----------------------------------------------------------------
// CROSSBAR REGISTER CONFIGURATION
//
// NOTE: The crossbar register should be configured before any  
// of the digital peripherals are enabled. The pinout of the 
// device is dependent on the crossbar configuration so caution 
// must be exercised when modifying the contents of the XBR0, 
// XBR1, and XBR2 registers. For detailed information on 
// Crossbar Decoder Configuration, refer to Application Note 
// AN001, "Configuring the Port I/O Crossbar Decoder". 
//----------------------------------------------------------------

// Configure the XBRn Registers

    SFRPAGE = 0x0F;
	XBR0 = 0x04;	// XBAR0: Initial Reset Value
	XBR1 = 0x14;	// XBAR1: Initial Reset Value
	XBR2 = 0x44;	// XBAR2: Initial Reset Value    	  
    XBR3 = 0x00;    // XBAR3: Initial Reset Value
// Select Pin I/0

// NOTE: Some peripheral I/O pins can function as either inputs or 
// outputs, depending on the configuration of the peripheral. By default,
// the configuration utility will configure these I/O pins as push-pull 
// outputs.
                    // Port configuration (1 = Push Pull Output)
    SFRPAGE = 0x0F;
  
    P0MDOUT = 0x05; // Output configuration for P0 
    P1MDOUT = 0x00; // Output configuration for P1 
    P2MDOUT = 0x00; // Output configuration for P2 
    P3MDOUT = 0xFF; // Output configuration for P3 

    P4MDOUT = 0xFF; // Output configuration for P4
    P5MDOUT = 0xFF; // Output configuration for P5

    P6MDOUT = 0xFF; // Output configuration for P6
    P7MDOUT = 0xFF; // Output configuration for P7

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

//----------------------------------------------------------------
// Comparators Register Configuration
//
// Bit 7  | Bit 6  | Bit 5  | Bit 4  | Bit 3 | Bit 2 | Bit 1 | Bit 0
//------------------------------------------------------------------     
//  R/W	  |    R   |  R/W   |  R/W   |  R/W  |  R/W  |  R/W  |  R/W
//------------------------------------------------------------------
// Enable | Output | Rising | Falling|  Positive     |  Negative    
//        | State  | Edge   | Edge   |  Hysterisis   |  Hysterisis    
//        | Flag   | Int.   | Int.   |  00: Disable  |  00: Disable
//        |        | Flag   | Flag   |  01:  5mV     |  01:  5mV  
//        |        |        |        |  10: 10mV     |  10: 10mV
//        |        |        |        |  11: 20mV     |  11: 20mV 
// ----------------------------------------------------------------
//----------------------------------------------------------------
// Comparator 0
//----------------------------------------------------------------

    SFRPAGE = 0x01;
    CPT0MD = 0x00;   // Comparator 0 Mode Selection Register
    CPT0CN = 0x00;   // Comparator 0 Control Register
	
//----------------------------------------------------------------
// Comparator 1
//----------------------------------------------------------------

    SFRPAGE = 0x02;
    CPT1MD = 0x00;   // Comparator 1 Mode Selection Register
    CPT1CN = 0x00;   // Comparator 1 Control Register

//----------------------------------------------------------------
// Comparator 2
//----------------------------------------------------------------
	
    SFRPAGE = 0x03;
    CPT2MD = 0x00;   // Comparator 2 Mode Selection Register
    CPT2CN = 0x00;   // Comparator 2 Control Register	
					
//----------------------------------------------------------------
// Oscillator Configuration
//----------------------------------------------------------------

    SFRPAGE = 0x0F;
	OSCXCN = 0x67;	// EXTERNAL Oscillator Control Register	
    for (n = 0; n < 255; n++) ;            // wait for osc to start
    while ( (OSCXCN & 0x80) == 0 );        // wait for xtal to stabilize
    CLKSEL = 0x01;  // Oscillator Clock Selector
	OSCICN = 0x80;	// Internal Oscillator Control Register

	
//----------------------------------------------------------------
// Reference Control Register Configuration
//----------------------------------------------------------------

    SFRPAGE = 0x00;
	REF0CN = 0x00;	// Reference Control Register

//----------------------------------------------------------------
// ADC0 Configuration
//----------------------------------------------------------------

    SFRPAGE = 0x00;
	AMX0CF = 0x08;	// AMUX0 Configuration Register
	AMX0SL = 0x00;	// AMUX0 Channel Select Register
    AMX0PRT = 0x03; // Port 3 Pin Selection Register
	ADC0CF = 0xF8;	// ADC0 Configuration Register
	ADC0CN = 0x00;	// ADC0 Control Register
	
    ADC0L = 0x00;   // ADC0 Data Word LSB
    ADC0H = 0x00;   // ADC0 Data Word MSB
	ADC0LTH = 0x00;	// ADC0 Less-Than High Byte Register
	ADC0LTL = 0x00;	// ADC0 Less-Than Low Byte Register
	ADC0GTH = 0xFF;	// ADC0 Greater-Than High Byte Register
	ADC0GTL = 0xFF;	// ADC0 Greater-Than Low Byte Register

//----------------------------------------------------------------
// ADC2 Configuration
//----------------------------------------------------------------

    SFRPAGE = 0x02;
    AMX2SL = 0x00;  // AMUX2 Chanel Select Register
    AMX2CF = 0x00;  // AMUX2 Configuration Register
    ADC2CF = 0xF8;  // ADC2 Configuration Register
    ADC2LT = 0xFF;  // ADC2 Less-Than Data Register
    ADC2GT = 0xFF;  // ADC2 Greater-Than Data Register
    ADC2CN = 0x00;  // ADC2 Control Register

    SFRPAGE = 0x00;
    HVA0CN = 0x00;  // High Voltage Amplifier Control Register

//----------------------------------------------------------------
// DAC Configuration
//----------------------------------------------------------------

    SFRPAGE = 0x00;
	DAC0L = 0x00;	// DAC0 Low Byte Register
	DAC0H = 0x00;	// DAC0 High Byte Register
    DAC0CN = 0x00;	// DAC0 Control Register

    SFRPAGE = 0x01;	
	DAC1L = 0x00;	// DAC1 Low Byte Register
	DAC1H = 0x00;	// DAC1 High Byte Register
    DAC1CN = 0x00;	// DAC1 Control Register

//----------------------------------------------------------------
// SPI Configuration
//----------------------------------------------------------------

    SFRPAGE = 0x00;	
	SPI0CFG = 0x00;	// SPI Configuration Register
	SPI0CKR = 0x00;	// SPI Clock Rate Register
    SPI0CN = 0x00;	// SPI Control Register

//----------------------------------------------------------------
// UART0 Configuration
//----------------------------------------------------------------

    SFRPAGE = 0x00;
    SADEN0 = 0x00;      // Serial 0 Slave Address Enable
    SADDR0 = 0x00;      // Serial 0 Slave Address Register
    SSTA0 = 0x15;       // UART0 Status and Clock Selection Register
    SCON0 = 0x50;       // Serial Port Control Register
    SCON0 &= 0xFC; 	//clear interrupt pending flags


    PCON = 0x00;        // Power Control Register

//----------------------------------------------------------------
// UART1 Configuration
//----------------------------------------------------------------

    SFRPAGE = 0x01;    
    SCON1 = 0x10;       // Serial Port 1 Control Register   
    SCON1 &= 0xFC; 	//clear interrupt pending flags

//----------------------------------------------------------------
// SMBus Configuration
//----------------------------------------------------------------	
    
	SFRPAGE = 0x00; 
	SMB0CN = 0x00;	// SMBus Control Register
	SMB0ADR = 0x00;	// SMBus Address Register
	SMB0CR = 0x00;	// SMBus Clock Rate Register


//----------------------------------------------------------------
// PCA Configuration
//----------------------------------------------------------------

    SFRPAGE = 0x00;
    PCA0MD = 0x00;       // PCA Mode Register
    PCA0CN = 0x00;      // PCA Control Register
    PCA0L = 0x00;       // PCA Counter/Timer Low Byte
    PCA0H = 0x00;       // PCA Counter/Timer High Byte	
	

    //Module 0
    PCA0CPM0 = 0x00;    // PCA Capture/Compare Register 0
    PCA0CPL0 = 0x00;    // PCA Counter/Timer Low Byte
    PCA0CPH0 = 0x00;    // PCA Counter/Timer High Byte

    //Module 1
    PCA0CPM1 = 0x00;    // PCA Capture/Compare Register 1
    PCA0CPL1 = 0x00;    // PCA Counter/Timer Low Byte
    PCA0CPH1 = 0x00;    // PCA Counter/Timer High Byte

    //Module 2
    PCA0CPM2 = 0x00;    // PCA Capture/Compare Register 2
    PCA0CPL2 = 0x00;    // PCA Counter/Timer Low Byte
    PCA0CPH2 = 0x00;    // PCA Counter/Timer High Byte

    //Module 3
    PCA0CPM3 = 0x00;    // PCA Capture/Compare Register 3
    PCA0CPL3 = 0x00;    // PCA Counter/Timer Low Byte
    PCA0CPH3 = 0x00;    // PCA Counter/Timer High Byte

    //Module 4
    PCA0CPM4 = 0x00;    // PCA Capture/Compare Register 4
    PCA0CPL4 = 0x00;    // PCA Counter/Timer Low Byte
    PCA0CPH4 = 0x00;    // PCA Counter/Timer High Byte
	
    //Module 5
    PCA0CPM5 = 0x00;    // PCA Capture/Compare Register 5
    PCA0CPL5 = 0x00;    // PCA Counter/Timer Low Byte
    PCA0CPH5 = 0x00;    // PCA Counter/Timer High Byte
//----------------------------------------------------------------
// Timers Configuration
//----------------------------------------------------------------

    SFRPAGE = 0x00;
//  CKCON = 0x18;   // Clock Control Register
    CKCON = 0x00;   // Clock Control Register    T0,T1均为系统时钟12分频
    TL0 = 0x66;     // Timer 0 Low Byte
    TH0 = 0xFC;     // Timer 0 High Byte
    TL1 = 0xD0;     // Timer 1 Low Byte          //0xD0  9600; 0X40  2400;
    TH1 = 0xD0;     // Timer 1 High Byte    
    TMOD = 0x21;    // Timer Mode Register
					// 定时器1工作于模式2（拥有串口1通讯）、定时器0工作于模式1（16位计数器/定时器）
    TCON = 0x50;    // Timer Control Register 
					// 允许定时器1、定时器0

    TMR2CF = 0x08;  // Timer 2 Configuration      
    RCAP2L = 0xB8;  // Timer 2 Reload Register Low Byte   //0xFFB8 : 9600 bps  0xFEE0 : 2400bps
    RCAP2H = 0xFF;  // Timer 2 Reload Register High Byte  //0xFFDC :19200bps   0xFFEE : 38400bps
    TMR2L = 0xB8;   // Timer 2 Low Byte	
    TMR2H = 0xFF;   // Timer 2 High Byte	
    TMR2CN = 0x04;  // Timer 2 CONTROL		
		
    SFRPAGE = 0x01;                                      //已用作定时器使用
    TMR3CF = 0x00;  // Timer 3 Configuration
    RCAP3L = 0x00;  // Timer 3 Reload Register Low Byte
    RCAP3H = 0x00;  // Timer 3 Reload Register High Byte
    TMR3H = 0x00;   // Timer 3 High Byte
    TMR3L = 0x00;   // Timer 3 Low Byte
    TMR3CN = 0x00;  // Timer 3 Control Register

/*
	/////////////////////////////////////////////////////////////////////////////////////////////
	///	Timer4工作在电平切换输出方式
	/// 产生的方波作为驱动步进电机旋转的步脉冲
	/// 使用系统时钟12分频，自动重装载方式计时
	/////////////////////////////////////////////////////////////////////////////////////////////
    SFRPAGE = 0x02;
    TMR4CF  = 0x02; //	00000010b;	// Timer 3 Configuration
					//	位7-5：000	// 保留
					//	位4-3：00	// SYSCLK
					//	位2  ：0	// 切换输出状态位；写时产生强制输出
					//	位1  ：1	// 电平切换输出在为定时器被分配的端口引脚可用
					//	位0  ：0	// 定时器向上计数，与TnEX 的状态无关
    RCAP4L  = 0x00;					// Timer 3 Reload Register Low Byte
    RCAP4H  = 0x00;					// Timer 3 Reload Register High Byte
    TMR4H   = 0x00;					// Timer 3 High Byte
    TMR4L   = 0x00;					// Timer 3 Low Byte
    TMR4CN  = 0x00; //	00001010b;	// Timer 3 Control Register
					//	位7  ：0	// 定时器上溢/下溢标志；本项目不用
					//	位6  ：0	// 定时器外部标志；本项目不用
					//	位5-4：00	// 保留
					//	位3  ：0	// TnEX上的跳变被忽略
					//	位2  ：0	// 定时器禁止
					//	位1  ：0	// 定时器功能
					//	位0  ：0	// 定时器工作在自动重装载方式
*/
    SFRPAGE = 0x02;
    TMR4CF  = 0x0a;
    RCAP4L  = 0x00;
    RCAP4H  = 0x00;
    TMR4H   = 0x00;
    TMR4L   = 0x00;
    TMR4CN  = 0x00;	// TR4 = 1;
//----------------------------------------------------------------
// Reset Source Configuration
//
// Bit 7  | Bit 6  | Bit 5  | Bit 4  | Bit 3 | Bit 2 | Bit 1 | Bit 0
//------------------------------------------------------------------     
//    R	 |   R/W  |  R/W   |  R/W   |   R   |   R   |  R/W  |  R
//------------------------------------------------------------------
//  JTAG  |Convert | Comp.0 | S/W    | WDT   | Miss. | POR   | HW
// Reset  |Start   | Reset/ | Reset  | Reset | Clock | Force | Pin
// Flag   |Reset/  | Enable | Force  | Flag  | Detect| &     | Reset
//        |Enable  | Flag   | &      |       | Flag  | Flag  | Flag
//        |Flag    |        | Flag   |       |       |       |
//------------------------------------------------------------------
// NOTE! : Comparator 0 must be enabled before it is enabled as a 
// reset source.
//
// NOTE! : External CNVSTR must be enalbed through the crossbar, and
// the crossbar enabled prior to enabling CNVSTR as a reset source 
//------------------------------------------------------------------

    SFRPAGE = 0x00;
	RSTSRC = 0x06;	// Reset Source Register


//----------------------------------------------------------------
// Interrupt Configuration
//----------------------------------------------------------------
    IE = 0x92;          //Interrupt Enable    UART0 ,TO 开取中断
    IP = 0x12;          //Interrupt Priority
    EIE1 = 0x00;        //Extended Interrupt Enable 1
    EIE2 = 0x40;        //Extended Interrupt Enable 2   UART1   
    EIP1 = 0x00;        //Extended Interrupt Priority 1
    EIP2 = 0x00;        //Extended Interrupt Priority 2
	

// other initialization code here...

}   //End of config
