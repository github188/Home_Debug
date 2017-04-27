//7.28灌云下载版本，主要 修改了高空输送线的运行时间，以及提升机构的极限位置。

#include <c8051f040.h>                 // SFR declaration
#include <math.h>
#include "Yaoping.h"
#include "CAN1.h"
#define T2RUN  temppage= SFRPAGE; SFRPAGE=0X00;TR2=1;SFRPAGE=temppage;	//允许定时器运行
#define T2STOP temppage= SFRPAGE; SFRPAGE=0X00;TR2=0;SFRPAGE=temppage		//禁止定时器运行

bit volatile Receive_command_finished = NO;
bit Receive485_command_finished = YES;
bit Time_FLAG = YES;
bit Time_FLAG1 = YES;
bit AUTO_FLAG = NO;
bit Step_FLAG = NO;
bit CANINFOR_FLAG = NO;
bit ONCE_FLAG = NO;
xdata volatile unsigned char Runmode = 0;
//定义定时器的软件计数器
xdata unsigned char  T0Counter5 = 0; //
xdata unsigned char  T0Counter2 = 0;
xdata volatile unsigned char  T0Counter3 = 0; //
xdata unsigned char  T0Counter4 = 0; //
xdata unsigned int  T0Counter6 = 0; //
xdata unsigned char  T0Counter8 = 0;
xdata unsigned int   T0Counter9 = 0;
xdata unsigned int   T0Counter10 = 0; //高空输送线定时器
xdata unsigned char  GAOKONG_COUT = 0; //高空输送线计数器
xdata unsigned char  YAOPINGTISH_COUT = 0; //药品提升计数器
xdata unsigned char  CANcomand = 0;  //CAN命令字
xdata volatile unsigned char  nCANcomand = 0;   //防止CAN中断修改运行中的命令
xdata unsigned char  AUTOCANcomand = 0;
xdata unsigned char  AUTOCMD_STATE = 0;
xdata unsigned char  AUTOCMD_TIME = 0;
xdata volatile unsigned char  AUTOCMD_CHECK = 0;
xdata unsigned char  temppage = 0;
xdata unsigned char BPSSET = 0;
xdata unsigned char IDSET = 0;
xdata unsigned char Shaft1_Run_mode = 0x00;
xdata unsigned long Shaft1_aim_pulse = 0;
xdata unsigned char Sampleperiod = 0;
xdata unsigned int ShowTime1 = 0;
//xdata FLASH Flash;            //定义Flash 缓冲区
xdata CANTRANSDATBUF CANTXBUF_ZKB;
xdata unsigned  int Pulse1 = 0;
xdata unsigned char Shaft1_Base_Speed_Value = 0;
xdata long int Shaft1_Velocity_Subsection_Parameter = 0;
xdata long int motor1_out = 0;                      //步进电机1实际的转速
xdata unsigned char Shaft1_ADD_Speed_Time_Base = 0;
xdata unsigned char Shaft1_SUB_Speed_Time_Base = 0;
xdata unsigned  long Pulse1_Cnt = 0;
xdata unsigned int  Shaft1_Add_Speed = 0;
xdata unsigned int  Shaft1_Speed_max = 0;
xdata long int Vel1 = 0;
xdata unsigned long speed_value = 0;
xdata unsigned char buf[30];
xdata unsigned char Rec1[30];
xdata unsigned  char RecPointer1 = 0;
//xdata  RECBUF1 RxBuf1;
xdata unsigned  long Servoparadisplace = 0;
xdata unsigned  long Servoparaspeed = 0;
xdata unsigned  long Servoparaaddtime = 0;
xdata unsigned  long Servoparasubtime = 0;
xdata unsigned  long ShowTime = 0;
xdata unsigned  char Servopara[107];
xdata volatile unsigned  char CANTRASTEMINFOR[20];
xdata unsigned  char repair_flag = 0;
xdata unsigned  char Servomotor_original = 0; //伺服原点标志位
xdata volatile systemcrtl system_crtl;
xdata Servomotordisplace  Servomotor_displace;
xdata Servomotorspeed     Servomotor_speed;
xdata Servomotoraddtime   Servomotor_addtime;
xdata Servomotorsubtime   Servomotor_subtime;
code unsigned int Speedvalue[2000] = { 60560,
                                       60657, 60751, 60842, 60929, 61012, 61093, 61171, 61246, 61319, 61389, 61457, 61523, 61587, 61649, 61708, 61766, 61823, 61877, 61930, 61982, 62032, 62081, 62128, 62174, 62219, 62262, 62305, 62346, 62387, 62426, 62465, 62502, 62539, 62574, 62609, 62643, 62676, 62709, 62741, 62772, 62802, 62832, 62861, 62889, 62917, 62945, 62971, 62997, 63023, 63048, 63073, 63097, 63121, 63144, 63167, 63189, 63211, 63233, 63254, 63274, 63295, 63315, 63334, 63354, 63373, 63391, 63410, 63428, 63445, 63463, 63480, 63497, 63513, 63530, 63546, 63562, 63577, 63593, 63608, 63622, 63637, 63651, 63666, 63680, 63693, 63707, 63720, 63733, 63746, 63759, 63772, 63784, 63796, 63809, 63820, 63832, 63844, 63855, 63866, 63878,
                                       63889, 63899, 63910, 63921, 63931, 63941, 63952, 63962, 63972, 63981, 63991, 64001, 64010, 64019, 64028, 64038, 64046, 64055, 64064, 64073, 64081, 64090, 64098, 64106, 64115, 64123, 64131, 64139, 64146, 64154, 64162, 64169, 64177, 64184, 64191, 64199, 64206, 64213, 64220, 64227, 64234, 64241, 64247, 64254, 64260, 64267, 64273, 64280, 64286, 64292, 64299, 64305, 64311, 64317, 64323, 64329, 64334, 64340, 64346, 64352, 64357, 64363, 64368, 64374, 64379, 64385, 64390, 64395, 64400, 64405, 64411, 64416, 64421, 64426, 64431, 64435, 64440, 64445, 64450, 64455, 64459, 64464, 64469, 64473, 64478, 64482, 64487, 64491, 64495, 64500, 64504, 64508, 64513, 64517, 64521, 64525, 64529, 64533, 64537, 64541,
                                       64545, 64549, 64553, 64557, 64561, 64565, 64568, 64572, 64576, 64579, 64583, 64587, 64590, 64594, 64598, 64601, 64605, 64608, 64611, 64615, 64618, 64622, 64625, 64628, 64632, 64635, 64638, 64641, 64645, 64648, 64651, 64654, 64657, 64660, 64663, 64666, 64669, 64673, 64675, 64678, 64681, 64684, 64687, 64690, 64693, 64696, 64699, 64701, 64704, 64707, 64710, 64713, 64715, 64718, 64721, 64723, 64726, 64729, 64731, 64734, 64736, 64739, 64742, 64744, 64747, 64749, 64752, 64754, 64756, 64759, 64761, 64764, 64766, 64769, 64771, 64773, 64776, 64778, 64780, 64782, 64785, 64787, 64789, 64791, 64794, 64796, 64798, 64800, 64802, 64805, 64807, 64809, 64811, 64813, 64815, 64817, 64819, 64821, 64824, 64826,
                                       64828, 64830, 64832, 64834, 64836, 64838, 64839, 64841, 64843, 64845, 64847, 64849, 64851, 64853, 64855, 64857, 64858, 64860, 64862, 64864, 64866, 64868, 64869, 64871, 64873, 64875, 64876, 64878, 64880, 64882, 64883, 64885, 64887, 64889, 64890, 64892, 64894, 64895, 64897, 64898, 64900, 64902, 64903, 64905, 64907, 64908, 64910, 64911, 64913, 64914, 64916, 64918, 64919, 64921, 64922, 64924, 64925, 64927, 64928, 64930, 64931, 64933, 64934, 64935, 64937, 64938, 64940, 64941, 64943, 64944, 64945, 64947, 64948, 64950, 64951, 64952, 64954, 64955, 64956, 64958, 64959, 64961, 64962, 64963, 64964, 64966, 64967, 64968, 64970, 64971, 64972, 64974, 64975, 64976, 64977, 64979, 64980, 64981, 64982, 64984,
                                       64985, 64986, 64987, 64988, 64990, 64991, 64992, 64993, 64994, 64996, 64997, 64998, 64999, 65000, 65001, 65003, 65004, 65005, 65006, 65007, 65008, 65009, 65010, 65012, 65013, 65014, 65015, 65016, 65017, 65018, 65019, 65020, 65021, 65022, 65023, 65025, 65026, 65027, 65028, 65029, 65030, 65031, 65032, 65033, 65034, 65035, 65036, 65037, 65038, 65039, 65040, 65041, 65042, 65043, 65044, 65045, 65046, 65047, 65048, 65049, 65050, 65051, 65051, 65052, 65053, 65054, 65055, 65056, 65057, 65058, 65059, 65060, 65061, 65062, 65063, 65063, 65064, 65065, 65066, 65067, 65068, 65069, 65070, 65071, 65071, 65072, 65073, 65074, 65075, 65076, 65077, 65077, 65078, 65079, 65080, 65081, 65082, 65082, 65083, 65084,
                                       65085, 65086, 65087, 65087, 65088, 65089, 65090, 65091, 65091, 65092, 65093, 65094, 65095, 65095, 65096, 65097, 65098, 65098, 65099, 65100, 65101, 65101, 65102, 65103, 65104, 65105, 65105, 65106, 65107, 65107, 65108, 65109, 65110, 65110, 65111, 65112, 65113, 65113, 65114, 65115, 65115, 65116, 65117, 65118, 65118, 65119, 65120, 65120, 65121, 65122, 65122, 65123, 65124, 65125, 65125, 65126, 65127, 65127, 65128, 65129, 65129, 65130, 65131, 65131, 65132, 65133, 65133, 65134, 65135, 65135, 65136, 65136, 65137, 65138, 65138, 65139, 65140, 65140, 65141, 65142, 65142, 65143, 65143, 65144, 65145, 65145, 65146, 65146, 65147, 65148, 65148, 65149, 65150, 65150, 65151, 65151, 65152, 65153, 65153, 65154,
                                       65154, 65155, 65155, 65156, 65157, 65157, 65158, 65158, 65159, 65159, 65160, 65161, 65161, 65162, 65162, 65163, 65163, 65164, 65165, 65165, 65166, 65166, 65167, 65167, 65168, 65168, 65169, 65169, 65170, 65171, 65171, 65172, 65172, 65173, 65173, 65174, 65174, 65175, 65175, 65176, 65176, 65177, 65177, 65178, 65178, 65179, 65179, 65180, 65181, 65181, 65182, 65182, 65183, 65183, 65184, 65184, 65185, 65185, 65186, 65186, 65187, 65187, 65188, 65188, 65188, 65189, 65189, 65190, 65190, 65191, 65191, 65192, 65192, 65193, 65193, 65194, 65194, 65195, 65195, 65196, 65196, 65197, 65197, 65197, 65198, 65198, 65199, 65199, 65200, 65200, 65201, 65201, 65202, 65202, 65202, 65203, 65203, 65204, 65204, 65205,
                                       65205, 65206, 65206, 65206, 65207, 65207, 65208, 65208, 65209, 65209, 65210, 65210, 65210, 65211, 65211, 65212, 65212, 65213, 65213, 65213, 65214, 65214, 65215, 65215, 65215, 65216, 65216, 65217, 65217, 65217, 65218, 65218, 65219, 65219, 65220, 65220, 65220, 65221, 65221, 65222, 65222, 65222, 65223, 65223, 65224, 65224, 65224, 65225, 65225, 65225, 65226, 65226, 65227, 65227, 65227, 65228, 65228, 65229, 65229, 65229, 65230, 65230, 65230, 65231, 65231, 65232, 65232, 65232, 65233, 65233, 65233, 65234, 65234, 65235, 65235, 65235, 65236, 65236, 65236, 65237, 65237, 65237, 65238, 65238, 65238, 65239, 65239, 65240, 65240, 65240, 65241, 65241, 65241, 65242, 65242, 65242, 65243, 65243, 65243, 65244,
                                       65244, 65244, 65245, 65245, 65245, 65246, 65246, 65246, 65247, 65247, 65247, 65248, 65248, 65249, 65249, 65249, 65249, 65250, 65250, 65250, 65251, 65251, 65251, 65252, 65252, 65252, 65253, 65253, 65253, 65254, 65254, 65254, 65255, 65255, 65255, 65256, 65256, 65256, 65257, 65257, 65257, 65258, 65258, 65258, 65258, 65259, 65259, 65259, 65260, 65260, 65260, 65261, 65261, 65261, 65262, 65262, 65262, 65262, 65263, 65263, 65263, 65264, 65264, 65264, 65265, 65265, 65265, 65265, 65266, 65266, 65266, 65267, 65267, 65267, 65267, 65268, 65268, 65268, 65269, 65269, 65269, 65270, 65270, 65270, 65270, 65271, 65271, 65271, 65272, 65272, 65272, 65272, 65273, 65273, 65273, 65273, 65274, 65274, 65274, 65275,
                                       65275, 65275, 65275, 65276, 65276, 65276, 65276, 65277, 65277, 65277, 65278, 65278, 65278, 65278, 65279, 65279, 65279, 65279, 65280, 65280, 65280, 65281, 65281, 65281, 65281, 65282, 65282, 65282, 65282, 65283, 65283, 65283, 65283, 65284, 65284, 65284, 65284, 65285, 65285, 65285, 65285, 65286, 65286, 65286, 65286, 65287, 65287, 65287, 65287, 65288, 65288, 65288, 65288, 65289, 65289, 65289, 65289, 65290, 65290, 65290, 65290, 65291, 65291, 65291, 65291, 65292, 65292, 65292, 65292, 65293, 65293, 65293, 65293, 65294, 65294, 65294, 65294, 65294, 65295, 65295, 65295, 65295, 65296, 65296, 65296, 65296, 65297, 65297, 65297, 65297, 65297, 65298, 65298, 65298, 65298, 65299, 65299, 65299, 65299, 65300,
                                       65300, 65300, 65300, 65300, 65301, 65301, 65301, 65301, 65302, 65302, 65302, 65302, 65302, 65303, 65303, 65303, 65303, 65304, 65304, 65304, 65304, 65304, 65305, 65305, 65305, 65305, 65305, 65306, 65306, 65306, 65306, 65307, 65307, 65307, 65307, 65307, 65308, 65308, 65308, 65308, 65308, 65309, 65309, 65309, 65309, 65309, 65310, 65310, 65310, 65310, 65310, 65311, 65311, 65311, 65311, 65312, 65312, 65312, 65312, 65312, 65313, 65313, 65313, 65313, 65313, 65314, 65314, 65314, 65314, 65314, 65315, 65315, 65315, 65315, 65315, 65316, 65316, 65316, 65316, 65316, 65316, 65317, 65317, 65317, 65317, 65317, 65318, 65318, 65318, 65318, 65318, 65319, 65319, 65319, 65319, 65319, 65320, 65320, 65320, 65320,
                                       65320, 65321, 65321, 65321, 65321, 65321, 65321, 65322, 65322, 65322, 65322, 65322, 65323, 65323, 65323, 65323, 65323, 65323, 65324, 65324, 65324, 65324, 65324, 65325, 65325, 65325, 65325, 65325, 65325, 65326, 65326, 65326, 65326, 65326, 65327, 65327, 65327, 65327, 65327, 65327, 65328, 65328, 65328, 65328, 65328, 65328, 65329, 65329, 65329, 65329, 65329, 65329, 65330, 65330, 65330, 65330, 65330, 65331, 65331, 65331, 65331, 65331, 65331, 65332, 65332, 65332, 65332, 65332, 65332, 65333, 65333, 65333, 65333, 65333, 65333, 65334, 65334, 65334, 65334, 65334, 65334, 65335, 65335, 65335, 65335, 65335, 65335, 65336, 65336, 65336, 65336, 65336, 65336, 65336, 65337, 65337, 65337, 65337, 65337, 65337,
                                       65338, 65338, 65338, 65338, 65338, 65338, 65339, 65339, 65339, 65339, 65339, 65339, 65339, 65340, 65340, 65340, 65340, 65340, 65340, 65341, 65341, 65341, 65341, 65341, 65341, 65341, 65342, 65342, 65342, 65342, 65342, 65342, 65343, 65343, 65343, 65343, 65343, 65343, 65343, 65344, 65344, 65344, 65344, 65344, 65344, 65345, 65345, 65345, 65345, 65345, 65345, 65345, 65346, 65346, 65346, 65346, 65346, 65346, 65346, 65347, 65347, 65347, 65347, 65347, 65347, 65347, 65348, 65348, 65348, 65348, 65348, 65348, 65348, 65349, 65349, 65349, 65349, 65349, 65349, 65349, 65350, 65350, 65350, 65350, 65350, 65350, 65350, 65351, 65351, 65351, 65351, 65351, 65351, 65351, 65351, 65352, 65352, 65352, 65352, 65352,
                                       65352, 65352, 65353, 65353, 65353, 65353, 65353, 65353, 65353, 65354, 65354, 65354, 65354, 65354, 65354, 65354, 65354, 65355, 65355, 65355, 65355, 65355, 65355, 65355, 65356, 65356, 65356, 65356, 65356, 65356, 65356, 65356, 65357, 65357, 65357, 65357, 65357, 65357, 65357, 65357, 65358, 65358, 65358, 65358, 65358, 65358, 65358, 65359, 65359, 65359, 65359, 65359, 65359, 65359, 65359, 65360, 65360, 65360, 65360, 65360, 65360, 65360, 65360, 65361, 65361, 65361, 65361, 65361, 65361, 65361, 65361, 65362, 65362, 65362, 65362, 65362, 65362, 65362, 65362, 65362, 65363, 65363, 65363, 65363, 65363, 65363, 65363, 65363, 65364, 65364, 65364, 65364, 65364, 65364, 65364, 65364, 65365, 65365, 65365, 65365,
                                       65365, 65365, 65365, 65365, 65365, 65366, 65366, 65366, 65366, 65366, 65366, 65366, 65366, 65367, 65367, 65367, 65367, 65367, 65367, 65367, 65367, 65367, 65368, 65368, 65368, 65368, 65368, 65368, 65368, 65368, 65368, 65369, 65369, 65369, 65369, 65369, 65369, 65369, 65369, 65369, 65370, 65370, 65370, 65370, 65370, 65370, 65370, 65370, 65371, 65371, 65371, 65371, 65371, 65371, 65371, 65371, 65371, 65371, 65372, 65372, 65372, 65372, 65372, 65372, 65372, 65372, 65372, 65373, 65373, 65373, 65373, 65373, 65373, 65373, 65373, 65373, 65374, 65374, 65374, 65374, 65374, 65374, 65374, 65374, 65374, 65375, 65375, 65375, 65375, 65375, 65375, 65375, 65375, 65375, 65375, 65376, 65376, 65376, 65376, 65376,
                                       65376, 65376, 65376, 65376, 65376, 65377, 65377, 65377, 65377, 65377, 65377, 65377, 65377, 65377, 65378, 65378, 65378, 65378, 65378, 65378, 65378, 65378, 65378, 65378, 65379, 65379, 65379, 65379, 65379, 65379, 65379, 65379, 65379, 65379, 65380, 65380, 65380, 65380, 65380, 65380, 65380, 65380, 65380, 65380, 65380, 65381, 65381, 65381, 65381, 65381, 65381, 65381, 65381, 65381, 65381, 65382, 65382, 65382, 65382, 65382, 65382, 65382, 65382, 65382, 65382, 65383, 65383, 65383, 65383, 65383, 65383, 65383, 65383, 65383, 65383, 65383, 65384, 65384, 65384, 65384, 65384, 65384, 65384, 65384, 65384, 65384, 65384, 65385, 65385, 65385, 65385, 65385, 65385, 65385, 65385, 65385, 65385, 65386, 65386, 65386,
                                       65386, 65386, 65386, 65386, 65386, 65386, 65386, 65386, 65387, 65387, 65387, 65387, 65387, 65387, 65387, 65387, 65387, 65387, 65387, 65387, 65388, 65388, 65388, 65388, 65388, 65388, 65388, 65388, 65388, 65388, 65388, 65389, 65389, 65389, 65389, 65389, 65389, 65389, 65389, 65389, 65389, 65389, 65390, 65390, 65390, 65390, 65390, 65390, 65390, 65390, 65390, 65390, 65390, 65390, 65391, 65391, 65391, 65391, 65391, 65391, 65391, 65391, 65391, 65391, 65391, 65391, 65392, 65392, 65392, 65392, 65392, 65392, 65392, 65392, 65392, 65392, 65392, 65393, 65393, 65393, 65393, 65393, 65393, 65393, 65393, 65393, 65393, 65393, 65393, 65393, 65394, 65394, 65394, 65394, 65394, 65394, 65394, 65394, 65394, 65394,
                                       65394, 65394, 65395, 65395, 65395, 65395, 65395, 65395, 65395, 65395, 65395, 65395, 65395, 65395, 65396, 65396, 65396, 65396, 65396, 65396, 65396, 65396, 65396, 65396, 65396, 65396, 65396, 65397, 65397, 65397, 65397, 65397, 65397, 65397, 65397, 65397, 65397, 65397, 65397, 65397, 65398, 65398, 65398, 65398, 65398, 65398, 65398, 65398, 65398, 65398, 65398, 65398, 65398, 65399, 65399, 65399, 65399, 65399, 65399, 65399, 65399, 65399, 65399, 65399, 65399, 65399, 65400, 65400, 65400, 65400, 65400, 65400, 65400, 65400, 65400, 65400, 65400, 65400, 65400, 65401, 65401, 65401, 65401, 65401, 65401, 65401, 65401, 65401, 65401, 65401, 65401, 65401, 65401, 65402, 65402, 65402, 65402, 65402, 65402, 65402,
                                       65402, 65402, 65402, 65402, 65402, 65402, 65403, 65403, 65403, 65403, 65403, 65403, 65403, 65403, 65403, 65403, 65403, 65403, 65403, 65403, 65404, 65404, 65404, 65404, 65404, 65404, 65404, 65404, 65404, 65404, 65404, 65404, 65404, 65404, 65404, 65405, 65405, 65405, 65405, 65405, 65405, 65405, 65405, 65405, 65405, 65405, 65405, 65405, 65405, 65406, 65406, 65406, 65406, 65406, 65406, 65406, 65406, 65406, 65406, 65406, 65406, 65406, 65406, 65406, 65407, 65407, 65407, 65407, 65407, 65407, 65407, 65407, 65407, 65407, 65407, 65407, 65407, 65407, 65408, 65408, 65408, 65408, 65408, 65408, 65408, 65408, 65408, 65408, 65408, 65408, 65408, 65408, 65408, 65409, 65409, 65409, 65409, 65409, 65409, 65409,
                                       65409, 65409, 65409, 65409, 65409, 65409, 65409, 65409, 65409, 65410, 65410, 65410, 65410, 65410, 65410, 65410, 65410, 65410, 65410, 65410, 65410, 65410, 65410, 65410, 65411, 65411, 65411, 65411, 65411, 65411, 65411, 65411, 65411, 65411, 65411, 65411, 65411, 65411, 65411, 65411, 65412, 65412, 65412, 65412, 65412, 65412, 65412, 65412, 65412, 65412, 65412, 65412, 65412, 65412, 65412, 65412, 65413, 65413, 65413, 65413, 65413, 65413, 65413, 65413, 65413, 65413, 65413, 65413, 65413, 65413, 65413, 65413, 65413, 65414, 65414, 65414, 65414, 65414, 65414, 65414, 65414, 65414, 65414, 65414, 65414, 65414, 65414, 65414, 65414, 65415, 65415, 65415, 65415, 65415, 65415, 65415, 65415, 65415, 65415
                                     };

void Dir_ZZ()
{
    Motor1_Dir = 1;	//正转
}
void Dir_FZ()
{
    Motor1_Dir = 0;	//反转
}

/**************************************************************************/
void delay_ms(unsigned int ms)
{
    unsigned int us = 1000;
    while(ms--)
    {
        WDTCN = 0xA5;
        delay1(us);
    }
}
/**************************************************************************/
void Watchdog_Init (void)
{
    WDTCN &= ~0x80;                     // WDTCN.7 must be logic 0 when setting
    // the interval.
    WDTCN |= 0x07;                      // Set the WDTCN[2-0] to 111b
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
    //init_T2 ();
    //init_INT0();
    //init_INT1();
    SFRPAGE  = CONFIG_PAGE;

    repair_flag = P7 & 0x03 ;
    //IDSET = 0x02;        //CAN接受地址
    IDSET = P7 & 0x7F ;
    IDSET = IDSET >> 4;
    BPSSET = 0x00;       //CAN波特率设置
    init_para();
    //PT2=1;
    PT0 = 1;
    //E2PROM初始化
    WDTCN = 0xA5;  //看门狗复位
    EA = 1;	//开中断
    //motor1_ENB=0;
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
    char SFRPAGE_SAVE = SFRPAGE;     // Save Current SFR page

    WDTCN = 0xA5;                    // 喂狗

    T0Counter2++;
    T0Counter3++;
    T0Counter4++;
    T0Counter5++;
    T0Counter6++;
    T0Counter8++;
    T0Counter9++;
    T0Counter10++;  //高空输送线定时器
    SFRPAGE = 0x0F;  //关闭高空输送线

    //高空输送线运行定时
    if(T0Counter10 > GAOKONG_TIMER)
    {
        if(Motor2 == 0)	//已启动
        {
            if((CANTRASTEMINFOR[8] & 0x04) == 0x04) //高空输送线传感器检测到药品
            {
                T0Counter10 = 0;
                GAOKONG_COUT = GAOKONG_COUT + 1;
                if(GAOKONG_COUT > 3)	//超出等待次数
                {
                    Motor2 = 1; //关闭高空输送线
                    AUTOCMD_STATE = 0XC4;
                    T0Counter8 = 51;
                    AUTOCMD_TIME = 0;
                }//报警*/
            }
            else	//高空输送线传感器未检测到药品
            {
                Motor2 = 1;
                T0Counter10 = 0;
                GAOKONG_COUT = 0;
            }
        }
        else	//未启动
        {
            T0Counter10 = 0;
            GAOKONG_COUT = 0;
        }
    }
    //提升输送线传感器信号检测并写入
    if(MONITOR_INPUT2 == 1)
    {
        /*SFRPAGE = 0x0F;
        Motor1_B=1;
        SFRPAGE = SFRPAGE_SAVE;*/
        CANTRASTEMINFOR[8] = CANTRASTEMINFOR[8] | 0x08;
    }
    else
    {
        CANTRASTEMINFOR[8] = CANTRASTEMINFOR[8] & 0xF7;
    }
    //高空输送线传感器信号检测并写入
    if(MONITOR_INPUT1 == 1)
    {
        //SFRPAGE = 0x0F;
        //Motor1_Power=1;
        //SFRPAGE = SFRPAGE_SAVE;
        CANTRASTEMINFOR[8] = CANTRASTEMINFOR[8] | 0x04;
    }
    else
    {
        CANTRASTEMINFOR[8] = CANTRASTEMINFOR[8] & 0xFB;
    }
    //
    SFRPAGE = SFRPAGE_SAVE;
    //
    SFRPAGE = TIMER01_PAGE;
    TH0 = 0xDC;	//重新装载定时值
    TL0 = 0x00;
    SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
    return;
}

//-----------------------------------------------------------------------------
// Port_Init ()
//-----------------------------------------------------------------------------

void PORT_Init (void)
{
    // Configure the XBRn Registers
    char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page
    SFRPAGE = CONFIG_PAGE;              // Set SFR page
    // Configure the XBRn Registers

    SFRPAGE = 0x0F;
    XBR0 = 0x04;	// XBAR0: Initial Reset Value  串口0
    XBR1 = 0x34;	// XBAR1: Initial Reset Value  T2定时器输出、外部中断1 和 外部中断0
    XBR2 = 0x40;	// XBAR2: Initial Reset Value  交叉总开关
    // XBR3 = 0x01;    // XBAR3: Initial Reset Value  T3定时器输出
    // Select Pin I/0

    // NOTE: Some peripheral I/O pins can function as either inputs or
    // outputs, depending on the configuration of the peripheral. By default,
    // the configuration utility will configure these I/O pins as push-pull
    // outputs.
    // Port configuration (1 = Push Pull Output)
    SFRPAGE = 0x0F;
    P0MDOUT = 0xF1; // 1111 1001b Output configuration for P0
    P1MDOUT = 0x00; // Output configuration for P1
    P2MDOUT = 0x00; // Output configuration for P2
    P3MDOUT = 0x00; // Output configuration for P3

    P4MDOUT = 0xFF; // Output configuration for P4
    P5MDOUT = 0xFF; // Output configuration for P5
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
    SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
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
    unsigned char i;
    for(i = 0; i < 8; i++)
    {
        CANTXBUF_ZKB.buf[i] = 0;
    }
    for(i = 0; i < 30; i++)
    {
        Rec1[i] = 0;
    }
    for(i = 0; i < 4; i++) //清伺服运行参数
    {
        Servomotor_displace.buf[i] = 0;
        Servomotor_speed.buf[i] = 0;;
        Servomotor_addtime.buf[i] = 0;;
        Servomotor_subtime.buf[i] = 0;;
    }
    Board_Parameter_Setup();    //各驱动板参数初始化
    AUTOCMD_CHECK = 0;
    Runmode = 2;
    /***********************************/
    //初始化步进电机1参数
    /**********************************/
    Shaft1_Run_mode = 0x00;
    Shaft1_aim_pulse = 0;
    RE1 = 0;
    DE1 = 0;
    Servoparadisplace = 500000;
    Servoparaspeed = 50000;
    Servoparaaddtime = 100;
    Servoparasubtime = 100;
    /*for(i=0;i<5;i++)
    {
    station[i]=0;
    }*/
    for(i = 0; i < 107; i++)
    {
        Servopara[i] = 0;
    }

    for(i = 0; i < 20; i++)
    {
        CANTRASTEMINFOR[i] = 0;
    }
    read_from_flash();
    system_crtl.AUTOsystem_command = 0;
    system_crtl.AUTOsystem_alarm1 = 0;
    //system_crtl.AUTOsystem_alarm2=0;


    Receive485_command_finished = YES;
}
void Board_Parameter_Setup(void)
{
    switch(IDSET)
    {
    case 01:    //X	轴参数
        init_can1_1();   //初始化c8051f040自带CAN

        //Current_Shaft1_Code=0x01;
        Shaft1_ADD_Speed_Time_Base = 10;
        Shaft1_SUB_Speed_Time_Base = 10;
        Shaft1_Base_Speed_Value = 50;
        Shaft1_Velocity_Subsection_Parameter = 248830;
        Shaft1_Add_Speed = 15;
        Shaft1_Speed_max = 400;   //40
        break;
    case 02:    //Y	轴参数					{
        init_can1_1();   //初始化c8051f040自带CAN
        break;
    case 03:    //Y	轴参数					{
        init_can1_1();   //初始化c8051f040自带CAN
        break;
    case 04:    //Y	轴参数					{
        init_can1_1();   //初始化c8051f040自带CAN
        break;
    default:
        break;
    }
}


/* 步进电机1驱动控制
在此函数中，正转代表为电机离开原点运行；反转代表为电机背离原点运行
*/
void Shaft1_CTL(void)
{
    //long int speed_value=0;

    if(Step_FLAG == YES)
    {
        T2STOP;
        Step_FLAG = YES;
        if(Shaft1_Run_mode == 0x01 || Shaft1_Run_mode == 0x02 || Shaft1_Run_mode == 0x03)
        {
            Shaft1_aim_pulse = 0;
            Pulse1_Cnt = 0;
            motor1_out = 0;
            T0Counter4 = 0;
            CANTRASTEMINFOR[8] = CANTRASTEMINFOR[8] & 0XFD; //清除运行标志
            Shaft1_Run_mode = 0x00;

            CANTRASTEMINFOR[13] = (unsigned char)((ShowTime & 0x0000FF00) >> 8);
            CANTRASTEMINFOR[14] = (unsigned char)(ShowTime & 0x000000FF);
        }
        return;
    }
}
/////////////////////////////////////////////////////////////////
//int ramp(int max,int step)  电机1 缓加减速
// 带有急停功能的子程序
// 单位为°/秒
////////////////////////////////////////////////////////////////
void  ramp1_1(long int max)
{
    //unsigned int nstep=0;
    /*if(!run_sign)
    {
    motor1_out =0;
    }
    else
    {*/
    if(motor1_out >= (max + Shaft1_Add_Speed))
    {
        //nstep=nstep+15000;
        motor1_out -= Shaft1_Add_Speed;
    }
    else if(motor1_out <= (max - Shaft1_Add_Speed))
    {
        if( T0Counter4 >= Shaft1_ADD_Speed_Time_Base )        //加速时加速度时间参数
        {
            T0Counter4 = 0;
            motor1_out += Shaft1_Add_Speed;
        }
    }
    else {
        motor1_out = max;
    }
    //}
    //return motor1_out;
}

//-----------------------------------------------------------
// 名    称：Serial_Port_Interrupt
// 功    能：串口中断服务程序
//-----------------------------------------------------------
//# pragma disable
# pragma enable
void Serial_Port_Interrupt(void) interrupt 4
{
    //unsigned int   TEMP,K=0;
    //unsigned char   xdata TEMP1,TEMP2,len=0,j=0,Check_Sum=0;
    xdata unsigned int   K = 0;
    xdata unsigned char  j = 0;
    char SFRPAGE_SAVE = SFRPAGE;

    SFRPAGE = UART0_PAGE;
    if(!RI0)		return ;		//误动作，退出
    WDTCN = 0xA5;
    //接收文件头
    RI0 = 0;
    Rec1[j] = SBUF0;	//接收的第一字节
    //TEMP1 = Receiv_Msg[j];
    //Check_Sum += Receiv_Msg[j];
    j++;
    K = 0;

    while(!RI0)			//超时处理
    {
        K++;
        WDTCN = 0xA5;
        if(K > Delay_Times)
            goto END_ISR;
    }
    RI0 = 0;
    Rec1[j] = SBUF0;		//接收的第二字节

    //TEMP2 = Receiv_Msg[j];
    //Check_Sum += Receiv_Msg[j];
    j++;
    /*TEMP=((UINT)TEMP1<<8)+TEMP2;	//合成接收的文件头

    if(TEMP != 0xEFEF)		//查接收的文件头
    	goto END_ISR;
    接收长度字节
    K=0;
    while(!RI0)
    {
    	K++;
    	if(K>Delay_Times)
    		goto END_ISR;
    }
    RI0=0;
    Receiv_Msg[j]=SBUF0;
    len=Receiv_Msg[j];
    Check_Sum += Receiv_Msg[j];
    j++;*/
    //接收数据、指令字节
    if((Rec1[0] == 0x01) && (Rec1[1] == 0x10))
    {
//        for(; j < 6 + 2; j++)
//        {
//            K = 0;
//            while(!RI0)
//            {
//                K++;
//                WDTCN = 0xA5;
//                if(K > Delay_Times)
//                    goto END_ISR ;
//            }
//            RI0 = 0;
//            Rec1[j] = SBUF0;
//            //Check_Sum += Receiv_Msg[j];

//        }
    }
    else if((Rec1[0] == 0x01) && (Rec1[1] == 0x03))
    {
        for(; j < 7 + 2; j++)
        {
            K = 0;
            while(!RI0)
            {
                K++;
                WDTCN = 0xA5;
                if(K > Delay_Times)
								{
//									    Receive_command_finished = YES;
										goto END_ISR ;
								}
            }
            RI0 = 0;
            Rec1[j] = SBUF0;
            //Check_Sum += Receiv_Msg[j];
        }
//				Receive_command_finished = YES;
    }
    else
    {
        ;
    }
    Receive_command_finished = YES;
END_ISR:
    RI0  = 0;
    ES0  = 1;
    SFRPAGE = SFRPAGE_SAVE;             // Restore SFRPAGE
    return;
}

/*void init_T2 (void)
{
	/////////////////////////////////////////////////////////////////////////////////////////////
	///	Timer2工作在电平切换输出方式
	/// 产生的方波步进电机的控制脉冲
	/// 使用系统时钟，自动重装载方式计时
	/// 方波频率：1MHz  0xFFFA；500kHz   0xFFF4
	/////////////////////////////////////////////////////////////////////////////////////////////
    char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page
    SFRPAGE = 0x00;
    TMR2CF  = 0x0A; //	00001010b;	// Timer 3 Configuration
	//	位7-5：000	// 保留
	//	位4-3：01	// SYSCLK
	//	位2  ：0	// 切换输出状态位；写时产生强制输出
	//	位1  ：1	// 电平切换输出在为定时器被分配的端口引脚可用
	//	位0  ：0	// 定时器向上计数，与TnEX 的状态无关
    RCAP2L  = 0x00;					// Timer 3 Reload Register Low Byte
    RCAP2H  = 0x00;					// Timer 3 Reload Register High Byte
    TMR2H   = 0x00;					// Timer 3 High Byte
    TMR2L   = 0x00;					// Timer 3 Low Byte
    TMR2CN  = 0x00; //	00001010b;	// Timer 3 Control Register
	//	位7  ：0	// 定时器上溢/下溢标志；本项目不用
	//	位6  ：0	// 定时器外部标志；本项目不用
	//	位5-4：00	// 保留
	//	位3  ：0	// TnEX上的跳变被忽略
	//	位2  ：0	// 定时器禁止
	//	位1  ：0	// 定时器功能
	//	位0  ：0	// 定时器工作在自动重装载方式

    ET2=1;
    SFRPAGE = SFRPAGE_SAVE;             // Restore SFRPAGE
}*/
/*void init_INT0 (void)
{
SFRPAGE  = CONFIG_PAGE;        //Port SFR's on Configuration page
XBR1  |= 0x04;                 //配置INT0管脚
SFRPAGE  = LEGACY_PAGE;
EA=0;
EX0 = 1;    //使能INT0
PX0 = 1;    //高优先级
IT0=1;      //下降沿触发中断
EA=1;
}
//对应于X轴为左边的限位开关，对应于Y轴为上边的限位开关
//外部中断INT0 ，高优先级
void INT0_ISR (void) interrupt 0
{
unsigned char temp;
temp=SFRPAGE;
SFRPAGE  = LEGACY_PAGE;
EX0 = 0 ;   //关外部中断0
Pulse1_Cnt=0;        //在此处进行清零电机1,当回到原点位置时，清零步进脉冲计数。
Shaft1_Run_mode=0x00;
Motor1_Begin_Run_Sign =NO;
IE0=0;      //INT0中断复位
EX0 = 1 ;   //开外部中断0
SFRPAGE=temp;
}*/
/*void init_INT0 (void)
{
SFRPAGE  = CONFIG_PAGE;        //Port SFR's on Configuration page
XBR1  |= 0x04;                 //配置INT0管脚
SFRPAGE  = LEGACY_PAGE;
EA=0;
EX0 = 1;    //使能INT0
PX0 = 1;    //高优先级
IT0=1;      //下降沿触发中断
EA=1;
}
//对应于X轴为左边的限位开关，对应于Y轴为上边的限位开关
//外部中断INT0 ，高优先级
void INT0_ISR (void) interrupt 0
{
unsigned char temp;
temp=SFRPAGE;
SFRPAGE  = LEGACY_PAGE;
EX0 = 0 ;   //关外部中断0
IE0=0;      //INT0中断复位
EX0 = 1 ;   //开外部中断0
SFRPAGE=temp;
}

 void init_INT1 (void)
 {

	 SFRPAGE  = CONFIG_PAGE; //Port SFR's on Configuration page
	 XBR1  |= 0x10;          //配置INT0管脚
	 SFRPAGE  = LEGACY_PAGE;
	 EA=0;
	 EX1 = 1;               //使能INT1
	 PX1 = 1;               //高优先级
	 IT1=1;                 //下降沿触发中断
	 EA=1;
	 }

	  //外部中断INT1 ，高优先级
	  //对应于X轴为右边的限位开关，对应于Y轴为下边的限位开关
	  void INT1_ISR (void) interrupt 2
	  {
	  unsigned char temp;
	  temp=SFRPAGE;
	  SFRPAGE  = LEGACY_PAGE;
	  EX1 = 0 ;           //关外部中断1

	   IE1=0;               //INT1中断复位
	   EX1 = 1 ;            //开外部中断1
	   SFRPAGE=temp;
}*/

//////////////////////////////////////////////////////////////////////
//T2溢出中断    电机1用
//为计输出脉冲数
//根据实际接线情况判断：步进电机控制没有使用T2输出脉冲，即定时器2未起作用
//////////////////////////////////////////////////////////////////////
void  ISR_T2(void)  interrupt 5
{
    unsigned char temp;
    temp = SFRPAGE;
    SFRPAGE = 0x00;
    TF2 = 0;
    Pulse1++;
    if (Pulse1 >= 10)
    {
        Pulse1 = 0;
        Pulse1_Cnt++;
        //T2STOP;
        switch(Shaft1_Run_mode)
        {
        case 1:
            if(Shaft1_aim_pulse >= Pulse1_Cnt)
            {
                ramp1_1((Shaft1_aim_pulse - Pulse1_Cnt) * 2);
                speed_value = motor1_out + Shaft1_Base_Speed_Value;
                //speed_value=1000;
                motor1_dir = 1;
                Vel1 = speed_value;
                Motor1_CTL();
            }
            break;

        case 2:
            if(Shaft1_aim_pulse >= Pulse1_Cnt)
            {
                ramp1_1((Shaft1_aim_pulse - Pulse1_Cnt) * 2);
                speed_value = motor1_out + Shaft1_Base_Speed_Value;
                motor1_dir = 0;
                Vel1 = speed_value;
                Motor1_CTL();
            }
            break;

        case 3:
            if(Shaft1_aim_pulse >= Pulse1_Cnt)
            {
                ramp1_1((Shaft1_aim_pulse - Pulse1_Cnt) * 2);
                speed_value = motor1_out + Shaft1_Base_Speed_Value;
                motor1_dir = 0;
                Vel1 = speed_value;
                Motor1_CTL();
            }
            break;

        default:
            speed_value = 0;
            break;
        }
    }
    SFRPAGE = temp;
}
///////////////////////////////////// /////////////////////////////////
//电机1转速表达:°/秒.
//输入范围:0~6000°/S;  （相当于步进电机1000r/min）
//T2=65536-1000000*11.0592*360/(200*800*Vel)=65536-24883/Vel
//T2=65536-1000000*11.0592*360/(200*400*Vel)=65536-49766/Vel
//T2=65536-1000000*11.0592*360/(200*64*Vel)=65536-311040/Vel
//T2=65536-1000000*11.0592*360/(200*16*Vel)=65536-1244160/Vel


//电机1转速表达:r/min.
//T2=65536-1000000*11.0592*3/(2*10*16*Vel)=65536-103680/Vel
//T2=65536-1000000*11.0592*3/(2*10*64*Vel)=65536-25920/Vel
//T2=65536-1000000*11.0592*3/(2*10*400*Vel)=65536-4147/Vel

//////////////////////////////////////////////////////////////////////
void Motor1_CTL(void)     //T2输出，控制电机1
{
    unsigned int temp1;
    //	long int motor1_v;
    //unsigned long ShowTime;

    if(Vel1 <= Shaft1_Base_Speed_Value)             //避免低速时出现啸叫
    {
        T2STOP;
        Time_FLAG1 = YES;
        Step_FLAG = YES;
        ShowTime = T0Counter6 * 10;
        return;
    } else
    {
        if(Vel1 > Shaft1_Speed_max)
            Vel1 = Shaft1_Speed_max;          //限制最大速度，
        //temp1 = (65536 -Shaft1_Velocity_Subsection_Parameter/Vel1);
        temp1 = Speedvalue[Vel1 - 50];
        SFRPAGE = 0x00;
        RCAP2L = temp1 & 0x00FF;
        temp1   = temp1 >> 8;
        RCAP2H = temp1 & 0x00FF;
        //T2RUN;
    }
    //else if(Vel1<50)
    //Vel1=50;
    // motor1_v = Vel1;
}
//仅配为RS485模式  与RS422的区别就是在发送程序中，增加了发送前，关闭接受端口；发送完即打开接受端口。
void Uart0Send(unsigned char *buf, unsigned char bufsize )
{
    unsigned char i = 0;
    unsigned int k = 0;
    unsigned int crc_z = 0;
    char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page

    SFRPAGE = CONFIG_PAGE;

    RE1 = 1;
    DE1 = 1;
    crc_z = crc_chk(buf, bufsize); //crc校验结果
    k = crc_z & 0x00ff;

    buf[bufsize] = (unsigned char)(k); //+CRC_Hi
    bufsize++;
    k = crc_z & 0xff00;
    k = k >> 8;
    buf[bufsize] = (unsigned char)(k); //+CRC_Li
    bufsize++;
    i = 0;
    delay_ms(10);
    ES0 = 0;
    SFRPAGE = UART0_PAGE;
    do
    {
//				ACC = buf[i];
//        TB80 = P;
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



//-----------------------------------------------------------------------------
//自定义延时
//延时时间约为(us N10)毫秒
//-----------------------------------------------------------------------------
void delay1( unsigned int us)
{
    unsigned int i = us;
    while(i--) ;
}

/*
**将设定的5个立即值数据写入伺服驱动器
**para1-para5:立即值状态、立即值位置、立即值速度、加速时间、减速时间
*/
void send_to_motor(void)
{
    unsigned char i;

    delay_ms(50);
    WDTCN = 0xA5;
    ES0 = 0;
    buf[0] = 0X01; //站点地址
    buf[1] = 0x10; //写
    buf[2] = 0x51; //寄存器地址高位
    buf[3] = 0x00; //寄存器地址低位
    buf[4] = 0x00; //所写内容高位
    buf[5] = 0x0A; //所写内容低位-5个寄存器
    buf[6] = 0x14;

    //#1 立即值状态
    if(Runmode == 1)
    {
        buf[7] = 0x00;
    }
    else
    {
        buf[7] = 0x01;
    }
    buf[8] = 0x00;
    buf[9] = 0x00;
    buf[10] = 0x00;
    //#2 立即值位置
    buf[11] = Servomotor_displace.buf[0];
    buf[12] = Servomotor_displace.buf[1];
    buf[13] = Servomotor_displace.buf[2];
    buf[14] = Servomotor_displace.buf[3];
    //#3 立即值速度
    buf[15] = Servomotor_speed.buf[0];
    buf[16] = Servomotor_speed.buf[1];
    buf[17] = Servomotor_speed.buf[2];
    buf[18] = Servomotor_speed.buf[3];
    //#4 立即值加速时间
    buf[19] = Servomotor_addtime.buf[0];
    buf[20] = Servomotor_addtime.buf[1];
    buf[21] = Servomotor_addtime.buf[2];
    buf[22] = Servomotor_addtime.buf[3];
    //#5 立即值减速时间
    buf[23] = Servomotor_subtime.buf[0];
    buf[24] = Servomotor_subtime.buf[1];
    buf[25] = Servomotor_subtime.buf[2];
    buf[26] = Servomotor_subtime.buf[3];

    for(i = 0; i < 30; i++)
    {
        Rec1[i] = 0;
    }
    Uart0Send(buf, 27);
    ES0 = 1;
    delay_ms(300);
    if(Receive_command_finished == YES) //暂时不做过多的检查和校验，后期补上，加强安全性
    {
        Receive_command_finished = NO;
    }
}


//-----------------------------------------------------------------------------
// 发送给伺服电机驱动器的通信CONT信号
//command:通信CONT信号-CONT9-CONT24（bit0-byte15）
//通过将CONT9-24绑定相应功能，从而通过modbus控制伺服驱动器
//-----------------------------------------------------------------------------
void Send_to_Motordriver_CTL(unsigned char command )
{
    UINTUNION temp;
    unsigned char i;

    temp.value = command;
    WDTCN = 0xA5;
    buf[0] = 0x01; //站点地址
    buf[1] = 0x10; //写

    buf[2] = 0x00; //寄存器地址高位   针对CONT输入信号地址
    buf[3] = 0x00; //寄存器地址低位

    buf[4] = 0x00; //记录数
    buf[5] = 0x02; //记录数

    buf[6] = 0x04; //字节数

    buf[7] = 0x00; //所写内容最高字节
    buf[8] = 0x00; //所写内容次高字节
    buf[9] = temp.buf[0]; //所写内容次低字节
    buf[10] = temp.buf[1]; //所写内容最低字节
    for(i = 0; i < 30; i++)
    {
        Rec1[i] = 0;
    }
    Uart0Send(buf, 11);
    delay_ms(300);
    if(Receive_command_finished == YES) //暂时不做过多的检查和校验，后期补上，加强安全性
    {
        Receive_command_finished = NO;
    }
}

//
void RunLEDDIS(void)//运行显示
{
    if(T0Counter5 >= 20) //200ms时间定时
    {
        T0Counter5 = 0 ;
        LED2 = !LED2;
    }
}

//直流电机控制
void DCmotorCTRL(void)
{
    char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page

    switch(CANcomand)
    {
        /****************高空输送线直流电机*****************/
    case 0xD6:
        SFRPAGE = 0x0F;
        Motor2 = 0;
        SFRPAGE = SFRPAGE_SAVE;
        Time_FLAG1 = NO;

        CANTRASTEMINFOR[13] = 0;
        CANTRASTEMINFOR[14] = 0;
        Step_FLAG = NO;
        nCANcomand = 0;
        CANcomand = 0x00;
        T0Counter6 = 0;
        T0Counter10 = 0;
        break;

    case 0xD7:
        SFRPAGE = 0x0F;
        Motor2 = 1;
        SFRPAGE = SFRPAGE_SAVE;
        Time_FLAG1 = NO;

        CANTRASTEMINFOR[13] = 0;
        CANTRASTEMINFOR[14] = 0;
        Step_FLAG = NO;
        nCANcomand = 0;
        CANcomand = 0x00;
        T0Counter6 = 0;
        break;
        /*************************************************/

        /****************倾倒步进电机*****************/
    case 0xD8:
        switch(CANINDEX)
        {
        case 0:
            SFRPAGE = 0x0F;
            Motor1_Power = 0;
            Motor1_Dir = 1;
            SFRPAGE = SFRPAGE_SAVE;
            break;

        default:
            break;
        }
        Time_FLAG1 = NO;
        CANTRASTEMINFOR[13] = 0;
        CANTRASTEMINFOR[14] = 0;
        Step_FLAG = NO;
        nCANcomand = 0;
        CANcomand = 0x00;
        T0Counter6 = 0;
        break;

    case 0xD9:
        Motor1_Power = 0;
        Motor1_Dir = 0;
        nCANcomand = 0;
        CANcomand = 0x00;
        break;
        /*************************************************/

    default:
        break;
    }
}

//步进电机控制
/*根据实际接线判断：滚筒步进电机没有使用T2输出信号，
而是由Motor1_Power输出信号直接控制中间继电器通断电实现转动，
要实现反转还需要增加一个中间继电器和一个数字信号输出（电机驱动板）*/
void SteppermotorCTRL(void)
{
    switch(CANcomand)
    {
    case 0xD5:
        Shaft1_aim_pulse = Servopara[CANINDEX * 9 + 80];
        Shaft1_aim_pulse = Shaft1_aim_pulse << 8;
        Shaft1_aim_pulse = Shaft1_aim_pulse + Servopara[CANINDEX * 9 + 81];
        Shaft1_aim_pulse = Shaft1_aim_pulse << 8;
        Shaft1_aim_pulse = Shaft1_aim_pulse + Servopara[CANINDEX * 9 + 82];
        Shaft1_aim_pulse = Shaft1_aim_pulse << 8;;
        Shaft1_aim_pulse = Shaft1_aim_pulse + Servopara[CANINDEX * 9 + 83];
        Shaft1_Speed_max = Servopara[CANINDEX * 9 + 84];
        Shaft1_Speed_max = Shaft1_Speed_max << 8;
        Shaft1_Speed_max = Shaft1_Speed_max + Servopara[CANINDEX * 9 + 85];
        Shaft1_Add_Speed = Servopara[CANINDEX * 9 + 86];
        Shaft1_Add_Speed = Shaft1_Add_Speed << 8;
        Shaft1_Add_Speed = Shaft1_Add_Speed + Servopara[CANINDEX * 9 + 87];
        Shaft1_ADD_Speed_Time_Base = Servopara[CANINDEX * 9 + 88];
        switch(CANINDEX)
        {
        case 0:
            Shaft1_Run_mode = 0x01;
            break;
        case 1:
            Shaft1_Run_mode = 0x02;
            break;
        case 2:
            Shaft1_Run_mode = 0x03;
            break;
        default:
            Shaft1_Run_mode = 0x00;
            break;
        }
        Time_FLAG1 = NO;
        Pulse1_Cnt = 0;
        motor1_out = 0;
        T2RUN;

        CANTRASTEMINFOR[13] = 0;
        CANTRASTEMINFOR[14] = 0;
        Step_FLAG = NO;
        nCANcomand = 0;
        CANcomand = 0x00;
        T0Counter6 = 0;
        T0Counter4 = 100;
        break;

    case 0x22:
        T2STOP;
        Shaft1_aim_pulse = 0;
        Pulse1_Cnt = 0;
        motor1_out = 0;
        T0Counter4 = 0;
        Shaft1_Run_mode = 0x00;
        nCANcomand = 0;
        CANcomand = 0x00;
        break;

    default:
        break;
    }
    Shaft1_CTL();    //1号电机

}
/*******************************************************************************************************
//伺服电机运行函数包括报警量采集
/*******************************************************************************************************/
void ServomotorCRTL(unsigned char comand)//伺服电机控制
{
    unsigned char CALU_m_pra = 0;

    CALU_m_pra = CANINDEX * 16;
    switch(comand)
    {
    case 0xD0:
        TR0  = 0;
        WDTCN = 0xA5;
        Send_to_Motordriver_CTL(0x01);
        Servomotor_displace.displace = Servoparadisplace;
        Servomotor_speed.speed = Servoparaspeed;
        Servomotor_addtime.addtime = 1000;
        Servomotor_subtime.subtime = 1000;
        send_to_motor();
        Send_to_Motordriver_CTL(0x03);
        Send_to_Motordriver_CTL(0x01);
        CANcomand = 0x00;
        nCANcomand = 0;
        TR0  = 1;
        break;

    case 0xD2:
        TR0  = 0;
        WDTCN = 0xA5;
        Send_to_Motordriver_CTL(0x00);
        nCANcomand = 0;
        CANcomand = 0x00;
        TR0  = 1;
        break;

    case 0x11:
        TR0  = 0;
        WDTCN = 0xA5;
        Send_to_Motordriver_CTL(0x01);
        nCANcomand = 0;
        CANcomand = 0x00;
        TR0  = 1;
        break;

    case 0xD3:
        TR0  = 0;
        WDTCN = 0xA5;
        Receive485_command_finished = NO;
        //
        Servomotor_displace.buf[0] = Servopara[CALU_m_pra];
        Servomotor_displace.buf[1] = Servopara[CALU_m_pra + 1];
        Servomotor_displace.buf[2] = Servopara[CALU_m_pra + 2];
        Servomotor_displace.buf[3] = Servopara[CALU_m_pra + 3];
        //
        Servomotor_speed.buf[0] = Servopara[CALU_m_pra + 4];
        Servomotor_speed.buf[1] = Servopara[CALU_m_pra + 5];
        Servomotor_speed.buf[2] = Servopara[CALU_m_pra + 6];
        Servomotor_speed.buf[3] = Servopara[CALU_m_pra + 7];
        //
        Servomotor_addtime.buf[0] = Servopara[CALU_m_pra + 8];
        Servomotor_addtime.buf[1] = Servopara[CALU_m_pra + 9];
        Servomotor_addtime.buf[2] = Servopara[CALU_m_pra + 10];
        Servomotor_addtime.buf[3] = Servopara[CALU_m_pra + 11];
        //
        Servomotor_subtime.buf[0] = Servopara[CALU_m_pra + 12];
        Servomotor_subtime.buf[1] = Servopara[CALU_m_pra + 13];
        Servomotor_subtime.buf[2] = Servopara[CALU_m_pra + 14];
        Servomotor_subtime.buf[3] = Servopara[CALU_m_pra + 15];
        //
        CANTRASTEMINFOR[13] = 0;
        CANTRASTEMINFOR[14] = 0;
        Send_to_Motordriver_CTL(0x01);
        send_to_motor();
        Send_to_Motordriver_CTL(0x03);
        Send_to_Motordriver_CTL(0x01);
        T0Counter6 = 0;
        CANcomand = 0x00;
        nCANcomand = 0;
        Time_FLAG = NO;
        CANTRASTEMINFOR[8] = CANTRASTEMINFOR[8] | 0X01; //置运行标志
        Receive485_command_finished = YES;
        TR0  = 1;
        break;

    case 0xD4:
        TR0  = 0;
        WDTCN = 0xA5;
        Returntopoint();
        CANcomand = 0x00;
        nCANcomand = 0;
        TR0  = 1;
        break;

    default:
        break;
    }
//上传命令反馈数据
}
/*******************************************************************************************************
//自动运行
/*******************************************************************************************************/
void AUTORUNMODE0(unsigned char comand)
{
    char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page

    switch(comand)
    {
    case 0xE0:
        Returntopoint();
        break;

    case 0xE1:
        /*if((CANTRASTEMINFOR[8]&0x08)==0x00)
        {
        SFRPAGE = 0x0F;
        if(Motor1_Power==0)
        {
        Motor1_Power=1;
        delay_ms(1000);
        Motor1_B=0;
        T0Counter6=0;
        }
        else
        {
        Motor1_Power=1;
        Motor1_B=0;
        T0Counter6=0;
        }

        }*/
        WDTCN = 0xA5;
        TR0  = 0;
        Send_to_Motordriver_CTL(0x00);//关闭伺服
        TR0  = 1;
        break;

    default:
        break;
    }
    SFRPAGE = SFRPAGE_SAVE;
}
/*******************************************************************************************************
//自动运行
/*******************************************************************************************************/

void AUTORUNMODE1(unsigned char comand)
{
    unsigned char CALU_m_pra = 0;
    char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page

    CALU_m_pra = CANINDEX * 16;
    if(CANINDEX >= 4) {
        CANINDEX = 4;
    }
    switch(comand)
    {
    case 0xF0:	//
        TR0  = 0;
        WDTCN = 0xA5;
        /*SFRPAGE = 0x0F;
        Motor2=0;
        SFRPAGE = SFRPAGE_SAVE;*/

        //开启提升伺服
        Receive485_command_finished = NO;
			  Servomotor_displace.displace = 6000;
        Servomotor_speed.speed = 40000;
        Servomotor_addtime.addtime = 200;
        Servomotor_subtime.subtime = 200;
		
//        //
//        Servomotor_displace.buf[0] = Servopara[CALU_m_pra];
//        Servomotor_displace.buf[1] = Servopara[CALU_m_pra + 1];
//        Servomotor_displace.buf[2] = Servopara[CALU_m_pra + 2];
//        Servomotor_displace.buf[3] = Servopara[CALU_m_pra + 3];
//        if(Servomotor_displace.displace > Max_displace)
//        {
//            Servomotor_displace.displace = Max_displace;
//        }
//        //
//        Servomotor_speed.buf[0] = Servopara[CALU_m_pra + 4];
//        Servomotor_speed.buf[1] = Servopara[CALU_m_pra + 5];
//        Servomotor_speed.buf[2] = Servopara[CALU_m_pra + 6];
//        Servomotor_speed.buf[3] = Servopara[CALU_m_pra + 7];
//        if(Servomotor_speed.speed > Max_speedvalue)
//        {
//            Servomotor_speed.speed = Max_speedvalue;
//        }
//        //
//        Servomotor_addtime.buf[0] = Servopara[CALU_m_pra + 8];
//        Servomotor_addtime.buf[1] = Servopara[CALU_m_pra + 9];
//        Servomotor_addtime.buf[2] = Servopara[CALU_m_pra + 10];
//        Servomotor_addtime.buf[3] = Servopara[CALU_m_pra + 11];
//        //
//        Servomotor_subtime.buf[0] = Servopara[CALU_m_pra + 12];
//        Servomotor_subtime.buf[1] = Servopara[CALU_m_pra + 13];
//        Servomotor_subtime.buf[2] = Servopara[CALU_m_pra + 14];
//        Servomotor_subtime.buf[3] = Servopara[CALU_m_pra + 15];
        //
        CANTRASTEMINFOR[13] = 0;
        CANTRASTEMINFOR[14] = 0;
        Send_to_Motordriver_CTL(0x01);//伺服S-ON
        send_to_motor();//设定立即值数据
        Send_to_Motordriver_CTL(0x03);//伺服S-ON并自动启动伺服
        Send_to_Motordriver_CTL(0x01);//伺服S-ON
        T0Counter6 = 0;
        Time_FLAG = NO;
        Servomotor_original = 0;	//原点位置标志清零
        CANTRASTEMINFOR[8] = CANTRASTEMINFOR[8] | 0X01;	//置运行标志
        Receive485_command_finished = YES;	//485数据接收完成
        TR0  = 1;
        break;

    case 0xF1:		//开启提升输送线和高空输送线
        WDTCN = 0xA5;
        SFRPAGE = 0x0F;
        Motor1_Power = 0;	//供电
        Dir_ZZ();	//正转
//        Motor2 = 0;	//启动高空输送线
        /*if(Motor1_B==0)
        {
        	Motor1_B=1;
        	delay_ms(1000);
        	Motor1_Power=0;
        }
        else
        {
        	Motor1_B=1;
        	Motor1_Power=0;
        }*/
        T0Counter10 = 0; //高空输送线定时器
        GAOKONG_COUT = 0; //高空输送线计数器
        SFRPAGE = SFRPAGE_SAVE;
        break;

    case 0xF2://回归药品倾倒、回归伺服
        WDTCN = 0xA5;
        SFRPAGE = 0x0F;
        Motor1_Power = 1;	//断电
        Dir_ZZ();	//正转
        /*if(Motor1_Power==0)
        {
        Motor1_Power=1;
        delay_ms(1000);
        Motor1_B=0;
        T0Counter6=0;
        }
        else
        {
        Motor1_Power=1;
        Motor1_B=0;
        T0Counter6=0;
        }*/
        TR0  = 0;
        SFRPAGE = SFRPAGE_SAVE;
        Receive485_command_finished = NO;
        Servomotor_displace.displace = Zero_backdisplace;//设置原点位置
        CANTRASTEMINFOR[13] = 0;
        CANTRASTEMINFOR[14] = 0;
        Send_to_Motordriver_CTL(0x01);//伺服S-ON
        send_to_motor();	//设定立即值
        Send_to_Motordriver_CTL(0x03);//伺服自动启动
        Send_to_Motordriver_CTL(0x01);//伺服S-ON
        T0Counter6 = 0;
        Time_FLAG = NO;
        CANTRASTEMINFOR[8] = CANTRASTEMINFOR[8] | 0X01; //置运行标志
        Receive485_command_finished = YES;
        TR0  = 1;
        break;

    case 0xF3:	//关闭伺服
        WDTCN = 0xA5;
        TR0  = 0;
        Send_to_Motordriver_CTL(0x00);
        TR0  = 1;
        break;

    case 0xF4:	//停止高空输送线
        WDTCN = 0xA5;
        SFRPAGE = 0x0F;  //关闭高空输送线
//        Motor2 = 1;
        SFRPAGE = SFRPAGE_SAVE;
        break;

    case 0xF5:	//回原点
        Returntopoint();
        break;

        //added begin
    case 0xF6:		//倾倒输送线反转
        WDTCN = 0xA5;
        SFRPAGE = 0x0F;
        Motor1_Power = 0;	//启动翻转输送线
        Dir_FZ();	//反转
        SFRPAGE = SFRPAGE_SAVE;
        break;

    case 0xF7:		//倾倒输送线反转
        WDTCN = 0xA5;
        SFRPAGE = 0x0F;
        Motor1_Power = 1;	//停止翻转输送线
        Dir_ZZ();	//反转
        SFRPAGE = SFRPAGE_SAVE;
        break;
        //added end

    default:
        break;
    }
    //Shaft1_CTL( Shaft1_Run_mode,Shaft1_aim_pulse);    //1号电机
    SFRPAGE = SFRPAGE_SAVE;
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

//伺服回原点
void Returntopoint(void)
{
    TR0  = 0;
    WDTCN = 0xA5;
//    Send_to_Motordriver_CTL(0x01);//伺服S-ON
    Send_to_Motordriver_CTL(0x05);//原点复归
//    Send_to_Motordriver_CTL(0x01);//伺服S-ON
    Time_FLAG = NO;
    CANTRASTEMINFOR[8] = CANTRASTEMINFOR[8] | 0X40; //置运行标志
    TR0  = 1;
}

//伺服电机数据采集
void ServomotorDataacquisition(unsigned char addressID)
{
    //if(ENABFLAG==YES)
    //{
    unsigned char i;

    if(Time_FLAG == YES) return;
    delay_ms(50);
    buf[0] = addressID; //站点地址
    buf[1] = 0x03; //读
    buf[2] = 0x10; //寄存器地址高位
    buf[3] = 0x03; //寄存器地址低位
    buf[4] = 0x00; //所写内容高位
    buf[5] = 0x06; //所写内容低位
    Uart0Send(buf, 6);
    AUTO_FLAG = YES;
    for(i = 0; i < 30; i++)
    {
        Rec1[i] = 0;
    }
    delay_ms(300);
    if(Receive_command_finished == YES)
    {
        //以后总传
        Receive_command_finished = NO;
        CANTRASTEMINFOR[0] = Rec1[7];
        CANTRASTEMINFOR[1] = Rec1[8];
        CANTRASTEMINFOR[2] = Rec1[9];
        CANTRASTEMINFOR[3] = Rec1[10];
        CANTRASTEMINFOR[4] = Rec1[11];
        CANTRASTEMINFOR[5] = Rec1[12];
        CANTRASTEMINFOR[6] = Rec1[13];
        CANTRASTEMINFOR[7] = Rec1[14];

    }
    AUTO_FLAG = NO;
}

//伺服电机电流采集
void CurrentCollection(void)
{
    //if(ENABFLAG==YES)
    //{
    unsigned char i;

    if(Time_FLAG1 == YES) return;
    delay_ms(50);
    buf[0] = 2; //站点地址
    buf[1] = 0x03; //读
    buf[2] = 0x00;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x08;
    for(i = 0; i < 30; i++)
    {
        Rec1[i] = 0;
    }
    Uart0Send(buf, 6);
    AUTO_FLAG = YES;
    delay_ms(300);
    if(Receive_command_finished == YES)
    {
        //以后总传
        Receive_command_finished = NO;
        CANTRASTEMINFOR[9] = Rec1[3];
        CANTRASTEMINFOR[10] = Rec1[4];
        CANTRASTEMINFOR[11] = Rec1[5];
        CANTRASTEMINFOR[12] = Rec1[6];
    }
    AUTO_FLAG = NO;;

    //}
}

//数据采集
void Dataacquisition()
{
    //没有处理命令时，可以执行数据采集任务
    if((CANcomand == 0 && nCANcomand == 0) || (nCANcomand == 0xFE) || (CANcomand == 0xFE))
    {
        //if(T0Counter3>=20)  //200ms时间定时
        //{
        //T0Counter3=0;
        ServomotorINP(0);
        switch(Sampleperiod)
        {
        case 0:
            ServomotorDataacquisition(1);//伺服电机数据采集
            break;

        case 1:
            CurrentCollection();//伺服电机电流采集
            break;

        default:
            break;
        }
        //采集对象循环交替
        Sampleperiod = Sampleperiod + 1;
        if(Sampleperiod >= 2)
        {
            Sampleperiod = 0;
        }

        //}
    }
}


//-----------------------------------------------------------------------------
//从伺服电机驱动器查询通信OUT信号
//get_value:通信OUT信号-OUT6-CONT21（bit0-byte15）
//通过将out6-21绑定相应功能，从而通过modbus获取伺服驱动器状态
//伺服电机定位运行结束(OUT6)、原点复归结束信号(OUT7)查询函数
//-----------------------------------------------------------------------------
void ServomotorINP(unsigned char mode)
{
    //unsigned int ShowTime1;
    unsigned char i;

    //if(ENABFLAG==YES)
    //{
	//暂时屏蔽
	if(mode==0)
	{
    if(Time_FLAG == YES) return;
	}
    WDTCN = 0xA5;
    delay_ms(50);
    buf[0] = 0x01; //站点地址
    buf[1] = 0x03; //读
    buf[2] = 0x01; //寄存器地址高位
    buf[3] = 0x00; //寄存器地址低位
    buf[4] = 0x00; //所写内容高位
    buf[5] = 0x02; //所写内容低位
    for(i = 0; i < 30; i++)
    {
        Rec1[i] = 0;
    }
    Uart0Send(buf, 6);
    AUTO_FLAG = YES;
    delay_ms(300);
    if(Receive_command_finished == YES)
    {
        Receive_command_finished = NO;
        ShowTime1 = T0Counter6 * 10;
        if((Rec1[6] & 0x01) == 0x01)//定位结束
        {
						if(mode==0)Time_FLAG = YES;
            CANTRASTEMINFOR[13] = (unsigned char)((ShowTime1 & 0xFF00) >> 8);
            CANTRASTEMINFOR[14] = (unsigned char)(ShowTime1 & 0x00FF);
            CANTRASTEMINFOR[8] = CANTRASTEMINFOR[8] & 0XFE; //清除运行标志
        }
        if((Rec1[6] & 0x02) == 0x02)//归位结束
        {
						if(mode==1)Time_FLAG = YES;
            CANTRASTEMINFOR[8] = CANTRASTEMINFOR[8] & 0XBF; //清除运行标志
        }
    }
    AUTO_FLAG = NO;
}

//系统控制
/*系统有手动运行模式和自动运行模式两种Runmode；
每种模式再根据CANcomand指令执行响应动作
CANINDEX作为辅助参数使用
*/
void SystemControl(void)
{
    switch(Runmode)
    {
    case 0:
        if(CANcomand == 0)
        {
            CANcomand = nCANcomand;
        }
        SteppermotorCTRL();  //步进电机控制
        DCmotorCTRL();       //直流电机控制
        if(Receive485_command_finished == YES) //防止连续发控制命令
        {
            ServomotorCRTL(CANcomand);//伺服电机控制
        }
        break;

    case 1:
        if(CANcomand == 0)
        {
            CANcomand = nCANcomand;
        }
        /****************************/
        //检查命令的合法性
        /****************************/
        else if(CANcomand == 0xFE)
        {
            AUTOCANcomand = AUTOTaskassignment1(); //自动运行任务安排
            AUTORUNMODE1(AUTOCANcomand);
        }
        else if(CANcomand == 0xFB)
        {
            AUTOCANcomand = AUTOTaskassignment0(); //原点复归
            AUTORUNMODE0(AUTOCANcomand);
        }
        //ADDED BEGIN
        else if(CANcomand == 0xB0)
        {
            AUTOCANcomand = AUTOTaskassignment1(); //
            AUTORUNMODE1(AUTOCANcomand);
        }
        //ADDED END
        else {}
        break;

    default:
        break;

    }
}

#define		QDM_FZ_RUNTIME		300	//单位：10ms
/********************************************************
////自动运行任务安排
********************************************************/
unsigned char AUTOTaskassignment1(void)
{
    unsigned char returnvalue = 0;
    char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page

    switch(system_crtl.AUTOsystem_command)//做到每步运行检查判断给出命令
    {
    case 1:
        //检查命令执行可行性
        returnvalue = 0xF0; //开启提升伺服
        CANTRASTEMINFOR[8] = CANTRASTEMINFOR[8] | 0X01; //置运行标志
        system_crtl.AUTOsystem_command = 2;
        break;
    case 2:
        //检查命令执行可行性
        //倾倒药品
        ServomotorINP(0);//查询伺服是否运行结束
        //	ServomotorDataacquisition(1);//伺服电机数据采集
        if(((CANTRASTEMINFOR[8] & 0X01) == 0x00)) //伺服提升结束
        {
            returnvalue = 0xF1; //直流电机运行倾倒药品 开启高空输送线
            T0Counter9 = 0;  //倾倒定时
            YAOPINGTISH_COUT = 0; //计数清0
            system_crtl.AUTOsystem_command = 4;
        }
        break;

    case 4:
        if(T0Counter9 > 200) //计时结束
        {
            if((CANTRASTEMINFOR[8] & 0x08) == 0x08) //提升输送线传感器检测
            {
                YAOPINGTISH_COUT = YAOPINGTISH_COUT + 1;
                if(YAOPINGTISH_COUT < 3)
                {
                    T0Counter9 = 0;  //倾倒定时 如果NPN低电平计时重来
                }
                else
                {
                    SFRPAGE = 0x0F;
                    Motor1_Power = 1;
										Dir_ZZ();	//正转
                    SFRPAGE = SFRPAGE_SAVE;
                    system_crtl.AUTOsystem_command = 7;
                }
            }
            else
            {
                returnvalue = 0xF2; //倾倒电机停机伺服回归
                CANTRASTEMINFOR[8] = CANTRASTEMINFOR[8] | 0X01; //置运行标志
                system_crtl.AUTOsystem_command = 5;
            }
        }
        break;
    case 5:
        ServomotorINP(0);//查询伺服是否运行结束
        //   ServomotorDataacquisition(1);//伺服电机数据采集
        if((CANTRASTEMINFOR[8] & 0X01) == 0x00)
        {
            returnvalue = 0xF5; //归零
            // T0Counter9=0;//启动延时停机
            system_crtl.AUTOsystem_command = 6; //
        }
        break;

    case 6:
        ServomotorINP(1);//查询伺服是否运行结束
        if((CANTRASTEMINFOR[8] & 0x40) == 0x00) //原点复归结束
        {
            Servomotor_original = 1; //伺服原点标志建立
            returnvalue = 0xF3; //关闭伺服
            system_crtl.AUTOsystem_command = 8; //
        }
        break;
    case 7:
        returnvalue = 0xF3; //关闭伺服
        system_crtl.AUTOsystem_command = 8;
        break;
    case 8:
        // T0Counter10=0;  //高空输送线定时器
        //  GAOKONG_COUT=0; //高空输送线计数器
        system_crtl.AUTOsystem_command = 0;
        CANcomand = 0x00;
        nCANcomand = 0x00;
        CANINDEX = 0;
        //nCANDate1=0;
        //自动运行结束上报信息参数

        AUTOCMD_STATE = 0XC2;
        T0Counter8 = 51;
        AUTOCMD_TIME = 0;
        break;

        //added begin
    case 9:	//倾倒输送线反转，在正转时被关闭，无运行时间设定
        switch(CANINDEX)
        {
        case 1:
            returnvalue = 0xF6;//提升输送线反转
            break;

        case 2:
            returnvalue = 0xF7;//提升输送线停止
            break;

        default:
            break;
        }
        system_crtl.AUTOsystem_command = 0;
        break;
        //added end

    default:
        break;
    }

    return  returnvalue;
}
/********************************************************
////自动运行任务安排模块系统复位-伺服回原点
********************************************************/
unsigned char AUTOTaskassignment0(void)
{
    unsigned char returnvalue = 0;

    switch(system_crtl.AUTOsystem_command)//做到每步运行检查判断给出命令
    {
    case 1:
        returnvalue = 0xE0; //伺服原点复归
        system_crtl.AUTOsystem_command = 2;
        break;

    case 2:
        ServomotorINP(1);//查询伺服是否运行结束
        if((CANTRASTEMINFOR[8] & 0x40) == 0x00) //原点复归结束
        {
            Servomotor_original = 1; //伺服原点标志建立
            returnvalue = 0xE1; //关闭使能，复归倾倒电机，复归步进电机
            system_crtl.AUTOsystem_command = 3;
        }
        break;

    case 3:
        //if(((CANTRASTEMINFOR[8]&0x08)==0x08)&&((CANTRASTEMINFOR[8]&0x20)==0x20))
        //if((CANTRASTEMINFOR[8]&0x08)==0x08)
        //{
        system_crtl.AUTOsystem_command = 0;
        CANcomand = 0x00;
        nCANcomand = 0;
        CANINDEX = 0;
        //nCANDate1=0;
        //自动运行结束上报信息参数
        AUTOCMD_STATE = 0XC3;
        T0Counter8 = 21;
        AUTOCMD_TIME = 0;
        //}
        break;

    default:
        break;
    }
    return returnvalue;
}

//
void CANsetpraback(void)
{
    unsigned char i;
    unsigned char sum;

    switch(AUTOCMD_CHECK)
    {
    case 0xFC:
        CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
        CANTXBUF_ZKB.normal_buf.index = CANINDEX;
        CANTXBUF_ZKB.normal_buf.command = AUTOCMD_CHECK;
        CANTXBUF_ZKB.normal_buf.data1 = CANTRASTEMINFOR[8]; //伺服状态传感器状态
        CANTXBUF_ZKB.normal_buf.data2 = Servomotor_original;
        CANTXBUF_ZKB.normal_buf.data3 = system_crtl.AUTOsystem_command; //模块运行情况
        CANTXBUF_ZKB.normal_buf.data4 = system_crtl.AUTOsystem_alarm1; //运行状态报警
        break;

    default:
        CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
        CANTXBUF_ZKB.normal_buf.index = CANINDEX;
        CANTXBUF_ZKB.normal_buf.command = CANcomand;
        CANTXBUF_ZKB.normal_buf.data1 = Runmode;
        CANTXBUF_ZKB.normal_buf.data2 = 0X22;
        CANTXBUF_ZKB.normal_buf.data3 = 0X33;
        CANTXBUF_ZKB.normal_buf.data4 = 0;
        break;
    }
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);
}

//
void CANsteminforback(void)
{
    unsigned char i;
    unsigned char sum;
    unsigned char mj;
    if(T0Counter3 >= 50) //200ms时间定时
    {
        T0Counter3 = 0;
        for(mj = 0; mj < 5; mj++)
        {
            CANTXBUF_ZKB.normal_buf.address1 = 0X01;
            CANTXBUF_ZKB.normal_buf.index = mj;
            CANTXBUF_ZKB.normal_buf.command = 0XC1;
            CANTXBUF_ZKB.normal_buf.data1 = CANTRASTEMINFOR[4 * mj + 0];
            CANTXBUF_ZKB.normal_buf.data2 = CANTRASTEMINFOR[4 * mj + 1];
            CANTXBUF_ZKB.normal_buf.data3 = CANTRASTEMINFOR[4 * mj + 2];
            CANTXBUF_ZKB.normal_buf.data4 = CANTRASTEMINFOR[4 * mj + 3];
            sum = 0;
            i = 0;
            do
            {
                sum += CANTXBUF_ZKB.buf[i];
                i++;
            } while(i < 7);
            CANTXBUF_ZKB.normal_buf.checkout = sum;
            can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);
        }
        LED3 = !LED3;
    }
}

//主动上传信息
void ActiveCANtransfer(void)//CAN参数主动上传
{
    unsigned char i = 0;
    unsigned char sum = 0;
    if(repair_flag == 0x00) return;
    if(T0Counter8 > 50)
    {
        T0Counter8 = 0;
        //if(AUTOCMD_STATE==0Xc2||AUTOCMD_STATE==0Xc3||AUTOCMD_STATE==0Xc4)
        if(AUTOCMD_STATE != 0)
        {
            TR0  = 0;
            WDTCN = 0xA5;
            CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
            CANTXBUF_ZKB.normal_buf.index = CANINDEX;
            CANTXBUF_ZKB.normal_buf.command = AUTOCMD_STATE;
            CANTXBUF_ZKB.normal_buf.data1 = CANTRASTEMINFOR[8]; //伺服状态传感器状态
            CANTXBUF_ZKB.normal_buf.data2 = Servomotor_original;
            CANTXBUF_ZKB.normal_buf.data3 = system_crtl.AUTOsystem_command; //模块运行情况
            CANTXBUF_ZKB.normal_buf.data4 = system_crtl.AUTOsystem_alarm1; //运行状态报警
            sum = 0;
            i = 0;
            do
            {
                sum = sum + CANTXBUF_ZKB.buf[i];
                i++;
            } while(i < 7);
            CANTXBUF_ZKB.normal_buf.checkout = sum;
            //EIE2 &= 0xDF;
            can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);
            //EIE2 |= 0x20;
            AUTOCMD_TIME = AUTOCMD_TIME + 1;
            //if(AUTOCMD_TIME>=3)
            //{
            AUTOCMD_STATE = 0;
            AUTOCMD_TIME = 0;
            //}
            TR0  = 1;

        }

    }
}

//非主动上传信息
void UnactiveCANtransfer(void)//CAN参数应答上传
{
    switch(CANcomand)
    {

    case 0xA0:
        CANsetpraback();
        CANcomand = 0;
        nCANcomand = 0;
        CANINFOR_FLAG = NO;
        break;
    case 0xA1:
        CANsetpraback();
        CANcomand = 0;
        nCANcomand = 0;
        CANINFOR_FLAG = NO;
        break;

    case 0xA4:
        CANsetpraback();
        CANcomand = 0;
        nCANcomand = 0;
        CANINFOR_FLAG = NO;
        break;
    case 0xA5:
        CANsetpraback();
        CANcomand = 0;
        nCANcomand = 0;
        CANINFOR_FLAG = NO;
        break;
    case 0xA6:
        CANsetpraback();
        CANcomand = 0;
        nCANcomand = 0;
        CANINFOR_FLAG = NO;
        break;
    case 0xA7:
        CANsetpraback();
        CANcomand = 0;
        nCANcomand = 0;
        CANINFOR_FLAG = NO;
        break;
    case 0xA8:
        CANsetpraback();
        CANcomand = 0;
        nCANcomand = 0;
        CANINFOR_FLAG = NO;
        break;
    case 0xA9:
        CANsetpraback();
        CANcomand = 0;
        nCANcomand = 0;
        CANINFOR_FLAG = NO;
        break;
    case 0xAA:
        CANsetpraback();
        CANcomand = 0;
        nCANcomand = 0;
        CANINFOR_FLAG = NO;
        break;
    case 0xAB:
        CANsetpraback();
        CANcomand = 0;
        nCANcomand = 0;
        CANINFOR_FLAG = NO;
        break;
    case 0xAC:
        CANsetpraback();
        CANcomand = 0;
        nCANcomand = 0;
        CANINFOR_FLAG = NO;
        break;
    case 0xC0://读取所有工位信息
        TRANFORSATATIONINFOR();
        CANcomand = 0;
        nCANcomand = 0;
        break;
    default:
        break;
    }
    switch(AUTOCMD_CHECK)
    {
    case 1:
        CANsetpraback();
        AUTOCMD_CHECK = 0;

        break;
    case 0xFC://自动模式查询
        if(repair_flag != 0x00) //如果等于零 查询模式不返回信息
        {
            CANsetpraback();
        }
        AUTOCMD_CHECK = 0;
        //CANcomand=0;
        //nCANcomand=0;
        CANINFOR_FLAG = NO;
        break;
    default:
        break;
    }
}

//
void TRANFORSATATIONINFOR(void)//读取所有工位信息
{
    unsigned char i;
    unsigned char sum;
    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 0;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[0];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[1];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[2];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[3];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);

    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 1;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[4];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[5];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[6];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[7];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);
    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 2;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[8];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[9];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[10];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[11];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);
    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 3;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[12];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[13];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[14];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[15];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);





    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 4;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[16];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[17];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[18];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[19];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);

    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 5;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[20];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[21];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[22];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[23];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);
    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 6;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[24];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[25];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[26];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[27];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);
    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 7;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[28];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[29];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[30];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[31];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);



    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 8;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[32];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[33];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[34];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[35];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);

    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 9;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[36];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[37];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[38];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[39];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);
    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 10;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[40];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[41];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[42];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[43];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);
    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 11;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[44];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[45];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[46];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[47];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);

    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 12;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[48];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[49];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[50];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[51];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);

    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 13;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[52];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[53];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[54];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[55];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);
    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 14;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[56];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[57];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[58];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[59];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);
    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 15;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[60];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[61];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[62];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[63];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);

    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 16;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[64];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[65];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[66];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[67];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);

    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 17;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[68];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[69];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[70];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[71];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);
    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 18;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[72];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[73];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[74];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[75];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);
    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 19;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[76];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[77];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[78];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[79];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);

    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 20;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[80];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[81];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[82];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[83];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);

    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 21;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[84];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[85];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[86];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[87];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);

    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 22;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[88];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[89];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[90];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[91];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);
    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 23;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[92];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[93];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[94];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[95];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);
    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 24;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[96];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[97];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[98];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[99];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);
    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 25;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[100];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[101];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[102];
    CANTXBUF_ZKB.normal_buf.data4 = Servopara[103];
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);
    CANTXBUF_ZKB.normal_buf.address1 = CANaddress1;
    CANTXBUF_ZKB.normal_buf.index = 26;
    CANTXBUF_ZKB.normal_buf.command = CANcomand;
    CANTXBUF_ZKB.normal_buf.data1 = Servopara[104];
    CANTXBUF_ZKB.normal_buf.data2 = Servopara[105];
    CANTXBUF_ZKB.normal_buf.data3 = Servopara[106];
    CANTXBUF_ZKB.normal_buf.data4 = 0;
    sum = 0;
    i = 0;
    do
    {
        sum += CANTXBUF_ZKB.buf[i];
        i++;
    } while(i < 7);
    CANTXBUF_ZKB.normal_buf.checkout = sum;
    can1_transmit(TX_MSGNUM_ZKB, CANTXBUF_ZKB.buf);
}
//////////////////////////
//将定义的Flash缓冲区写入Flash
//////////////////////////
void write_to_flash(void)
{
    unsigned char xdata *pwrite ;
    unsigned char i;
    char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page
    EA = 0; //关中断
    SFRPAGE = LEGACY_PAGE;
    FLSCL = FLSCL | 0x01; //允许写擦除flash
    PSCTL = PSCTL | 0x07; //允许写擦除flash
    pwrite = 0x00;
    *pwrite = 0;  //擦除flash
    PSCTL = PSCTL & 0xFD; //禁止擦除
    i = 0;
    do {
        *pwrite++ = Servopara[i];
        i++;
    } while(i < 107);
    PSCTL = PSCTL & 0xFA; //禁止flash写
    FLSCL = FLSCL & 0xFE; //禁止写擦除flash
    EA = 1; //开中断
    SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
}

//////////////////////////
//将Flash中保存的数据读入Flash缓冲区
//////////////////////////
void read_from_flash(void)
{
    unsigned char code *pread ;
    unsigned char i;
    char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page
    EA = 0;
    SFRPAGE = LEGACY_PAGE;
    PSCTL = PSCTL | 0x04; //指向flash地址0～0x7f
    pread = 0x00;
    i = 0;
    do {
        Servopara[i] = *pread++;
        i++;
    } while(i < 107);
    PSCTL = PSCTL & 0xFB; //恢复 64k flash
    EA = 1; // 允许全局中断
    SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
}


/*void Deviationremoval(void)//偏差清除
{
Send_to_Motordriver_CTL(0x01);//保持S-ON
Send_to_Motordriver_CTL(0x09);//
Send_to_Motordriver_CTL(0x01);//
Time_FLAG=NO;

}*/
void init_machine(void)
{
    CANaddress1 = 0x01;
    nCANcomand = 0xFB;
    CANINDEX = 0X00;
    system_crtl.AUTOsystem_command = 1;			//AUTOCMD_CHECK=1;
    T0Counter3 = 51; //尽快的定时发送清除状态类表
    Runmode = 1; //自动运行
}
//////////////////////////
//         主程序
//////////////////////////
unsigned char buffer[8];
void main (void)
{
    SFRPAGE = CONFIG_PAGE;
    Initial();
    delay_ms(1000);
    init_machine();//上电系统复位
//	Send_to_Motordriver_CTL(0x00);
    while(1)
    {

        SystemControl();              //系统控制（控制信息来源，can\RS485\中断信号）
        UnactiveCANtransfer();        //CAN参数应答上传
        ActiveCANtransfer();          //CAN参数主动上传

			
        //Dataacquisition();            //数据采集
        /*if((CANINFOR_FLAG==NO)&&(Runmode==0x00))//CAN主动连续上传
        {
        	//EIE2 &= 0xDF;
        	CANsteminforback();            //系统信息上传
        	//EIE2 |= 0x20;
        }*/
        RunLEDDIS();                  //运行显示
    }
}




























