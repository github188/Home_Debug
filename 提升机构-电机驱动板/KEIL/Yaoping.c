//7.28�������ذ汾����Ҫ �޸��˸߿������ߵ�����ʱ�䣬�Լ����������ļ���λ�á�

#include <c8051f040.h>                 // SFR declaration
#include <math.h>
#include "Yaoping.h"
#include "CAN1.h"
#define T2RUN  temppage= SFRPAGE; SFRPAGE=0X00;TR2=1;SFRPAGE=temppage;	//����ʱ������
#define T2STOP temppage= SFRPAGE; SFRPAGE=0X00;TR2=0;SFRPAGE=temppage		//��ֹ��ʱ������

bit volatile Receive_command_finished = NO;
bit Receive485_command_finished = YES;
bit Time_FLAG = YES;
bit Time_FLAG1 = YES;
bit AUTO_FLAG = NO;
bit Step_FLAG = NO;
bit CANINFOR_FLAG = NO;
bit ONCE_FLAG = NO;
xdata volatile unsigned char Runmode = 0;
//���嶨ʱ�������������
xdata unsigned char  T0Counter5 = 0; //
xdata unsigned char  T0Counter2 = 0;
xdata volatile unsigned char  T0Counter3 = 0; //
xdata unsigned char  T0Counter4 = 0; //
xdata unsigned int  T0Counter6 = 0; //
xdata unsigned char  T0Counter8 = 0;
xdata unsigned int   T0Counter9 = 0;
xdata unsigned int   T0Counter10 = 0; //�߿������߶�ʱ��
xdata unsigned char  GAOKONG_COUT = 0; //�߿������߼�����
xdata unsigned char  YAOPINGTISH_COUT = 0; //ҩƷ����������
xdata unsigned char  CANcomand = 0;  //CAN������
xdata volatile unsigned char  nCANcomand = 0;   //��ֹCAN�ж��޸������е�����
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
//xdata FLASH Flash;            //����Flash ������
xdata CANTRANSDATBUF CANTXBUF_ZKB;
xdata unsigned  int Pulse1 = 0;
xdata unsigned char Shaft1_Base_Speed_Value = 0;
xdata long int Shaft1_Velocity_Subsection_Parameter = 0;
xdata long int motor1_out = 0;                      //�������1ʵ�ʵ�ת��
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
xdata unsigned  char Servomotor_original = 0; //�ŷ�ԭ���־λ
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
    Motor1_Dir = 1;	//��ת
}
void Dir_FZ()
{
    Motor1_Dir = 0;	//��ת
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
// ��    �ƣ�Initial
// ��    �ܣ���ʼ��ģ��
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
    //IDSET = 0x02;        //CAN���ܵ�ַ
    IDSET = P7 & 0x7F ;
    IDSET = IDSET >> 4;
    BPSSET = 0x00;       //CAN����������
    init_para();
    //PT2=1;
    PT0 = 1;
    //E2PROM��ʼ��
    WDTCN = 0xA5;  //���Ź���λ
    EA = 1;	//���ж�
    //motor1_ENB=0;
}
void Timer01_Init(void)
{
    char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page
    SFRPAGE = TIMER01_PAGE;             // Set SFR page
    TMOD = (TMOD & 0x00) | 0x21; //T0�����ڷ�ʽ1��T1�����ڷ�ʽ2
    PCON = 0x00;		//��Դ��SMOD������
    TH1  = 0xfd;		//9600 bit band:TH1=0xfd;1200 bit:TH1=0xe8
    TL1  = 0xfd;		//T1��������bt=11059200/(32*12*(256-TH1))
    TF1  = 0;		//��ʱ��1�����־λ����
    TR1  = 1;		//T1��ʼ���� bt=28800/(256-TH1)
    TH0  = 0xDC;      //Reset 10ms interrupt
    TL0  = 00;
    TF0  = 0;
    TR0  = 1;
    ET0 = 1;
    SCON0 = 0x50;		//����ͨѶ��ʽһ����ʼλ(L)��8λ����λ����У��λ��ֹͣλ(H)
//    SCON0 = 0xD0;       // Serial Port Control Register
    //SCON = 0x40;		//����ͨѶ��ʽһ����ʼλ(L)��8λ����λ����У��λ��ֹͣλ(H)
    ES0 = 1;	//�����ж�����
    SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
}


/////////////////////////////
//��ʱ0�ж�,ģʽ1,16λ��ʱ������, ʱ��4��Ƶ ,�����ȼ�
//T0=65536-1000us*11.0592/4=0xF533      ʱ��4��Ƶ�µ�1����
//T0=65536-1000us*11.0592/12=0xFC66     ʱ��12��Ƶ�µ�1����
//T0=65536-10000us*11.0592/12=0xDC00    ʱ��12��Ƶ�µ�10����
void Timer0_ISR (void) interrupt 1
{
    char SFRPAGE_SAVE = SFRPAGE;     // Save Current SFR page

    WDTCN = 0xA5;                    // ι��

    T0Counter2++;
    T0Counter3++;
    T0Counter4++;
    T0Counter5++;
    T0Counter6++;
    T0Counter8++;
    T0Counter9++;
    T0Counter10++;  //�߿������߶�ʱ��
    SFRPAGE = 0x0F;  //�رո߿�������

    //�߿����������ж�ʱ
    if(T0Counter10 > GAOKONG_TIMER)
    {
        if(Motor2 == 0)	//������
        {
            if((CANTRASTEMINFOR[8] & 0x04) == 0x04) //�߿������ߴ�������⵽ҩƷ
            {
                T0Counter10 = 0;
                GAOKONG_COUT = GAOKONG_COUT + 1;
                if(GAOKONG_COUT > 3)	//�����ȴ�����
                {
                    Motor2 = 1; //�رո߿�������
                    AUTOCMD_STATE = 0XC4;
                    T0Counter8 = 51;
                    AUTOCMD_TIME = 0;
                }//����*/
            }
            else	//�߿������ߴ�����δ��⵽ҩƷ
            {
                Motor2 = 1;
                T0Counter10 = 0;
                GAOKONG_COUT = 0;
            }
        }
        else	//δ����
        {
            T0Counter10 = 0;
            GAOKONG_COUT = 0;
        }
    }
    //���������ߴ������źż�Ⲣд��
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
    //�߿������ߴ������źż�Ⲣд��
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
    TH0 = 0xDC;	//����װ�ض�ʱֵ
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
    XBR0 = 0x04;	// XBAR0: Initial Reset Value  ����0
    XBR1 = 0x34;	// XBAR1: Initial Reset Value  T2��ʱ��������ⲿ�ж�1 �� �ⲿ�ж�0
    XBR2 = 0x40;	// XBAR2: Initial Reset Value  �����ܿ���
    // XBR3 = 0x01;    // XBAR3: Initial Reset Value  T3��ʱ�����
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
//��ʼ������
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
    for(i = 0; i < 4; i++) //���ŷ����в���
    {
        Servomotor_displace.buf[i] = 0;
        Servomotor_speed.buf[i] = 0;;
        Servomotor_addtime.buf[i] = 0;;
        Servomotor_subtime.buf[i] = 0;;
    }
    Board_Parameter_Setup();    //�������������ʼ��
    AUTOCMD_CHECK = 0;
    Runmode = 2;
    /***********************************/
    //��ʼ���������1����
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
    case 01:    //X	�����
        init_can1_1();   //��ʼ��c8051f040�Դ�CAN

        //Current_Shaft1_Code=0x01;
        Shaft1_ADD_Speed_Time_Base = 10;
        Shaft1_SUB_Speed_Time_Base = 10;
        Shaft1_Base_Speed_Value = 50;
        Shaft1_Velocity_Subsection_Parameter = 248830;
        Shaft1_Add_Speed = 15;
        Shaft1_Speed_max = 400;   //40
        break;
    case 02:    //Y	�����					{
        init_can1_1();   //��ʼ��c8051f040�Դ�CAN
        break;
    case 03:    //Y	�����					{
        init_can1_1();   //��ʼ��c8051f040�Դ�CAN
        break;
    case 04:    //Y	�����					{
        init_can1_1();   //��ʼ��c8051f040�Դ�CAN
        break;
    default:
        break;
    }
}


/* �������1��������
�ڴ˺����У���ת����Ϊ����뿪ԭ�����У���ת����Ϊ�������ԭ������
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
            CANTRASTEMINFOR[8] = CANTRASTEMINFOR[8] & 0XFD; //������б�־
            Shaft1_Run_mode = 0x00;

            CANTRASTEMINFOR[13] = (unsigned char)((ShowTime & 0x0000FF00) >> 8);
            CANTRASTEMINFOR[14] = (unsigned char)(ShowTime & 0x000000FF);
        }
        return;
    }
}
/////////////////////////////////////////////////////////////////
//int ramp(int max,int step)  ���1 ���Ӽ���
// ���м�ͣ���ܵ��ӳ���
// ��λΪ��/��
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
        if( T0Counter4 >= Shaft1_ADD_Speed_Time_Base )        //����ʱ���ٶ�ʱ�����
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
// ��    �ƣ�Serial_Port_Interrupt
// ��    �ܣ������жϷ������
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
    if(!RI0)		return ;		//�������˳�
    WDTCN = 0xA5;
    //�����ļ�ͷ
    RI0 = 0;
    Rec1[j] = SBUF0;	//���յĵ�һ�ֽ�
    //TEMP1 = Receiv_Msg[j];
    //Check_Sum += Receiv_Msg[j];
    j++;
    K = 0;

    while(!RI0)			//��ʱ����
    {
        K++;
        WDTCN = 0xA5;
        if(K > Delay_Times)
            goto END_ISR;
    }
    RI0 = 0;
    Rec1[j] = SBUF0;		//���յĵڶ��ֽ�

    //TEMP2 = Receiv_Msg[j];
    //Check_Sum += Receiv_Msg[j];
    j++;
    /*TEMP=((UINT)TEMP1<<8)+TEMP2;	//�ϳɽ��յ��ļ�ͷ

    if(TEMP != 0xEFEF)		//����յ��ļ�ͷ
    	goto END_ISR;
    ���ճ����ֽ�
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
    //�������ݡ�ָ���ֽ�
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
	///	Timer2�����ڵ�ƽ�л������ʽ
	/// �����ķ�����������Ŀ�������
	/// ʹ��ϵͳʱ�ӣ��Զ���װ�ط�ʽ��ʱ
	/// ����Ƶ�ʣ�1MHz  0xFFFA��500kHz   0xFFF4
	/////////////////////////////////////////////////////////////////////////////////////////////
    char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page
    SFRPAGE = 0x00;
    TMR2CF  = 0x0A; //	00001010b;	// Timer 3 Configuration
	//	λ7-5��000	// ����
	//	λ4-3��01	// SYSCLK
	//	λ2  ��0	// �л����״̬λ��дʱ����ǿ�����
	//	λ1  ��1	// ��ƽ�л������Ϊ��ʱ��������Ķ˿����ſ���
	//	λ0  ��0	// ��ʱ�����ϼ�������TnEX ��״̬�޹�
    RCAP2L  = 0x00;					// Timer 3 Reload Register Low Byte
    RCAP2H  = 0x00;					// Timer 3 Reload Register High Byte
    TMR2H   = 0x00;					// Timer 3 High Byte
    TMR2L   = 0x00;					// Timer 3 Low Byte
    TMR2CN  = 0x00; //	00001010b;	// Timer 3 Control Register
	//	λ7  ��0	// ��ʱ������/�����־������Ŀ����
	//	λ6  ��0	// ��ʱ���ⲿ��־������Ŀ����
	//	λ5-4��00	// ����
	//	λ3  ��0	// TnEX�ϵ����䱻����
	//	λ2  ��0	// ��ʱ����ֹ
	//	λ1  ��0	// ��ʱ������
	//	λ0  ��0	// ��ʱ���������Զ���װ�ط�ʽ

    ET2=1;
    SFRPAGE = SFRPAGE_SAVE;             // Restore SFRPAGE
}*/
/*void init_INT0 (void)
{
SFRPAGE  = CONFIG_PAGE;        //Port SFR's on Configuration page
XBR1  |= 0x04;                 //����INT0�ܽ�
SFRPAGE  = LEGACY_PAGE;
EA=0;
EX0 = 1;    //ʹ��INT0
PX0 = 1;    //�����ȼ�
IT0=1;      //�½��ش����ж�
EA=1;
}
//��Ӧ��X��Ϊ��ߵ���λ���أ���Ӧ��Y��Ϊ�ϱߵ���λ����
//�ⲿ�ж�INT0 �������ȼ�
void INT0_ISR (void) interrupt 0
{
unsigned char temp;
temp=SFRPAGE;
SFRPAGE  = LEGACY_PAGE;
EX0 = 0 ;   //���ⲿ�ж�0
Pulse1_Cnt=0;        //�ڴ˴�����������1,���ص�ԭ��λ��ʱ�����㲽�����������
Shaft1_Run_mode=0x00;
Motor1_Begin_Run_Sign =NO;
IE0=0;      //INT0�жϸ�λ
EX0 = 1 ;   //���ⲿ�ж�0
SFRPAGE=temp;
}*/
/*void init_INT0 (void)
{
SFRPAGE  = CONFIG_PAGE;        //Port SFR's on Configuration page
XBR1  |= 0x04;                 //����INT0�ܽ�
SFRPAGE  = LEGACY_PAGE;
EA=0;
EX0 = 1;    //ʹ��INT0
PX0 = 1;    //�����ȼ�
IT0=1;      //�½��ش����ж�
EA=1;
}
//��Ӧ��X��Ϊ��ߵ���λ���أ���Ӧ��Y��Ϊ�ϱߵ���λ����
//�ⲿ�ж�INT0 �������ȼ�
void INT0_ISR (void) interrupt 0
{
unsigned char temp;
temp=SFRPAGE;
SFRPAGE  = LEGACY_PAGE;
EX0 = 0 ;   //���ⲿ�ж�0
IE0=0;      //INT0�жϸ�λ
EX0 = 1 ;   //���ⲿ�ж�0
SFRPAGE=temp;
}

 void init_INT1 (void)
 {

	 SFRPAGE  = CONFIG_PAGE; //Port SFR's on Configuration page
	 XBR1  |= 0x10;          //����INT0�ܽ�
	 SFRPAGE  = LEGACY_PAGE;
	 EA=0;
	 EX1 = 1;               //ʹ��INT1
	 PX1 = 1;               //�����ȼ�
	 IT1=1;                 //�½��ش����ж�
	 EA=1;
	 }

	  //�ⲿ�ж�INT1 �������ȼ�
	  //��Ӧ��X��Ϊ�ұߵ���λ���أ���Ӧ��Y��Ϊ�±ߵ���λ����
	  void INT1_ISR (void) interrupt 2
	  {
	  unsigned char temp;
	  temp=SFRPAGE;
	  SFRPAGE  = LEGACY_PAGE;
	  EX1 = 0 ;           //���ⲿ�ж�1

	   IE1=0;               //INT1�жϸ�λ
	   EX1 = 1 ;            //���ⲿ�ж�1
	   SFRPAGE=temp;
}*/

//////////////////////////////////////////////////////////////////////
//T2����ж�    ���1��
//Ϊ�����������
//����ʵ�ʽ�������жϣ������������û��ʹ��T2������壬����ʱ��2δ������
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
//���1ת�ٱ��:��/��.
//���뷶Χ:0~6000��/S;  ���൱�ڲ������1000r/min��
//T2=65536-1000000*11.0592*360/(200*800*Vel)=65536-24883/Vel
//T2=65536-1000000*11.0592*360/(200*400*Vel)=65536-49766/Vel
//T2=65536-1000000*11.0592*360/(200*64*Vel)=65536-311040/Vel
//T2=65536-1000000*11.0592*360/(200*16*Vel)=65536-1244160/Vel


//���1ת�ٱ��:r/min.
//T2=65536-1000000*11.0592*3/(2*10*16*Vel)=65536-103680/Vel
//T2=65536-1000000*11.0592*3/(2*10*64*Vel)=65536-25920/Vel
//T2=65536-1000000*11.0592*3/(2*10*400*Vel)=65536-4147/Vel

//////////////////////////////////////////////////////////////////////
void Motor1_CTL(void)     //T2��������Ƶ��1
{
    unsigned int temp1;
    //	long int motor1_v;
    //unsigned long ShowTime;

    if(Vel1 <= Shaft1_Base_Speed_Value)             //�������ʱ����Х��
    {
        T2STOP;
        Time_FLAG1 = YES;
        Step_FLAG = YES;
        ShowTime = T0Counter6 * 10;
        return;
    } else
    {
        if(Vel1 > Shaft1_Speed_max)
            Vel1 = Shaft1_Speed_max;          //��������ٶȣ�
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
//����ΪRS485ģʽ  ��RS422����������ڷ��ͳ����У������˷���ǰ���رս��ܶ˿ڣ������꼴�򿪽��ܶ˿ڡ�
void Uart0Send(unsigned char *buf, unsigned char bufsize )
{
    unsigned char i = 0;
    unsigned int k = 0;
    unsigned int crc_z = 0;
    char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page

    SFRPAGE = CONFIG_PAGE;

    RE1 = 1;
    DE1 = 1;
    crc_z = crc_chk(buf, bufsize); //crcУ����
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
//�Զ�����ʱ
//��ʱʱ��ԼΪ(us N10)����
//-----------------------------------------------------------------------------
void delay1( unsigned int us)
{
    unsigned int i = us;
    while(i--) ;
}

/*
**���趨��5������ֵ����д���ŷ�������
**para1-para5:����ֵ״̬������ֵλ�á�����ֵ�ٶȡ�����ʱ�䡢����ʱ��
*/
void send_to_motor(void)
{
    unsigned char i;

    delay_ms(50);
    WDTCN = 0xA5;
    ES0 = 0;
    buf[0] = 0X01; //վ���ַ
    buf[1] = 0x10; //д
    buf[2] = 0x51; //�Ĵ�����ַ��λ
    buf[3] = 0x00; //�Ĵ�����ַ��λ
    buf[4] = 0x00; //��д���ݸ�λ
    buf[5] = 0x0A; //��д���ݵ�λ-5���Ĵ���
    buf[6] = 0x14;

    //#1 ����ֵ״̬
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
    //#2 ����ֵλ��
    buf[11] = Servomotor_displace.buf[0];
    buf[12] = Servomotor_displace.buf[1];
    buf[13] = Servomotor_displace.buf[2];
    buf[14] = Servomotor_displace.buf[3];
    //#3 ����ֵ�ٶ�
    buf[15] = Servomotor_speed.buf[0];
    buf[16] = Servomotor_speed.buf[1];
    buf[17] = Servomotor_speed.buf[2];
    buf[18] = Servomotor_speed.buf[3];
    //#4 ����ֵ����ʱ��
    buf[19] = Servomotor_addtime.buf[0];
    buf[20] = Servomotor_addtime.buf[1];
    buf[21] = Servomotor_addtime.buf[2];
    buf[22] = Servomotor_addtime.buf[3];
    //#5 ����ֵ����ʱ��
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
    if(Receive_command_finished == YES) //��ʱ��������ļ���У�飬���ڲ��ϣ���ǿ��ȫ��
    {
        Receive_command_finished = NO;
    }
}


//-----------------------------------------------------------------------------
// ���͸��ŷ������������ͨ��CONT�ź�
//command:ͨ��CONT�ź�-CONT9-CONT24��bit0-byte15��
//ͨ����CONT9-24����Ӧ���ܣ��Ӷ�ͨ��modbus�����ŷ�������
//-----------------------------------------------------------------------------
void Send_to_Motordriver_CTL(unsigned char command )
{
    UINTUNION temp;
    unsigned char i;

    temp.value = command;
    WDTCN = 0xA5;
    buf[0] = 0x01; //վ���ַ
    buf[1] = 0x10; //д

    buf[2] = 0x00; //�Ĵ�����ַ��λ   ���CONT�����źŵ�ַ
    buf[3] = 0x00; //�Ĵ�����ַ��λ

    buf[4] = 0x00; //��¼��
    buf[5] = 0x02; //��¼��

    buf[6] = 0x04; //�ֽ���

    buf[7] = 0x00; //��д��������ֽ�
    buf[8] = 0x00; //��д���ݴθ��ֽ�
    buf[9] = temp.buf[0]; //��д���ݴε��ֽ�
    buf[10] = temp.buf[1]; //��д��������ֽ�
    for(i = 0; i < 30; i++)
    {
        Rec1[i] = 0;
    }
    Uart0Send(buf, 11);
    delay_ms(300);
    if(Receive_command_finished == YES) //��ʱ��������ļ���У�飬���ڲ��ϣ���ǿ��ȫ��
    {
        Receive_command_finished = NO;
    }
}

//
void RunLEDDIS(void)//������ʾ
{
    if(T0Counter5 >= 20) //200msʱ�䶨ʱ
    {
        T0Counter5 = 0 ;
        LED2 = !LED2;
    }
}

//ֱ���������
void DCmotorCTRL(void)
{
    char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page

    switch(CANcomand)
    {
        /****************�߿�������ֱ�����*****************/
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

        /****************�㵹�������*****************/
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

//�����������
/*����ʵ�ʽ����жϣ���Ͳ�������û��ʹ��T2����źţ�
������Motor1_Power����ź�ֱ�ӿ����м�̵���ͨ�ϵ�ʵ��ת����
Ҫʵ�ַ�ת����Ҫ����һ���м�̵�����һ�������ź��������������壩*/
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
    Shaft1_CTL();    //1�ŵ��

}
/*******************************************************************************************************
//�ŷ�������к��������������ɼ�
/*******************************************************************************************************/
void ServomotorCRTL(unsigned char comand)//�ŷ��������
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
        CANTRASTEMINFOR[8] = CANTRASTEMINFOR[8] | 0X01; //�����б�־
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
//�ϴ����������
}
/*******************************************************************************************************
//�Զ�����
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
        Send_to_Motordriver_CTL(0x00);//�ر��ŷ�
        TR0  = 1;
        break;

    default:
        break;
    }
    SFRPAGE = SFRPAGE_SAVE;
}
/*******************************************************************************************************
//�Զ�����
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

        //���������ŷ�
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
        Send_to_Motordriver_CTL(0x01);//�ŷ�S-ON
        send_to_motor();//�趨����ֵ����
        Send_to_Motordriver_CTL(0x03);//�ŷ�S-ON���Զ������ŷ�
        Send_to_Motordriver_CTL(0x01);//�ŷ�S-ON
        T0Counter6 = 0;
        Time_FLAG = NO;
        Servomotor_original = 0;	//ԭ��λ�ñ�־����
        CANTRASTEMINFOR[8] = CANTRASTEMINFOR[8] | 0X01;	//�����б�־
        Receive485_command_finished = YES;	//485���ݽ������
        TR0  = 1;
        break;

    case 0xF1:		//�������������ߺ͸߿�������
        WDTCN = 0xA5;
        SFRPAGE = 0x0F;
        Motor1_Power = 0;	//����
        Dir_ZZ();	//��ת
//        Motor2 = 0;	//�����߿�������
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
        T0Counter10 = 0; //�߿������߶�ʱ��
        GAOKONG_COUT = 0; //�߿������߼�����
        SFRPAGE = SFRPAGE_SAVE;
        break;

    case 0xF2://�ع�ҩƷ�㵹���ع��ŷ�
        WDTCN = 0xA5;
        SFRPAGE = 0x0F;
        Motor1_Power = 1;	//�ϵ�
        Dir_ZZ();	//��ת
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
        Servomotor_displace.displace = Zero_backdisplace;//����ԭ��λ��
        CANTRASTEMINFOR[13] = 0;
        CANTRASTEMINFOR[14] = 0;
        Send_to_Motordriver_CTL(0x01);//�ŷ�S-ON
        send_to_motor();	//�趨����ֵ
        Send_to_Motordriver_CTL(0x03);//�ŷ��Զ�����
        Send_to_Motordriver_CTL(0x01);//�ŷ�S-ON
        T0Counter6 = 0;
        Time_FLAG = NO;
        CANTRASTEMINFOR[8] = CANTRASTEMINFOR[8] | 0X01; //�����б�־
        Receive485_command_finished = YES;
        TR0  = 1;
        break;

    case 0xF3:	//�ر��ŷ�
        WDTCN = 0xA5;
        TR0  = 0;
        Send_to_Motordriver_CTL(0x00);
        TR0  = 1;
        break;

    case 0xF4:	//ֹͣ�߿�������
        WDTCN = 0xA5;
        SFRPAGE = 0x0F;  //�رո߿�������
//        Motor2 = 1;
        SFRPAGE = SFRPAGE_SAVE;
        break;

    case 0xF5:	//��ԭ��
        Returntopoint();
        break;

        //added begin
    case 0xF6:		//�㵹�����߷�ת
        WDTCN = 0xA5;
        SFRPAGE = 0x0F;
        Motor1_Power = 0;	//������ת������
        Dir_FZ();	//��ת
        SFRPAGE = SFRPAGE_SAVE;
        break;

    case 0xF7:		//�㵹�����߷�ת
        WDTCN = 0xA5;
        SFRPAGE = 0x0F;
        Motor1_Power = 1;	//ֹͣ��ת������
        Dir_ZZ();	//��ת
        SFRPAGE = SFRPAGE_SAVE;
        break;
        //added end

    default:
        break;
    }
    //Shaft1_CTL( Shaft1_Run_mode,Shaft1_aim_pulse);    //1�ŵ��
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

//�ŷ���ԭ��
void Returntopoint(void)
{
    TR0  = 0;
    WDTCN = 0xA5;
//    Send_to_Motordriver_CTL(0x01);//�ŷ�S-ON
    Send_to_Motordriver_CTL(0x05);//ԭ�㸴��
//    Send_to_Motordriver_CTL(0x01);//�ŷ�S-ON
    Time_FLAG = NO;
    CANTRASTEMINFOR[8] = CANTRASTEMINFOR[8] | 0X40; //�����б�־
    TR0  = 1;
}

//�ŷ�������ݲɼ�
void ServomotorDataacquisition(unsigned char addressID)
{
    //if(ENABFLAG==YES)
    //{
    unsigned char i;

    if(Time_FLAG == YES) return;
    delay_ms(50);
    buf[0] = addressID; //վ���ַ
    buf[1] = 0x03; //��
    buf[2] = 0x10; //�Ĵ�����ַ��λ
    buf[3] = 0x03; //�Ĵ�����ַ��λ
    buf[4] = 0x00; //��д���ݸ�λ
    buf[5] = 0x06; //��д���ݵ�λ
    Uart0Send(buf, 6);
    AUTO_FLAG = YES;
    for(i = 0; i < 30; i++)
    {
        Rec1[i] = 0;
    }
    delay_ms(300);
    if(Receive_command_finished == YES)
    {
        //�Ժ��ܴ�
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

//�ŷ���������ɼ�
void CurrentCollection(void)
{
    //if(ENABFLAG==YES)
    //{
    unsigned char i;

    if(Time_FLAG1 == YES) return;
    delay_ms(50);
    buf[0] = 2; //վ���ַ
    buf[1] = 0x03; //��
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
        //�Ժ��ܴ�
        Receive_command_finished = NO;
        CANTRASTEMINFOR[9] = Rec1[3];
        CANTRASTEMINFOR[10] = Rec1[4];
        CANTRASTEMINFOR[11] = Rec1[5];
        CANTRASTEMINFOR[12] = Rec1[6];
    }
    AUTO_FLAG = NO;;

    //}
}

//���ݲɼ�
void Dataacquisition()
{
    //û�д�������ʱ������ִ�����ݲɼ�����
    if((CANcomand == 0 && nCANcomand == 0) || (nCANcomand == 0xFE) || (CANcomand == 0xFE))
    {
        //if(T0Counter3>=20)  //200msʱ�䶨ʱ
        //{
        //T0Counter3=0;
        ServomotorINP(0);
        switch(Sampleperiod)
        {
        case 0:
            ServomotorDataacquisition(1);//�ŷ�������ݲɼ�
            break;

        case 1:
            CurrentCollection();//�ŷ���������ɼ�
            break;

        default:
            break;
        }
        //�ɼ�����ѭ������
        Sampleperiod = Sampleperiod + 1;
        if(Sampleperiod >= 2)
        {
            Sampleperiod = 0;
        }

        //}
    }
}


//-----------------------------------------------------------------------------
//���ŷ������������ѯͨ��OUT�ź�
//get_value:ͨ��OUT�ź�-OUT6-CONT21��bit0-byte15��
//ͨ����out6-21����Ӧ���ܣ��Ӷ�ͨ��modbus��ȡ�ŷ�������״̬
//�ŷ������λ���н���(OUT6)��ԭ�㸴������ź�(OUT7)��ѯ����
//-----------------------------------------------------------------------------
void ServomotorINP(unsigned char mode)
{
    //unsigned int ShowTime1;
    unsigned char i;

    //if(ENABFLAG==YES)
    //{
	//��ʱ����
	if(mode==0)
	{
    if(Time_FLAG == YES) return;
	}
    WDTCN = 0xA5;
    delay_ms(50);
    buf[0] = 0x01; //վ���ַ
    buf[1] = 0x03; //��
    buf[2] = 0x01; //�Ĵ�����ַ��λ
    buf[3] = 0x00; //�Ĵ�����ַ��λ
    buf[4] = 0x00; //��д���ݸ�λ
    buf[5] = 0x02; //��д���ݵ�λ
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
        if((Rec1[6] & 0x01) == 0x01)//��λ����
        {
						if(mode==0)Time_FLAG = YES;
            CANTRASTEMINFOR[13] = (unsigned char)((ShowTime1 & 0xFF00) >> 8);
            CANTRASTEMINFOR[14] = (unsigned char)(ShowTime1 & 0x00FF);
            CANTRASTEMINFOR[8] = CANTRASTEMINFOR[8] & 0XFE; //������б�־
        }
        if((Rec1[6] & 0x02) == 0x02)//��λ����
        {
						if(mode==1)Time_FLAG = YES;
            CANTRASTEMINFOR[8] = CANTRASTEMINFOR[8] & 0XBF; //������б�־
        }
    }
    AUTO_FLAG = NO;
}

//ϵͳ����
/*ϵͳ���ֶ�����ģʽ���Զ�����ģʽ����Runmode��
ÿ��ģʽ�ٸ���CANcomandָ��ִ����Ӧ����
CANINDEX��Ϊ��������ʹ��
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
        SteppermotorCTRL();  //�����������
        DCmotorCTRL();       //ֱ���������
        if(Receive485_command_finished == YES) //��ֹ��������������
        {
            ServomotorCRTL(CANcomand);//�ŷ��������
        }
        break;

    case 1:
        if(CANcomand == 0)
        {
            CANcomand = nCANcomand;
        }
        /****************************/
        //�������ĺϷ���
        /****************************/
        else if(CANcomand == 0xFE)
        {
            AUTOCANcomand = AUTOTaskassignment1(); //�Զ�����������
            AUTORUNMODE1(AUTOCANcomand);
        }
        else if(CANcomand == 0xFB)
        {
            AUTOCANcomand = AUTOTaskassignment0(); //ԭ�㸴��
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

#define		QDM_FZ_RUNTIME		300	//��λ��10ms
/********************************************************
////�Զ�����������
********************************************************/
unsigned char AUTOTaskassignment1(void)
{
    unsigned char returnvalue = 0;
    char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page

    switch(system_crtl.AUTOsystem_command)//����ÿ�����м���жϸ�������
    {
    case 1:
        //�������ִ�п�����
        returnvalue = 0xF0; //���������ŷ�
        CANTRASTEMINFOR[8] = CANTRASTEMINFOR[8] | 0X01; //�����б�־
        system_crtl.AUTOsystem_command = 2;
        break;
    case 2:
        //�������ִ�п�����
        //�㵹ҩƷ
        ServomotorINP(0);//��ѯ�ŷ��Ƿ����н���
        //	ServomotorDataacquisition(1);//�ŷ�������ݲɼ�
        if(((CANTRASTEMINFOR[8] & 0X01) == 0x00)) //�ŷ���������
        {
            returnvalue = 0xF1; //ֱ����������㵹ҩƷ �����߿�������
            T0Counter9 = 0;  //�㵹��ʱ
            YAOPINGTISH_COUT = 0; //������0
            system_crtl.AUTOsystem_command = 4;
        }
        break;

    case 4:
        if(T0Counter9 > 200) //��ʱ����
        {
            if((CANTRASTEMINFOR[8] & 0x08) == 0x08) //���������ߴ��������
            {
                YAOPINGTISH_COUT = YAOPINGTISH_COUT + 1;
                if(YAOPINGTISH_COUT < 3)
                {
                    T0Counter9 = 0;  //�㵹��ʱ ���NPN�͵�ƽ��ʱ����
                }
                else
                {
                    SFRPAGE = 0x0F;
                    Motor1_Power = 1;
										Dir_ZZ();	//��ת
                    SFRPAGE = SFRPAGE_SAVE;
                    system_crtl.AUTOsystem_command = 7;
                }
            }
            else
            {
                returnvalue = 0xF2; //�㵹���ͣ���ŷ��ع�
                CANTRASTEMINFOR[8] = CANTRASTEMINFOR[8] | 0X01; //�����б�־
                system_crtl.AUTOsystem_command = 5;
            }
        }
        break;
    case 5:
        ServomotorINP(0);//��ѯ�ŷ��Ƿ����н���
        //   ServomotorDataacquisition(1);//�ŷ�������ݲɼ�
        if((CANTRASTEMINFOR[8] & 0X01) == 0x00)
        {
            returnvalue = 0xF5; //����
            // T0Counter9=0;//������ʱͣ��
            system_crtl.AUTOsystem_command = 6; //
        }
        break;

    case 6:
        ServomotorINP(1);//��ѯ�ŷ��Ƿ����н���
        if((CANTRASTEMINFOR[8] & 0x40) == 0x00) //ԭ�㸴�����
        {
            Servomotor_original = 1; //�ŷ�ԭ���־����
            returnvalue = 0xF3; //�ر��ŷ�
            system_crtl.AUTOsystem_command = 8; //
        }
        break;
    case 7:
        returnvalue = 0xF3; //�ر��ŷ�
        system_crtl.AUTOsystem_command = 8;
        break;
    case 8:
        // T0Counter10=0;  //�߿������߶�ʱ��
        //  GAOKONG_COUT=0; //�߿������߼�����
        system_crtl.AUTOsystem_command = 0;
        CANcomand = 0x00;
        nCANcomand = 0x00;
        CANINDEX = 0;
        //nCANDate1=0;
        //�Զ����н����ϱ���Ϣ����

        AUTOCMD_STATE = 0XC2;
        T0Counter8 = 51;
        AUTOCMD_TIME = 0;
        break;

        //added begin
    case 9:	//�㵹�����߷�ת������תʱ���رգ�������ʱ���趨
        switch(CANINDEX)
        {
        case 1:
            returnvalue = 0xF6;//���������߷�ת
            break;

        case 2:
            returnvalue = 0xF7;//����������ֹͣ
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
////�Զ�����������ģ��ϵͳ��λ-�ŷ���ԭ��
********************************************************/
unsigned char AUTOTaskassignment0(void)
{
    unsigned char returnvalue = 0;

    switch(system_crtl.AUTOsystem_command)//����ÿ�����м���жϸ�������
    {
    case 1:
        returnvalue = 0xE0; //�ŷ�ԭ�㸴��
        system_crtl.AUTOsystem_command = 2;
        break;

    case 2:
        ServomotorINP(1);//��ѯ�ŷ��Ƿ����н���
        if((CANTRASTEMINFOR[8] & 0x40) == 0x00) //ԭ�㸴�����
        {
            Servomotor_original = 1; //�ŷ�ԭ���־����
            returnvalue = 0xE1; //�ر�ʹ�ܣ������㵹��������鲽�����
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
        //�Զ����н����ϱ���Ϣ����
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
        CANTXBUF_ZKB.normal_buf.data1 = CANTRASTEMINFOR[8]; //�ŷ�״̬������״̬
        CANTXBUF_ZKB.normal_buf.data2 = Servomotor_original;
        CANTXBUF_ZKB.normal_buf.data3 = system_crtl.AUTOsystem_command; //ģ���������
        CANTXBUF_ZKB.normal_buf.data4 = system_crtl.AUTOsystem_alarm1; //����״̬����
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
    if(T0Counter3 >= 50) //200msʱ�䶨ʱ
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

//�����ϴ���Ϣ
void ActiveCANtransfer(void)//CAN���������ϴ�
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
            CANTXBUF_ZKB.normal_buf.data1 = CANTRASTEMINFOR[8]; //�ŷ�״̬������״̬
            CANTXBUF_ZKB.normal_buf.data2 = Servomotor_original;
            CANTXBUF_ZKB.normal_buf.data3 = system_crtl.AUTOsystem_command; //ģ���������
            CANTXBUF_ZKB.normal_buf.data4 = system_crtl.AUTOsystem_alarm1; //����״̬����
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

//�������ϴ���Ϣ
void UnactiveCANtransfer(void)//CAN����Ӧ���ϴ�
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
    case 0xC0://��ȡ���й�λ��Ϣ
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
    case 0xFC://�Զ�ģʽ��ѯ
        if(repair_flag != 0x00) //��������� ��ѯģʽ��������Ϣ
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
void TRANFORSATATIONINFOR(void)//��ȡ���й�λ��Ϣ
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
//�������Flash������д��Flash
//////////////////////////
void write_to_flash(void)
{
    unsigned char xdata *pwrite ;
    unsigned char i;
    char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page
    EA = 0; //���ж�
    SFRPAGE = LEGACY_PAGE;
    FLSCL = FLSCL | 0x01; //����д����flash
    PSCTL = PSCTL | 0x07; //����д����flash
    pwrite = 0x00;
    *pwrite = 0;  //����flash
    PSCTL = PSCTL & 0xFD; //��ֹ����
    i = 0;
    do {
        *pwrite++ = Servopara[i];
        i++;
    } while(i < 107);
    PSCTL = PSCTL & 0xFA; //��ֹflashд
    FLSCL = FLSCL & 0xFE; //��ֹд����flash
    EA = 1; //���ж�
    SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
}

//////////////////////////
//��Flash�б�������ݶ���Flash������
//////////////////////////
void read_from_flash(void)
{
    unsigned char code *pread ;
    unsigned char i;
    char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page
    EA = 0;
    SFRPAGE = LEGACY_PAGE;
    PSCTL = PSCTL | 0x04; //ָ��flash��ַ0��0x7f
    pread = 0x00;
    i = 0;
    do {
        Servopara[i] = *pread++;
        i++;
    } while(i < 107);
    PSCTL = PSCTL & 0xFB; //�ָ� 64k flash
    EA = 1; // ����ȫ���ж�
    SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
}


/*void Deviationremoval(void)//ƫ�����
{
Send_to_Motordriver_CTL(0x01);//����S-ON
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
    T0Counter3 = 51; //����Ķ�ʱ�������״̬���
    Runmode = 1; //�Զ�����
}
//////////////////////////
//         ������
//////////////////////////
unsigned char buffer[8];
void main (void)
{
    SFRPAGE = CONFIG_PAGE;
    Initial();
    delay_ms(1000);
    init_machine();//�ϵ�ϵͳ��λ
//	Send_to_Motordriver_CTL(0x00);
    while(1)
    {

        SystemControl();              //ϵͳ���ƣ�������Ϣ��Դ��can\RS485\�ж��źţ�
        UnactiveCANtransfer();        //CAN����Ӧ���ϴ�
        ActiveCANtransfer();          //CAN���������ϴ�

			
        //Dataacquisition();            //���ݲɼ�
        /*if((CANINFOR_FLAG==NO)&&(Runmode==0x00))//CAN���������ϴ�
        {
        	//EIE2 &= 0xDF;
        	CANsteminforback();            //ϵͳ��Ϣ�ϴ�
        	//EIE2 |= 0x20;
        }*/
        RunLEDDIS();                  //������ʾ
    }
}




























