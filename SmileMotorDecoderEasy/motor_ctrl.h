// DCC Decoder for DS-DCC decode
// By yaasan
// Based on Nicolas's sketch http://blog.nicolas.cx
// Inspired by Geoff Bunza and his 17 Function DCC Decoder & updated library
//
//
//

//---------------------------------------------------------------------
// Declaration of Symbols
// 2018/6/30 aya add 
//---------------------------------------------------------------------
 #define MOTOR_PWM_A 1    // Attiny85 PB1(6pin) OC1A analogWrite使えない,OCR1A=nで設定
 #define MOTOR_PWM_B 4    // Attiny85 PB4(3pin) OC1B analogWrite使える(OCR1B)


/*************************************************
 * 
 * Declaration of Classes
 * 
 *************************************************/

/*************************************************
 * 
 * Declaration of Variables
 * 
 *************************************************/


typedef struct _MOTOR_PARAM
{
	byte mStartVoltage;
	byte mMaxVoltage;
	byte mAccRatio;
	byte mDecRatio;
	byte mBEMFcoefficient;
	byte mPI_P;	
	byte mPI_I;	
	byte mBEMFcutoff;
} MOTOR_PARAM;
	
	


/*************************************************
 * 
 * Declaration of Functions
 * 
 *************************************************/

extern void MOTOR_Init();
extern void MOTOR_SetCV(byte iNo, byte inData);
extern void MOTOR_Sensor();
extern void MOTOR_Ack(void);
extern void MOTOR_Main(int inSpeedCmd, byte inDirection);
int MOTOR_limitSpeed(int inSpeed);
int MOTOR_GetBEMF();
