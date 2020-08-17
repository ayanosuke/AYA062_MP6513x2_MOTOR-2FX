// Motor Control for DS-DCC decode
// By yaasan
//
// MP3V5_DL109T by nagoden

#include <Arduino.h>
#include <string.h>
#include "motor_ctrl.h"
#include "motor_func.h"


/*************************************************
 * 
 * Declaration of Symbols
 * 
 *************************************************/

#define COUNT_ZEROMAX 10
#define ONBRAKEPWM


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


//モータ制御関連の変数
long gMotorLPF_buf = 0;
long gBEMFLPF_buf = 0;

long gMotorPI_buf = 0;
long gMotorSpeed = 0;
byte gDirection = 128;
int gPrevSpeed = 0;
int gPrevSpeedRef = 0;
int gPWMRef = 0;
byte gZeroCount = 0;

MOTOR_PARAM gParam;

//--------------------------------------------------------------------------------
// Declaration of Functions
//--------------------------------------------------------------------------------
void MOTOR_Init();
void MOTOR_SetCV(byte iNo, byte inData);
void MOTOR_Sensor();
void MOTOR_Ack(void);
void MOTOR_Main(int inSpeedCmd, byte inDirection);


//--------------------------------------------------------------------------------
// MOTOR_Init() ATtiny85用
// Functions
//--------------------------------------------------------------------------------
void MOTOR_Init()
{
	gParam.mStartVoltage = 32;
  gParam.mBEMFcoefficient = 38;
	gParam.mPI_P = 32;
	gParam.mPI_I = 96;
 	gParam.mBEMFcutoff = 32;
	gParam.mAccRatio = 4 * 16;
	gParam.mDecRatio = 4 * 16;

#if 0
	#if defined(__AVR_ATtiny1634__)
	//D2,D14 PWM キャリア周期:16kHz (8MHz internal)
	TCCR1B &= B11111000; // Counter stopped
	TCCR1B |= B00000001; // No prescaling
	#else
	//D3,D11 PWM キャリア周期:31kHz
	TCCR2B &= B11111000;
	TCCR2B |= B00000001;
	#endif
#endif

   //タイマー1初期化(タイマー1入力4MHz, PWM 15.625KHz)
   // TCCR1 Timer/Counter 0 Control Register
   //      OCR1A有効   low出力          分周無し　
   // TCCR1 = 1 << CTC1 | 1 << PWM1A | 2 << COM1A0 | 0<<CS13 | 2<<CS10; 
   TCCR1 = B11100011;//
   
   // General Timer / Counter Control Register
   //       OCR1B有効   low出力　
   GTCCR = 1 << PWM1B | 2 << COM1B0;
   OCR1C = 255;   //full count
   OCR1A = 0;   // duty
   OCR1B = 0;

	//PWM出力ピン D3,D11を出力にセット
	pinMode(MOTOR_PWM_B, OUTPUT);
	pinMode(MOTOR_PWM_A, OUTPUT);
}

//--------------------------------------------------------------------------------
// MOTOR_SetCV()
//--------------------------------------------------------------------------------
void MOTOR_SetCV(byte iNo, byte inData)
{
	switch(iNo){
	  case 2:
		      gParam.mStartVoltage = inData;
		      break;
	  case 3:
		      if( inData > 16){
			      gParam.mAccRatio = 160;                   // Readjust CV3 Para set by MECY 2017/4/11
		      } else {
			      gParam.mAccRatio = inData * 10 ;          // Readjust CV3 Para set by MECY 2017/4/11
		      }
		      break;
	  case 4:
		      if( inData > 16){
			      gParam.mDecRatio = 160;                   // Readjust CV4 Para set by MECY 2017/4/11
		      } else {
			      gParam.mDecRatio = inData * 10;           // Readjust CV4 Para set by MECY 2017/4/11
		      }
		      break;
	  case 5:
		      gParam.mMaxVoltage = inData;
		      break;
	  case 54:
    	    gParam.mBEMFcoefficient = inData;
    	    break;
	  case 55:
		      gParam.mPI_P = inData;
		      break;
	  case 56:
		      gParam.mPI_I = inData;
		      break;
    case 57:
    	    gParam.mBEMFcutoff = inData;
    	    break;
	}
}

//--------------------------------------------------------------------------------
// MOTOR_Ack()
// MP6513で方向のトグルを実行すると破損の可能性があるので片側のみの対応とする  2020/03/01 Nagoden
// ATtiny85用にPWM_A側をレジスタ直接操作に変更
// OnBreak(255)なので変更
//--------------------------------------------------------------------------------
void MOTOR_Ack(void)
{
  #ifdef ONBRAKEPWM
    analogWrite(MOTOR_PWM_B, 255);    //Change  by MP6513.
    OCR1A = 30; //analogWrite(MOTOR_PWM_A, 30);     //Change  by MP6513.

    delay( 6 );
  
    analogWrite(MOTOR_PWM_B, 255);    //Change  by MP6513.
    OCR1A = 255; //analogWrite(MOTOR_PWM_A, 255);    //Change  by MP6513.    
  #else
    analogWrite(MOTOR_PWM_B, 0);
    nalogWrite(MOTOR_PWM_A, 250);
  
    delay( 6 );  
  
    OCR1A = 0; //analogWrite(MOTOR_PWM_A, 0);
  #endif
}

//--------------------------------------------------------------------------------
// MOTOR_Main()
// Motor control Task (10Hz) 
// inDirection 0 or 128
//--------------------------------------------------------------------------------
void MOTOR_Main(int inSpeedCmd, byte inDirection)
{
  int aSpeedRef = (double)inSpeedCmd * ((double)gParam.mMaxVoltage  - (double)gParam.mStartVoltage) / (double)gParam.mMaxVoltage ;
  int aPWMRef = 0;
  long aSpeedRef_offseted = 0;
  static char aSlowdownFlg = 0;
  
  //8bit LPF
  if( gPrevSpeedRef <= aSpeedRef)
  {
    aSpeedRef = MOTOR_LPF(aSpeedRef, gParam.mAccRatio, &gMotorLPF_buf);
  }
  else
  {
    aSpeedRef = MOTOR_LPF(aSpeedRef, gParam.mDecRatio, &gMotorLPF_buf);
  }
  
  //バックアップ
  //gPrevSpeedRef = inSpeedCmd;
    gPrevSpeedRef = aSpeedRef;

    //PI Control
    //BEMF無いので削除 aya
  
    //PI無効
    if( aSpeedRef == 0)
    {
      aSpeedRef_offseted = aSpeedRef;
    }
    else if( aSpeedRef >= 1)
    {
      aSpeedRef_offseted = aSpeedRef + gParam.mStartVoltage ; //2016/06/19 Modify Nagoden 
    }

  //前回速度として保存
  gPrevSpeed = aSpeedRef_offseted ;

  //Limiter (8bit PWM Output)
  aPWMRef = MOTOR_limitSpeed((uint16_t)(aSpeedRef_offseted));
  //Clear output and LPF buffer when direction changes  2019/08/10 Nagoden
  if ( inDirection != gDirection )
  {
       aPWMRef = 0;
       gMotorLPF_buf = 0;
       aSlowdownFlg = 1;
  } 
    
  //PWM出力
  if( aPWMRef == 0){
    #ifdef ONBRAKEPWM
      OCR1A = 255; // analogWrite(MOTOR_PWM_A, 255);
      analogWrite(MOTOR_PWM_B, 255);
    #else
      OCR1A = 0; // analogWrite(MOTOR_PWM_A, 0);
      analogWrite(MOTOR_PWM_B, 0);
    #endif
  //Turn off the output for a certain time when the direction is changed 2019/08/10 Nagoden
    if ( inDirection != gDirection )
    {
        delay(1000); 
        gDirection = inDirection;
    }
  }
  else
  {
    //PWM出力
    //進行方向でPWMのABを切り替える
    if( inDirection > 0)
    {
      #ifdef ONBRAKEPWM
        analogWrite(MOTOR_PWM_B, 255);
        OCR1A = 255 - aPWMRef; // analogWrite(MOTOR_PWM_A, 255 - aPWMRef);
      #else
        analogWrite(MOTOR_PWM_B, 0);
        OCR1A = aPWMRef; // analogWrite(MOTOR_PWM_A, aPWMRef);
        #endif
      }
      else
      {
      #ifdef ONBRAKEPWM
        OCR1A = 255; // analogWrite(MOTOR_PWM_A, 255);
        analogWrite(MOTOR_PWM_B, 255 - aPWMRef);
      #else
        OCR1A = 0; // analogWrite(MOTOR_PWM_A, 0);
        analogWrite(MOTOR_PWM_B, aPWMRef);
        #endif
      }
  }

  gDirection = inDirection;
  gPWMRef = aPWMRef;
  
  /*
  Serial.print("SCMD= ");
  Serial.print(inSpeedCmd);

  Serial.print(" ,SREF= ");
  Serial.print(aSpeedRef);
  
  Serial.print(" ,BEMF= ");
  Serial.print(aSpeedDet);

  Serial.print(" ,PI_IBUF= ");
  Serial.print(gMotorPI_buf);

  Serial.print(" ,PWM=");
  Serial.println(aPWMRef);
  */
}

//--------------------------------------------------------------------------------
// MOTOR_limitSpeed(()
//--------------------------------------------------------------------------------
int MOTOR_limitSpeed(int inSpeed)
{
	uint16_t aSpeedref = 0;                                   //MECY Adjust 2016/06/04
	
	if( inSpeed >= gParam.mMaxVoltage)
	{
		aSpeedref = gParam.mMaxVoltage;                            //MECY Adjust 2016/06/04
	}
	else if( inSpeed <= 0)
	{
		aSpeedref = 0;                                        //MECY Adjust 2016/06/04
	}
	else if( gParam.mBEMFcutoff == 0 )                        //MECY Adjust 2016/06/04
	{
		aSpeedref = inSpeed;
	}
	else if( gParam.mBEMFcutoff > 0 )
	{
		aSpeedref = inSpeed;
	}
	
	return aSpeedref;
}
