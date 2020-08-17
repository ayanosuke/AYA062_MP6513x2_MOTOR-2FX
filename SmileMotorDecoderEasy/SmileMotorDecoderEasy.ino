//
// Smile Motor Decoder Easy
// Copyright(C)'2020 Ayanosuke(Maison de DCC) / twaydcc / Desktop Station / MECY
//
// AYA062-1 MP6513 x 2ch
//
// PB1-IN1 OUT1(6) [3] 橙 MOTOR
// PB4-IN2 OUT2(4) [2] 灰 MOTOR
// PB0-IN1 OUT1(6) [7] 白 PIN_FHEAD
// PB3-IN2 OUT2(4) [6] 黄 PIN_RHEAD
//
// http://maison-dcc.sblo.jp/ http://dcc.client.jp/ http://ayabu.blog.shinobi.jp/
// https://twitter.com/masashi_214
//
// DCC電子工作連合のメンバーです
// https://desktopstation.net/tmi/ https://desktopstation.net/bb/index.php
//
// This software is released under the MIT License.
// http://opensource.org/licenses/mit-license.php
//--------------------------------------------------------------------------------

#include <Arduino.h>
#include <string.h>
#include <avr/pgmspace.h> 
#include <avr/eeprom.h>	 //required by notifyCVRead() function if enabled below
#include <avr/wdt.h>
#include "motor_func.h"
#include "motor_ctrl.h"
#include "NmraDcc.h"


/*************************************************
 * 
 * Declaration of Symbols
 * 
 *************************************************/

//#define DEBUG			//リリースのときはコメントアウトすること
//#define DSFOX			//DSfoxでデバッグする


//各種設定、宣言

#define MAN_VER_NUMBER  0   /* Release Ver CV07 */
#define DECODER_ADDRESS 3
#define PIN_FHEAD	0
#define PIN_RHEAD	3

#define LIGHT_FLASH   5
#define MOTOR_LIM_MAX 255

#define CV_VSTART 2
#define CV_ACCRATIO 3
#define CV_DECCRATIO  4
#define CV_VMAX 5
#define CV_LIGHT 33
#define CV_BRIGHTRUN  34
#define CV_BRIGHTSTOP 35
#define CV_BEMFcoefficient  54

#define CV_PI_P 55
#define CV_PI_I 56
#define CV_BEMFCUTOFF 57

/*************************************************
 * 
 * Declaration of Classes
 * 
 *************************************************/

NmraDcc	 Dcc;
DCC_MSG	 Packet;

/*************************************************
 * 
 * Declaration of Variables
 * 
 *************************************************/


//Task Schedule
unsigned long gPreviousL1 = 0;
//unsigned long gPreviousL2 = 0; // VVVF control
unsigned long gPreviousL3 = 0;
unsigned long gPreviousL4 = 0;
//unsigned long gPreviousL5 = 0; // gIntervalRail

boolean gbMyActive = false;

//モータ制御関連の変数

uint16_t gSpeedCmd = 0;
uint8_t gPwmDirv = 128;
uint8_t gCV29_Vstart = 0;
uint8_t gCV29Direction = 0;
uint8_t gCV33_BiDir = 1;
uint8_t gCV34_Britrun = 255;
uint8_t gCV35_Britstop = 2;

// CV

uint8_t gCV1_SAddr = 3;
uint8_t gCVx_LAddr = 3;
uint8_t gCV2_Vstart = 20;
uint8_t gCV3_AccRatio = 16;
uint8_t gCV4_DecRatio = 16;
uint8_t gCV5_VMAX = 255;


//ファンクションの変数
uint8_t gFuncBits[2] = {0};

struct CVPair{
  uint16_t	CV;
  uint8_t	Value;
};

CVPair FactoryDefaultCVs [] = {
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, DECODER_ADDRESS},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, 0},		//The LSB is set CV 1 in the libraries .h file, which is the regular address location, so by setting the MSB to 0 we tell the library to use the same address as the primary address. 0 DECODER_ADDRESS
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},	 //XX in the XXYY address
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, 0},	 //YY in the XXYY address
  {CV_29_CONFIG, 2},	 //Make sure this is 0 or else it will be random based on what is in the eeprom which could caue headaches
//  {CV_29_CONFIG, 128},   //Make sure this is 0 or else it will be random based on what is in the eeprom which could caue headaches
  {CV_VSTART, 20},
  {CV_ACCRATIO, 16},
  {CV_DECCRATIO, 8},
  {CV_LIGHT, 1},            // headlight ctrl. 1:bi-direction, 0:single
  {CV_BRIGHTRUN, 255},        // headlight britness max255
  {CV_BRIGHTSTOP, 2},
  {CV_BEMFcoefficient, 64}, // 0-127 step / Reajust by MECY 2017/04/16
  {CV_PI_P, 18},
  {CV_PI_I, 96},
  {CV_BEMFCUTOFF,0},        // 0でBEMF Off 1でBEMF On
};

/*************************************************
 * 
 * Declaration of Functions
 * 
 *************************************************/

void(* resetFunc) (void) = 0;  //declare reset function at address 0

uint8_t FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);

void ControlRoomLight(void);
void ControlHeadLight(void);

//------------------------------------------------------------------
// CV8 によるリセットコマンド受信処理
//------------------------------------------------------------------
void notifyCVResetFactoryDefault()
{
	//When anything is writen to CV8 reset to defaults. 

	resetCVToDefault();	 
#ifdef DEBUG
	Serial.println("Resetting...");
#endif
	delay(1000);  //typical CV programming sends the same command multiple times - specially since we dont ACK. so ignore them by delaying
	resetFunc();
}

//------------------------------------------------------------------
// CVをデフォルトにリセット(Initialize cv value)
// Serial.println("CVs being reset to factory defaults");
//------------------------------------------------------------------
void resetCVToDefault()
{
	//CVをデフォルトにリセット
#ifdef DEBUG
	Serial.println("CVs being reset to factory defaults");
#endif
	
	for (int j=0; j < FactoryDefaultCVIndex; j++ ){
		Dcc.setCV( FactoryDefaultCVs[j].CV, FactoryDefaultCVs[j].Value);
	}
}


//------------------------------------------------------------------
// CV値が変化した時の処理（特に何もしない）
//------------------------------------------------------------------
extern void	   notifyCVChange( uint16_t CV, uint8_t Value)
{
   //CVが変更されたときのメッセージ
#ifdef DEBUG
   Serial.print("CV "); 
   Serial.print(CV); 
   Serial.print(" Changed to "); 
   Serial.println(Value, DEC);
#endif
}

//------------------------------------------------------------------
// CV Ackの処理
// そこそこ電流を流さないといけない
//------------------------------------------------------------------
void notifyCVAck(void)
{
#ifdef DEBUG
  Serial.println("notifyCVAck");
#endif
  
	MOTOR_Ack();
}

//------------------------------------------------------------------
// Arduino固有の関数 setup() :初期設定
//------------------------------------------------------------------
void setup()
{
	//ファンクションの割り当てピン初期化
	
	pinMode(PIN_FHEAD, OUTPUT);
//	digitalWrite(PIN_FHEAD, 0);
	
	pinMode(PIN_RHEAD, OUTPUT);
//	digitalWrite(PIN_RHEAD, 0);


	if ( Dcc.getCV(CV_MULTIFUNCTION_PRIMARY_ADDRESS) == 0xFF )
	{
		//if eeprom has 0xFF then assume it needs to be programmed
#ifdef DEBUG
	  Serial.println("CV Defaulting due to blank eeprom");
#endif
	  
	  notifyCVResetFactoryDefault();
	  
   } else{
#ifdef DEBUG
	 Serial.println("CV Not Defaulting");
#endif
   }
  
	// Setup which External Interrupt, the Pin it's associated with that we're using, disable pullup.
//	Dcc.pin(0, 11, 0);
  Dcc.pin(0, 2, 0);   // ATtiny85

	// Call the main DCC Init function to enable the DCC Receiver
	Dcc.init( MAN_ID_DIY, MAN_VER_NUMBER,   FLAGS_MY_ADDRESS_ONLY , 0 ); 

	//Reset task
	gPreviousL1 = millis();
	gPreviousL3 = millis();
	gPreviousL4 = millis();

	//Init CVs
	gCV1_SAddr = Dcc.getCV( CV_MULTIFUNCTION_PRIMARY_ADDRESS ) ;
	gCV2_Vstart = Dcc.getCV( CV_VSTART ) ;
	gCV3_AccRatio = Dcc.getCV( CV_ACCRATIO ) ;
	gCV4_DecRatio = Dcc.getCV( CV_DECCRATIO ) ;
	gCV5_VMAX = Dcc.getCV( CV_VMAX ) ;
	gCVx_LAddr = (Dcc.getCV( CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB ) << 8) + Dcc.getCV( CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB );
	
	gCV29_Vstart = Dcc.getCV( CV_29_CONFIG ) ;
	//cv29 Direction Check
	if ( (gCV29_Vstart & 0x01) > 0)
	{
		gCV29Direction = 1;//REVをFWDにする
	}
	else
	{
		gCV29Direction = 0;//FWDをFWDにする
	}
	
	gCV33_BiDir = Dcc.getCV(CV_LIGHT);
	gCV34_Britrun = Dcc.getCV(CV_BRIGHTRUN);
	gCV35_Britstop = Dcc.getCV(CV_BRIGHTSTOP);

	MOTOR_Init();	
	MOTOR_SetCV(2, Dcc.getCV(CV_VSTART));
	MOTOR_SetCV(3, Dcc.getCV(CV_ACCRATIO));
	MOTOR_SetCV(4, Dcc.getCV(CV_DECCRATIO));
	MOTOR_SetCV(5, Dcc.getCV(CV_VMAX));
	MOTOR_SetCV(54, Dcc.getCV(CV_BEMFcoefficient));	//Nagoden Adjust1 2016/06/02
	MOTOR_SetCV(55, Dcc.getCV(CV_PI_P));			// MECY Adjust1 2016/04/09
	MOTOR_SetCV(56, Dcc.getCV(CV_PI_I));			// MECY Adjust1 2016/04/09
	MOTOR_SetCV(57, Dcc.getCV(CV_BEMFCUTOFF));

#ifdef DEBUG
	Serial.print("CV1(ShortAddr): ");
	Serial.println(gCV1_SAddr);
	Serial.print("CV17/18(LongAddr): ");
	Serial.println(gCVx_LAddr);
	Serial.print("CV29: ");
	Serial.println(gCV29_Vstart);
	
	Serial.println("Ready");
#endif



}

//---------------------------------------------------------------------
// Arduino main loop
//---------------------------------------------------------------------
void loop()
{

   //analogWrite(PIN_FHEAD,255); //PB0-IN1 OUT1(6) [7] 橙 PIN_FHEAD 255でLO 125で50%
   //analogWrite(PIN_RHEAD,25); //PB3-IN2 OUT2(4) [6] 灰 PIN_RHEAD 255でLO  うごきません
   //digitalWrite(PIN_RHEAD,HIGH); //LOWでON HIGHでOFF
  
	// You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
	Dcc.process();

	if( (millis() - gPreviousL3) >= 100){
		//Motor drive control
		MOTOR_Main(gSpeedCmd, gPwmDirv);
		//Reset task
		gPreviousL3 = millis();
	}
	
	
	if( (millis() - gPreviousL4) >= 250){	
//		ControlRoomLight();
		ControlHeadLight();
		//Reset task
		gPreviousL4 = millis();
	}
}

//---------------------------------------------------------------------
//DCC速度信号の受信によるイベント
//extern void notifyDccSpeed( uint16_t Addr, uint8_t Speed, uint8_t ForwardDir, uint8_t MaxSpeed )
//---------------------------------------------------------------------
extern void notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps )
{
	uint16_t aSpeedRef = 0;
	
	if(!gbMyActive) {
		wdt_enable(WDTO_1S);
		gbMyActive = true;
	}

	wdt_reset();
	
	//速度値の正規化(255を100%とする処理)
	if( Speed >= 1){
		aSpeedRef = ((Speed - 1) * 255) / SpeedSteps;
	} else {
		//緊急停止信号受信時の処理 //Nagoden comment 2016/06/11
#ifdef DEBUG
		Serial.println("***** Emagency STOP **** ");
#endif
		aSpeedRef = 0;
	}

  aSpeedRef = aSpeedRef * gCV5_VMAX / 255; 
	
	gSpeedCmd = aSpeedRef;

  gPwmDirv = Dir;
  if ( gCV29Direction > 0) {
     if ( Dir == DCC_DIR_FWD){
          gPwmDirv = 128;
     } else {
          gPwmDirv = 0;     
     }     
  } else {
     if ( Dir == DCC_DIR_FWD){
          gPwmDirv = 0;
     } else {
         gPwmDirv =128;     
     }     
  }
 
  wdt_reset();
}
//---------------------------------------------------------------------------
//ファンクション信号受信のイベント
//FN_0_4とFN_5_8は常時イベント発生（DCS50KはF8まで）
//FN_9_12以降はFUNCTIONボタンが押されたときにイベント発生
//前値と比較して変化あったら処理するような作り。
//---------------------------------------------------------------------------
extern void notifyDccFunc( uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)
{
	wdt_reset();
	
	switch(FuncGrp)
	{
    case FN_0_4:
//      Function F00
         if  ((FuncState & FN_BIT_00) == 16){
            if (gFuncBits[0] == 0){ // OFFだったらON
              gFuncBits[0] = 1;
            }
         } else {
            if (gFuncBits[0] == 1){ // ONだったらOFF
              gFuncBits[0] = 0;
            }
         }
             
//      Function F01
         if  ((FuncState & FN_BIT_01) == 1){
            if (gFuncBits[1] == 0){ // OFFだったらON
              gFuncBits[1] = 1;
            }
         } else {
            if (gFuncBits[1] == 1){ // ONだったらOFF
              gFuncBits[1] = 0;
            }
         }
		break;
	}
}

void exec_function (int function, int pin, int FuncState)
{
	//digitalWrite (pin, FuncState);
}

void ControlRoomLight(void)
{
}

//---------------------------------------------------------------------
// ヘッドライトの制御
// ControlRoomLight()
//---------------------------------------------------------------------
void ControlHeadLight(void)
{
	if(gFuncBits[0] == 0){ // FR共に消灯
//		digitalWrite(PIN_FHEAD, 0);
//		digitalWrite(PIN_RHEAD, 0);
    analogWrite(PIN_FHEAD,255);   // MP6513は255でOFF
    digitalWrite(PIN_RHEAD,HIGH); //analogWrite(PIN_RHEAD,255);PB3はanalogWrite()が使えない;
		return;
	}
	
	if( gPwmDirv == 0){ //FWD
//		digitalWrite(PIN_FHEAD, 1);
//		digitalWrite(PIN_RHEAD, 0);
    analogWrite(PIN_FHEAD,255-gCV34_Britrun);
    digitalWrite(PIN_RHEAD,HIGH); //analogWrite(PIN_RHEAD,255);
	} else {
		//REV
		if(gCV33_BiDir == 1){
//			digitalWrite(PIN_FHEAD, 0);
//			digitalWrite(PIN_RHEAD, 1);
    analogWrite(PIN_FHEAD,255);
    digitalWrite(PIN_RHEAD,LOW); //analogWrite(PIN_RHEAD,0);

		} else {
//			digitalWrite(PIN_FHEAD, 0);
//			digitalWrite(PIN_RHEAD, 0);
    analogWrite(PIN_FHEAD,255);
    digitalWrite(PIN_RHEAD,HIGH); //analogWrite(PIN_RHEAD,255);
		}
	}
}
