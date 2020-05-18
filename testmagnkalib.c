#include "libschsat.h"
const float kd = 290.0;
const float time_step = 0.1;
uint16_t  sr=0;
float omega_goal = 0.0;// РµСЃР»Рё РЅР°РґРѕ СЂР°СЃРєСЂСѓС‚РєСѓ С‚Рѕ 10 РёР»Рё -10,РµСЃР»Рё СЃС‚Р°Р±РёР»РёР·Р°С†РёСЋ С‚Рѕ 0.0
int16_t magx, magy, magz;
double angle=0;
// РќРѕРјРµСЂ РјР°С…РѕРІРёРєР°
const int mtr_num = 1;
const int num=1;
// РњР°РєСЃРёРјР°Р»СЊРЅРѕ РґРѕРїСѓСЃС‚РёРјР°СЏ СЃРєРѕСЂРѕСЃС‚СЊ РјР°С…РѕРІРёРєР°, РѕР±/РјРёРЅ
const int mtr_max_speed = 3000;
// РќРѕРјРµСЂ Р”РЈРЎ (РґР°С‚С‡РёРєР° СѓРіР»РѕРІРѕР№ СЃРєРѕСЂРѕСЃС‚Рё)
const  uint16_t hyr_num = 1;
int16_t mtr_new_speed;
int16_t mgx_cal=0;
	int16_t mgy_cal=0;
	int16_t mgz_cal=0;
	int16_t *magx_cal = &mgx_cal;
	int16_t *magy_cal = &mgy_cal;
	int16_t *magz_cal = &mgz_cal;
float mag_alpha;
//РєР°Р»РёР±СЂРѕРІРєР°
float deg(int16_t *magx, int16_t *magy, int16_t *magz ){
	float magx_cal;
	magx_cal = 2.965555*(*magx + 57.770324) + 0.089700*(*magy + 82.501177) + 0.209399*(*magz + 107.336800);
	float magy_cal;
	magy_cal = 0.089700*(*magx + 57.770324) + 3.204481*(*magy + 82.501177) + 0.234796*(*magz + 107.336800);
	float magz_cal;
	magz_cal = 0.209399*(*magx + 57.770324) + 0.234796*(*magy + 82.501177) + 2.632183*(*magz + 107.336800);
	*magx = (int16_t) magx_cal;
	*magy = (int16_t) magy_cal;
	*magz = (int16_t) magz_cal;
	double alfa_rad = atan2(mgz_cal, mgy_cal);
			if(alfa_rad < 0) 
				alfa_rad += 2 * M_PI; 
			mag_alpha = alfa_rad/ M_PI *180;
	return mag_alpha;
}
void initialize_all(void){
/*Р¤СѓРЅРєС†РёСЏ РІРєР»СЋС‡Р°РµС‚ РІСЃРµ РїСЂРёР±РѕСЂС‹,РєРѕС‚РѕСЂС‹Рµ Р±СѓРґСѓС‚ РёСЃРїРѕР»СЊР·РѕРІР°С‚СЊСЃСЏ РІ РѕСЃРЅРѕРІРЅРѕР№ РїСЂРѕРіСЂР°РјРјРµ.*/
	printf("Enable motor в„–%d\n", mtr_num); 
	motor_turn_on(mtr_num);
	Sleep(1);
	printf("Enable angular velocity sensor в„–%d\n", hyr_num); 
	hyro_turn_on(hyr_num);
	Sleep(1);
	magnetometer_turn_on(num);  
	Sleep(1);
 }
 void switch_off_all(void){
/* Р¤СѓРЅРєС†РёСЏ РѕС‚РєР»СЋС‡Р°РµС‚ РІСЃРµ РїСЂРёР±РѕСЂС‹,РєРѕС‚РѕСЂС‹Рµ Р±СѓРґСѓС‚ РёСЃРїРѕР»СЊР·РѕРІР°С‚СЊСЃСЏ РІ РѕСЃРЅРѕРІРЅРѕР№ РїСЂРѕРіСЂР°РјРјРµ.*/
	printf("Finishing...");
	int16_t new_speed = mtr_new_speed; //
	printf("\nDisable angular velocity sensor в„–%d\n", hyr_num);
	hyro_turn_off(hyr_num);
	motor_set_speed(mtr_num, 0, &new_speed);
	Sleep (1);
	motor_turn_off(mtr_num);
	printf("Finish program\n");
}

 void control(void){
	initialize_all();
	sun_sensor_turn_on(num);
  		uint16_t value1;
  		uint16_t value2;
	// РРЅРёС†РёР°Р»РёР·РёСЂСѓРµРј СЃС‚Р°С‚СѓСЃ РјР°С…РѕРІРёРєР°
	int16_t mtr_state;
	// РРЅРёС†РёР°Р»РёР·РёСЂСѓРµРј СЃС‚Р°С‚СѓСЃ Р”РЈРЎ	
	int16_t hyro_state = 0;
	int16_t pRAW_dataX = 0;
	int16_t pRAW_dataY = 0;
	int16_t pRAW_dataZ = 0;
	int16_t *gx_raw = &pRAW_dataX; 
	int16_t *gy_raw = &pRAW_dataY; 
	int16_t *gz_raw = &pRAW_dataZ;
	int16_t speed = 0;
	int16_t	*mtr_speed = &speed;
	float omega;
	int i;
	while(1){
		if (LSS_OK == magnetometer_request_raw(num, magx_cal,magy_cal,magz_cal	)) {
		angle=deg(magx_cal,magy_cal,magz_cal);
		printf("angle=%f\n",angle);
		}
		 else {
  			puts("Fail!");
  		}
	}
	Sleep(time_step);
	switch_off_all();
	sun_sensor_turn_off(num);
	magnetometer_turn_off(num);  
 }