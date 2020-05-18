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
 
int motor_new_speed_PD(int mtr_speed, int omega, float omega_goal){
 /* Р¤СѓРЅРєС†РёСЏ РґР»СЏ РѕРїСЂРµРґРµР»РµРЅРёСЏ РЅРѕРІРѕР№ СЃРєРѕСЂРѕСЃС‚Рё РјР°С…РѕРІРёРєР°.
 РќРѕРІР°СЏ СЃРєРѕСЂРѕСЃС‚СЊ РјР°С…РѕРІРёРєР° СЃРєР»Р°РґС‹РІР°РµС‚СЃСЏ РёР·
 С‚РµРєСѓС‰РµР№ СЃРєРѕСЂРѕСЃС‚Рё РјР°С…РѕРІРёРєР° Рё РїСЂРёСЂР°С‰РµРЅРёСЏ СЃРєРѕСЂРѕСЃС‚Рё.
 РџСЂРёСЂР°С‰РµРЅРёРµ СЃРєРѕСЂРѕСЃС‚Рё РїСЂРѕРїРѕСЂС†РёРѕРЅР°Р»СЊРЅРѕ РѕС€РёР±РєРµ РїРѕ СѓРіР»Сѓ Рё РѕС€РёР±РєРµ РїРѕ СѓРіР»РѕРІРѕР№ СЃРєРѕСЂРѕСЃС‚Рё.
 mtr_speed - С‚РµРєСѓС‰Р°СЏ СѓРіР»РѕРІР°СЏ СЃРєРѕСЂРѕСЃС‚СЊ РјР°С…РѕРІРёРєР°, РѕР±/РјРёРЅ
 omega - С‚РµРєСѓС‰Р°СЏ СѓРіР»РѕРІР°СЏ СЃРєРѕСЂРѕСЃС‚СЊ СЃРїСѓС‚РЅРёРєР°, РіСЂР°Рґ/СЃ
 omega_goal - С†РµР»РµРІР°СЏ СѓРіР»РѕРІР°СЏ СЃРєРѕСЂРѕСЃС‚СЊ СЃРїСѓС‚РЅРёРєР°, РіСЂР°Рґ/СЃ
 mtr_new_speed - С‚СЂРµР±СѓРµРјР°СЏ СѓРіР»РѕРІР°СЏ СЃРєРѕСЂРѕСЃС‚СЊ РјР°С…РѕРІРёРєР°, РѕР±/РјРёРЅ*/
 
	mtr_new_speed = (int)(mtr_speed + kd * (omega - omega_goal));
 
	if (mtr_new_speed > mtr_max_speed)
	{
		mtr_new_speed = mtr_max_speed;
	}
	else if (mtr_new_speed < -mtr_max_speed)
	{
		mtr_new_speed = -mtr_max_speed;
	}
	return mtr_new_speed;
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
void take_photo(){
	int i;
 
		if (LSS_OK == camera_turn_on()) {		
			for (i = 1; i < 10; i++) {
				printf("Take photo #%d\n", i);
				if (camera_take_photo(i)) {
					puts("\tFail!");
				}
			}
		}
		else
			puts("\tFail!");
 
		printf("Turn-on transmitter #1\n");
		if (LSS_OK == transmitter_turn_on(1)) {
			for (i = 1; i < 10; i++) {
				printf("Transmit photo #%d\n", i);
				if (transmitter_transmit_photo(1, i)) {
					puts("\tFail!");
				}
			}
		} else {
			puts("\tFail!");
		}
		printf("Turn-off transmitter #1\n");
		if (transmitter_turn_off(1))
			puts("\tFail!");	
}
void tramsmit(){
	int i;
	printf("Turn-on transmitter #1\n");
		if (LSS_OK == transmitter_turn_on(1)) {
			for (i = 1; i < 10; i++) {
				printf("Transmit photo #%d\n", i);
				if (transmitter_transmit_photo(1, i)) {
					puts("\tFail!");
				}
			}
		} else {
			puts("\tFail!");
		}
		printf("Turn-off transmitter #1\n");
		if (transmitter_turn_off(1))
			puts("\tFail!");
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
	
	//СЂР°СЃРєСЂСѓС‚РєР° РґРѕ РѕРїСЂРµРґРµР»РµРЅРЅРѕРіРѕ РіСЂР°РґСѓСЃР°
	/*
	while(1){
		if (LSS_OK == magnetometer_request_raw(num, magx_cal,magy_cal,magz_cal	)) {
		angle=deg(magx_cal,magy_cal,magz_cal);
		printf("angle=%f\n",angle);
		}
		 else {
  			puts("Fail!");
  		}
  		mSleep(10);
	    if(angle>0 && angle<35){
			omega_goal=1.0;
			break;
		}
		hyro_state = hyro_request_raw(hyr_num, gx_raw, gy_raw, gz_raw); 	
		mtr_state = motor_request_speed(mtr_num, mtr_speed);
		if (!hyro_state){
			float gx_degs = *gx_raw * 0.00875;
			float gy_degs = *gy_raw * 0.00875;
			float gz_degs = *gz_raw * 0.00875;
			omega = gz_degs;
		}
		int mtr_new_speed;
		if (!mtr_state)	{
			int16_t mtr_speed=0;
			motor_request_speed(mtr_num, &mtr_speed);	
			mtr_new_speed = motor_new_speed_PD(mtr_speed,omega,omega_goal);
			motor_set_speed(mtr_num, mtr_new_speed, &omega);
		}	
	}
	*/
	printf("Start _stab");
	for(i = 0; i < 1000; i++){
		hyro_state = hyro_request_raw(hyr_num, gx_raw, gy_raw, gz_raw); 	
		mtr_state = motor_request_speed(mtr_num, mtr_speed);
		if (!hyro_state){
			float gx_degs = *gx_raw * 0.00875;
			float gy_degs = *gy_raw * 0.00875;
			float gz_degs = *gz_raw * 0.00875;
			omega = gz_degs;
			//printf(" omega = %f  \n", omega);
		}
		int mtr_new_speed;
		if (!mtr_state)	{
			int16_t mtr_speed=0;
			motor_request_speed(mtr_num, &mtr_speed);
			mtr_new_speed = motor_new_speed_PD(mtr_speed,omega,omega_goal);
			motor_set_speed(mtr_num, mtr_new_speed, &omega);
		}
		
	}
	//take_photo();
	//tramsmit();
	
	//Р”Р»СЏ СЃС‚Р°Р±РёР»РёР·Р°С†РёРё
	/*
	
	for(i = 0; i < 1000; i++){
		
		if (LSS_OK == magnetometer_request_raw(num, magx_cal,magy_cal,magz_cal	)) {
		angle=deg(magx_cal,magy_cal,magz_cal);
		printf("angle=%f\n",angle);
		}
		 else {
  			puts("Fail!");
  		}
  		mSleep(10);
	    if(angle>0 && angle<35){
			omega_goal=0.0;
		}
		else{
			omega_goal=-10.0;
		}
		*/
		/*if (LSS_OK == sun_sensor_request_raw(num, &value1, &value2)) {
			sr= sqrt((value1*value1)+(value2*value2));
  			printf("sr=%d\n",sr);r
			if(sr>300){
				angle=0;
				omega_goal=10.0;
			}
  		} else {
  			puts("Fail!");
  		}
		*/
		/*printf("i = %d\n", i);
		hyro_state = hyro_request_raw(hyr_num, gx_raw, gy_raw, gz_raw); 	
		mtr_state = motor_request_speed(mtr_num, mtr_speed);
		if (!hyro_state){
			float gx_degs = *gx_raw * 0.00875;
			float gy_degs = *gy_raw * 0.00875;
			float gz_degs = *gz_raw * 0.00875;
			omega = gz_degs;
			printf(" omega = %f  ", omega);
		    //printf(" gy_degs = %f  ", gy_degs);
			//printf(" gz_degs = %f\n", gz_degs);
		}
		int mtr_new_speed;
		if (!mtr_state)	{
			int16_t mtr_speed=0;
			motor_request_speed(mtr_num, &mtr_speed);
			//printf("Motor_speed: %d\n", mtr_speed);	
			mtr_new_speed = motor_new_speed_PD(mtr_speed,omega,omega_goal);
			motor_set_speed(mtr_num, mtr_new_speed, &omega);
		}
	}*/
	Sleep(time_step);
	switch_off_all();
	sun_sensor_turn_off(num);
	magnetometer_turn_off(num);  
}

 
