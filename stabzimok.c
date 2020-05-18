#include "libschsat.h"
const float kd = 290.0;
const float time_step = 0.1;
uint16_t  sr=0;
float omega_goal = 10.0;// если надо раскрутку то 10 или -10,если стабилизацию то 0.0
int16_t magx, magy, magz;
double angle=0;
// Номер маховика
const int mtr_num = 1;
const int num=1;
// Максимально допустимая скорость маховика, об/мин
const int mtr_max_speed = 3000;
// Номер ДУС (датчика угловой скорости)
const  uint16_t hyr_num = 1;
int16_t mtr_new_speed;
int16_t mgx_cal=0;
	int16_t mgy_cal=0;
	int16_t mgz_cal=0;
	int16_t *magx_cal = &mgx_cal;
	int16_t *magy_cal = &mgy_cal;
	int16_t *magz_cal = &mgz_cal;
float mag_alpha;
//калибровка
float deg(int16_t *magx, int16_t *magy, int16_t *magz ){
	float magx_cal;
	magx_cal = 1.307189*(*magx + 65.572081) + -0.035110*(*magy + 51.484602) + 0.117041*(*magz + 8.965048);
	float magy_cal;
	magy_cal = -0.035110*(*magx + 65.572081) + 1.466554*(*magy + 51.484602) + 0.049757*(*magz + 8.965048);
	float magz_cal;
	magz_cal = 0.117041*(*magx + 65.572081) + 0.049757*(*magy + 51.484602) + 1.200149*(*magz + 8.965048);
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
 /* Функция для определения новой скорости маховика.
 Новая скорость маховика складывается из
 текущей скорости маховика и приращения скорости.
 Приращение скорости пропорционально ошибке по углу и ошибке по угловой скорости.
 mtr_speed - текущая угловая скорость маховика, об/мин
 omega - текущая угловая скорость спутника, град/с
 omega_goal - целевая угловая скорость спутника, град/с
 mtr_new_speed - требуемая угловая скорость маховика, об/мин*/
 
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
/*Функция включает все приборы,которые будут использоваться в основной программе.*/
	printf("Enable motor №%d\n", mtr_num); 
	motor_turn_on(mtr_num);
	Sleep(1);
	printf("Enable angular velocity sensor №%d\n", hyr_num); 
	hyro_turn_on(hyr_num);
	Sleep(1);
	magnetometer_turn_on(num);  
	Sleep(1);
 }
 
void switch_off_all(void){
/* Функция отключает все приборы,которые будут использоваться в основной программе.*/
	printf("Finishing...");
	int16_t new_speed = mtr_new_speed; //
	printf("\nDisable angular velocity sensor №%d\n", hyr_num);
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
	// Инициализируем статус маховика
	int16_t mtr_state;
	// Инициализируем статус ДУС	
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
	
	//раскрутка до определенного градуса
	
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
	take_photo();
	//tramsmit();
	
	//Для стабилизации
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

 
