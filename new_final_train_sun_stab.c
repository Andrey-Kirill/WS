#include"libschsat.h"
void control(void){
	int i = 0;
	hyro_turn_on(1);
	float omega = 0;
	float omega_goal = 10;
	int new_speed = 0;
	motor_turn_on(1);
	float angle = 0;

	sun_sensor_turn_on(1);
	Sleep(1);
	while(1){
		uint16_t value1;
		uint16_t value2;
		if(LSS_OK == sun_sensor_request_raw(1,&value1,&value2)){
			printf("%d %d\n",value1,value2);
		}
		if((value1 > 1111 && value1 <1200 ) && (value2 > 1040 && value2<1110)){
			omega_goal = 1.0;
			break;
		}
		
		int16_t x,y,z;
		int16_t speed;
		if(LSS_OK == hyro_request_raw(1,&x,&y,&z)){
		    omega = x*0.00875;
		}
		motor_request_speed(1,&speed);
		new_speed = (speed + 200*(omega-omega_goal));
		if(LSS_OK == motor_set_speed(1,new_speed,&omega)){
			printf("speed = %f\n",omega);
		}
	
		mSleep(50);
		
	}
	for(i = 0;i<700;i++){
		int16_t x,y,z;
		int16_t speed;
		if(LSS_OK == hyro_request_raw(1,&x,&y,&z)){
		    omega = x*0.00875;
		}
		
		motor_request_speed(1,&speed);
		new_speed = (speed + 200*(omega-omega_goal));
		if(LSS_OK == motor_set_speed(1,new_speed,&omega)){
			printf("speed = %f\n",omega);
		}
		if(i == 0){
			arduino_send(0,1,NULL,NULL,100);
			
		}
		mSleep(50);
		
		
	}
	omega_goal = -10;
	
	for(i = 0;i<155;i++){
		int16_t x,y,z;
		int16_t speed;
		if(LSS_OK == hyro_request_raw(1,&x,&y,&z)){
		    omega = x*0.00875;
		}
		angle += omega*0.05;
		printf("angle = %f\n",angle);
		motor_request_speed(1,&speed);
		new_speed = (speed + 200*(omega-omega_goal));
		if(LSS_OK == motor_set_speed(1,new_speed,&omega)){
			printf("speed = %f\n",omega);
		}
		mSleep(50);
	}
	omega_goal = 0;
	for(i = 0;i<150;i++){
		int16_t x,y,z;
		int16_t speed;
		if(LSS_OK == hyro_request_raw(1,&x,&y,&z)){
		    omega = x*0.00875;
		}
		motor_request_speed(1,&speed);
		new_speed = (speed + 200*(omega-omega_goal));
		if(LSS_OK == motor_set_speed(1,new_speed,&omega)){
			printf("speed = %f\n",omega);
		}
		mSleep(50);
	}
	
}