#include"libschsat.h"
int16_t mgx_cal = 0;
int16_t mgy_cal = 0;
int16_t mgz_cal = 0;
int16_t *magx_cal = &mgx_cal;
int16_t *magy_cal = &mgy_cal;
int16_t *magz_cal = &mgz_cal;
float mag_alpha;
float degs(int16_t *magx,int16_t *magy,int16_t *magz){
	float magx_cal = 0.0*(*magx + 0.0)+0.0*(*magy + 0.0)+0.0*(*magz + 0.0);
	float magy_cal = 0.0*(*magx + 0.0)+0.0*(*magy + 0.0)+0.0*(*magz + 0.0);
	float magz_cal = 0.0*(*magx + 0.0)+0.0*(*magy + 0.0)+0.0*(*magz + 0.0);
    *magx = (int16_t)magx_cal;
	*magy = (int16_t)magy_cal;
	*magz = (int16_t)magz_cal;
	double alpha_rad = atan2(mgx_cal,mgz_cal);
	  if(alpha_rad<0){
		  alpha_rad += 2*M_PI;
	  }
	  mag_alpha = alpha_rad/180*M_PI;
	  return mag_alpha;
}