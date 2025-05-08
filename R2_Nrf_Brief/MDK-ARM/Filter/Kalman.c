#include "Kalman.h"


float Kalman_Filter(struct Kalman_filter* K_Flt, float Input)
{
    /*量测更新，3组方程*/
    K_Flt->Input = Input;
	  //if (isnan(K_Flt->X_last)) 
     //   K_Flt->X_last = 0.0; 
    K_Flt->K = (K_Flt->C_last) / (K_Flt->C_last + K_Flt->R);
    K_Flt->X  = K_Flt->X_last + K_Flt->K * (K_Flt->Input - K_Flt->X_last);
    K_Flt->C =  (1 - K_Flt->K) * (K_Flt->C_last);

    /*时间更新，2组方程*/
    K_Flt->X_last = K_Flt->X;
    K_Flt->C_last = K_Flt->C + K_Flt->Q;

    return K_Flt->X;
}
float EKF_Filter(struct EKF * ekf, float input ,float gain){
	ekf->a_hat_prior = ekf->a_hat + gain;
	if (isnan(ekf->a_hat_prior)) 
      ekf->a_hat_prior = 0.0; 
	ekf->p_prior = ekf->p + ekf->q;
	ekf->y = input - ekf->a_hat_prior;
	ekf->k = ekf->p_prior / (ekf->p_prior + ekf->r);
	ekf->a_hat = ekf->a_hat_prior + ekf->k * ekf->y;
	ekf->p = (1 - ekf->k)* ekf->p_prior;
	return ekf->a_hat;
}

