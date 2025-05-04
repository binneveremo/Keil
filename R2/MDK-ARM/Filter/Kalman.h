#ifndef __KALMAN_H__
#define __KALMAN_H__
struct Kalman_filter
{
	float C;						/*最优估计协方差矩阵C(k|k)*/ 
	float Q;						/*过程噪声协方差*/
	float R;						/*量测噪声协方差*/	
	float C_last;				    /*上次预测过程协方差矩阵 C(k|k-1)*/
	float X;				    /*系统状态预测矩阵，列矩阵*/
	float X_last;				    /*系统状态预测矩阵，列矩阵*/
	float K;						/*卡尔曼增益，列矩阵*/  
	float Input;				    /*量测值，即Z(k)*/
};
struct EKF{
	float r;
	float q;
	float a_hat_prior;
	float p_prior;
	float y;
	float k;
	float a_hat;
	float p;
};
float Kalman_Filter(struct Kalman_filter* K_Flt, float Input);
float EKF_Filter(struct EKF * ekf, float input ,float gain);
#endif
