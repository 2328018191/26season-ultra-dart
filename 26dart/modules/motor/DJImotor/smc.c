#include "main.h"
#include "smc.h"
#include "math.h"
void SMC_Init(SMC_Config *config,float K,float alpha,float beta,float lambda,float dt,SMC *SMC_control)
{
	config->K = K;
	config->alpha = alpha;
	config->beta = beta;
	config->lambda = lambda;
	config->dt = dt;
	SMC_control->actual_speed = 0;
	SMC_control->target_speed = 0;
	SMC_control->error = 0;
	SMC_control->output = 0;
	SMC_control->error_integral = 0;
}
float SMC_Calculate(SMC *SMC_control,SMC_Config *config,float set,float now)
{
	 float s, u_eq, u_sw, u_total;
	SMC_control->actual_speed = now;
	SMC_control->target_speed = set;
	SMC_control->error = set - now;
	s = SMC_control->error + SMC_control->config.lambda * SMC_control->error_integral;//构造滑模面
	u_eq = config->K*s;
	u_sw = config->beta * (s / (fabs(s) + config->alpha));
	u_total = u_eq + u_sw;
	if(u_total > 16000)
		u_total = 16000;
	else if(u_total < -16000)
		u_total = -16000;
	SMC_control->output = u_total;
	SMC_control->error_integral += SMC_control->error * config->dt;
	return SMC_control->output;
}