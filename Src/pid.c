
/**
  ******************************************************************************
  * @file			pid.c
  * @version		V1.0.0
  * @date			2016��11��11��17:21:36
  * @brief   		����PID�� ����/����ϰ���Խ�get/measure/real/fdb,
						  ��������һ���set/target/ref
  *******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "sys.h"
#include <math.h>
//#include "cmsis_os.h"



void abs_limit(float *a, float ABS_MAX)
{
	if (*a > ABS_MAX)
		*a = ABS_MAX;
	if (*a < -ABS_MAX)
		*a = -ABS_MAX;
}

/*������ʼ��--------------------------------------------------------------*/
static void pid_param_init(
	PID_t *pid,
	uint32_t mode,
	uint32_t maxout,
	uint32_t intergral_limit,
	float kp,
	float ki,
	float kd)
{

	pid->IntegralLimit = intergral_limit;
	pid->MaxOutput = maxout;
	pid->pid_mode = mode;

	pid->p = kp;
	pid->i = ki;
	pid->d = kd;
}

/**
    *@bref. calculate delta PID and position PID
    *@param[in] set�� target
    *@param[in] real	measure
    */
float PID_Calc(PID_t *pid, float get, float set)
{
	pid->get[NOW] = get;
	pid->set[NOW] = set;
	pid->err[NOW] = set - get; //set - measure
	if (pid->max_err != 0 && ABS(pid->err[NOW]) > pid->max_err)
		return 0;
	if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
		return 0;

	if (pid->pid_mode == POSITION_PID) //λ��ʽp
	{
		pid->pout = pid->p * pid->err[NOW];
		pid->iout += pid->i * pid->err[NOW];
		pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);
		abs_limit(&(pid->iout), pid->IntegralLimit);  //�����޷�
		pid->pos_out = pid->pout + pid->iout + pid->dout;
		abs_limit(&(pid->pos_out), pid->MaxOutput);   //����޷�
		pid->last_pos_out = pid->pos_out; //update last time
	}
	else if (pid->pid_mode == DELTA_PID) //����ʽP
	{
		pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
		pid->iout = pid->i * pid->err[NOW];
		pid->dout = pid->d * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);

		abs_limit(&(pid->iout), pid->IntegralLimit);
		pid->delta_u = pid->pout + pid->iout + pid->dout;
		pid->delta_out = pid->last_delta_out + pid->delta_u;
		abs_limit(&(pid->delta_out), pid->MaxOutput);
		pid->last_delta_out = pid->delta_out; //update last time
	}

	pid->err[LLAST] = pid->err[LAST];
	pid->err[LAST] = pid->err[NOW];
	pid->get[LLAST] = pid->get[LAST];
	pid->get[LAST] = pid->get[NOW];
	pid->set[LLAST] = pid->set[LAST];
	pid->set[LAST] = pid->set[NOW];
	return pid->pid_mode == POSITION_PID ? pid->pos_out : pid->delta_out;
}

/*pid�����ʼ��-----------------------------------------------------------------*/
void PID_Init(
	PID_t *pid,
	uint32_t mode,
	uint32_t maxout,
	uint32_t intergral_limit,

	float kp,
	float ki,
	float kd)
{
	/*init function pointer*/
	pid->f_param_init = pid_param_init;

	/*init pid param */
	pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
}
