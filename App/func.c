/*
版 本 号：v1.8
创 建 者：纸飞机工作室
修 改 者：
功能描述：纸飞机四轴程序，勿用于商业用途

 */



#include "stm32f10x.h"
#include <stdio.h>
#include <stdio.h>                    /* standard I/O .h-file                 */
#include <ctype.h>                    /* character functions                  */
#include <string.h>                   /* string and memory functions          */
#include "USART.h"
#include "adc_dma.h"
#include "func.h"
#include "imu.h"
#include "controller.h"
#include "eeprom.h"
#include "filter.h"

uint16_t VirtAddVarTab[NumbOfVar] = {0xAF00, 0xAF01, 0xAF02, 0xAF03, 0xAF04, 0xAF05, 0xAF06, 0xAF07, 0xAF08, 
									 0xAF09, 0xAF0A, 0xAF0B, 0xAF0C, 0xAF0D, 0xAF0E, 0xAF0F, 0xAF10, 0xAF11, 
									 0xAF12, 0xAF13, 0xAF14, 0xAF15, 0xAF16, 0xAF17};
void EE_Write_ACC_GYRO_Offset(void)
{
	EE_WriteVariable(VirtAddVarTab[EE_ACC_X_OFFSET], mpu_info.offset_acc.x);
	EE_WriteVariable(VirtAddVarTab[EE_ACC_Y_OFFSET], mpu_info.offset_acc.y);
	EE_WriteVariable(VirtAddVarTab[EE_ACC_Z_OFFSET], mpu_info.offset_acc.z);
	EE_WriteVariable(VirtAddVarTab[EE_GYRO_X_OFFSET], mpu_info.offset_gyro.x);
	EE_WriteVariable(VirtAddVarTab[EE_GYRO_Y_OFFSET], mpu_info.offset_gyro.y);
	EE_WriteVariable(VirtAddVarTab[EE_GYRO_Z_OFFSET], mpu_info.offset_gyro.z);
}
void EE_Read_ACC_GYRO_Offset(void)
{
	EE_ReadVariable(VirtAddVarTab[EE_ACC_X_OFFSET], (u16 *)(&mpu_info.offset_acc.x));
	EE_ReadVariable(VirtAddVarTab[EE_ACC_Y_OFFSET], (u16 *)(&mpu_info.offset_acc.y));
	EE_ReadVariable(VirtAddVarTab[EE_ACC_Z_OFFSET], (u16 *)(&mpu_info.offset_acc.z));
	EE_ReadVariable(VirtAddVarTab[EE_GYRO_X_OFFSET], (u16 *)(&mpu_info.offset_gyro.x));
	EE_ReadVariable(VirtAddVarTab[EE_GYRO_Y_OFFSET], (u16 *)(&mpu_info.offset_gyro.y));
	EE_ReadVariable(VirtAddVarTab[EE_GYRO_Z_OFFSET], (u16 *)(&mpu_info.offset_gyro.z));
}
																		 																		 
void EE_Write_PID(void)	//
{
	u16 _temp;
	_temp = pidRoll.kp * 100;
	EE_WriteVariable(VirtAddVarTab[EE_PID_ROL_P],_temp);
	_temp = pidRoll.ki * 100;
	EE_WriteVariable(VirtAddVarTab[EE_PID_ROL_I],_temp);
	_temp = pidRoll.kd * 100;
	EE_WriteVariable(VirtAddVarTab[EE_PID_ROL_D],_temp);
	_temp = pidPitch.kp * 100;
	EE_WriteVariable(VirtAddVarTab[EE_PID_PIT_P],_temp);
	_temp = pidPitch.ki * 100;
	EE_WriteVariable(VirtAddVarTab[EE_PID_PIT_I],_temp);
	_temp = pidPitch.kd * 100;
	EE_WriteVariable(VirtAddVarTab[EE_PID_PIT_D],_temp);
	_temp = pidYaw.kp * 100;
	EE_WriteVariable(VirtAddVarTab[EE_PID_YAW_P],_temp);
	_temp = pidYaw.ki * 100;
	EE_WriteVariable(VirtAddVarTab[EE_PID_YAW_I],_temp);
	_temp = pidYaw.kd * 100;
	EE_WriteVariable(VirtAddVarTab[EE_PID_YAW_D],_temp);
}
void EE_Read_PID(void)
{
	u16 _temp;
	EE_ReadVariable(VirtAddVarTab[EE_PID_ROL_P],&_temp);
	pidRoll.kp = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_ROL_I],&_temp);
	pidRoll.ki = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_ROL_D],&_temp);
	pidRoll.kd = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_PIT_P],&_temp);
	pidPitch.kp = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_PIT_I],&_temp);
	pidPitch.ki = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_PIT_D],&_temp);
	pidPitch.kd = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_YAW_P],&_temp);
	pidYaw.kp = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_YAW_I],&_temp);
	pidYaw.ki = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_PID_YAW_D],&_temp);
	pidYaw.kd = (float)_temp / 100;
}

void EE_Read_Rate_PID(void)
{
	u16 _temp;
	EE_ReadVariable(VirtAddVarTab[EE_RATE_PID_ROL_P],&_temp);
	pidRollRate.kp = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_RATE_PID_ROL_I],&_temp);
	pidRollRate.ki = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_RATE_PID_ROL_D],&_temp);
	pidRollRate.kd = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_RATE_PID_PIT_P],&_temp);
	pidPitchRate.kp = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_RATE_PID_PIT_I],&_temp);
	pidPitchRate.ki = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_RATE_PID_PIT_D],&_temp);
	pidPitchRate.kd = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_RATE_PID_YAW_P],&_temp);
	pidYawRate.kp = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_RATE_PID_YAW_I],&_temp);
	pidYawRate.ki = (float)_temp / 100;
	EE_ReadVariable(VirtAddVarTab[EE_RATE_PID_YAW_D],&_temp);
	pidYawRate.kd = (float)_temp / 100;
}

void EE_Write_Rate_PID(void)	//
{
	u16 _temp;
	_temp = pidRollRate.kp * 100;
	EE_WriteVariable(VirtAddVarTab[EE_RATE_PID_ROL_P],_temp);
	_temp = pidRollRate.ki * 100;
	EE_WriteVariable(VirtAddVarTab[EE_RATE_PID_ROL_I],_temp);
	_temp = pidRollRate.kd * 100;
	EE_WriteVariable(VirtAddVarTab[EE_RATE_PID_ROL_D],_temp);
	_temp = pidPitchRate.kp * 100;
	EE_WriteVariable(VirtAddVarTab[EE_RATE_PID_PIT_P],_temp);
	_temp = pidPitchRate.ki * 100;
	EE_WriteVariable(VirtAddVarTab[EE_RATE_PID_PIT_I],_temp);
	_temp = pidPitchRate.kd * 100;
	EE_WriteVariable(VirtAddVarTab[EE_RATE_PID_PIT_D],_temp);
	_temp = pidYawRate.kp * 100;
	EE_WriteVariable(VirtAddVarTab[EE_RATE_PID_YAW_P],_temp);
	_temp = pidYawRate.ki * 100;
	EE_WriteVariable(VirtAddVarTab[EE_RATE_PID_YAW_I],_temp);
	_temp = pidYawRate.kd * 100;
	EE_WriteVariable(VirtAddVarTab[EE_RATE_PID_YAW_D],_temp);
}
																		 
void SetAccGyroOffset(u8 *buf)
{
    mpu_info.offset_acc.x = (short)((buf[3]<<8)|buf[4]);
    mpu_info.offset_acc.y = (short)((buf[5]<<8)|buf[6]);
    mpu_info.offset_acc.z = (short)((buf[7]<<8)|buf[8]);
    mpu_info.offset_gyro.x = (short)((buf[9]<<8)|buf[10]);
    mpu_info.offset_gyro.y = (short)((buf[11]<<8)|buf[12]);
    mpu_info.offset_gyro.z = (short)((buf[13]<<8)|buf[14]);
	EE_Write_ACC_GYRO_Offset();

}
void GetAccGyroOffset(u8 *buf)
{
	u16 tmp16;
	buf[0] = 0xAA;
	buf[1] = 0x21;
	buf[2] = 12;
	tmp16 = (u16)mpu_info.offset_acc.x;
	buf[3] = BYTE1(tmp16);
	buf[4] = BYTE0(tmp16);
	tmp16 = (u16)mpu_info.offset_acc.y;
	buf[5] = BYTE1(tmp16);
	buf[6] = BYTE0(tmp16);
	tmp16 = (u16)mpu_info.offset_acc.z;
	buf[7] = BYTE1(tmp16);
	buf[8] = BYTE0(tmp16);
	tmp16 = (u16)mpu_info.offset_gyro.x;
	buf[9] = BYTE1(tmp16);
	buf[10] = BYTE0(tmp16);
	tmp16 = (u16)mpu_info.offset_gyro.y;
	buf[11] = BYTE1(tmp16);
	buf[12] = BYTE0(tmp16);
	tmp16 = (u16)mpu_info.offset_gyro.z;
	buf[13] = BYTE1(tmp16);
	buf[14] = BYTE0(tmp16);
 	buf[15] = CheckSum(buf,buf[2]+3);
}
																		 
u8 CheckSum(u8 *buf,u8 len)
{
	u8 i,sum=0;
	for(i=0;i<len;i++)
		sum+=buf[i];
	return sum;
}

void GetState(u8 *buf)
{
	u16 tmp16;
	
	buf[0] = 0xAA;
	buf[1] = 0x30;
	buf[2] = 28;
	tmp16 = (short)(angle.y*100);
	buf[3] = BYTE1(tmp16);
	buf[4] = BYTE0(tmp16);
	tmp16 = (short)(angle.x*100);
	buf[5] = BYTE1(tmp16);
	buf[6] = BYTE0(tmp16);
	tmp16 = (short)(angle.z*100);
	buf[7] = BYTE1(tmp16);
	buf[8] = BYTE0(tmp16);
	
	tmp16 = (short)(mpu_info.gyro.x*Gyro2Degree);
	buf[9] = BYTE1(tmp16);
	buf[10] = BYTE0(tmp16);
	tmp16 = (short)(mpu_info.gyro.y*Gyro2Degree);
	buf[11] = BYTE1(tmp16);
	buf[12] = BYTE0(tmp16);
	tmp16 = (short)(mpu_info.gyro.z*Gyro2Degree);
	buf[13] = BYTE1(tmp16);
	buf[14] = BYTE0(tmp16);

	tmp16 = (short)(rollRateDesired*10);
	buf[15] = BYTE1(tmp16);
	buf[16] = BYTE0(tmp16);
	tmp16 = (short)(pitchRateDesired*10);
	buf[17] = BYTE1(tmp16);
	buf[18] = BYTE0(tmp16);
	tmp16 = (short)(yawRateDesired*10);
	buf[19] = BYTE1(tmp16);
	buf[20] = BYTE0(tmp16);

	tmp16 = rollOutput;
	buf[21] = BYTE1(tmp16);
	buf[22] = BYTE0(tmp16);
	tmp16 = pitchOutput;
	buf[23] = BYTE1(tmp16);
	buf[24] = BYTE0(tmp16);
	tmp16 = yawOutput;
	buf[25] = BYTE1(tmp16);
	buf[26] = BYTE0(tmp16);
	tmp16 = Motor_Thr;
	buf[27] = BYTE1(tmp16);
	buf[28] = BYTE0(tmp16);
/*
	buf[21] = MOTOR1/5;
	buf[22] = MOTOR2/5;
	buf[23] = MOTOR3/5;
	buf[24] = MOTOR4/5;
	buf[25] = BYTE3(freqMeas);
	buf[26] = BYTE2(freqMeas);
	buf[27] = BYTE1(freqMeas);
	buf[28] = BYTE0(freqMeas);
*/
	tmp16 = GetADCValue();
	tmp16 = (tmp16*5*110/4096 +0.5);
	buf[30] = tmp16-250;

	buf[31] = CheckSum(buf,buf[2]+3);
}

