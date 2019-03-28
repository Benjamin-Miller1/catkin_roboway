#ifndef _MotorDriver_H_
#define _MotorDriver_H_

#ifndef __cpluscplus
extern "C"{
#endif
/*以下为控制函数，address为想要驱动的驱动器地址, 返回8个字节字节流的首地址，函数不能重入*/
unsigned char *PRIM_Enable(unsigned char address);
unsigned char *PRIM_Disable(unsigned char address);
unsigned char *PRIM_SetVelocity(unsigned char address, int velocity);
unsigned char *PRIM_Stop(unsigned char address);
unsigned char *PRIM_GetActVelocity(unsigned char address);
unsigned char *PRIM_GetActualPos(unsigned char address);
unsigned char *PRIM_ClearError(unsigned char address);
/*以下是解析函数， ptr为外部获取到can数据域的数据（8字节）*/
int PRIM_ExplainActVelocity(unsigned char address, unsigned char *ptr, double *result);
int PRIM_ExplainActualPos(unsigned char address, unsigned char *ptr, int *result);

#ifndef __cpluscplus
}
#endif

#endif