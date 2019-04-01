#include "motor_control/motor_driver.h"
#include <stdio.h>
#include <stdlib.h>

#ifndef __cpluscplus
extern "C"{
#endif

unsigned short getCRC16(unsigned char *ptr,unsigned char len)
{
    unsigned char i;
    unsigned short crc = 0xFFFF;
    if(len==0)
    {
        len = 1;
    }
    while(len--)
    {
        crc ^= *ptr;
        for(i=0; i<8; i++)
        {
            if(crc&1)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
        ptr++;
    }
    return(crc);
}
/*2字节类型数据转为字节流：高位在第一个字节，低位在第二个字节*/
void PRIM_ushortTostream(unsigned short value, unsigned char *ptr)
{
    *ptr = value & 0xFF;
    *(ptr + 1) = (value >> 8) & 0xFF;
}

unsigned short PRIM_streamToushort(unsigned char *ptr)
{
    unsigned short value = 0;
    value = ptr[1];
    value <<= 8;
    value += ptr[0];
    return value;
}
/*4字节类型数据转为字节流：最高位在第一个字节，次高位在第二个字节，次低位在第三个字节，最低位在第四个字节*/
void PRIM_intTostream(int value, unsigned char *ptr)
{
    *(ptr + 3) = value & 0xFF;
    *(ptr + 2) = (value >> 8) & 0xFF;
    *(ptr + 1) = (value >> 16) & 0xFF;
    *ptr       = (value >> 24) & 0xFF;
}

int PRIM_streamToint(unsigned char *ptr)
{
    int value = 0;
    value = ptr[0];
    value <<= 8;
    value += ptr[1];
    value <<= 8;
    value += ptr[2];
    value <<= 8;
    value += ptr[3];
    return value;
}

/*用于电机使能*/
unsigned char *PRIM_Enable(unsigned char address)
{
    static unsigned char data[8] = {0x00, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char *pdata = data;
    data[0] = address;
    unsigned short crc_result = getCRC16(data, 6);
    PRIM_ushortTostream(crc_result, pdata + 6);
    return data;
}
/*用于电机释放*/
unsigned char *PRIM_Disable(unsigned char address)
{
    static unsigned char data[8] = {0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char *pdata = data;
    data[0] = address;
    unsigned short crc_result= getCRC16(data, 6);
    PRIM_ushortTostream(crc_result, pdata + 6);
    return data;
}
/*停止运动*/
unsigned char *PRIM_Stop(unsigned char address)
{
    static unsigned char data[8] = {0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char *pdata = data;
    data[0] = address;
    unsigned short crc_result= getCRC16(data, 6);
    PRIM_ushortTostream(crc_result, pdata + 6);
    return data;
}
/*用于设置速度控制模式速度的设定值*/
unsigned char *PRIM_SetVelocity(unsigned char address, int velocity)
{
    static unsigned char data[8] = {0x00, 0x6F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char *pdata = data;
    data[0] = address;
    PRIM_intTostream(velocity * 10, data + 2);//can协议中的单位为0.1RPM
    unsigned short crc_result= getCRC16(data, 6);
    PRIM_ushortTostream(crc_result, pdata + 6);
    return data;
}

/*用于获取电机速度信息*/
unsigned char *PRIM_GetActVelocity(unsigned char address)
{
    static unsigned char data[8] = {0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char *pdata = data;
    data[0] = address;
    unsigned short crc_result= getCRC16(data, 6);
    PRIM_ushortTostream(crc_result, pdata + 6);
    return data;
}
/*用于获取电机位置实际值*/
unsigned char *PRIM_GetActualPos(unsigned char address)
{
    static unsigned char data[8] = {0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char *pdata = data;
    data[0] = address;
    unsigned short crc_result= getCRC16(data, 6);
    PRIM_ushortTostream(crc_result, pdata + 6);
    return data;
}

int PRIM_ExplainActVelocity(unsigned char address, unsigned char *ptr, double *result)
{
    if(ptr[0] != address || ptr[1] != 0x3F)
        return -1;
    *result = PRIM_streamToint(ptr+2) / 10.0;//can协议中的单位为0.1RPM
    return 0 ;
}
int PRIM_ExplainActualPos(unsigned char address, unsigned char *ptr, int *result)
{
    if(ptr[0] != address || ptr[1] != 05)
        return -1;
    *result = PRIM_streamToint(ptr+2);
    return 0;
}
/*用于获取电机位置实际值*/
unsigned char *PRIM_GetError(unsigned char address)
{
    static unsigned char data[8] = {0x00, 0x3D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char *pdata = data;
    data[0] = address;
    unsigned short crc_result= getCRC16(data, 6);
    PRIM_ushortTostream(crc_result, pdata + 6);
    return data;
}
int PRIM_ExplainError(unsigned char address, unsigned char *ptr, int *result)
{
    if(ptr[0] != address || ptr[1] != 0x3D)
        return -1;
    *result = PRIM_streamToint(ptr+2);
    return 0;
}
/*清除告警*/
unsigned char *PRIM_ClearError(unsigned char address)
{
    static unsigned char data[8] = {0x00, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char *pdata = data;
    data[0] = address;
    unsigned short crc_result= getCRC16(data, 6);
    PRIM_ushortTostream(crc_result, pdata + 6);
    return data;
}

#ifndef __cpluscplus
}
#endif
