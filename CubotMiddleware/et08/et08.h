#include "stm32h7xx_hal.h"
#include "usart.h"

#ifndef ET08_H
#define ET08_H

#include "driver_usart.h"



#define ET08_rxBufferLengh 25	 //< et08接收缓存区数据长度


/**
  * @brief  接收机接收数据类型, 包含rc遥控器数据、mouse鼠标数据和 keyflag 按键数据
  */
typedef struct { 
 struct {       
	 uint16_t ch0;       
	 uint16_t ch1;       
	 uint16_t ch2;      
	 uint16_t ch3;  
     uint8_t  sA; 	 
	 uint8_t  s1; 
	 uint8_t  s2;
     uint8_t  sD;	 
	 uint8_t  s1_last; 
	 uint8_t  s2_last; 
	}rc; 
 	uint8_t  isUnpackaging;  	 //< 解算状态标志位，解算过程中不读取数据
	uint8_t  isOnline;
	uint32_t onlineCheckCnt;
}RC_Ctrl_ET; 
 

/**
  * @brief  dr16数据拆分解算函数
	* @param[in] rc_ctrl  	为该结构体赋值
	* @param[in] recBuffer  串口接收缓存区
	* @param[in] len        缓存区数组数据长度，未使用，仅用来匹配函数类型
  */


/**
  * @brief  dr16数据拆分解算函数
	* @param[in] recBuffer  串口接收缓存区
	* @param[in] len        缓存区数组数据长度，未使用，仅用来匹配函数类型
  * @retval    RC_Ctl     接收机数据类型，储存杆量和按键信息
  */
uint8_t ET08_callback(uint8_t * recBuffer, uint16_t len);

/**
  * @brief  初始化接收机数据类型的数据，将杆量和按键信息归零
	* @param[in] RC_Ctl  接收机数据类型首地址，储存杆量和按键信息
  * @retval    RC_Ctl  接收机数据类型首地址，储存杆量和按键信息
  */
void ET08Init(RC_Ctrl_ET* RC_Ctl);

void ET08_online_protection();



void ET08_DataUnpack(RC_Ctrl_ET* rc_ctrl, uint8_t * recBuffer, uint16_t len );


extern RC_Ctrl_ET rc_Ctrl_et;


#endif

