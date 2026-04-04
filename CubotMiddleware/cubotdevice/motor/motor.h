#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <stm32h7xx_hal.h>
#include <driver_can.h>

#include "linux_list.h"

#define K_ECD_TO_ANGLE 0.043945f	 //< �Ƕ�ת���������̶ȵ�ϵ����360/8192
#define ECD_RANGE_FOR_3508 8191		 //< �������̶�ֵΪ0-8191
#define CURRENT_LIMIT_FOR_3508 16000 //< ���Ƶ�����ΧΪ����16384
#define ECD_RANGE_FOR_6020 8191		 //< �������̶�ֵΪ0-8191
#define CURRENT_LIMIT_FOR_6020 29000 //< ���Ƶ�����ΧΪ����30000
#define ECD_RANGE_FOR_2006 8191		 //< �������̶�ֵΪ0-8191
#define CURRENT_LIMIT_FOR_2006 10000 //< ���Ƶ�����ΧΪ����16384

typedef enum
{
	Motor3508 = 0x00U, // wu fu hao
	Motor6020 = 0x01U,
	Motor2006 = 0x02U,
	MIT = 0x03U
} MotorType;

typedef struct
{

	int16_t Ecd;		   //< ��ǰ����������ֵ
	int16_t SpeedRPM;	   //< ÿ������תȦ��
	int16_t TorqueCurrent; //< ��������
	uint8_t Temperature;   //< �¶�

	float RawEcd;		//< ԭʼ����������
	int16_t LastEcd;	//< ��һʱ�̱���������ֵ
	float Angle;		//< �����ı������Ƕ�
	int16_t AngleSpeed; //< �����ı��������ٶ�
	int32_t RoundCnt;	//< �ۼ�ת��Ȧ��
	int32_t TotalEcd;	//< �������ۼ�����ֵ
	float TotalAngle;	//< �ۼ���ת�Ƕ�

	float Target;	//< �������������
	int32_t Output; //< ������ֵ��ͨ��Ϊ�����͵�ѹ
	float CanEcd[20];
	float CanAngleSpeed[20];
	float LvboAngle;
	int16_t LvboEcd;
	int16_t LvboSpeedRPM;
	int16_t moter_id;
	struct
	{
		uint16_t Cnt;
		uint16_t FPS;
		uint8_t Status;
		uint8_t StatusCnt;
	} Online_check;
	// �������������
	float MITangle;
	float MITspeed;
	float MITtorque;
} MotorData;

typedef struct
{
	uint8_t CanNumber;	   //< �����ʹ�õ�CAN�˿ں�
	uint16_t CanId;		   //< ���ID
	uint8_t MotorType;	   //< �������
	uint16_t EcdOffset;	   //< �����ʼ���
	uint16_t EcdFullRange; //< ����������
	int16_t CurrentLimit;  //< ����ܳ��ܵ�������
} MotorParam;

static uint8_t CAN_update_data(MotorData *motor, CAN_RxBuffer rxBuffer);
typedef uint8_t (*Motor_DataUpdate)(MotorData *motor_data, CAN_RxBuffer rxBuffer);

static uint8_t CAN_fill_3508_2006_data(CAN_Object can, MotorData motor_data, uint16_t id);
typedef uint8_t (*CAN_FillMotorData)(CAN_Object can, MotorData motor_data, uint16_t id);

typedef struct
{
	MotorData Data;
	MotorParam Param;
	list_t list;
	Motor_DataUpdate MotorUpdate; //< ���µ���������ݵĺ���ָ��
	CAN_FillMotorData FillMotorData;
} Motor;

static void MotorEcdtoAngle(Motor *motor);
static void MotorLvboEcdtoAngle(Motor *motor);
static void MotorOutputLimit(Motor *motor);
static uint8_t CAN_fill_3508_2006_data(CAN_Object can, MotorData motor_data, uint16_t id);
static uint8_t CAN_fill_6020_data(CAN_Object can, MotorData motor_data, uint16_t id);
static uint8_t CAN_update_data(MotorData *motor, CAN_RxBuffer rxBuffer);
static void CAN_RegisteMotor(CAN_Object *canx, Motor *motor);
static void CAN_DeleteMotor(Motor *motor);
int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
static uint8_t CAN_MIT_update_data(MotorData *motor, CAN_RxBuffer rxBuffer);
void MotorInit(Motor *motor, uint16_t ecd_Offset, MotorType type, CanNumber canx, uint16_t id);
Motor *MotorFind(uint16_t canid, CAN_Object canx);
void MotorRxCallback(CAN_Object canx, CAN_RxBuffer rxBuffer);
uint16_t MotorReturnID(Motor motor);
void MotorFillData(Motor *motor, int32_t output);
uint16_t MotorCanOutput(CAN_Object can, int16_t IDforTxBuffer);

#endif
