#ifndef IMU_H_
#define IMU_H_

#include "main.h"
#include "struct_typedef.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "pid.h"

// 定义常量
#define INS_TASK_INIT_TIME 10         // IMU任务初始化时间
#define INS_GYRO_OFFSET_SCALE 0.0003f // 陀螺仪偏移量系数

// 欧拉角偏移地址
#define INS_YAW_ADDRESS_OFFSET    2
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   0

// 陀螺仪数据偏移地址
#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

// 加速度计数据偏移地址
#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

// 磁力计数据偏移地址
#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2

extern TIM_HandleTypeDef htim3;

// IMU控制结构体
typedef struct {
	fp32 angle[3]; // 欧拉角数据
	fp32 gyroscope[3]; // 陀螺仪数据（单位：°/s）
	fp32 gyro_correct[3]; // 陀螺仪校准数据
	fp32 accelerometer[3]; // 加速度计数据（单位：g）
	fp32 magnetometer[3]; // 磁力计数据（单位：uT）

	pid_type_def temp_pid; // 温度PID
	float temperature; // 温度数据

	SemaphoreHandle_t xAccSemaphore; // 加速度计信号量
	SemaphoreHandle_t xGyroSemaphore; // 陀螺仪信号量
	SemaphoreHandle_t xMagSemaphore; // 磁力计信号量

	TickType_t last_update_time; // 上次更新时间戳
	TickType_t last_second_time; // 上次统计时间戳
	fp32 delta_time; // 间隔时间

	uint32_t solves_per_second; // 每秒解算次数
	uint32_t solve_count; // 当前解算次数

	uint32_t time_count; // 时间计数
	uint32_t step_status; // 步态状态
} imu_control_t;

// 初始化IMU控制结构体
void imu_control_init(imu_control_t *imu_control);

// 获取IMU控制指针
imu_control_t *get_imu_control_point(void);

// 模块功能函数
void imu_gpio_init(void);

void imu_pwm_init(void);

void imu_hardware_init(void);

void imu_data_update(imu_control_t *imu_control);

void imu_temperature_control(imu_control_t *imu_control);

void imu_statistics_update(imu_control_t *imu_control);

fp32 *get_INS_angle_point(void);

fp32 *get_gyro_data_point(void);

#endif // IMU_H_
