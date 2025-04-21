#include "imu.h"
#include "BMI088driver.h"
#include "ist8310driver.h"
#include "bsp_gpio.h"
#include "bsp_pwm.h"
#include "cmsis_os.h"
#include "QuaternionEKF.h"

#define SAMPLE_RATE 1000
#define DEG_TO_RAD(angle) ((angle) * (PI / 180.0f))

// 全局IMU控制指针
static imu_control_t imu_control_instance;

// 中断回调函数声明
static void acc_int_callback(void);

static void gyro_int_callback(void);

/**
 * @brief 初始化IMU控制结构体
 *
 * @param imu_control IMU控制结构体指针
 */
void imu_control_init(imu_control_t *imu_control) {
	// 初始化信号量
	imu_control->xAccSemaphore = xSemaphoreCreateBinary();
	imu_control->xGyroSemaphore = xSemaphoreCreateBinary();

	// 初始化时间戳
	imu_control->last_update_time = 0;
	imu_control->last_second_time = 0;
	imu_control->solve_count = 0;
	imu_control->solves_per_second = 0;

	// AHRS初始化
	IMU_QuaternionEKF_Init(10, 0.001f, 1000000,  0.9996, (fp32) 1 / SAMPLE_RATE, 0); //ekf初始化

	// 初始化温度PID控制器
	const fp32 PID_params[3] = {50.0f, 0.1f, 0.0f}; // Kp, Ki, Kd
	PID_init(&imu_control->temp_pid, PID_POSITION, PID_params, 1000.0f, 50.0f);
}


/**
 * @brief 获取IMU控制指针
 *
 * @return imu_control_t* IMU控制结构体指针
 */
imu_control_t *get_imu_control_point(void) {
	return &imu_control_instance;
}

/**
 * @brief 初始化IMU硬件
 */
void imu_hardware_init(void) {
	imu_pwm_init();
	imu_gpio_init();
	while (BMI088_init()) {
		// 等待IMU初始化完成
	}
	while (ist8310_init()) {
		// 等待磁力计初始化完成
	}
}

/**
 * @brief 更新IMU数据
 *
 * @param imu_control IMU控制结构体指针
 */
void imu_data_update(imu_control_t *imu_control) {
	// 获取当前时间戳
	const TickType_t current_time = xTaskGetTickCount();
	imu_control->delta_time = (float) (current_time - imu_control->last_update_time) / (float) configTICK_RATE_HZ;
	imu_control->last_update_time = current_time;

	// 更新陀螺仪数据
	if (imu_control->step_status == 1) {
		imu_control->gyroscope[0] -= imu_control->gyro_correct[0]; //减去陀螺仪0飘
		imu_control->gyroscope[1] -= imu_control->gyro_correct[1];
		imu_control->gyroscope[2] -= imu_control->gyro_correct[2];

		//ekf姿态解算部分
		IMU_QuaternionEKF_Update(imu_control->gyroscope[0], imu_control->gyroscope[1], imu_control->gyroscope[2],
		                         imu_control->accelerometer[0], imu_control->accelerometer[1],
		                         imu_control->accelerometer[2]);
		//ekf获取姿态角度函数
		imu_control->angle[0] = DEG_TO_RAD(Get_Roll());
		imu_control->angle[1] = DEG_TO_RAD(Get_Pitch());
		imu_control->angle[2] = DEG_TO_RAD(Get_Yaw());
	} else if (imu_control->step_status == 0) {
		//gyro correct
		imu_control->gyro_correct[0] += imu_control->gyroscope[0];
		imu_control->gyro_correct[1] += imu_control->gyroscope[1];
		imu_control->gyro_correct[2] += imu_control->gyroscope[2];
		imu_control->time_count++;
		if (imu_control->time_count >= 5 * SAMPLE_RATE) {
			imu_control->gyro_correct[0] /= 5 * SAMPLE_RATE;
			imu_control->gyro_correct[1] /= 5 * SAMPLE_RATE;
			imu_control->gyro_correct[2] /= 5 * SAMPLE_RATE;
			imu_control->step_status = 1;
		}
	}
}

/**
 * @brief 更新统计信息
 *
 * @param imu_control IMU控制结构体指针
 */
void imu_statistics_update(imu_control_t *imu_control) {
	imu_control->solve_count++;

	// 每秒钟更新一次解算次数
	const TickType_t current_time = xTaskGetTickCount();
	if (current_time - imu_control->last_second_time >= configTICK_RATE_HZ) {
		imu_control->solves_per_second = imu_control->solve_count;
		imu_control->solve_count = 0;
		imu_control->last_second_time = current_time;
		BMI088_read_temperature(&imu_control->temperature);
	}
}

/**
 * @brief 使用PID控制IMU温度在40度
 *
 * @param imu_control IMU控制结构体指针
 */
void imu_temperature_control(imu_control_t *imu_control) {
	// 目标温度
	const float target_temperature = 40.0f;

	// 使用PID控制器计算PWM占空比
	float pwm_duty_cycle = PID_calc(&imu_control->temp_pid, imu_control->temperature, target_temperature);

	// 限制PWM占空比在0到100之间
	if (pwm_duty_cycle < 0.0f) {
		pwm_duty_cycle = 0.0f;
	} else if (pwm_duty_cycle > 10000.0f) {
		pwm_duty_cycle = 10000.0f;
	}

	// 更新PWM占空比
	BSP_PWM_SetDutyCycle(&htim3, PWM_CHANNEL_4, (uint16_t) pwm_duty_cycle);
}

/**
 * @brief 加速度计中断回调函数
 */
static void acc_int_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// 读取加速度计数据
	BMI088_read_accel(imu_control_instance.accelerometer);

	// 在中断中释放信号量
	xSemaphoreGiveFromISR(imu_control_instance.xAccSemaphore, &xHigherPriorityTaskWoken);

	// 如果需要，触发上下文切换
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken)
}

/**
 * @brief 陀螺仪中断回调函数
 */
static void gyro_int_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// 读取陀螺仪数据
	BMI088_read_gyro(imu_control_instance.gyroscope);

	// 在中断中释放信号量
	xSemaphoreGiveFromISR(imu_control_instance.xGyroSemaphore, &xHigherPriorityTaskWoken);

	// 如果需要，触发上下文切换
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken)
}

/**
 * @brief IMU GPIO初始化函数
 *
 * @details 该函数用于初始化IMU模块的GPIO引脚，包括加速度计和陀螺仪的中断引脚。
 *          每个引脚的配置包括端口、引脚号、模式、上拉/下拉、速度和中断号。
 *          初始化完成后，会为每个中断引脚注册一个回调函数，用于处理中断事件。
 *
 * @note
 * 1. 加速度计和陀螺仪的中断引脚需要分别配置为外部中断模式（BSP_GPIO_MODE_EXTI）。
 * 2. 中断引脚的上拉/下拉配置为GPIO_NOPULL，表示不使用上拉或下拉电阻。
 * 3. 中断引脚的速度配置为GPIO_SPEED_FREQ_LOW，表示低速模式。
 * 4. 中断引脚的中断号需要与硬件设计一致，确保中断能够正确触发。
 * 5. 回调函数需要在调用该函数之前定义，并在初始化时注册。
 *
 * @param
 * @return 无
 */
void imu_gpio_init(void) {
	// 配置加速度计中断引脚
	BSP_GPIO_InitTypeDef acc_int_config = {
		.port = ACC_INT_GPIO_Port,
		.pin = ACC_INT_Pin,
		.mode = BSP_GPIO_MODE_EXTI,
		.pull = GPIO_NOPULL,
		.speed = GPIO_SPEED_FREQ_LOW,
		.irqn = ACC_INT_EXTI_IRQn
	};
	BSP_GPIO_Init(&acc_int_config);

	// 配置陀螺仪中断引脚
	BSP_GPIO_InitTypeDef gyro_int_config = {
		.port = GYRO_INT_GPIO_Port,
		.pin = GYRO_INT_Pin,
		.mode = BSP_GPIO_MODE_EXTI,
		.pull = GPIO_NOPULL,
		.speed = GPIO_SPEED_FREQ_LOW,
		.irqn = GYRO_INT_EXTI_IRQn
	};
	BSP_GPIO_Init(&gyro_int_config);

	// 注册回调函数
	BSP_GPIO_RegisterCallback(ACC_INT_Pin, acc_int_callback);
	BSP_GPIO_RegisterCallback(GYRO_INT_Pin, gyro_int_callback);
}

/**
 * @brief IMU PWM初始化函数
 *
 * @details 该函数用于初始化IMU模块的PWM接口，用于控制IMU的温度。
 *          初始化包括配置定时器句柄、PWM通道、频率和占空比。
 *          初始化完成后，PWM输出将用于控制IMU的温度。
 *
 * @note
 * 1. 定时器句柄（htim）需要与硬件设计一致，确保PWM输出能够正确连接到IMU。
 * 2. PWM通道需要与硬件设计一致，确保PWM信号能够正确输出。
 * 3. PWM频率和占空比可以根据实际需求进行调整。
 * 4. 初始化时占空比设置为0，表示PWM输出初始为低电平。
 * 5. 初始化完成后，调用BSP_PWM_Start函数启动PWM输出。
 *
 * @param
 * @return 无
 */
void imu_pwm_init(void) {
	// 定义PWM初始化结构体
	PWM_InitTypeDef pwm_config = {
		.htim = &htim3, // 定时器句柄
		.channel = PWM_CHANNEL_4, // PWM通道
		.frequency = 1000, // PWM频率
		.duty_cycle = 0 // PWM占空比
	};

	// 初始化PWM
	BSP_PWM_Init(&pwm_config);

	// 启动PWM
	BSP_PWM_Start(&htim3, PWM_CHANNEL_4);
}

/**
 * @brief 获取欧拉角数据指针
 * @return
 */
fp32 *get_INS_angle_point() {
	return imu_control_instance.angle;
}

/**
 * @brief 获取陀螺仪数据指针
 * @return
 */
fp32 *get_gyro_data_point() {
	return imu_control_instance.gyroscope;
}
