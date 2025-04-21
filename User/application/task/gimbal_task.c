//
// Created by Rick on 2024/12/15.
//

#include "cmsis_os.h"
#include "gimbal.h"
#include "shoot.h"

gimbal_control_t *gimbal_control;

void gimbal_task(void *argument) {
	// 等待陀螺仪任务更新数据
	osDelay(GIMBAL_TASK_INIT_TIME);
	// 获取云台对象
	gimbal_control = get_gimbal_control_point();
	// 云台初始化
	gimbal_init(gimbal_control);

	// 判断电机是否都上线
	while (toe_is_error(PITCH_GIMBAL_MOTOR_TOE) && toe_is_error(YAW_GIMBAL_MOTOR_TOE)) {
		osDelay(GIMBAL_CONTROL_TIME);
		gimbal_feedback_update(gimbal_control);
	}

	while (1) {
		// 设置云台控制模式
		gimbal_set_mode(gimbal_control);
		// 控制模式切换 控制数据过渡
		gimbal_mode_change_control_transit(gimbal_control);
		// 云台数据反馈
		gimbal_feedback_update(gimbal_control);
		// 设置云台控制量
		gimbal_set_control(gimbal_control);
		// 云台控制PID计算
		gimbal_control_loop(gimbal_control);

#if YAW_TURN
		gimbal_control->gimbal_yaw_motor.given_current = -gimbal_control->gimbal_yaw_motor.given_current;
#else
		gimbal_control->gimbal_yaw_motor.given_current = gimbal_control->gimbal_yaw_motor.given_current;
#endif

#if PITCH_TURN
		gimbal_control->gimbal_pitch_motor.given_current = -gimbal_control->gimbal_pitch_motor.given_current;
#else
		gimbal_control->gimbal_pitch_motor.given_current = gimbal_control->gimbal_pitch_motor.given_current;
#endif

		if (!(toe_is_error(YAW_GIMBAL_MOTOR_TOE) && toe_is_error(PITCH_GIMBAL_MOTOR_TOE))) {
			if (toe_is_error(DBUS_TOE)) {
				gimbal_control->CAN_cmd_gimbal(0, 0, 0, 0);
			} else {
				// gimbal_control->CAN_cmd_gimbal(0,
				//                                (int16_t) gimbal_control->gimbal_pitch_motor.given_current,
				//                                0,
				//                                0);
				gimbal_control->CAN_cmd_gimbal((int16_t)gimbal_control->gimbal_yaw_motor.given_current,
					0,
					get_shoot_control_point()->trigger_motor.give_current,
					0);
			}
		}
		osDelay(GIMBAL_CONTROL_TIME);
	}
}
