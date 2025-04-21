//
// Created by Rick on 2024/12/10.
//

#ifndef GIMBAL_BEHAVIOUR_H_
#define GIMBAL_BEHAVIOUR_H_

#include "main.h"
#include "struct_typedef.h"
#include "stdbool.h"
#include "gimbal.h"
#include "feedforward.h"
#include "auto_shoot.h"
#include "user_lib.h"

typedef enum
{
	GIMBAL_ZERO_FORCE = 0,
	GIMBAL_INIT,
	GIMBAL_CALI,
	GIMBAL_ABSOLUTE_ANGLE,
	GIMBAL_RELATIVE_ANGLE,
	GIMBAL_MOTIONLESS,
	GIMBAL_AUTO,
} gimbal_behaviour_e;

void gimbal_behaviour_set(gimbal_control_t *gimbal_mode_set);
void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set);
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set);

bool_t gimbal_cmd_to_chassis_stop(void);
bool_t get_gimbal_behaviour(void);

#endif //GIMBAL_BEHAVIOUR_H_
