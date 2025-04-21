//
// Created by Rick on 2025/1/4.
//

#include "cmsis_os.h"

void watch_task(void* argument){

	// 喂狗任务,间隔20ms
	while (1){
		// 延时
		osDelay(200);
	}
}
