#include "FreeRTOS.h"
#include "semphr.h"
#include "imu.h"
#include "ist8310driver.h"

#define SAMPLE_RATE 1000

imu_control_t *imu_control;

uint32_t ist_count = 0;

void imu_task(void *argument) {
	imu_control = get_imu_control_point();
	imu_control_init(imu_control);
	imu_hardware_init();

	while (1) {
		if (xSemaphoreTake(imu_control->xAccSemaphore, portMAX_DELAY) == pdTRUE &&
			xSemaphoreTake(imu_control->xGyroSemaphore, portMAX_DELAY) == pdTRUE) {

			ist_count++;
			imu_data_update(imu_control);
			imu_statistics_update(imu_control);
			imu_temperature_control(imu_control);
		}
	}
}
