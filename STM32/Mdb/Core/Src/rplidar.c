#include "rplidar.h"
#include "main.h"

#define LIDAR_REQUEST 0xA5

static UART_HandleTypeDef* serial;

void lidar_init(UART_HandleTypeDef* uart) {
	// Starts the motor. We can use PWM if we want to change the speed
	HAL_GPIO_WritePin(MCTL_GPIO_Port, MCTL_Pin, GPIO_PIN_SET);
	// Saves the serial object
	serial = uart;
}

void lidar_send_command(uint8_t command) {
	uint8_t cmd[2] = { LIDAR_REQUEST, command};
	HAL_UART_Transmit(serial, cmd, 2, 0xFFFF);
}
