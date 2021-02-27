#include "rplidar.h"
#include "main.h"

#define LIDAR_REQUEST 0xA5

static UART_HandleTypeDef* serial;

void lidar_init(UART_HandleTypeDef* uart) {
	// Starts the motor. We can use PWM if we want to change the speed
	HAL_GPIO_WritePin(MCTL_GPIO_Port, MCTL_Pin, GPIO_PIN_RESET);
	// Saves the serial object
	serial = uart;
}

void lidar_send_command(uint8_t command) {
	uint8_t cmd[2] = { LIDAR_REQUEST, command};
	HAL_UART_Transmit(serial, cmd, 2, 0xFFFF);
}









struct scan {
	int newScan;    // 1 for the first packet of each new turn
	int quality;    // 0-63
	float angle;    // degrees
	float distance; // mm
};

#define S_FLAG   0x0000000001
#define NS_FLAG  0x0000000002
#define QUALITY  0x00000000FC
#define C_FLAG   0x0000000100
#define ANGLE    0x0000FFFE00
#define DISTANCE 0xFFFF000000

static int decode(struct scan* s, uint64_t raw) {

	s->newScan = raw & S_FLAG;

	// Sanity check
	if(!(raw & C_FLAG) || !!(raw & NS_FLAG) == s->newScan)
		return 0;

	s->quality = (raw & QUALITY) >> 2;
	// Shift of 9 because angle starts at bit 9, 6 because angle = raw_value/64.0
	s->angle = (float) ((raw & ANGLE) >> (6 + 9));
	// Shift of 24 because angle starts at bit 24, 2 because distance = raw_value/4.0
	s->distance = (float) ((raw & DISTANCE) >> (2 + 24));

	return 1;
}








