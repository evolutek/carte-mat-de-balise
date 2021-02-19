#ifndef INC_RPLIDAR_H_
#define INC_RPLIDAR_H_

#include "main.h"

#define LIDAR_START_SCAN 0x20

void lidar_init(UART_HandleTypeDef* uart);
void lidar_send_command(uint8_t command);

#endif /* INC_RPLIDAR_H_ */
