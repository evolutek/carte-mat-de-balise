/**
  ******************************************************************************
  * @file    rplidar.c
  * @create  2024/04/14
  * @author  LeCrabe
  * @brief   This file contains everything for the RPLidar of the Evolutek team
  ******************************************************************************
  */
#include "main.h"
#include "rplidar.h"
#include <stdio.h>
#include <string.h>

/* Request -------------------------------------------------------------------*/
descriptor new_req(UART_HandleTypeDef *huart, const uint8_t cmd) {
	request req_struct;
	descriptor desc_res;
	req_struct.start_flag = START_FLAG1;
	req_struct.command = cmd;
	if (cmd > 0x80) {
		//todo
	}
	HAL_UART_Transmit(huart, (uint8_t *)&req_struct, sizeof(req_struct), 100);
	HAL_UART_Receive(huart, (uint8_t *)&desc_res, sizeof(desc_res), 1000);
	return desc_res;
}

void start_scan(UART_HandleTypeDef *huart, descriptor *res_desc, uint8_t *rx_buffer) {
	new_req(huart, SCAN);
	get_res_descriptor(huart, (uint8_t *)res_desc);
	HAL_UART_Receive_DMA(huart, (uint8_t *)rx_buffer, 256);
}
void start_force_scan(UART_HandleTypeDef *huart, descriptor *res_desc) {
	new_req(huart, FORCE_SCAN);
	get_res_descriptor(huart, (uint8_t *)res_desc);
}
void start_express_scan(UART_HandleTypeDef *huart, uint8_t payload, descriptor *res_desc) {
	// TODO #1
}


/* Response ----------------------------------------------------------------*/
void get_res_descriptor(UART_HandleTypeDef *huart, uint8_t *pData) {
	HAL_UART_Receive(huart, pData, 7, 1000);
}
void get_res_data(UART_HandleTypeDef *huart, uint8_t *pData, descriptor *res_desc) {
//	uint16_t data_size = res_desc->res_length_type; // TODO #2
	//HAL_UART_Receive(huart, pData, data_size, 1000);
	HAL_UART_Receive(huart, pData, 5, 1000);
}

/* No response */
void stop(UART_HandleTypeDef *huart) {
	new_req(huart, STOP);
}
void reset(UART_HandleTypeDef *huart) {
	new_req(huart, RESET);
}

/* Multiple response */
//scan_pack get_scan(UART_HandleTypeDef *huart) {
//
//}
//scan_pack force_scan(UART_HandleTypeDef *huart);
//express_scan_data *get_express_scan(UART_HandleTypeDef *huart);


/* Single response */
void get_single_res(UART_HandleTypeDef *huart, uint8_t *res_data, const uint8_t cmd) {
	new_req(huart, cmd);

	descriptor res_desc;
	get_res_descriptor(huart, (uint8_t *)&res_desc);

	get_res_data(huart, res_data, &res_desc);
}

info_data get_info(UART_HandleTypeDef *huart) {
	info_data res_data;
	get_single_res(huart, (uint8_t *)&res_data, GET_INFO);

	return res_data;
}
health_data get_health(UART_HandleTypeDef *huart) {
	health_data res_data;
	get_single_res(huart, (uint8_t *)&res_data, GET_HEALTH);

	return res_data;
}
samplerate_data get_samplerate(UART_HandleTypeDef *huart) {
	samplerate_data res_data;
	get_single_res(huart, (uint8_t *)&res_data, GET_SAMPLERATE);

	return res_data;
}
// TODO #1
lidar_conf_data get_lidar_conf(UART_HandleTypeDef *huart) {
	lidar_conf_data res_data;
	get_single_res(huart, (uint8_t *)&res_data, GET_LIDAR_CONF);

	return res_data;
}

//health_data get_res_data(UART_HandleTypeDef *huart) {
//	health_data res_data_struct;
//	HAL_UART_Receive(huart, (uint8_t *)&res_data_struct, 3, 1000);
//	return res_data_struct;
//}
