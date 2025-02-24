/**
  ******************************************************************************
  * @file    rplidar.h
  * @create  2024/04/11
  * @author  LeCrabe
  * @brief   This file contains everything for the A2M8 RPLidar of Evolutek<<
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_RPLIDAR_H_
#define INC_RPLIDAR_H_

#include <stdint.h>



/******************************** PAQUET FIELDS *******************************/
/* Start field ---------------------------------------------------------------*/
#define START_FLAG1		0xA5
#define START_FLAG2		0x5A

/* Command field -------------------------------------------------------------*/
/* No response */
#define STOP 			0x25
#define RESET 			0x40

/* Multiple response */
#define SCAN 			0x20
#define FORCE_SCAN 		0x21
#define EXPRESS_SCAN 	0x82	/* Payload */

/* Single response */
#define GET_INFO 		0x50
#define GET_HEALTH 		0x52
#define GET_SAMPLERATE 	0x59
#define GET_LIDAR_CONF 	0x84	/* Payload */

/* Descriptor field ----------------------------------------------------------*/
#define	RES_LENGTH_MODE 0x40000005
#define DATA_TYPE		0x81

/* MACROS --------------------------------------------------------------------*/
#define CHECK_BIT(var, pos) ((var >> pos) & (1))

/* State Machines ------------------------------------------------------------*/
enum state_scan { STANDBY, REQUEST, DESCRIPTOR, SCANNING, STOPPING, UART_ERROR };


/* Request packet format -----------------------------------------------------*/
typedef struct Request {
	uint8_t start_flag;		/* Mandatory (=START1) -----*/
	uint8_t command;		/* Mandatory (c.f. line 22) */
	uint8_t payload[];		/* Payload size + payload data (default=nill) ----*/
} __attribute__((__packed__)) request;


/* Response descriptor packet format -----------------------------------------*/
typedef struct Descriptor {
	uint8_t	start_flag1;		/* Mandatory (=START1) ----------------------*/
	uint8_t start_flag2;		/* Mandatory (=START2) ----------------------*/
	uint32_t res_length_mode;	/* Mandatory (30b res length + 2b send mode) */
	uint8_t type;				/* Mandatory --------------------------------*/
} __attribute__((__packed__)) descriptor;


/* Response data packet format -----------------------------------------------*/
typedef struct Scan_Data { 	// Also used by FORCE_SCAN
	uint8_t quality;		// Reflected laser pulse strength
	uint16_t angle_q6;		// In ° (angle_q6/64.0 ° ?)
	uint16_t distance_q2;	// In mm (distance_q2/4.0 mm ?)
} __attribute__((__packed__)) scan_data;

typedef struct Scan_Pack {
	uint16_t quantity;		// Number of measure
	scan_data mes[];		// Measures
} __attribute__((__packed__)) scan_pack;

typedef struct Express_scan_Data {
	uint8_t sync1; 				// 0xA + checkSums
	uint8_t sync2;				// 0x5 + checkSums
	uint16_t start_angle_q6;	// Ref angle. In ° (start_angle_q6/64.0 ° ?)
	uint8_t cabin[];			// WARNING: legacy+extended+dense versions
} __attribute__((__packed__)) express_scan_data;

typedef struct Info_Data {
	uint8_t model;				// RPLIDAR model ID
	uint8_t firmware_minor; 	// decimal part of version number
	uint8_t firmware_major; 	// integer part of version number
	uint8_t hardware;			// hardware version number
	uint8_t serialnumber[16];	// 128bit unique serial number
} __attribute__((__packed__)) info_data;

typedef struct Health_Data {
	uint8_t status;		// O: Good, 1: Warning, 2: Error
	uint8_t error[];	// related code for warning/error
} __attribute__((__packed__)) health_data;

typedef struct Samplerate_Data {
	uint16_t Tstandard;	// time for a single laser ranging in SCAN mode. In us
	uint16_t Texpress;	// same but in EXPRESS_SCAN mode. In us
} __attribute__((__packed__)) samplerate_data;

typedef struct Lidar_conf_Data {
	uint32_t type;		// configuration entry id
	uint8_t payload[];	// configuration value
} __attribute__((__packed__)) lidar_conf_data;


/******************************** PAQUET FIELDS *******************************/



/**************************** FUNCTIONS PROTOTYPES ****************************/
/* Request -------------------------------------------------------------------*/
descriptor new_req(UART_HandleTypeDef *huart, const uint8_t cmd); // TODO #1
void start_scan(UART_HandleTypeDef *huart, descriptor *res_desc, uint8_t *rx_buffer);
void start_force_scan(UART_HandleTypeDef *huart, descriptor *res_desc);
void start_express_scan(UART_HandleTypeDef *huart, const uint8_t payload, descriptor *res_desc); // TODO #1


/* Response ------------------------------------------------------------------*/
void get_res_descriptor(UART_HandleTypeDef *huart, uint8_t *pData);
void get_res_data(UART_HandleTypeDef *huart, uint8_t *pData, descriptor *res_desc);

/* No response */
void stop(UART_HandleTypeDef *huart);
void reset(UART_HandleTypeDef *huart);

/* Multiple response */
scan_pack get_scan(UART_HandleTypeDef *huart);
scan_pack force_scan(UART_HandleTypeDef *huart);
express_scan_data *get_express_scan(UART_HandleTypeDef *huart); // TODO #1

/* Single response */
void get_single_res(UART_HandleTypeDef *huart, uint8_t *res_data, const uint8_t cmd);
info_data get_info(UART_HandleTypeDef *huart);
health_data get_health(UART_HandleTypeDef *huart);
samplerate_data get_samplerate(UART_HandleTypeDef *huart);
lidar_conf_data get_lidar_conf(UART_HandleTypeDef *huart); // TODO #1


/**************************** FUNCTIONS PROTOTYPES ****************************/



#endif /* INC_RPLIDAR_H_ */
