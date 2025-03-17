/**
  ******************************************************************************
  * @file    rplidar.h
  * @create  2024/04/11
  * @author  LeCrabe
  * @brief   This file contains everything for the A2M8 RPLidar of Evolutek
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INC_RPLIDAR_H_
#define INC_RPLIDAR_H_

#include <stdint.h>
#include "usart.h"

/******************************** PAQUET FIELDS *******************************/
/* Start field ---------------------------------------------------------------*/
#define START_FLAG1         0xA5
#define START_FLAG2         0x5A

/* Command field -------------------------------------------------------------*/
/* No response */
#define STOP                	0x25
#define RESET               	0x40
//#define LIDAR_CONF_MOTOR_PWM	0xA802  // Get/set motor speed

/* Multiple response */
#define SCAN                0x20
#define FORCE_SCAN          0x21
#define EXPRESS_SCAN        0x82    /* Payload */

/* Single response */
#define GET_INFO            0x50
#define GET_HEALTH          0x52
#define GET_SAMPLERATE      0x59
#define GET_LIDAR_CONF      0x84    /* Payload */

/* Descriptor field ----------------------------------------------------------*/
#define RES_LENGTH_MODE     0x40000005
#define DATA_TYPE           0x81

/* Response paquet format ----------------------------------------------------*/
#define RPLIDAR_ANS_FORMAT_LEGACY   0x82
#define RPLIDAR_ANS_FORMAT_EXTENDED 0x84
#define RPLIDAR_ANS_FORMAT_DENSE    0x85

/* Packets size --------------------------------------------------------------*/
#define RPLIDAR_SCAN_PACKET_SIZE 		5
#define RPLIDAR_LEGACY_PACKET_SIZE  	84
#define RPLIDAR_EXTENDED_PACKET_SIZE 	132
#define RPLIDAR_DENSE_PACKET_SIZE   	84

/* LIDAR configuration types -------------------------------------------------*/
#define LIDAR_CONF_SCAN_MODE_COUNT     		0x00000070  // Get available scan modes
#define LIDAR_CONF_SCAN_MODE_US_PER_SAMPLE	0x00000071  // Get speed info
#define LIDAR_CONF_SCAN_MODE_MAX_DISTANCE   0x00000074  // Get max distance
#define LIDAR_CONF_SCAN_MODE_ANS_TYPE   	0x00000075  // Get answer type
#define LIDAR_CONF_SCAN_MODE_TYPICAL    	0x0000007C  // Get typical mode
#define LIDAR_CONF_SCAN_MODE_NAME			0x0000007F  // Get user friendly scan mode

/* Buffer size ---------------------------------------------------------------*/
#define LIDAR_BUFFER_SIZE   720     // Circular buffer positions

/* MACROS --------------------------------------------------------------------*/
#define CHECK_BIT(var, pos) ((var >> pos) & (1))

/* Request header format -----------------------------------------------------*/
typedef struct {
    uint8_t start_flag;     // Mandatory (=START1)
    uint8_t command;        // Mandatory (c.f. line 22)
} __attribute__((__packed__)) request_header;

/* Response descriptor packet format -----------------------------------------*/
typedef struct {
    uint8_t start_flag1;        // Mandatory (=START1)
    uint8_t start_flag2;        // Mandatory (=START2)
    uint32_t res_length_mode;   // Mandatory (30b res length + 2b send mode)
    uint8_t type;               // Mandatory
} __attribute__((__packed__)) descriptor;

/* Response data packet format -----------------------------------------------*/
typedef struct {
    uint8_t quality;        // Reflected laser pulse strength
    uint16_t angle_q6;      // In ° (angle_q6/64.0 ° ?)
    uint16_t distance_q2;   // In mm (distance_q2/4.0 mm ?)
} __attribute__((__packed__)) scan_data;

typedef struct {
    uint16_t quantity;      // Number of measure
} __attribute__((__packed__)) scan_pack;

typedef struct {
    uint8_t sync1;          // 0xA + checkSums
    uint8_t sync2;          // 0x5 + checkSums
    uint16_t start_angle_q6;// Ref angle. In ° (start_angle_q6/64.0 ° ?)
} __attribute__((__packed__)) express_scan_data;

typedef struct {
    uint8_t model;              // RPLIDAR model ID
    uint8_t firmware_minor;     // decimal part of version number
    uint8_t firmware_major;     // integer part of version number
    uint8_t hardware;           // hardware version number
    uint8_t serialnumber[16];   // 128bit unique serial number
} __attribute__((__packed__)) info_data;

typedef struct {
    uint8_t status;         // O: Good, 1: Warning, 2: Error
} __attribute__((__packed__)) health_data;

typedef struct {
    uint16_t Tstandard;     // time for a single laser ranging in SCAN mode. In us
    uint16_t Texpress;      // same but in EXPRESS_SCAN mode. In us
} __attribute__((__packed__)) samplerate_data;

typedef struct {
    uint32_t type;          // configuration entry id
} __attribute__((__packed__)) lidar_conf_data;

/* LiDAR conf response packet format ---------------------------------------- */
typedef struct {
    uint16_t count;         // Number of available scan modes
} __attribute__((__packed__)) scan_mode_count;

typedef struct {
    uint8_t scan_mode;      // Scan mode number
    uint32_t us_per_sample; // Microseconds per sample
} __attribute__((__packed__)) scan_mode_us_per_sample;

typedef struct {
    uint16_t pwm_value;     // PWM value (0-1023)
} __attribute__((__packed__)) motor_pwm;
/******************************** PAQUET FIELDS *******************************/



/******************************* CONTEXT FIELDS *******************************/
/* State Machines ------------------------------------------------------------*/
typedef enum {
    LIDAR_STATE_STANDBY,
    LIDAR_STATE_REQUEST,
    LIDAR_STATE_DESCRIPTOR,
    LIDAR_STATE_SCANNING,
    LIDAR_STATE_STOPPING,
    LIDAR_STATE_ERROR
} LidarState;

/* Processed measurement -----------------------------------------------------*/
typedef struct {
    uint8_t quality;        // quality
    float angle;            // angle °
    float distance;         // distance mm
    uint8_t startBit;       // new scan (1 = start)
} LidarMeasurement;

/* LiDAR context -------------------------------------------------------------*/
typedef struct {
    // hardware peripherals
    UART_HandleTypeDef *uart;
    GPIO_TypeDef *motorPort;
    uint16_t motorPin;

    // LiDAR state
    LidarState state;

    // Reception buffer
    descriptor responseDescriptor;
    uint8_t dataPacket[1000 * RPLIDAR_SCAN_PACKET_SIZE];

    // Circular buffer for processed points
    LidarMeasurement points[LIDAR_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;

    // Decoding informations
    uint8_t format;
    uint16_t start_angle_q6;
    uint16_t prev_start_angle_q6;

    // Flags and counters
    volatile uint8_t receivingDescriptor;
    volatile uint8_t dmaTransferComplete;
    volatile uint32_t lastPacketTime;
    volatile uint32_t packetCount;
    volatile uint32_t errorCount;
} LidarContext;
/******************************* CONTEXT FIELDS *******************************/



/**************************** FUNCTIONS PROTOTYPES ****************************/
// Initialisation and control
void LIDAR_Init(LidarContext *ctx, UART_HandleTypeDef *uart, GPIO_TypeDef *motorPort, uint16_t motorPin);

// Commands
HAL_StatusTypeDef LIDAR_SendCommand(LidarContext *ctx, uint8_t cmd, uint8_t* payload, uint8_t payloadSize);
void LIDAR_Stop(LidarContext *ctx);
uint8_t LIDAR_Scan(LidarContext *ctx);
uint8_t LIDAR_ExpressScan(LidarContext *ctx, uint8_t mode);
uint8_t LIDAR_Info(LidarContext *ctx);
uint8_t LIDAR_Health(LidarContext *ctx);
uint8_t LIDAR_Conf(LidarContext *ctx, uint32_t type);
uint8_t LIDAR_HealthCheck(LidarContext *ctx);

// Receiving and processing
void LIDAR_ProcessPacket(LidarContext *ctx, uint8_t cplt);
uint16_t LIDAR_GetPoints(LidarContext *ctx, LidarMeasurement *points, uint16_t maxPoints);
void LIDAR_DMA_Callback(UART_HandleTypeDef *huart, uint8_t cplt);

// Compatibility with existing code
void LIDAR_RxCpltCallback(UART_HandleTypeDef *huart);
/**************************** FUNCTIONS PROTOTYPES ****************************/

// Global variable for active context
extern LidarContext *activeLidarContext;

#endif /* INC_RPLIDAR_H_ */
