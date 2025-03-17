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

// Global variable for storing the active context (used in callbacks)
LidarContext *activeLidarContext = NULL;

/**
 * @brief RPLidar initialisation
 * @param ctx RPLidar context to initialise
 * @param uart Handle UART for communication
 * @param motorPort GPIO Port GPIO for motor control
 * @param motorPin GPIO Pin GPIO for motor control
 */
void LIDAR_Init(LidarContext *ctx, UART_HandleTypeDef *uart, GPIO_TypeDef *motorPort, uint16_t motorPin)
{
  // Initialise context struct
  memset(ctx, 0, sizeof(LidarContext));

  // Save hardware references
  ctx->uart = uart;
  ctx->motorPort = motorPort;
  ctx->motorPin = motorPin;

  // Initialise state
  ctx->state = LIDAR_STATE_STANDBY;
  ctx->head = 0;
  ctx->tail = 0;
  ctx->receivingDescriptor = 0;
  ctx->dmaTransferComplete = 0;
  ctx->lastPacketTime = 0;
  ctx->packetCount = 0;
  ctx->errorCount = 0;

  // Motor off by default
  HAL_GPIO_WritePin(ctx->motorPort, ctx->motorPin, GPIO_PIN_RESET);

  // Set this context as active for callback
  activeLidarContext = ctx;

  printf("Lidar initialised\r\n");
}

/**
 * @brief Start the motor
 * @param ctx RPLidar context
 */
static void LIDAR_Motor_Start(LidarContext *ctx)
{
  HAL_GPIO_WritePin(ctx->motorPort, ctx->motorPin, GPIO_PIN_SET);
}

/**
 * @brief Stop the motor
 * @param ctx RPLidar context
 */
static void LIDAR_Motor_Stop(LidarContext *ctx)
{
  HAL_GPIO_WritePin(ctx->motorPort, ctx->motorPin, GPIO_PIN_RESET);
}

/* Request -------------------------------------------------------------------*/
/**
 * @brief Send a command to the RPLidar
 * @param ctx RPLidar context
 * @param cmd Command to send
 * @param payload Pointer to the payload data (or NULL if no payload)
 * @param payloadSize Size of the payload in bytes (0 if no payload)
 * @return HAL_StatusTypeDef Result of the UART transmission
 */
HAL_StatusTypeDef LIDAR_SendCommand(LidarContext *ctx, uint8_t cmd, uint8_t* payload, uint8_t payloadSize)
{
    uint8_t buffer[2 + payloadSize]; // Buffer for complete packet
    uint8_t totalSize = 0;

    // Prepare header in buffer
    buffer[0] = START_FLAG1;
    buffer[1] = cmd;
    totalSize = sizeof(request_header);

    // Add payload to buffer if exists
    if (payload != NULL && payloadSize > 0) {
        // Check buffer overflow
        if (totalSize + payloadSize > sizeof(buffer)) {
            return HAL_ERROR;
        }

        memcpy(buffer + totalSize, payload, payloadSize);
        totalSize += payloadSize;
    }

    // Send the complete packet
    return HAL_UART_Transmit(ctx->uart, buffer, totalSize, 100);
}

/**
 * @brief Wait and check response descriptor
 * @param ctx RPLidar context
 * @return 0 if success, 1 otherwise
 */
static uint8_t LIDAR_WaitResponseDescriptor(LidarContext *ctx)
{
  ctx->receivingDescriptor = 1;

  // Receive response descriptor
  if (HAL_UART_Receive(ctx->uart, (uint8_t *)&ctx->responseDescriptor, sizeof(descriptor), 1000) != HAL_OK) {
    ctx->receivingDescriptor = 0;
    ctx->errorCount++;
    return 1;  // Reception error
  }

  ctx->receivingDescriptor = 0;

  // Check if reception is valid
  if (ctx->responseDescriptor.start_flag1 != START_FLAG1 ||
      ctx->responseDescriptor.start_flag2 != START_FLAG2) {
    ctx->errorCount++;
    return 1;  // Synchronisation error
  }

  return 0;  // Success
}

/**
 * @brief Stop RPLidar scan
 * @param ctx RPLidar context
 */
void LIDAR_Stop(LidarContext *ctx)
{
  ctx->state = LIDAR_STATE_STOPPING;
  printf("Stopping lidar\r\n");

  // Stop DMA reception
  HAL_UART_AbortReceive(ctx->uart);

  // Send stop command
  LIDAR_SendCommand(ctx, STOP, NULL, 0);
  HAL_Delay(20);

  // Stop the motor
  LIDAR_Motor_Stop(ctx);

  ctx->state = LIDAR_STATE_STANDBY;
}

/**
 * @brief Start RPLidar scan
 * @param ctx RPLidar context
 * @return 0 if success, 1 otherwise
 */
uint8_t LIDAR_Scan(LidarContext *ctx)
{
  if (ctx->state != LIDAR_STATE_STANDBY) {
    return 1;  // Already in use
  }

  ctx->state = LIDAR_STATE_REQUEST;
  printf("Starting lidar: request\r\n");

  // Stop any previous scan
  LIDAR_SendCommand(ctx, STOP, NULL, 0);
  HAL_Delay(20);

  // Start the motor
  LIDAR_Motor_Start(ctx);
  HAL_Delay(2000);  // Time for the motor to reach its speed

  // Send scan command
  LIDAR_SendCommand(ctx, SCAN, NULL, 0);

  ctx->state = LIDAR_STATE_DESCRIPTOR;

  // Wait for and check response descriptor
  if (LIDAR_WaitResponseDescriptor(ctx) != 0) {
    LIDAR_Motor_Stop(ctx);
    ctx->state = LIDAR_STATE_ERROR;
    printf("Error receiving descriptor\r\n");
    return 1;
  }

  // Start DMA reception
  HAL_UART_Receive_DMA(ctx->uart, ctx->dataPacket, sizeof(ctx->dataPacket));

  ctx->state = LIDAR_STATE_SCANNING;
  ctx->lastPacketTime = HAL_GetTick();
  return 0;
}

/**
 * @brief Start RPLidar express scan
 * @param ctx RPLidar context
 * @param ctx Express scan working mode
 * @return 0 if success, 1 otherwise
 */
uint8_t LIDAR_ExpressScan(LidarContext *ctx, uint8_t mode)
 {
   if (ctx->state != LIDAR_STATE_STANDBY) {
     return 1;  // Already in use
   }

   ctx->state = LIDAR_STATE_REQUEST;
   printf("Starting lidar: request\r\n");

   // Stop any previous scan
   LIDAR_SendCommand(ctx, STOP, NULL, 0);
   HAL_Delay(20);

   // Start the motor
   LIDAR_Motor_Start(ctx);
   HAL_Delay(2000);  // Time for the motor to reach its speed

   // Send scan command
   LIDAR_SendCommand(ctx, EXPRESS_SCAN, NULL, 0);

   ctx->state = LIDAR_STATE_DESCRIPTOR;

   // Wait for and check response descriptor
   if (LIDAR_WaitResponseDescriptor(ctx) != 0) {
     LIDAR_Motor_Stop(ctx);
     ctx->state = LIDAR_STATE_ERROR;
     printf("Error receiving descriptor\r\n");
     return 1;
   }

   // Start DMA reception
   if (mode == 0) {
	   HAL_UART_Receive_DMA(ctx->uart, ctx->dataPacket, RPLIDAR_LEGACY_PACKET_SIZE);
   } else {
	   HAL_UART_Receive_DMA(ctx->uart, ctx->dataPacket, RPLIDAR_EXTENDED_PACKET_SIZE);
   }

   ctx->state = LIDAR_STATE_SCANNING;
   ctx->lastPacketTime = HAL_GetTick();
   return 0;
 }

/**
 * @brief Get RPLidar info
 * @param ctx RPLidar context
 * @return 0 if success, 1 otherwise
 */
uint8_t LIDAR_Info(LidarContext *ctx)
{
	  if (ctx->state != LIDAR_STATE_STANDBY) {
	    return 1;  // Already in use
	  }

	  ctx->state = LIDAR_STATE_REQUEST;
	  printf("Starting lidar: request\r\n");

	  // Stop any previous scan
	  LIDAR_SendCommand(ctx, STOP, NULL, 0);
	  HAL_Delay(20);

	  // Send scan command
	  LIDAR_SendCommand(ctx, GET_INFO, NULL, 0);

	  ctx->state = LIDAR_STATE_DESCRIPTOR;

	  // Wait for and check response descriptor
	  if (LIDAR_WaitResponseDescriptor(ctx) != 0) {
	    LIDAR_Motor_Stop(ctx);
	    ctx->state = LIDAR_STATE_ERROR;
	    printf("Error receiving descriptor\r\n");
	    return 1;
	  }

	  // Start DMA reception
	  HAL_UART_Receive_DMA(ctx->uart, ctx->dataPacket, 20);

	  ctx->state = LIDAR_STATE_SCANNING;
	  ctx->lastPacketTime = HAL_GetTick();
	  return 0;
}

/**
 * @brief Get RPLidar health
 * @param ctx RPLidar context
 * @return 0 if success, 1 otherwise
 */
uint8_t LIDAR_Health(LidarContext *ctx)
{
	  if (ctx->state != LIDAR_STATE_STANDBY) {
	    return 1;  // Already in use
	  }

	  ctx->state = LIDAR_STATE_REQUEST;
	  printf("Starting lidar: request\r\n");

	  // Stop any previous scan
	  LIDAR_SendCommand(ctx, STOP, NULL, 0);
	  HAL_Delay(20);

	  // Send scan command
	  LIDAR_SendCommand(ctx, GET_HEALTH, NULL, 0);

	  ctx->state = LIDAR_STATE_DESCRIPTOR;

	  // Wait for and check response descriptor
	  if (LIDAR_WaitResponseDescriptor(ctx) != 0) {
	    LIDAR_Motor_Stop(ctx);
	    ctx->state = LIDAR_STATE_ERROR;
	    printf("Error receiving descriptor\r\n");
	    return 1;
	  }

	  // Start DMA reception
	  HAL_UART_Receive_DMA(ctx->uart, ctx->dataPacket, 3);

	  ctx->state = LIDAR_STATE_SCANNING;
	  ctx->lastPacketTime = HAL_GetTick();
	  return 0;
}

/**
 * @brief Get LiDAR configuration
 * @param ctx RPLiDAR context
 * @param type Configuration type to request
 * @return 0 if success, 1 otherwise
 */
// TODO
uint8_t LIDAR_Conf(LidarContext *ctx, uint32_t type)
{
    if (ctx->state != LIDAR_STATE_STANDBY) {
        printf("Error: LiDAR not in standby state\r\n");
        return 1;  // Already in use
    }

    ctx->state = LIDAR_STATE_REQUEST;
//    printf("Requesting LiDAR configuration: 0x%08X\r\n", type);

    // Stop any previous operation
    LIDAR_SendCommand(ctx, STOP, NULL, 0);
    HAL_Delay(20);

    // Prepare the payload (type is a 32-bit value)
    uint8_t payloadType[4];
    payloadType[0] = type & 0xFF;
    payloadType[1] = (type >> 8) & 0xFF;
    payloadType[2] = (type >> 16) & 0xFF;
    payloadType[3] = (type >> 24) & 0xFF;

//    uint8_t payloadData[2];
    switch (type) {
    case LIDAR_CONF_SCAN_MODE_COUNT:
    	break;
    case LIDAR_CONF_SCAN_MODE_US_PER_SAMPLE:
    	break;
    case LIDAR_CONF_SCAN_MODE_MAX_DISTANCE:
    	break;
    case LIDAR_CONF_SCAN_MODE_ANS_TYPE:
    	break;
    case LIDAR_CONF_SCAN_MODE_TYPICAL:
    	break;
    case LIDAR_CONF_SCAN_MODE_NAME:
    	break;
    }

    // Send the command with payload
    LIDAR_SendCommand(ctx, GET_LIDAR_CONF, payloadType, sizeof(payloadType));

    ctx->state = LIDAR_STATE_DESCRIPTOR;

    // Wait for and check response descriptor
    if (LIDAR_WaitResponseDescriptor(ctx) != 0) {
        ctx->state = LIDAR_STATE_ERROR;
        printf("Error receiving descriptor\r\n");
        return 1;
    }

    // Calculate the response size based on the type
    uint16_t responseSize;
    switch (type) {
	case LIDAR_CONF_SCAN_MODE_COUNT:
		responseSize = sizeof(scan_mode_count);
		break;
	case LIDAR_CONF_SCAN_MODE_US_PER_SAMPLE:
		responseSize = sizeof(scan_mode_us_per_sample);
		break;
//	case LIDAR_CONF_MOTOR_PWM:
//		responseSize = sizeof(motor_pwm);
//		break;
	default:
		responseSize = 20; // Default size if unknown
		break;
    }

    // Receive the response data
    if (HAL_UART_Receive(ctx->uart, ctx->dataPacket, responseSize, 1000) != HAL_OK) {
        ctx->state = LIDAR_STATE_ERROR;
        printf("Error receiving configuration data\r\n");
        return 1;
    }

    ctx->state = LIDAR_STATE_STANDBY;
    printf("Configuration data received successfully\r\n");
    return 0;
}

/**
 * @brief Process received data packet
 * @param ctx RPLidar context
 * @param cplt 0 if halfCplt callback
 */
void LIDAR_ProcessPacket(LidarContext *ctx, uint8_t cplt)
{
  if (ctx->state != LIDAR_STATE_SCANNING) {
    return;
  }

  // Update packet counter and timestamp
  ctx->packetCount++;
  ctx->lastPacketTime = HAL_GetTick();

  scan_data *data = (scan_data*)ctx->dataPacket[cplt];

  for (int i = 0; i < 5; i++) {  // Process 5 measurements per packet
    // Extract information
    uint8_t quality = data[i].quality & 0x3F;  // 6 lower bits
    float angle = (float)data[i].angle_q6 / 64.0f;
    float distance = (float)data[i].distance_q2 / 4.0f;

    // Check if it's the beginning of a new turn
    uint8_t startBit = 0;
    if (i == 0 && ctx->prev_start_angle_q6 > data[i].angle_q6 &&
        ctx->prev_start_angle_q6 - data[i].angle_q6 > 3000) {
      startBit = 1;  // Transition from ~360° to ~0°
    }
    ctx->prev_start_angle_q6 = data[i].angle_q6;

    // If quality is valid
    if (quality > 0 && distance > 0) {
      // Store measurement in circular buffer
      ctx->points[ctx->head].quality = quality;
      ctx->points[ctx->head].angle = angle;
      ctx->points[ctx->head].distance = distance;
      ctx->points[ctx->head].startBit = startBit;

      // Advance the head of the circular buffer
      ctx->head = (ctx->head + 1) % LIDAR_BUFFER_SIZE;

      // If buffer is full, also advance the tail
      if (ctx->head == ctx->tail) {
        ctx->tail = (ctx->tail + 1) % LIDAR_BUFFER_SIZE;
      }
    }
  }

  // Restart DMA reception for the next packet
  HAL_UART_Receive_DMA(ctx->uart, (uint8_t *)ctx->dataPacket, sizeof(scan_data) * 5);
}

/**
 * @brief Callback for DMA transfer completion
 * This function must be called from HAL_UART_Rx(Half)CpltCallback
 * @param huart UART handle
 * @param cplt 0 if HalfCpltCallback
 */
void LIDAR_DMA_Callback(UART_HandleTypeDef *huart, uint8_t cplt)
{
  if (activeLidarContext != NULL && huart == activeLidarContext->uart) {
    if (activeLidarContext->state == LIDAR_STATE_SCANNING) {
      activeLidarContext->dmaTransferComplete = 1;
      LIDAR_ProcessPacket(activeLidarContext, cplt);
      activeLidarContext->dmaTransferComplete = 0;
    }
  }
}

/**
 * @brief Get points from the circular buffer
 * @param ctx RPLidar context
 * @param points Array to store the points
 * @param maxPoints Maximum number of points to retrieve
 * @return Number of points actually retrieved
 */
uint16_t LIDAR_GetPoints(LidarContext *ctx, LidarMeasurement *points, uint16_t maxPoints)
{
  if (ctx->state != LIDAR_STATE_SCANNING) {
    return 0;
  }

  // Temporarily disable interrupts to protect buffer access
  __disable_irq();

  uint16_t count = 0;
  uint16_t localTail = ctx->tail;

  while (localTail != ctx->head && count < maxPoints) {
    points[count] = ctx->points[localTail];
    localTail = (localTail + 1) % LIDAR_BUFFER_SIZE;
    count++;
  }

  // Update the tail after copying points
  ctx->tail = localTail;

  // Re-enable interrupts
  __enable_irq();

  return count;
}

/**
 * @brief Check RPLidar status and try to recover in case of error
 * @param ctx RPLidar context
 * @return Health status: 0 = OK, 1 = Error, 2 = Recovered
 */
uint8_t LIDAR_HealthCheck(LidarContext *ctx)
{
  uint32_t currentTime = HAL_GetTick();

  // If no packet has been received for 1 second and lidar is supposed to be scanning
  if (ctx->state == LIDAR_STATE_SCANNING &&
      (currentTime - ctx->lastPacketTime) > 1000 &&
      !ctx->dmaTransferComplete) {

    ctx->errorCount++;
    printf("Health check: no packet received for 1s\r\n");

    // Recovery attempt: restart the scan
    LIDAR_Stop(ctx);
    HAL_Delay(100);

    if (LIDAR_Scan(ctx) == 0) {
      return 2;  // Recovery successful
    } else {
      return 1;  // Persistent error
    }
  }

  return 0;  // All good
}
