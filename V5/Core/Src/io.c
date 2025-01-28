#include "main.h"

int _read(int file, char *ptr, int len)
{
  (void)file;

  HAL_UART_Receive(&huart2, (uint8_t *)ptr, len, 100);

  return len;
}
int _write(int file, char *ptr, int len)
{
  (void)file;

  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 100);

  return len;
}
