#ifndef __UART3_H
#define __UART3_H

#include "system.h"
#include "sys.h"
#include "oled.h"

#define SHOW_UART_PRIO 6	//higher number lower prior
#define SHOW_UART_STK_SIZE 128

void uart3_init(u32 bound);
void usart3_send(u8 data);
#endif

