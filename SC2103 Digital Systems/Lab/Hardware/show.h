#ifndef __SHOW_H
#define __SHOW_H
#include "sys.h"
#include "oled.h"
#include "system.h"
#include "uart3.h"

#define SHOW_TASK_PRIO		3
#define SHOW_STK_SIZE 		512  

void show_task(void *pvParameters);
void oled_show(void);
void show_uart(void *pvParameters);

#endif
