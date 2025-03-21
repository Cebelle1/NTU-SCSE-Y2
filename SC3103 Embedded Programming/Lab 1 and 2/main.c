#include "system.h"


#define START_TASK_PRIO 4
#define START_STK_SIZE 256
TaskHandle_t StartTask_Handler;
TimerHandle_t xAutoTimer;

void start_task (void *pvParameters);

u32 baud_rate = 115200;
u8 data = 'c';

int main(void){
	systemInit();
	uart3_init(baud_rate);
	xTaskCreate((TaskFunction_t) start_task,
							(const char*	  ) "start_task",
							(uint16_t				) START_STK_SIZE,
							(void*					) NULL,
							(UBaseType_t	  ) START_TASK_PRIO,
							(TaskHandle_t *	) &StartTask_Handler);
	//Automatically reload (pdTRUE) at period of RATE_1_HZ, and calls the same led_task function.
	xAutoTimer = xTimerCreate("timer1", F2T(RATE_1_HZ), pdTRUE, 0, led_task);	//same led task
	
	if (xTimerStart(xAutoTimer,0)==pdPASS)
		
	vTaskStartScheduler();
}

void start_task(void *pvpParameters){
	taskENTER_CRITICAL(); //pause
	//xTaskCreate(led_task, "led_task", LED_STK_SIZE, NULL, LED_TASK_PRIO, NULL); //create task
	//xTaskCreate(buzzer_task, "buzzer_task", BUZZER_STK_SIZE, NULL, BUZZER_TASK_PRIO, NULL);
	//xTaskCreate(show_task, "show_task", SHOW_STK_SIZE, NULL, SHOW_TASK_PRIO, NULL);
	//============ Lab2======//
	xTaskCreate(song_task, "song_task", SONG_TASK_STK_SIZE, NULL, SONG_TASK_PRIO, NULL);	//song.c in Hardware folder
	xTaskCreate(show_uart, "show_uart", SHOW_UART_STK_SIZE, NULL, SHOW_UART_PRIO, NULL); //uart3.c	in Hardware folder
	//IRQhandler in uart3, should shift to here?
	
	vTaskDelete(StartTask_Handler);
	taskEXIT_CRITICAL();	//resume 
}





