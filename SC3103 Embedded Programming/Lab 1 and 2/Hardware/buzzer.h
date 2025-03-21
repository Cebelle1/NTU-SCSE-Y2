#ifndef __BUZZER_H
#define __BUZZER_H
#include "sys.h"
#include "system.h"

#define BUZZER_TASK_PRIO		3 //-1 to 32, lower number higher prioritiy
#define BUZZER_STK_SIZE			128 //stack size

//Buzzer pin PB10
#define BUZZER_PORT GPIOB	//port b
#define BUZZER_PIN GPIO_Pin_10 //pin 10
#define BUZZER PBout(10) //PB10 as output

//Function prototypes
void BUZZER_Init(void);
void buzzer_task(void *pvParameters);
extern int Buzz_Count; //variable to be edited by other files/classes hence need to be extern
#endif
