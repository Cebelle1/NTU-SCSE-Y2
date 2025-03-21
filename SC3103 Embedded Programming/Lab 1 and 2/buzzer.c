#include "buzzer.h"

int Buzz_Count = 7000; //delay count in ms

void BUZZER_Init(void){
	GPIO_InitTypeDef	GPIO_InitStructure; //Create the GPIO init object to specify GPIO operating mode
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //APB clock
	
	GPIO_InitStructure.GPIO_Pin =  BUZZER_PIN;                  //LED Pin
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;            //Push pull output
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;						//Output type push pull; _OD for open drain
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        //100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;					//internal resistors; NOPULL; _UP = pull up; _DOWN = pull down
  GPIO_Init(BUZZER_PORT, &GPIO_InitStructure);                    //Initialize LED GPIO
	//GPIO_SetBits(BUZZER_PORT,BUZZER_PIN);
	GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);
}

void buzzer_task(void *pvParameters){
	while(1) //task shall not return
		{ 
		BUZZER = ~BUZZER;
		vTaskDelay(Buzz_Count);	 //task delay instead of cpu delay_ms();
	}
}
