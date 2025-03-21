#include "show.h"

void show_task(void *pvParameters)
{
   u32 lastWakeTime = getSysTickCnt();
   while(1)
    {	

			vTaskDelayUntil(&lastWakeTime, F2T(RATE_50_HZ));

			oled_show();
    }
}  

void oled_show(void)
{  
     //To DO
		uint8_t* p ="  Loo Si Hui";
		uint8_t min = 0;
		uint8_t sec = 0;
		uint8_t time[5];

		while(1){
			OLED_ShowString(122, 58, p);
			sec++;
			if(sec == 59){
				min++;
				sec=0;
			}
			
			sprintf((char*) time, "%02d min %02d sec", min, sec);
			OLED_ShowString(0, 50, (uint8_t*) time);
			OLED_Refresh_Gram(); //refresh the OLED RAM
			vTaskDelay(1000);
		}

}




