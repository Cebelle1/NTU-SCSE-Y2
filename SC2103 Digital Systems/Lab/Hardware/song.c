#include "song.h"
#include "buzzer.h"

void Button_Init(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //APB clock
	
	GPIO_InitStructure.GPIO_Pin =  UBUTTON_PIN;                  //LED Pin
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;            //Push pull output
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        //100MHz
  GPIO_Init(UBUTTON_PORT, &GPIO_InitStructure);                    //Initialize LED GPIO
	//GPIO_SetBits(UBUTTON_PORT,UBUTTON_PIN);
	GPIO_ReadInputDataBit(UBUTTON_PORT, UBUTTON_PIN);
	
}	
static u8* song = 
(uint8_t*) "e2,d2,e2,d2,e2,B2,d2,c2,A2_C2,E2,A2,B2_E2,G2,B2,c4_E2,e2,d2,e2,d2,e2,B2,d2,c2,A2_C2,E2,A2,B2_E2,c2,B2,A4";

static u8* empty = (uint8_t*) "";
static u32 notes[] = {
 2272, // A - 440 Hz
 2024, // B - 494 Hz
 3822, // C - 261 Hz
 3401, // D - 294 Hz
 3030, // E - 330 Hz
 2865, // F - 349 Hz
 2551, // G - 392 Hz
 1136, // a - 880 Hz
 1012, // b - 988 Hz
 1912, // c - 523 Hz
 1703, // d - 587 Hz
 1517, // e - 659 Hz
 1432, // f - 698 Hz
 1275, // g - 784 Hz
};


uint32_t getNote(uint8_t ch)
{
 if (ch >= 'A' && ch <= 'G')
	 return notes[ch - 'A'];
	 if (ch >= 'a' && ch <= 'g')
	 return notes[ch - 'a' + 7];
	 return 0;
}

uint32_t getDuration(uint8_t ch)
{
 if (ch < '0' || ch > '9')
	 return 500;
		/* number of ms */
	 return (ch - '0') * 250;
}

uint32_t getPause(uint8_t ch)
{
	 switch (ch) {
		 case '+':
			return 0;
		 case ',':
			return 5;
		 case '.':
			return 20;
		 case '_':
			return 30;
		 default:
			return 5;
 }
}

void playNote(uint32_t note, uint32_t durationMs)
{
	uint32_t t = 0;
	if (note > 0) {
		while (t < (durationMs*1000)) {
			BUZZER = 1; // Turn on your buzzer (Please Edit)
			delay_us(note/2);
			BUZZER = 0; // Turn off your buzzer (Please Edit)
			delay_us(note/2);
			t += note;
		}
	}
	else {
		delay_xms(durationMs); // ms timer
	}
}

void playSong(uint8_t *song) {
	uint32_t note = 0;
	uint32_t dur = 0;
	uint32_t pause = 0;
	/*
	* A song is a collection of tones where each tone is
	* a note, duration and pause, e.g.
	* "E2,F4,"
	*/
	while(*song != '\0') {
		note = getNote(*song++);
		if (*song == '\0')
			break;
		dur = getDuration(*song++);
		if (*song == '\0')
			break;
		pause = getPause(*song++);
		playNote(note, dur);
		delay_us(pause);
	}
}

void song_task(void *pvParameters){
	uint8_t button;
	while(1){
		button = GPIO_ReadInputDataBit(UBUTTON_PORT, UBUTTON_PIN); //stm32f4xx_gpio.c
		if(!button){	//Active Low
			playSong(song);
		}else{
			playSong(empty);
		}
	}
}
