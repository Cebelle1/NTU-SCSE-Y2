#ifndef __SONG_H
#define __SONG_H
#include "sys.h"
#include "system.h"

#define SONG_TASK_PRIO 4 //higher number, lower priority
#define SONG_TASK_STK_SIZE 128

#define UBUTTON_PORT GPIOD
#define UBUTTON_PIN GPIO_Pin_8
#define UBUTTON PDin(8)

void Button_Init(void);
uint32_t getNote(uint8_t ch);
uint32_t getDuration(uint8_t ch);
uint32_t getPause(uint8_t ch);
void playNote(uint32_t note, uint32_t durationMs);
void playSong(uint8_t *song);
void song_task(void *pvParameters);

#endif