#include <stdio.h>
#include "hello.h"

int helloCount(int count){

	for(int i=0; i<0x7fffffff; i++);
	printf("Hello World from funct2!\n");
	
	count++;
	return count;
}
