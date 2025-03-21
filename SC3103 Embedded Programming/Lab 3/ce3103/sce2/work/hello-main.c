#include "hello.h"
#include <stdio.h>

int main(){
	int count =0;
	
	while(1){
		printf("Hello World from main!\n");
		for(int i=0; i<0x5fffff; i++);
		char c = getchar();
		helloprint();	//delayed inside
		count = helloCount(count);	//delayed inside
		printf("Count: %d\n", count);
		if(c=='q'){
			printf("back");
			break;
		}
		//break;
	}
	printf("End");
}
