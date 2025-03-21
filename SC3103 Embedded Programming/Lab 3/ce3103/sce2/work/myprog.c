#include <stdio.h>
int main(){
	char i, c;
	while(1){
		i = fread(&c, 1, 1, stdin);
		if (i>0){
			fwrite(&c, 1, 1, stdout);
		}
	}
}
