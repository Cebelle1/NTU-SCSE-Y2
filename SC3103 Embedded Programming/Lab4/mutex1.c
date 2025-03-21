#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

int g_var1 =0;
pthread_mutex_t mutexA = PTHREAD_MUTEX_INITIALIZER;

void *inc_gv()
{
	int  i,j;
	for(i=0; i<10; i++){
	pthread_mutex_lock(&mutexA);
	g_var1++;
	for(j=0; j<5000000; j++);
	printf(" %d", g_var1);
	pthread_mutex_unlock(&mutexA);
	fflush(stdout);
	}
}

int main(){
	pthread_t  TA,TB;
	
	pthread_mutex_init(&mutexA,NULL);
	
	int TAret, TBret;
	TAret = pthread_create(&TA, NULL, inc_gv, NULL);
	TBret = pthread_create(&TB, NULL, inc_gv, NULL);
	
	pthread_join( TA, NULL);
	pthread_join( TB, NULL);
	return 0;
}
        
