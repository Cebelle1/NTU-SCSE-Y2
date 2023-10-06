#include <stdio.h>
#include <stdlib.h>

int i;
int main()
{
    int s,e;

    printf("Size of array: ");
    scanf("%d", &s);
    int array[s];
    printf("Input array elements: \n");
    for(i=0; i<s; i++){
        scanf("%d", &e);
        array[i] = e;
    }

    //Sort
    InsertionSort(s, array);

    //Print
    Print(s, array);
    return 0;
}

int InsertionSort(int s, int *array){
    int i;
    int j=0;
    int cur, temp;
    for(i=1; i<s; i++){
        PrintSequence(s, array);
        cur = array[i];
        for(j=i-1; j>=0; j--){      //Scan down the sorted
            if(array[j] > cur){     //If sorted is bigger, shift to right to make space
                array[j+1] = array[j];      //Swap
                array[j] = cur;
            }else{
                break;              //Since alr sorted, no point continuing
            }
        }

    }
}

void PrintSequence(int s, int* ar){
    for(int x=0; x<s; x++){
        printf("%d ", ar[x]);
    }
    printf("\n");
}

void Print(int s, int *array){
    for(i=0; i<s; i++){
        printf("%d ",array[i]);
    }
}
