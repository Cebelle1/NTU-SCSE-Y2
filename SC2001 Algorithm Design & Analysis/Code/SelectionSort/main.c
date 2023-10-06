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
    SelectionSort(s, array);

    //Print
    Print(s, array);
    return 0;
}


//Nested Loop > O(n^2)
int SelectionSort(int s, int *array){
    int temp,x,y,index;

    //Sort
    for(x=0; x<s; x++){      //current index to swap
        PrintSequence(s, array);
        int min = x;
        for(y=x; y<s; y++){  //scanner to find min
            if(array[y] < array[min]){
                min = y;

            }
        }
        //Swap current index with min and move to next
        temp = array[x];
        array[x] = array[min];
        array[min] = temp;
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
