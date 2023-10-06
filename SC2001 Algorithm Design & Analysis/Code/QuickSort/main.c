#include <stdio.h>
#include <stdlib.h>

int i;
void QuickSort(int* array, int start, int end);
int Partition(int* array, int start, int end);

int main()
{
    int s,e;
    int *p;
    printf("Size of array: ");
    scanf("%d", &s);
    int array[s];
    printf("Input array elements: \n");
    for(i=0; i<s; i++){
        scanf("%d", &e);
        array[i] = e;
    }

    //Sort
    QuickSort(array, 0, s-1);

    for(i=0; i<s; i++){
        printf("%d ", array[i]);
    }
    return 0;
}
void QuickSort(int *array, int start, int end){

    int pivot_pos;
    if(start >= end) return;
    pivot_pos = Partition(array, start, end);
    QuickSort(array, start, pivot_pos-1);   //Exclude the pivot
    QuickSort(array,pivot_pos +1, end);

}

//Partition the array s.t. number lower than pivot goes to left of pivot, bigger than pivot goes right
int Partition(int* array, int start, int end){
    int pIndex = start;
    int pivot = array[end];
    int temp;
    int i;

    for(i=start; i<end; i++){
        if(array[i] <= pivot){          //Compare each element with pivot
            temp = array[i];
            array[i] = array[pIndex];
            array[pIndex] = temp;
            pIndex++;                   //Only if swapping, then increase pIndex so next swap can take next position
        }
    }
    temp = array[end];                  //At the end, swap pivot in
    array[end] = array[pIndex];
    array[pIndex] = temp;

    return pIndex;                      //Returns pivot index so next partition halves the array excluding the pivot

}



