#include <stdio.h>
#include <stdlib.h>

int i;
void MergeSort(int s, int* array);
void MergeArray(int l_size, int r_size, int *l_array, int *r_array, int *array);
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
    MergeSort(s, array);

    return 0;
}
void MergeSort(int s, int *array){

    int mid, i;
    if (s <2 ) return;

    mid = s/2;

    int l_array[mid];
    int l_size = mid;
    int r_array[s-mid];
    int r_size = s-mid;

    //Split into left right arrays
    for(i=0; i<mid; i++){
        l_array[i] = array[i];
    }

    for(i=mid; i<s; i++){
        r_array[i-mid] = array[i];
    }

    //Recursion
    MergeSort(l_size, l_array);
    MergeSort(r_size, r_array);

    //Sorting occurs while merging
    MergeArray(l_size, r_size, l_array, r_array, array); //Edits on the og array since pointers

    for(int x=0; x<s; x++){
        printf("%d ", array[x]);
    }
    printf("\n");
}

void MergeArray(int l_size, int r_size, int *l_array, int *r_array, int *array){

    int i =0; //Left array pointer
    int j=0; //Right array pointer
    int k=0; //Main array pointer

    //Compare left and right array and sort them back into main array
    //Do until one of the array reaches the end
    while(i<l_size && j<r_size){
        if(l_array[i] <= r_array[j]){
            array[k] = l_array[i];
            i++;
        }else{
            array[k] = r_array[j];
            j++;
        }
        k++;
    }

    //Left overs
    while(i< l_size){
        array[k] = l_array[i];
        i++;
        k++;
    }

    while(j< r_size){
        array[k] = r_array[j];
        j++;
        k++;
    }

    //Since arrays are pointers already and are declared in function, the temp small arrays should disappear
    //after their respective recursions except for the main array, declared in main
}



