#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define ARRAY_SIZE  15000000 //array size min 1000 max 1mil
#define TESTS       30 //run each array size multiple times then average the total time
#define MAX         500
#define THRESHOLD   13  //Threshold to switch to insertion

void generateInput(int size, int *array);
int random(int min, int max);
void InsertionSort(int s, int *array);
void printArray(int s, int *array);
void MergeSort(int s, int* array);
void MergeArray(int l_size, int r_size, int *l_array, int *r_array, int *array);

double insert_times[ARRAY_SIZE];   //each index correspond to insertion time of n=index+1
double merge_times[ARRAY_SIZE];
double hyb_times[ARRAY_SIZE];
time_t begin, end;
clock_t start, stop;


unsigned long key_comp_ins[ARRAY_SIZE];
unsigned long key_comp_merg[ARRAY_SIZE];
unsigned long key_comp_hyb[ARRAY_SIZE];
//unsigned long array[ARRAY_SIZE];

unsigned long merge_key_comp;
unsigned long hyb_key_comp;

//FOR EACH SIZE, RUN ITERATION, ITERATION TIME/SIZE = TIME TAKEN FOR THAT SIZE
int main()
{

    FILE *fp, *fp1;
    fp = fopen("KeyComparison.csv", "w");
    fp1 = fopen("TimeAnalysis.csv", "w");
    unsigned long i;
    int *array = malloc(sizeof(int) * ARRAY_SIZE);


    for(i=1; i<= ARRAY_SIZE; i++){        //Sort the same size for TESTS times
        //Insertion Sort
        generateInput(i, array);
        InsertionSort(i,array);    //pass in to do sort

        //================Merge Sort=================
        merge_key_comp=0;
        //time(&begin);     //Measures in seconds only, not useful
        start = clock();

        for(int test=0; test<TESTS; test++){
            generateInput(i, array);
            MergeSort(i, array);
        }
        //time(&end);
        stop = clock();

        key_comp_merg[i] = merge_key_comp/TESTS;
        //merge_times[i] = (end-begin)/TESTS;
        merge_times[i] = ((double)(stop - start)/CLOCKS_PER_SEC/TESTS)*1000;

        //================Hybrid Sort======================
        *array = malloc(sizeof(int) * ARRAY_SIZE);
        hyb_key_comp=0;
        //time(&begin);
        start = clock();
        for(int test=0; test<TESTS; test++){
            generateInput(i, array);
            Hyb_MergeSort(i, array);
            //printArray(i, array);
        }
        //time(&end);
        stop = clock();
        key_comp_hyb[i] = hyb_key_comp/TESTS;
        //hyb_times[i] = (end-begin)/TESTS;
        hyb_times[i] = ((double)(stop - start)/CLOCKS_PER_SEC/TESTS)*1000;

        //Key Comparison of all 3 algo
        printf("Ins: %lu | Ms: %lu | Hyb:%lu \n", key_comp_ins[i],key_comp_merg[i], key_comp_hyb[i]);
        //fprintf(fp,"%lu,%lu,%lu,%lu\n",i, key_comp_ins[i], key_comp_merg[i],key_comp_hyb[i]);

        //Time Analysis of Merge and Hyb
        //printf("Time size %lu: \t MS: %f Ins: %f | Hyb: %f\n", i, insert_times[i], merge_times[i], hyb_times[i]);
        //fprintf(fp1,"%lu,%f,%f,%f\n",i, insert_times[i],merge_times[i], hyb_times[i]);

        //Key Comparison with Merge vs Hybrid only
        //printf("Comp size %lu: \t MS: %lu | Hyb %lu\n",i, key_comp_merg[i], key_comp_hyb[i]);
        //fprintf(fp,"%lu,%lu,%lu\n", i, key_comp_merg[i],key_comp_hyb[i]);

        //Time Analysis of Merge and Hyb
        //printf("Time size %lu: \t MS: %f | Hyb: %f\n", i, merge_times[i], hyb_times[i]);
        //fprintf(fp1,"%lu,%f,%f\n",i, merge_times[i], hyb_times[i]);

        //printf("Time: %f | Comp: %lu \n", merge_times[i], key_comp_merg[i]);
    }

    return 0;
}

void InsertionSort(int s, int *array){

    int test, i, j, temp;
    unsigned long count = 0;    //For key comparison

    //time(&begin);
    start = clock();

    for(test =0; test< TESTS; test++){    //For the same size, sort for TESTS times and return the average time
        //INSERTION SORT BEGINS HERE

        generateInput(s, array);

        for(i=1; i<s; i++){                 //Loop through each item in array (subject), start at index 1 since index0 has nth to compare to
            temp = array[i];            //save pivot to a variable else swapping will cause confusion
            for(j=i-1; j>=0; j--){             //Sub compare, move through every element before i (main) to decide
                count++;
                //printf("Count : %.02f\n", count);
                if(temp < array[j]){    //if main
                    array[j+1] = array[j]; //shift j to j+1 to make space
                    array[j] = temp;

                }else {
                    break;  //since going from right to left, alr sorted, if most right is alr smaller than just skip
                }
            }

        }

    }

    //time(&end);
    stop= clock();
    //insert_times[s] = (end-begin)/TESTS;
    insert_times[s] = (double)(stop - start)/CLOCKS_PER_SEC/TESTS;
    key_comp_ins[s] = count/TESTS;

}

void MergeSort(int s, int *array){

    int mid, i;
    if (s <2 ) return;

    mid = s/2;

    int l_size = mid;
    int r_size = s-mid;
    int *l_array = (int*)malloc(sizeof(int) * l_size);
    int *r_array = (int*)malloc(sizeof(int) * r_size);

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
    free(l_array);
    free(r_array);

}

void Hyb_InsertionSort(int s, int *array){

    int i, j, temp;

    for(i=1; i<s; i++){

        temp = array[i];
        for(j=i-1; j>=0; j--){
            hyb_key_comp++;

            if(temp < array[j]){    //if main
                array[j+1] = array[j]; //shift j to j+1 to make space
                array[j] = temp;
            }else {
                break;  //since going from right to left, alr sorted, if most right is alr smaller than just skip
            }
        }
    }
}


void Hyb_MergeSort(int s, int *array){

    int mid, i;
    //printf("S!!!!!   %d\n", s);
    if (s <2 ) return;
    if(s <= THRESHOLD){
        //printf("Switching to insertion\n");
        Hyb_InsertionSort(s, array);
    } else {
        mid = s/2;
        int l_size = mid;
        int r_size = s-mid;
        int *l_array = (int*)malloc(sizeof(int) * l_size);
        int *r_array = (int*)malloc(sizeof(int) * r_size);

        //Split into left right arrays
        for(i=0; i<mid; i++){
            l_array[i] = array[i];
        }

        for(i=mid; i<s; i++){
            r_array[i-mid] = array[i];
        }

        //Recursion
        Hyb_MergeSort(l_size, l_array);
        Hyb_MergeSort(r_size, r_array);

        //Sorting occurs while merging
        MergeArray(l_size, r_size, l_array, r_array, array); //Edits on the og array since pointers
        free(l_array);
        free(r_array);
    }



}

void MergeArray(int l_size, int r_size, int *l_array, int *r_array, int *array){

    int i =0; //Left array pointer
    int j=0; //Right array pointer
    int k=0; //Main array pointer

    //Compare left and right array and sort them back into main array
    //Do until one of the array reaches the end
    while(i<l_size && j<r_size){
        merge_key_comp++;
        hyb_key_comp++;
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
        merge_key_comp++;
        hyb_key_comp++;
        array[k] = l_array[i];
        i++;
        k++;
    }

    while(j< r_size){
            merge_key_comp++;
        hyb_key_comp++;
        array[k] = r_array[j];
        j++;
        k++;
    }

    //Since arrays are pointers already and are declared in function, the temp small arrays should disappear
    //after their respective recursions except for the main array, declared in main
}

//Generating random number
void generateInput(int size, int *array){
    int i;
    int rand;

    for(i=0; i< size; i++){         //size determine by main
        rand = random(1, MAX);
        array[i] = rand;
    }
}

int random(int min, int max){
   return min + rand() / (RAND_MAX / (max - min + 1) + 1);
}

void printArray(int s, int *array){
    for(int x=0; x<s; x++){
        printf("%lu ", array[x]);
    }
    printf("\n");
}


