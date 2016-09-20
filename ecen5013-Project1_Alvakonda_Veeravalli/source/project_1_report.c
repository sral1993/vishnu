#include<stdio.h>
#include<stdint.h>
#include"memory.h"


void project_1_report()
{
int8_t i ;                                     
int8_t arr[32];                                //Creating an array of 32 bytes
uint8_t *aptr1, *aptr2, *aptr3;                //Declaring 3 pointers

//Initializing the addresses to which the pointesr should point to

aptr1=(uint8_t *) &arr[0];                     
aptr2=(uint8_t *)&arr[8];
aptr3=(uint8_t *)&arr[16];

//Initializing 16 elements of the array starting from the address aptr1 points to with the values 31-46

for (i=0; i<16; i++)
{
*aptr1=i+31;
aptr1++;
}
aptr1-=16;

//Initializing elements starting from the address aptr 3 to the values zero

my_memzero(aptr3,16);

//Moving 8 bytes of data from aptr1 to aptr3

my_memmove(aptr1, aptr3,8 );

//Moving 16 bytes of data from aptr2 to aptr1

my_memmove(aptr2, aptr1, 16);

//Reversing the entire array

my_reverse(aptr1, 32);

printf("The array after reschuffling is as follows:\n");
for(i=0; i<32; i++)
{
printf("%u,", *aptr1++);
}
aptr1-=32;
printf("\n");
} 





