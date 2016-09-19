
//Including Header files
#include "memory.h"
#include<stdint.h>
#include<stdio.h>

//Definition of the function memmove

int8_t my_memmove(uint8_t * src,uint8_t *dst,uint32_t length)
{
int k=0;
if (length >0)      //checks if length is greater than 0
{
src=src+length-1;
dst=dst+length-1;
//Moving data from last address in Source to last address of the destination
//This method make sure that the data transfers securely even when there is an overlap in source and destination addresses.
while(k<length)                     
{
*dst=*src;
src--;
dst--;
k++;
}
return 0;
}
else
{
printf("error");     //since length is 0, it returns error
}
}

//Definition of the function memzero



int8_t my_memzero(uint8_t * src, uint32_t length)
{
int k=0;
//uint8_t j;
if(length>0)            //checks if length is greater than 0
{

// zeroes all the data in source addresses by making use of pointers 
for(k=0;k<length;k++)
{
*src=0;                   
src++;
}
}
else
{
printf("error  \n");    //since length is 0, it returns error
}
src=src-length+1;
}


//Definition of the function reverse

int8_t my_reverse(uint8_t *src,uint32_t length)
{
uint8_t  *trgt;
int temp;
trgt=src+length-1;

//Initializing the target with source last address and then transferring the data to source using temporary variable 
while(trgt>src)
{
temp=*trgt;
*trgt=*src;
*src=temp;
trgt--;
src++;
}
src=src-length+1;
return 1;
}
