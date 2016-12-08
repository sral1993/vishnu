/*
 * itoa.c
 *
 *  Created on: Nov 25, 2016
 *      Author: Prithvi
 */


#ifndef SOURCES_LOG_H_
#define SOURCES_LOG_H_
#include <stdint.h>

void LOG(char *p,int len);
void LOG1(char *p,int l,int param,int data_type_size);



#endif /* SOURCES_LOG_H_ */

#include "itoa.h"

int8_t* my_itoa(int8_t *str, int32_t data, int32_t base)
{
int ascii_length, i, temp;
int8_t *reverse;
int8_t temp0;
//Initializing the pointer reverse with the address passed in as the argument
reverse=str;
ascii_length=0;
//If the entered integer input data is positive
if(data>=0)
{
temp=data;
//Finding the length of the ascii string to be generated
while(temp!=0)
{
temp=temp/base;
ascii_length++;
}
temp=data;
i=0;
/*Isolating the individual numbers (0-9) of the input integer data and storing them in a string which is to be converted to ascii. However the order of the numbers in the string are in reverse order. So the string need to be reversed before converting them in to ascii values*/
while(i<ascii_length)
{
*str=(int8_t) (temp%10);
temp=temp/base;
i++;
str++;
}
str--;
//Reversing the string of individual numbers(0-9) of the input integer data so that it can be //converted to their respective ascii values later
while(str>reverse)
{
temp0=*str;
*str=*reverse;
*reverse=temp0;
str--;
reverse++;
}
if(ascii_length%2)
str=str-ascii_length/2;
else
str=str-(ascii_length/2-1);
i=0;
//Replacing the string of numbers with their respective ASCII values and now the the //ASCII string of the given input integer data is ready
while(i<ascii_length)
{
*str=*str+48;
str++;
i++;
}
*str=0;
//Making the str pointer point to the 1st element of the ASCII string for the given integer //input data
str=str-ascii_length;
}
//If the integer input data to be converted is negative
else
{
*str=45;
str++;
temp=(data)*(-1);
//Finding the length of the ascii string of the given integer input data
while(temp!=0)
{
temp=temp/base;
ascii_length++;
}
temp=(data)*(-1);
i=0;
/*Isolating the individual numbers (0-9) of the input integer data and storing them in a string which is to be converted to ascii. However the order of the numbers in the string are in reverse order. So the string need to be reversed before converting them in to ascii values*/
while(i<ascii_length)
{
*str=(int8_t) (temp%10);
temp=temp/base;
i++;
str++;
}
str--;
reverse++;
//Reversing the string of individual numbers(0-9) of the input integer data so that it can be //converted to their respective ascii values later
while(str>reverse)
{
temp0=*str;
*str=*reverse;
*reverse=temp0;
str--;
reverse++;
}
if(ascii_length%2)
str=str-ascii_length/2;
else
str=str-(ascii_length/2-1);
i=0;
while(i<ascii_length)
{
*str += 48;
str++;
i++;
}
*str=0;
str=str-ascii_length;
return str;
}
}

