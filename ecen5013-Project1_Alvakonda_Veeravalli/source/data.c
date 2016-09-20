#include<stdio.h>
#include<stdint.h>
#include"data.h"

//Defining a function my_itoa which is used to convert integer data into an ascii string and return it

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

//Reversing the string of individual numbers(0-9) of the input integer data so that it can be converted to their respective ascii values later

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

//Replacing the string of numbers with their respective ASCII values and now the the ASCII string of the given input integer data is ready

while(i<ascii_length)
{
*str=*str+48;
str++;
i++;
}
*str=0;

//Making the str pointer point to the 1st element of the ASCII string for the given integer input data

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
//Reversing the string of individual numbers(0-9) of the input integer data so that it can be converted to their respective ascii values later

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
*str=*str+48;
str++;
i++;
}
*str=0;
str=str-ascii_length;
return str;
}
}



//Defining a function my_atoi which is used to convert a given ascii string into integer data and return it


int32_t my_atoi(int8_t *str)
{
int32_t integer=0;
int8_t zero_ascii=48;

//Checking if the integer data is positive or negative

//If the integer data is positive

if(*str==45)
{
str++;
while((*str)!=0)
{
integer=integer*10+(int32_t)(*str-zero_ascii);
str++;
}
integer=-integer;
}

//If the integer data is negative

else
{
while((*str)!=0)
{
integer=integer*10+(int32_t)(*str-zero_ascii);
str++;
}
}
return integer;
}


//Defining a function dump_memory which takes a pointer to a string and prints out the hex output of the individual elements in the string

void dump_memory(uint8_t* start,uint32_t length)
{
uint32_t quo;
int count;
int i=1,j,k,tmp;
char hex[10];
for(k=1;k<=length;k++)
{
count=0;
quo=*start;       //Assigning *start to quotient which is the initial value of the pointing address
while(quo!=0)
{
count++;
tmp=quo % 16;     // Assigning quo mod 16 to the temporary varaible in order to find whether the values are 0 to 9 or from A to F
if(tmp <10)
tmp +=48;         // Adding 48 to temp moves the decimal value to Hexadecimal value for the range 0 to 9
else
tmp +=55;         // Adding 55 to temp moves the decimal value to Hexadecimal value for the range A to F
hex[i++]=tmp;
quo=quo/16;
}
if(count==1)
hex[i++]=48;            // Appending zero for single digit output
else if (count==0)
{
hex[i++]=48;            // Appending zero for double digit output
hex[i++]=48;
}
printf("\n hexadecimal value of decimal number  %u is ",*start);
for(j=i-1;j>i-3;j--)
{
printf("%c",hex[j]);     // printing the Hexadecimal OUTPUT
}

start++;
}
printf("\n");

}

//Defining function big_to_little which converts data in big endian representation to little endian representation


uint32_t big_to_little(uint32_t data)
{
uint8_t *big_ptr, *small_ptr, temp;
uint32_t little_endian,*small_endian_pointer;

//Assigning the address of data to big_ptr and typecasting the pointer to uint8_t i.e. it points to data of length 1 byte

big_ptr=(uint8_t *)&data;

//Assigning the address of the byte with the highest address i.e the least significant bytes address to the pointer variable small_ptr

small_ptr=big_ptr+3;

//Converting data from big endian to little endian format i.e. reversing the order of the bytes

while(big_ptr<small_ptr)
{
temp=*big_ptr;
*big_ptr=*small_ptr;
*small_ptr=temp;
small_ptr--;
big_ptr++;
}

small_ptr--;
small_endian_pointer=(uint32_t *) small_ptr;
little_endian= *small_endian_pointer;
return little_endian;
}


//Defining a function little_to_big which converts data in big endian representation to small endian


uint32_t little_to_big(uint32_t data)
{
uint8_t *little_ptr, *big_ptr, temp;
uint32_t big_endian,*big_endian_pointer;

//Assigning the address of data to little_ptr and typecasting the pointer to uint8_t i.e. it points to data of length 1 byte

little_ptr=(uint8_t *)&data;

//Assigning the address of the byte with the highest address to the variable big_ptr

big_ptr=little_ptr+3;

//Converting data from little endian representation to big endian representation

while(big_ptr>little_ptr)
{
temp=*big_ptr;
*big_ptr=*little_ptr;
*little_ptr=temp;
little_ptr++;
big_ptr--;
}
big_ptr--;
big_endian_pointer=(uint32_t *) big_ptr;
big_endian= *big_endian_pointer;
return big_endian;
}
