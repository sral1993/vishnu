#include "MKL25Z4.h"
#include<stdint.h>
#include<string.h>
#include"log.h"
#include<math.h>
#include<stdlib.h>
#include "itoa.h"


int8_t my_memmove(uint8_t * src,uint8_t *dst,uint32_t length);
int8_t my_memzero(uint8_t * src, uint32_t length);
int8_t my_reverse(uint8_t *src,uint32_t length);

void start();
void TPM2_Setup();
int stop();
typedef struct
{
uint8_t* HEAD;
uint8_t* TAIL;
uint8_t* Buffer;
}structure;

void my_ftoa(float n, char *res, int afterpoint);
int intToStr(int x, char str[], int d);
void reverse(char *str, int len);
int32_t my_atoi(int8_t *str);
//int32_t my_atoi(int8_t *str);

void TPM2_Setup()
{

	SIM_BASE_PTR->SCGC6 |= SIM_SCGC6_TPM2_MASK;
	SIM_BASE_PTR->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	TPM2_BASE_PTR->SC = TPM_SC_CMOD(1) | TPM_SC_PS(0);
	TPM2_BASE_PTR->MOD = 32000;
	TPM2_BASE_PTR->CONTROLS[0].CnSC=TPM_CnSC_MSB_MASK|TPM_CnSC_ELSA_MASK;
	TPM2_BASE_PTR->CONTROLS[0].CnV=TPM2_BASE_PTR->MOD;
}

void start()
{
	TPM2_Setup();
}

int stop()
{
	TPM2_BASE_PTR->SC = TPM_SC_CMOD(0);
	int count=TPM2_CNT;
	int x=count/21;
	return x;
}

void main()
{

	uint8_t *src,*dst;
	uint8_t arr[5000];
	uint8_t b[5000];
	src=(uint8_t *) arr;
	dst= (uint8_t *)b;
	uint32_t length=5000;
start();

my_reverse(src,100);
int time2=stop();
char str1[]="time for my_reverse for 100bytes in us :/n";
int l1=strlen(str1);
LOG1(str1,l1,time2,8);


start();

my_reverse(src,10);
int time3=stop();
char str2[]="time for my_reverse for 10bytes in us :/n";
int l2=strlen(str2);
LOG1(str2,l2,time3,8);

start();

my_reverse(src,1000);
int time4=stop();
char str3[]="time for my_reverse for 1000bytes in us :/n";
int l3=strlen(str3);
LOG1(str3,l3,time4,8);



}





void my_ftoa(float n, char *res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}
void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}
//Definition of the function memmove
int8_t my_memmove(uint8_t * src,uint8_t *dst,uint32_t length)
{
int k=0;
if (length >0) //checks if length is greater than 0
{
src=src+length-1;
dst=dst+length-1;
//Moving data from last address in Source to last address of the destination
//This method make sure that the data transfers securely even when there is an overlap in //source and destination addresses.
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
//printf("error"); //since length is 0, it returns error
}
}


int8_t my_memzero(uint8_t * src, uint32_t length)
{
int k=0;
//uint8_t j;
if(length>0) //checks if length is greater than 0
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
printf("error \n"); //since length is 0, it returns error
}
src=src-length+1;
}
int8_t my_reverse(uint8_t *src,uint32_t length)
{
uint8_t *trgt;
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
