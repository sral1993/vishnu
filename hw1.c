#include<stdio.h>
#include<stdint.h>
#include<string.h>
#include<hw1.h>

char reverse(char*str,int length)
{
char tmp;
char *out;
out= str+length-1;
if(length>0)
{
while(out>str)
{
tmp=*out;
*out=*str;
*str=tmp;
out--;
str++;
}
}
else
{
return(1);
}
}
