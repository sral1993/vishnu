#include<stdio.h>
#include<string.h>

void main()
{ 
char  str[10];
int x,y,z,l1,l2,l3;
str[1]="This is a string";
str[2]="some NUMmbers12345";
str[3]="Does it reverse\n\0\t  correctly?";
l1=strlen(str[1]);
l2=strlen(str[2]);
l3=strlen(str[3]);
//method1
str[4]=  reverse(char *str, int length);
printf("%s",str[4]);
str[5]= char reverse(char *str, int length);
printf("%s",str5);
x=strcmp(str[4],str[5]);
if(x=0)
{
printf("strings are reversed properly");
}
else
{
printf("strings are not reversed properly");
}
//method2

str[6]= reverse(char *str, int length);
str[7]= strrev(str6);
y=strcmp(str[6],str[7]);
if(y=0)
{
printf("strings are reversed properly");
}
else
{
printf("strings are not reversed properly");
}

//method3---negative testing

x=strlen(str[1]);
str5=reverse(char *str, int length);
y=strlen(str[5]);

if(x==y)
{
printf("strings are reversed");
}
else
{
printf("strings are not reversed");
}


}


