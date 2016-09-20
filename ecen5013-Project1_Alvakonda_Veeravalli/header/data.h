#ifndef  _DATA_H
#define  _DATA_H
#include<stdio.h>
#include<stdint.h>

//Declaring all the functions of the user defined library memory

int8_t* my_itoa(int8_t *str, int32_t data, int32_t base);  //This function converts integer datatype to ascii string 
int32_t my_atoi(int8_t *str);                              //This function converts a given ascii string to integer datatype
void dump_memory(uint8_t* start,uint32_t length);          /*This function takes a pointer to a string and length of it and prints out the hex output                t
                                                            of individual elements in the string*/
uint32_t big_to_little(uint32_t data);                     //This function converts data fom big endian representation to little endian representation
uint32_t little_to_big(uint32_t data);                     //This function converts data from little endian representation to big endian representation

#endif
