

#ifndef _MEMORY_H
#define _MEMORY_H     
#include<stdint.h>

//DECLARING THE FUNCTION my_memmove

int8_t my_memmove(uint8_t * src,uint8_t *dst,uint32_t length);

//DECLARING THE FUNCTION my_memzero
int8_t my_memzero(uint8_t * src, uint32_t length);

//DECLARING THE FUNCTION my_memmove
int8_t my_reverse(uint8_t *src,uint32_t length);

#endif
