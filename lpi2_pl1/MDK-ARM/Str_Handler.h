#ifndef __STR_HANDLER_
#define __STR_HANDLER_


#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

_Bool convert_str_hexa_uint32_t(uint32_t* value,char * addr);
_Bool convert_str_hexa_uint8_t(uint8_t* value,char * addr);
_Bool convert_str_hexa_uint16_t(uint16_t* value,char* addr);

#endif