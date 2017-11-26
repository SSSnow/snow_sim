#ifndef _MONITORM_H_
#define _MONITORM_H_

#include "../devdriver/monitor.h"

void comm_data_send(COMM_DATA* pcomm);
void comm_str_send(unsigned char* buffer,unsigned int size);
unsigned int str_len(unsigned char* buffer);
void str_add(unsigned char* buffer,unsigned char* bufferAdd,unsigned int len);
unsigned char* get_monitor_buffer_ptr(void);
unsigned char MonitorRecData(RecPackTypeDef* RecData);
unsigned char MonitorCommData(RecCommTypeDef* RecData);

#endif 
