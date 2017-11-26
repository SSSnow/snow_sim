#include "monitorM.h"

static unsigned char testBuffer[300]={0};

//数据内容发送
void comm_data_send(COMM_DATA* pcomm)
{
	unsigned char checksum=0;
	pcomm->head = 0x55;
	pcomm->cmd = 0xDD;
	pcomm->length = 36;
	for(unsigned char kk=0;kk<36;kk++)
	{
		checksum += pcomm->data[kk/4].c_data[kk%4];
	}
	pcomm->checksum = checksum;
	ioctrl(monitor_getID(),MONITOR_IOCTRL_COMM_DATA_WRITE,pcomm);
}


//数据标题发送
void comm_str_send(unsigned char* buffer,unsigned int size)
{
	unsigned char ChkSum = 0;
	for(unsigned char Index = size; Index > 0; Index --)
	{
		buffer[Index + 2] = buffer[Index - 1];
		ChkSum += buffer[Index - 1];
	}
	buffer[0] = 0x55;
	buffer[1] = 0xCC;
	buffer[2] = size;
	buffer[size + 3] = ChkSum;

	write(monitor_getID(), buffer, size + 4);
}



unsigned int str_len(unsigned char* buffer)
{
	unsigned int count = 0;
	while(buffer[count] != '\0')
	{
		 count ++;
	}
	return count;
}


void str_add(unsigned char* buffer,unsigned char* bufferAdd,unsigned int len)
{
	for(unsigned int i=0;i<len;i++)
	{
		buffer[i] = bufferAdd[i];
	}
}

unsigned char* get_monitor_buffer_ptr(void)
{
	return testBuffer;
}

unsigned char MonitorRecData(RecPackTypeDef* RecData)
{
	if(read(monitor_getID(), RecData, ParPack))
	{
		return 1;
	}
	else return 0;
}

unsigned char MonitorCommData(RecCommTypeDef* RecData)
{
	if(read(monitor_getID(), RecData, CommPack))
	{
		return 1;
	}
	else return 0;
}
