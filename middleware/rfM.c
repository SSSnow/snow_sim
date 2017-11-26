/*
 * rfM.c
 *
 *  Created on: Oct 14, 2017
 *      Author: snow
 */

#include "rfM.h"

static unsigned char recPackageFlag = 0;
static unsigned int rfLostCount = 1000;//默认是掉信号
unsigned char rfRelinkClose = 0;

NJZY_RF_STR recRFPackage = {0};

NJZY_CONTROL_DATA remoteData = {0};

void rc_data_updata(void)
{
	recPackageFlag = read(cc2530_getID(), &recRFPackage, 0);

	if(recPackageFlag == 1)
	{
		if(recRFPackage.status < 50){//not lost signal rssi is dbm when value in -50~0 is better < -70 is lost
				for(unsigned char i = 0; i < 12; i ++)
					remoteData.RemoteD[i] = ((unsigned short *)recRFPackage.channel)[i];
				rfLostCount = 0;
		}else{
				rfLostCount ++;
				if(rfLostCount >= 2000)
				{
					rfLostCount = 2000;
				}
		}
		rfRelinkClose = 1;
	}
	else
	{
		rfLostCount ++;
		if(rfLostCount >= 2000)
		{
			 rfLostCount = 2000;
		}
	}
}


NJZY_SEND_STR NJZY_Send_Data = {0};
void rf_relink(void)
{
	NJZY_Send_Data.h1 = 0x4E;
	NJZY_Send_Data.h2 = 0x59;
	NJZY_Send_Data.len = 8;
	NJZY_Send_Data.type = NJZY_PACKET_TYPE_BIND;
	NJZY_Send_Data.buffer[0] = 3;
	NJZY_Send_Data.buffer[1] = 6;
	NJZY_Send_Data.buffer[2] = 'B';
	NJZY_Send_Data.buffer[3] = 'I';
	NJZY_Send_Data.buffer[4] = 'N';
	NJZY_Send_Data.buffer[5] = 'D';
	NJZY_Send_Data.buffer[6] = crcRfCal((unsigned char *)&NJZY_Send_Data.len, NJZY_Send_Data.len);
	write(cc2530_getID(),(unsigned char* )&NJZY_Send_Data, 11);
}


uint8_t rfIsLost(void)
{
	if(rfLostCount > 200)
		return 1;
	else
		return 0;
}

NJZY_CONTROL_DATA* get_rf_data(void)
{
	return &remoteData;
}

unsigned char get_rf_relink_flag(void)
{
	return rfRelinkClose;
}
