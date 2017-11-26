/*
 * device.c
 *
 *  Created on: Oct 11, 2017
 *      Author: snow
 */
#include "driver.h"
#include "device.h"

static DEV_DRV* devDrv[DEV_DRV_NUMBER] = {NULL};

unsigned int register_driver(DEV_DRV* regDevDrv)
{
	 unsigned int retdevDrvID = 0;
	 static unsigned int  devDrvCount = 0;
	 if(devDrvCount < DEV_DRV_NUMBER)
	 {
		 retdevDrvID = devDrvCount;
		 devDrv[retdevDrvID] = regDevDrv;
		 devDrvCount ++;
	 }
	 else
	 {
		 retdevDrvID = 0xFFFFFFFF;
	 }
	 return retdevDrvID;
}


int init(unsigned int devID)
{
	if(devID >= DEV_DRV_NUMBER)
	{
		return 0;
	}
	else
	{
		if(devDrv[devID]->init != NULL)
		{
			 return devDrv[devID]->init();
		}
	  else
		{
			 return 0;
		}
	}
}

int open(unsigned int devID)
{
	if(devID >= DEV_DRV_NUMBER)
	{
		return 0;
	}
	else
	{
		if(devDrv[devID]->open != NULL)
		{
			 return devDrv[devID]->open();
		}
	  else
		{
			 return 0;
		}
	}
}


int close(unsigned int devID)
{
	if(devID >= DEV_DRV_NUMBER)
	{
		return 0;
	}
	else
	{
		if(devDrv[devID]->close != NULL)
		{
			 return devDrv[devID]->close();
		}
	  else
		{
			 return 0;
		}
	}
}


int suspend(unsigned int devID)
{
	if(devID >= DEV_DRV_NUMBER)
	{
		return 0;
	}
	else
	{
		if(devDrv[devID]->suspend != NULL)
		{
			 return devDrv[devID]->suspend();
		}
	  else
		{
			 return 0;
		}
	}
}


int resume(unsigned int devID)
{
	if(devID >= DEV_DRV_NUMBER)
	{
		return 0;
	}
	else
	{
		if(devDrv[devID]->resume != NULL)
		{
			 return devDrv[devID]->resume();
		}
	  else
		{
			 return 0;
		}
	}
}


int write(unsigned int devID,void* buffer,unsigned int len)
{
	if(devID >= DEV_DRV_NUMBER)
	{
		return 0;
	}
	else
	{
		if(devDrv[devID]->write != NULL)
		{
			 return devDrv[devID]->write(buffer,len);
		}
	  else
		{
			 return 0;
		}
	}
}


int read(unsigned int devID,void* buffer,unsigned int len)
{
	if(devID >= DEV_DRV_NUMBER)
	{
		return 0;
	}
	else
	{
		if(devDrv[devID]->read != NULL)
		{
			 return devDrv[devID]->read(buffer,len);
		}
	  else
		{
			 return 0;
		}
	}
}


int ioctrl(unsigned int devID,unsigned char cmd,void* arg)
{
	if(devID >= DEV_DRV_NUMBER)
	{
		return 0;
	}
	else
	{
		if(devDrv[devID]->ioctrl != NULL)
		{
			 return devDrv[devID]->ioctrl(cmd,arg);
		}
	  else
		{
			 return 0;
		}
	}
}


