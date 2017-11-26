/*
 * deriver.h
 *
 *  Created on: Oct 11, 2017
 *      Author: snow
 */

#ifndef _DRIVER_H_
#define _DRIVER_H_

#include <stddef.h>

typedef int (*pfunc_init)(void);
typedef int (*pfunc_open)(void);
typedef int (*pfunc_close)(void);
typedef int (*pfunc_suspend)(void);
typedef int (*pfunc_resume)(void);
typedef int (*pfunc_write)(void* buffer,unsigned int len);
typedef int (*pfunc_read)(void* buffer,unsigned int len);
typedef int (*pfunc_ioctrl)(unsigned char cmd,void* arg);

typedef struct
{
	pfunc_init init;
	pfunc_open open;
	pfunc_close close;
	pfunc_suspend suspend;
	pfunc_resume resume;
	pfunc_write write;
	pfunc_read read;
	pfunc_ioctrl ioctrl;
}DEV_DRV;

unsigned int register_driver(DEV_DRV* regDevDrv);
int init(unsigned int devID);
int open(unsigned int devID);
int close(unsigned int devID);
int suspend(unsigned int devID);
int resume(unsigned int devID);
int write(unsigned int devID,void* buffer,unsigned int len);
int read(unsigned int devID,void* buffer,unsigned int len);
int ioctrl(unsigned int devID,unsigned char cmd,void* arg);

#endif /* SRC_DEVDRIVER_DRIVER_H_ */
