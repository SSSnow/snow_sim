/*
 * device.h
 *
 *  Created on: Oct 11, 2017
 *      Author: snow
 */

#ifndef _DEVICE_H_
#define _DEVICE_H_

#include "driver.h"

#define DEV_DRV_NUMBER 10
#define DEV_NAME_LEN 15

typedef struct
{
  char name[DEV_NAME_LEN];
  unsigned int devID;
  DEV_DRV devDrv;
  void* private;
}DEV;


#endif /* SRC_DEVDRIVER_DEVICE_H_ */
