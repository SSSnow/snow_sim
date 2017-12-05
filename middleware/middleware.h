/*
 * middleware.h
 *
 *  Created on: Oct 14, 2017
 *      Author: snow
 */

#ifndef _MIDDLEWARE_H_
#define _MIDDLEWARE_H_

#include<math.h>
#include "../devdriver/bsp.h"
#include "../modules/global_type.h"
#include "voltM.h"
#include "motoM.h"
#include "ledM.h"
#include "rfM.h"
#include "pressureM.h"
#include "acc_gyro_temp_M.h"
#include "monitorM.h"
#include "ano.h"

uint8_t middleware_init(void);

#endif /* SRC_MIDDLEWARE_MIDDLEWARE_H_ */
