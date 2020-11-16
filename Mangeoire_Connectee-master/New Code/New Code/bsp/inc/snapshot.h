/*
 * snapshot.h
 *
 *  Created on: 14 déc. 2018
 *      Author: alexandre.ferroni
 */

#ifndef BSP_INC_SNAPSHOT_H_
#define BSP_INC_SNAPSHOT_H_

#include "main.h"

void init_snap(void);
void snap_irq_init(void);
void snap_irq_enable(void);

#endif /* BSP_INC_SNAPSHOT_H_ */
