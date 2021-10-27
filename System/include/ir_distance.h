/*
 * irDistance.h
 *
 *  Created on: 29 Jul 2020
 *      Author: an
 */

#include <stdint.h>

#ifndef SYSTEM_INCLUDE_IRDISTANCE_H_
#define SYSTEM_INCLUDE_IRDISTANCE_H_

void ir_distances(uint32_t *left, uint32_t *center, uint32_t *right);
void read_adc_17_14_16(uint32_t *ch17, uint32_t *ch12, uint32_t *ch16);
void adc_init_channel_17_14_16(void);

#endif /* SYSTEM_INCLUDE_IRDISTANCE_H_ */
