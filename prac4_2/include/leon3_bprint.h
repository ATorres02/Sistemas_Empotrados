/*
 * leon3_bprint.h
 *
 *  Created on: 05/03/2022
 *      Author: torres
 */

#ifndef LEON3_BPRINT_H_
#define LEON3_BPRINT_H_

#include "leon3_types.h"
#include "leon3_uart.h"

int8_t leon3_print_string(char* str);
int8_t leon3_print_uint8(uint8_t i);
int8_t leon3_print_uint32(uint32_t i);

#endif /* LEON3_BPRINT_H_ */
