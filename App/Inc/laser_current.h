/*
 * laser_current.h
 *
 *  Created on: Feb 10, 2026
 *      Author: Elizabeth
 */

#ifndef INC_LASER_CURRENT_H_
#define INC_LASER_CURRENT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void LaserCurrent_NotifyThermalError(uint8_t error_code);

#ifdef __cplusplus
}
#endif

#endif /* INC_LASER_CURRENT_H_ */
