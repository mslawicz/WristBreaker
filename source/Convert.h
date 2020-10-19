/*
 * Convert.h
 *
 *  Created on: 19.10.2020
 *      Author: Marcin
 */

#ifndef CONVERT_H_
#define CONVERT_H_

#define LO8(x)  static_cast<uint8_t>((x)&0xFFU) // NOLINT(hicpp-signed-bitwise)
#define HI8(x)  static_cast<uint8_t>(((x)&0xFF00U)>>8U) // NOLINT(hicpp-signed-bitwise)

#endif /* CONVERT_H_ */