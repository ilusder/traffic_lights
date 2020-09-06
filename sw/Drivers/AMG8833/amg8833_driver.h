//******************************************************************************

//    _____ _               _____                        _     _
//   |_   _| |             |  __ \                      | |   (_)
//     | | | |_   _  __ _  | |  | | ___ _ __ _   _  __ _| |__  _ _ __
//     | | | | | | |/ _` | | |  | |/ _ \ '__| | | |/ _` | '_ \| | '_ \
//    _| |_| | |_| | (_| | | |__| |  __/ |  | |_| | (_| | |_) | | | | |
//   |_____|_|\__, |\__,_| |_____/ \___|_|   \__, |\__,_|_.__/|_|_| |_|
//             __/ |                          __/ |
//            |___/                          |___/

//******************************************************************************
#include "stm32f0xx_hal.h"

#ifndef AMG8833_DRIVER_H_
#define AMG8833_DRIVER_H_



#endif /* AMG8833_DRIVER_H_ */


uint16_t AMG88GetImage(I2C_HandleTypeDef * i2cHandle);
uint16_t AMG88GetTemp(I2C_HandleTypeDef * i2cHandle);