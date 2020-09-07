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

#include "amg8833_driver.h"

//*****************************************************************************
// AMG88 Device I2C address
//*****************************************************************************
#define AMG88_ADDR        (0x69 << 1)
#define  AMG88_THEMP       0X0E
#define  AMG88_IMAGE       0X80



uint16_t AMG88GetImage(I2C_HandleTypeDef * i2cHandle, 
                        uint16_t * pic, 
                        uint16_t pic_size)
{
    HAL_StatusTypeDef i2c_status;


    /* Read the data */
      
    i2c_status = HAL_I2C_Mem_Read(i2cHandle, AMG88_ADDR, 
                                 AMG88_IMAGE, I2C_MEMADD_SIZE_8BIT, 
                                 (uint8_t *) pic, 
                                 pic_size, 
                                 1000);
    if(i2c_status == HAL_OK)
    {

    }
    return(0);
}

//****************************************************************************
//
//! \brief Initialize the AMG8833 Camera device with defaults
//!         Reads the CHIP ID.
//!
//! \param[in]    i2cHandle        the handle to the openned i2c device
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
uint16_t AMG88GetTemp(I2C_HandleTypeDef * i2cHandle)
{
    uint16_t ucRegVal;

   HAL_StatusTypeDef i2c_status;

    /* Read the temp */
    i2c_status = HAL_I2C_Mem_Read(i2cHandle, AMG88_ADDR, 
                                 AMG88_THEMP, I2C_MEMADD_SIZE_8BIT, 
                                 (uint8_t *) &ucRegVal, 2, 
                                 100);
    if(i2c_status == HAL_OK)
    {
        return ucRegVal;
    }
    return(0);
}
