#include "amg8833_driver.h"

//*****************************************************************************
// AMG88 Device I2C address
//*****************************************************************************
#define AMG88_ADDR        (0x69 << 1)
#define  AMG88_CONTROL     0x00
#define  AMG88_DATA        0X01
#define  AMG88_THEMP       0X0E
#define  AMG88_IMAGE       0X80



int AMG88GetImage(I2C_HandleTypeDef i2cHandle)
{
    HAL_StatusTypeDef i2c_status;
    uint16_t ucRegVal[64];


    /* Read the data */
      
    i2c_status = HAL_I2C_Mem_Read(&i2cHandle, AMG88_ADDR, 
                                 AMG88_IMAGE, 64 * 2, 
                                 (uint8_t *) ucRegVal, 64 * 2, 
                                 100);
    if(i2c_status == 0)
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
int AMG88GetTemp(I2C_HandleTypeDef i2cHandle, float * temp)
{
    uint16_t ucRegVal;

   HAL_StatusTypeDef i2c_status;

    /* Read the temp */
    i2c_status = HAL_I2C_Mem_Read(&i2cHandle, AMG88_ADDR, 
                                 AMG88_THEMP, 2, 
                                 (uint8_t *) &ucRegVal, 2, 
                                 100);
    if(i2c_status == 0)
    {
        * temp = ucRegVal * 0.0625;
        return(1);
    }
    return(0);
}
