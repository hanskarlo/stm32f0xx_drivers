/*
 * scd41.h
 *
 *  Created on: Jul. 28, 2023
 *      Author: hansj
 */

#ifndef INCLUDE_SCD41_H_
#define INCLUDE_SCD41_H_

#include <stdint.h>
#include "stm32f0xx_i2c.h"


class SCD41
{
    private:
		int test;
        
    public:
        SCD41();
        ~SCD41();


        uint8_t* getSerialNumber();
};

SCD41::SCD41(/* args */)
{
}

SCD41::~SCD41(){ }





#endif /* INCLUDE_SCD41_H_ */


