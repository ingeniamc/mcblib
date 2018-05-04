/**
 * @file mcb_usr.c
 * @brief This file contains functions to be implemented by user to
 *        migrate platform dependencies
 *
 * @author  Firmware department
 * @copyright Ingenia Motion Control (c) 2018. All rights reserved.
 */

#include "mcb_usr.h"

__attribute__((weak))uint32_t Mcb_GetMillis(void)
{
    /** Return millisecons */
    return 0;
}

__attribute__((weak))bool Mcb_IntfIsReady(uint16_t u16Id)
{
    /** Check if SPI instance is ready for initiate a new transmission */
    return false;
}

__attribute__((weak))bool Mcb_IntfCheckCrc(uint16_t u16Id)
{
    /** Compute crc */
    return false;
}

__attribute__((weak))void Mcb_IntfSPITransfer(uint16_t u16Id, uint16_t* pu16In, uint16_t* pu16Out, uint16_t u16Sz)
{
    /** Set to low chip select pin */

    /** Launch a SPI transfer */

    /** Set to high chip select pint */
}

__attribute__((weak))void Mcb_IntfSyncSignal(uint16_t u16Id)
{

}
