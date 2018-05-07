/**
 * @file mcb_usr.c
 * @brief This file contains functions to be implemented by user to
 *        migrate platform dependencies
 *
 * @author  Firmware department
 * @copyright Ingenia Motion Control (c) 2018. All rights reserved.
 */

#include "mcb_usr.h"
#include "mcb_checksum.h"

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

__attribute__((weak))uint16_t Mcb_IntfComputeCrc(const uint16_t* pu16Buf, uint16_t u16Sz)
{
    uint16_t u16Crc = CRC_START_XMODEM;

    for (uint16_t u16Idx = 0; u16Idx < u16Sz; u16Idx++)
    {
        u16Crc = update_crc_ccitt(u16Crc, (pu16Buf[u16Idx] >> 8) & 0xFF);
        u16Crc = update_crc_ccitt(u16Crc, (pu16Buf[u16Idx] & 0xFF));
    }
    return u16Crc;
}

__attribute__((weak))bool Mcb_IntfCheckCrc(uint16_t u16Id, const uint16_t* pu16Buf, uint16_t u16Sz)
{
    /** Check the CRC of the incoming data */

    bool isCrcOk = true;

    if (Mcb_IntfComputeCrc(pu16Buf, u16Sz) != 0)
    {
        isCrcOk = false;
    }
    return isCrcOk;
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
