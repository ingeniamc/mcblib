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

/** Struct used when no resource instance defined by user */
volatile bool ptFlag[MCB_NUMBER_RESOURCES];

__attribute__((weak))uint8_t Mcb_IntfReadIRQ(uint16_t u16Id)
{
    /*
     * Return a 0 which means LOW level indicating that the slave is not
     * available
     */
    return (uint8_t)0U;
}

__attribute__((weak))uint32_t Mcb_GetMillis(void)
{
    /** Return milliseconds */
    return (uint32_t)0U;
}

__attribute__((weak))void Mcb_RelinquishCPU(void)
{
    return;
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
    bool isCrcOk = true;
    /** Compute the CRC of the incoming frame, excluding its own CRC */
    uint16_t u16CompCrc = Mcb_IntfComputeCrc(pu16Buf, (u16Sz - 1));

    if (u16CompCrc != pu16Buf[u16Sz - 1])
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

__attribute__((weak))void Mcb_IntfInitResource(uint16_t u16Id)
{
    /** Init the resource instance, release state */
    if (u16Id < MCB_NUMBER_RESOURCES)
    {
        ptFlag[u16Id] = true;
    }
}

__attribute__((weak))void Mcb_IntfDeinitResource(uint16_t u16Id)
{
    /** Remove the resource instance */
    if (u16Id < MCB_NUMBER_RESOURCES)
    {
        ptFlag[u16Id] = false;
    }
}

__attribute__((weak))bool Mcb_IntfTryTakeResource(uint16_t u16Id)
{
    /** Non blocking resource take */
    bool isResTak = false;

    if (u16Id < MCB_NUMBER_RESOURCES)
    {
        if (ptFlag[u16Id] != false)
        {
            ptFlag[u16Id] = false;
            isResTak = true;
        }
    }
    return isResTak;
}

__attribute__((weak))bool Mcb_IntfTakeResource(uint16_t u16Id)
{
    /** Blocking resource take */
    bool isResTak = false;

    if (u16Id < MCB_NUMBER_RESOURCES)
    {
        while (ptFlag[u16Id] == false);

        ptFlag[u16Id] = false;
        isResTak = true;
    }
    return isResTak;
}

__attribute__((weak))void Mcb_IntfReleaseResource(uint16_t u16Id)
{
    if (u16Id < MCB_NUMBER_RESOURCES)
    {
        ptFlag[u16Id] = true;
    }
}
