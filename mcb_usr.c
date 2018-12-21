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

/** Struct used when no mutex instance defined by user */
volatile bool ptFlag[MCB_NUMBER_MUTEX_RESOURCES];

__attribute__((weak))uint32_t Mcb_GetMillis(void)
{
    /** Return milliseconds */
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

__attribute__((weak))void Mcb_IntfInitMutex(uint16_t u16Id)
{
    /** Init the mutex instance, unlock state */
    if (u16Id < MCB_NUMBER_MUTEX_RESOURCES)
    {
        ptFlag[u16Id] = true;
    }
}

__attribute__((weak))void Mcb_IntfDeinitMutex(uint16_t u16Id)
{
    /** Remove the mutex instance */
    if (u16Id < MCB_NUMBER_MUTEX_RESOURCES)
    {
        ptFlag[u16Id] = false;
    }
}

__attribute__((weak))bool Mcb_IntfTryLockMutex(uint16_t u16Id)
{
    /** Non blocking mutex lock */
    bool isMutexLock = false;

    if (u16Id < MCB_NUMBER_MUTEX_RESOURCES)
    {
        if (ptFlag[u16Id] != false)
        {
            ptFlag[u16Id] = false;
            isMutexLock = true;
        }
    }
    return isMutexLock;
}

__attribute__((weak))void Mcb_IntfUnlockMutex(uint16_t u16Id)
{
    /** Unlock mutex instance */
    if (u16Id < MCB_NUMBER_MUTEX_RESOURCES)
    {
        ptFlag[u16Id] = true;
    }
}
