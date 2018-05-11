/**
 * @file mcb_frame.c
 * @brief This file contains useful functions for framing purpose
 *
 * @author  Firmware department
 * @copyright Ingenia Motion Control (c) 2018. All rights reserved.
 */

#include "mcb_frame.h"
#include "mcb_usr.h"
#include <string.h>

/** Frame description
 * Word 0       - Header
 * Word 1..4    - Config data
 * Word 5..N-1  - Cyclic  data (optional)
 * Word N       - CRC
 */

/** Header elements */
typedef union
{
    struct
    {
        /** Segmented message */
        uint16_t u1Pending :1;
        /** Frame command identification */
        uint16_t u3Cmd :3;
        /** Address of the Static Data */
        uint16_t u12Addr :12;
    };
    uint16_t u16All;
} THeader;

int32_t Mcb_FrameCreateConfig(Mcb_TFrame* tFrame, uint16_t u16Addr, uint8_t u8Cmd, uint8_t u8Pending,
        const void* pCfgBuf, bool bCalcCrc)
{
    int32_t i32Err = 0;

    while (1)
    {
        THeader tHeader;

        if (tFrame == NULL)
        {
            i32Err = -1;
            break;
        }

        tHeader.u12Addr = u16Addr;
        tHeader.u3Cmd = u8Cmd;
        tHeader.u1Pending = u8Pending;
        tFrame->u16Buf[MCB_FRM_HEAD_IDX] = tHeader.u16All;

        /* Copy config & cyclic buffer (if any) */
        if (pCfgBuf != NULL)
        {
            memcpy(&tFrame->u16Buf[MCB_FRM_CONFIG_IDX], pCfgBuf, (sizeof(tFrame->u16Buf[0]) * MCB_FRM_CONFIG_SZ));
        }
        else
        {
            memset(&tFrame->u16Buf[MCB_FRM_CONFIG_IDX], 0, (sizeof(tFrame->u16Buf[0]) * MCB_FRM_CONFIG_SZ));
        }

        tFrame->u16Sz = MCB_FRM_HEAD_SZ + MCB_FRM_CONFIG_SZ;

        if (bCalcCrc != false)
        {
            /* Compute CRC and add it to buffer */
            tFrame->u16Buf[tFrame->u16Sz] = Mcb_IntfComputeCrc(tFrame->u16Buf, tFrame->u16Sz);
            tFrame->u16Sz += MCB_FRM_CRC_SZ;
        }
        break;
    }

    return i32Err;
}

int32_t Mcb_FrameAppendCyclic(Mcb_TFrame* tFrame, const void* pCyclicBuf, uint16_t u16SzCyclic, bool bCalcCrc)
{
    int32_t i32Err = 0;

    while (1)
    {
        if (tFrame == NULL)
        {
            i32Err = -1;
            break;
        }

        /* Check dynamic buffer size */
        if (u16SzCyclic > MCB_FRM_MAX_CYCLIC_SZ)
        {
            i32Err = -2;
            break;
        }

        /* Copy config & cyclic buffer (if any) */
        if (pCyclicBuf != NULL)
        {
            memcpy(&tFrame->u16Buf[MCB_FRM_CYCLIC_IDX], pCyclicBuf, (sizeof(tFrame->u16Buf[0]) * u16SzCyclic));
        }
        else
        {
            memset(&tFrame->u16Buf[MCB_FRM_CYCLIC_IDX], 0, (sizeof(tFrame->u16Buf[0]) * u16SzCyclic));
        }

        tFrame->u16Sz += u16SzCyclic;

        if (bCalcCrc != false)
        {
            /* Compute CRC and add it to buffer */
            tFrame->u16Buf[tFrame->u16Sz] = Mcb_IntfComputeCrc(tFrame->u16Buf, tFrame->u16Sz);
            tFrame->u16Sz += MCB_FRM_CRC_SZ;
        }
        break;
    }

    return i32Err;
}

bool Mcb_FrameGetSegmented(const Mcb_TFrame* tFrame)
{
    THeader tHeader;

    tHeader.u16All = tFrame->u16Buf[MCB_FRM_HEAD_IDX];
    
    return (bool)tHeader.u1Pending;
}

uint16_t Mcb_FrameGetAddr(const Mcb_TFrame* tFrame)
{
    THeader tHeader;

    tHeader.u16All = tFrame->u16Buf[MCB_FRM_HEAD_IDX];
    
    return (uint16_t)tHeader.u12Addr;
}

uint8_t Mcb_FrameGetCmd(const Mcb_TFrame* tFrame)
{
    THeader tHeader;

    tHeader.u16All = tFrame->u16Buf[MCB_FRM_HEAD_IDX];
    
    return (uint8_t)tHeader.u3Cmd;
}

uint16_t Mcb_FrameGetConfigData(const Mcb_TFrame* tFrame, uint16_t* pu16Buf)
{
    memcpy(pu16Buf, &tFrame->u16Buf[MCB_FRM_CONFIG_IDX], (sizeof(tFrame->u16Buf[0]) * MCB_FRM_CONFIG_SZ));

    return MCB_FRM_CONFIG_SZ;
}

uint16_t Mcb_FrameGetCyclicData(const Mcb_TFrame* tFrame, uint16_t* pu16Buf, uint16_t u16Size)
{
    memcpy(pu16Buf, &tFrame->u16Buf[MCB_FRM_CYCLIC_IDX], (sizeof(tFrame->u16Buf[0]) * u16Size));

    return MCB_FRM_CONFIG_SZ;
}
