/**
 * @file mcb_frame.c
 * @brief This file contains useful functions for framing purpose
 *
 * @author  Firmware department
 * @copyright Ingenia Motion Control (c) 2018. All rights reserved.
 */

#include "mcb_frame.h"
#include <string.h>
#include "mcb_checksum.h"

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

/** Ingenia protocol frame header size */
#define MCB_FRM_HDR_SZ         1U
/** Ingenia protocol frame CRC size */
#define MCB_FRM_CRC_SZ         1U

/**
 * Computes the CRC of the input frame
 *
 * @param[in] ptFrame
 *  Pointer to target frame
 *
 * @retval result of the CRC 
 */ 
uint16_t
Mcb_FrameCRC(const Mcb_TFrame* ptFrame);

int32_t Mcb_FrameCreate(Mcb_TFrame* tFrame, uint16_t u16Addr, uint8_t u8Cmd, uint8_t u8Pending, const void* pCfgBuf,
        const void* pCyclicBuf, uint16_t u16SzCyclic, bool calcCRC)
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

        /* Check dynamic buffer size */
        if (u16SzCyclic > 10)
        {
            i32Err = -2;
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

		memcpy(&tFrame->u16Buf[MCB_FRM_CYCLIC_IDX], pCyclicBuf,
              (sizeof(tFrame->u16Buf[0]) * u16SzCyclic));

        tFrame->u16Sz = MCB_FRM_HEAD_SZ + MCB_FRM_CONFIG_SZ + u16SzCyclic;
        if (calcCRC != false)
        {
            /* Compute CRC and add it to buffer */
            tFrame->u16Buf[tFrame->u16Sz] = Mcb_FrameCRC(tFrame);
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
    memcpy(pu16Buf, &tFrame->u16Buf[MCB_FRM_HEAD_SZ],
			(sizeof(tFrame->u16Buf[0]) * MCB_FRM_CONFIG_SZ));

    return MCB_FRM_CONFIG_SZ;
}

bool Mcb_FrameCheckCRC(const Mcb_TFrame* tFrame)
{
    bool bCRC = true;

    if (Mcb_FrameCRC(tFrame) != 0)
    {
        bCRC = false;
    }

    return bCRC;
}

uint16_t Mcb_FrameCRC(const Mcb_TFrame* tFrame)
{
    uint16_t u16Crc = CRC_START_XMODEM;

    for (uint16_t u16Idx = 0; u16Idx < tFrame->u16Sz; u16Idx++)
    {
        u16Crc = update_crc_ccitt(u16Crc, (tFrame->u16Buf[u16Idx] >> 8) & 0xFF);
        u16Crc = update_crc_ccitt(u16Crc, (tFrame->u16Buf[u16Idx] & 0xFF));
    }
    return u16Crc;
}
