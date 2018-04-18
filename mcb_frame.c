/**
 * @file mcb_frame.c
 * @brief This file contains useful functions for framing purpose
 *
 * @author  Firmware department
 * @copyright Ingenia Motion Control (c) 2018. All rights reserved.
 */

#include "mcb_frame.h"
#include <stdint.h>
#include <stdio.h>
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
/** Ingenia protocol frame dynamic buffer size */
#define MCB_FRM_MAX_CYCLIC_SZ   (MCB_FRM_MAX_DATA_SZ - MCB_FRM_HDR_SZ - \
                                 MCB_FRM_CONFIG_SZ - MCB_FRM_CRC_SZ)

/**
 * Computes the CRC of the input frame
 *
 * @param[in] ptFrame
 *  Pointer to target frame
 *
 * @retval result of the CRC 
 */ 
uint16_t
McbFrameCRC(const TMcbFrame* ptFrame);

int32_t McbFrameCreate(TMcbFrame* tFrame, uint16_t u16Addr, uint8_t u8Cmd,
                    uint8_t u8Pending, const void* pCfgBuf, const void* pCyclicBuf,
                    size_t szCyclic, bool calcCRC)
{
    int32_t err = 0;

    while (1)
    {
        THeader tHeader;     
        
        if (tFrame == NULL)
        {
            err = -1;
            break;
        }

        /* Check dynamic buffer size */
        if (szCyclic > 10)
        {
            err = -2;
            break;
        }
        
        tHeader.u12Addr = u16Addr;
        tHeader.u3Cmd = u8Cmd;
        tHeader.u1Pending = u8Pending;
        tFrame->u16Buf[MCB_FRM_HEAD_IDX] = tHeader.u16All;

        /* Copy config & cyclic buffer (if any) */
        if (pCfgBuf != NULL)
        {
			memcpy(&tFrame->u16Buf[MCB_FRM_CONFIG_IDX], pCfgBuf,
					(sizeof(tFrame->u16Buf[0]) * MCB_FRM_CONFIG_SZ));
        }
        else
        {
			memset(&tFrame->u16Buf[MCB_FRM_CONFIG_IDX], 0,
					(sizeof(tFrame->u16Buf[0]) * MCB_FRM_CONFIG_SZ));
        }

		memcpy(&tFrame->u16Buf[MCB_FRM_CYCLIC_IDX], pCyclicBuf,
              (sizeof(tFrame->u16Buf[0]) * szCyclic));

		tFrame->u16Sz = MCB_FRM_HEAD_SZ + MCB_FRM_CONFIG_SZ + szCyclic;
        if (calcCRC != false)
        {
            /* Compute CRC and add it to buffer */
            tFrame->u16Buf[tFrame->u16Sz] = McbFrameCRC(tFrame);
			tFrame->u16Sz += MCB_FRM_CRC_SZ;
        }
        break;
    }

    return err;
}

bool McbFrameGetSegmented(const TMcbFrame* tFrame)
{
    THeader tHeader;

    tHeader.u16All = tFrame->u16Buf[MCB_FRM_HEAD_IDX];
    
    return (bool)tHeader.u1Pending;
}

uint16_t McbFrameGetAddr(const TMcbFrame* tFrame)
{
    THeader tHeader;

    tHeader.u16All = tFrame->u16Buf[MCB_FRM_HEAD_IDX];
    
    return (uint16_t)tHeader.u12Addr;
}

uint8_t McbFrameGetCmd(const TMcbFrame* tFrame)
{
    THeader tHeader;

    tHeader.u16All = tFrame->u16Buf[MCB_FRM_HEAD_IDX];
    
    return (uint8_t)tHeader.u3Cmd;
}

uint16_t McbFrameGetConfigData(const TMcbFrame* tFrame, uint16_t* pu16Buf)
{
    memcpy(pu16Buf, &tFrame->u16Buf[MCB_FRM_HEAD_SZ],
			(sizeof(tFrame->u16Buf[0]) * MCB_FRM_CONFIG_SZ));

    return MCB_FRM_CONFIG_SZ;
}

bool McbFrameCheckCRC(const TMcbFrame* tFrame)
{
    bool bCRC = true;

    if (IpbFrameCRC(tFrame) != 0)
    {
        bCRC = false;
    }

    return bCRC;
}

uint16_t McbFrameCRC(const TMcbFrame* tFrame)
{
    uint16_t u16Crc = CRC_START_XMODEM;

    for (uint16_t u16Idx = 0; u16Idx < tFrame->u16Sz; u16Idx++)
    {
        u16Crc = update_crc_ccitt(u16Crc, (tFrame->u16Buf[u16Idx] >> 8) & 0xFF);
        u16Crc = update_crc_ccitt(u16Crc, (tFrame->u16Buf[u16Idx] & 0xFF));
    }
    return u16Crc;
}
