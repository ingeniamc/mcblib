/**
 * @file mcb_frame.h
 * @brief This file contains useful functions for framing purpose
 *
 * @author  Firmware department
 * @copyright Ingenia Motion Control (c) 2018. All rights reserved.
 */

/**
 * \addtogroup InternalAPI MCB library
 * @{
 *
 *  Internal headers of the motion control bus library
 */
 
#ifndef MCB_FRAME_H
#define MCB_FRAME_H

#include <stdint.h>
#include <stdbool.h>

/** Maximum data size of the buffers */
#define MCB_MAX_DATA_SZ 128

/** Motion control frame config buffer header size (words) */
#define MCB_FRM_HEAD_SZ         1U
/** Motion control frame config buffer size (words)*/
#define MCB_FRM_CONFIG_SZ       4U
/** Motion control frame CRC size (words)*/
#define MCB_FRM_CRC_SZ          1U
/** Motion control frame MAX cyclic size (words)*/
#define MCB_FRM_MAX_CYCLIC_SZ   32U

/** Header position on raw buffer */
#define MCB_FRM_HEAD_IDX        0U
/** Configuration position on raw buffer */
#define MCB_FRM_CONFIG_IDX      1U
/** Cyclic position on raw buffer */
#define MCB_FRM_CYCLIC_IDX      5U

/** Ingenia protocol config function requests/replies */
/** Read request */
#define MCB_REQ_GETINFO         0U
/** Read request */
#define MCB_REQ_READ            1U
/** Write request */
#define MCB_REQ_WRITE           2U
/** Idle request  */
#define MCB_REQ_IDLE            7U

/** Acknowledge */
#define MCB_REP_ACK             3U
/** Error detected during read */
#define MCB_REP_GETINFO_ERROR   4U
/** Error detected during read */
#define MCB_REP_READ_ERROR      5U
/** Error detected during write */
#define MCB_REP_WRITE_ERROR     6U
/** General Error */
#define MCB_REP_ERROR           4U

/** Ingenia protocol segmentation definitions */
/** Bit value for not segmented frames */
#define MCB_FRM_NOTSEG          0U
/** Bit value for segmented frames */
#define MCB_FRM_SEG             1U

/** Get info cyclic Tx */
#define CYCLIC_TX (uint8_t)1
/** Get info cyclic Rx */
#define CYCLIC_RX (uint8_t)2

/** Get info int16 type */
#define INT16_TYPE          (uint16_t)0
/** Get info uint16 type */
#define UINT16_TYPE         (uint16_t)1
/** Get info int32 type */
#define INT32_TYPE          (uint16_t)2
/** Get info uint32 type */
#define UINT32_TYPE         (uint16_t)3
/** Get info float type */
#define FLOAT_TYPE          (uint16_t)4
/** Get info int16 type */
#define STRING_TYPE         (uint16_t)5

/** Get info struct */
typedef struct Mcb_TInfoData
{
    /** Register size in bytes */
    unsigned int u8Size         : 8;
    /** Register type info */
    unsigned int u8DataType     : 6;
    /** Register type of cyclic info */
    unsigned int u8CyclicType   : 2;
    /** Register access type info */
    unsigned int u8AccessType   : 3;
} Mcb_TInfoData;

/** Get info config data struct */
typedef union Mcb_TInfoMsgData
{
    /** Static data */
    uint16_t u16Data[MCB_MAX_DATA_SZ];
    /** Get info command structure */
    Mcb_TInfoData tInfoData;
} Mcb_TInfoMsgData;

/** High speed Ingenia protocol frame */
typedef struct {
	/** Data buffer */
	uint16_t u16Buf[MCB_MAX_DATA_SZ];
    /** Frame size */
	uint16_t u16Sz;
} Mcb_TFrame;

/**
 * Creates a configuration MCB frame.
 *
 * @param [out] tFrame
 *      Destination frame
 * @param [in] u16Addr
 *      Register address.
 * @param [in] u8Cmd
 *      Frame command (request or reply)
 * @param [in] u8Pending
 *      Indicates if the config data will be segmented.
 * @param [in] pCfgBuf
 *      Buffer with config data.
 * @param [in] bCalcCrc
 *  If true, @ref Mcb_IntfComputeCrc is called. This function has a built-in CRC,
 *  or it can be replaced with user-specific implementation.
 *  If false, no CRC function is called. Used when the CRC is automatically
 *  computed by hardware.
 *
 * @retval 0 success, error code otherwise
 */
int32_t
Mcb_FrameCreateConfig(Mcb_TFrame* tFrame, uint16_t u16Addr, uint8_t u8Cmd,
                      uint8_t u8Pending, const void* pCfgBuf, bool bCalcCrc);

/**
 * Add cyclic data into a pre-created config frame
 *
 * @note Mcb_FrameCreateConfig has to be used before this function
 *
 * @param [out] tFrame
 *      Destination frame
 * @param [in] pCyclicBuf
 *      Buffer with cyclic data.
 * @param [in] u16SzCyclic
 *      Size of the cyclic data.
 * @param [in] bCalcCrc
 *  If true, @ref Mcb_IntfComputeCrc is called. This function has a built-in CRC,
 *  or it can be replaced with user-specific implementation.
 *  If false, no CRC function is called. Used when the CRC is automatically
 *  computed by hardware.
 *
 * @retval 0 success, error code otherwise
 */
int32_t
Mcb_FrameAppendCyclic(Mcb_TFrame* tFrame, const void* pCyclicBuf,
                      uint16_t u16SzCyclic, bool bCalcCrc);

/**
 * Returns the address of the header.
 *
 * @param [in] tFrame
 *      Input frame.
 * @retval Address.
 */
uint16_t
Mcb_FrameGetAddr(const Mcb_TFrame* tFrame);

/**
 * Returns the command (request or reply) of the config data.
 *
 * @param [in] tFrame
 *      Input frame.
 * @retval Command.
 */
uint8_t
Mcb_FrameGetCmd(const Mcb_TFrame* tFrame);

/**
 * Checks if the config data is segmented and requires further data.
 *
 * @param [in] tFrame
 *      Input frame.
 * @retval true if config data is segmented.
 */
bool
Mcb_FrameGetSegmented(const Mcb_TFrame* tFrame);

/**
 * Returns the config data of a frame.
 *
 * @param [in] tFrame
 *  Input frame.
 * @param [out] pu16Buf
 *  Pointer to config data 
 * @retval config data
 */
uint16_t
Mcb_FrameGetConfigData(const Mcb_TFrame* tFrame, uint16_t* pu16Buf);

/**
 * Returns the cyclic data of a frame.
 *
 * @param [in] tFrame
 *  Input frame.
 * @param [out] pu16Buf
 *  Pointer to config data
 * @param[in] u16Size
 *  Sioze to be read
 *
 * @retval cyclic data
 */
uint16_t
Mcb_FrameGetCyclicData(const Mcb_TFrame* tFrame, uint16_t* pu16Buf, uint16_t u16Size);

/**
 * Indicates if the crc for the input frame is correct
 *
 * @param[in] tFrame
 *  Input frame
 *
 * @retval true if crc is correct
 *         false if crc is wrong
 */
bool
Mcb_FrameCheckCRC(const Mcb_TFrame* tFrame);

#endif /* MCB_FRAME_H */

/** @} */
