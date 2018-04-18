/**
 * @file mcb.h
 * @brief This file contains API functions of the
 *        motion control bus (MCB)
 *
 * @author  Firmware department
 * @copyright Ingenia Motion Control (c) 2018. All rights reserved.
 */

#ifndef MCB_H
#define MCB_H

#include <stdint.h>
#include "mcb_intf.h"

#define HSP_MAX_DATA_SZ 128

#define DFLT_TIMEOUT 100

typedef enum
{
    /* Blocking mode, each request block until response */
    MCB_BLOCKING = 0,
    /* Non Blocking mode, if not ready, return state */
    MCB_NON_BLOCKING
} EMcbMode;

typedef enum
{
    /* Message not ready */
    MCB_MESSAGE_NOT_READY = 0,
    /* Success request */
    MCB_MESSAGE_SUCCESS,
    /* Request error */
    MCB_MESSAGE_ERROR
} EMcbReqStatus;

/** Motion control but instance */
typedef struct
{
    /** Indicates if mcb is in cyclic mode */
    bool isCyclic;
    /** Linked Hsp module */
    McbIntf tIntf;
    /** Transmission mode */
    EMcbMode eMode;
} McbInst;

/** Frame data struct */
typedef struct
{
    /* Node data */
    uint16_t u16Node;
    /* Address data */
    uint16_t u16Addr;
    /* Command data */
    uint16_t u16Cmd;
    /* Message total size (bytes) */
    uint16_t u16Size;
    /* Static data */
    uint16_t u16Data[HSP_MAX_DATA_SZ];
    /* Message status */
    EMcbReqStatus eStatus;
} McbMsg;

/** 
 * Initialization of a mcb instance 
 *
 * @param[in] ptInst
 *  Instance to be initialized
 * @param[in] eMode
 *  Indicates if blocking or non-blocking mode is applied
 */
void McbInit(McbInst* ptInst,  EMcbMode eMode);

/** Deinitializes a mcb instance */
void McbDeinit(McbInst* ptInst);

/**
 * Generic write function
 *
 * @param[in] ptInst
 *  Specifies the target instance
 * @param[in/out] mcbMsg
 *  Request to be send and load with reply
 * @param[in] u32Timeout
 *  Timeout duration
 */
EMcbReqStatus
McbWrite(McbInst* ptInst, McbMsg* mcbMsg, uint32_t u32Timeout);

/**
 * Generic read function
 *
 * @param[in] ptInst
 *  Specifies the target instance
 * @param[in/out] mcbMsg
 *  Request to be send and load with reply
 * @param[in] u32Timeout
 *  Timeout duration
 */
EMcbReqStatus
McbRead(McbInst* ptInst, McbMsg* mcbMsg, uint32_t u32Timeout);

/** Motion read/write functions */

/** Mapping functions */
void*
McbTxMap(McbInst* ptInst, uint16_t u16Addr, uint16_t u16Sz);
void*
McbRxMap(McbInst* ptInst, uint16_t u16Addr, uint16_t u16Sz);

/** Enabling cyclic mode.
 * Blocking function, while the config is written into driver. */
int32_t
McbEnableCyclic(McbInst* ptInst);
/** Disable cyclic mode. */
int32_t
McbDisableCyclic(McbInst* ptInst);

#endif
