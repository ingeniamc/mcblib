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

#include "mcb_intf.h"

#define HSP_MAX_DATA_SZ 128

#define DFLT_TIMEOUT 100

typedef enum
{
    /* Blocking mode, each request block until response */
    MCB_BLOCKING = 0,
    /* Non Blocking mode, if not ready, return state */
    MCB_NON_BLOCKING
} Mcb_EMode;

typedef enum
{
    /* Message not ready */
    MCB_MESSAGE_NOT_READY = 0,
    /* Success request */
    MCB_MESSAGE_SUCCESS,
    /* Request error */
    MCB_MESSAGE_ERROR
} Mcb_EReqStatus;

/** Motion control but instance */
typedef struct
{
    /** Indicates if mcb is in cyclic mode */
    bool isCyclic;
    /** Linked Hsp module */
    Mcb_TIntf tIntf;
    /** Transmission mode */
    Mcb_EMode eMode;
} Mcb_TInst;

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
    Mcb_EReqStatus eStatus;
} Mcb_TMsg;

/** 
 * Initialization of a mcb instance 
 *
 * @param[in] ptInst
 *  Instance to be initialized
 * @param[in] eMode
 *  Indicates if blocking or non-blocking mode is applied
 */
void Mcb_Init(Mcb_TInst* ptInst, Mcb_EMode eMode, uint16_t u16Id);

/** Deinitializes a mcb instance */
void Mcb_Deinit(Mcb_TInst* ptInst);

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
Mcb_EReqStatus
Mcb_Write(Mcb_TInst* ptInst, Mcb_TMsg* mcbMsg, uint32_t u32Timeout);

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
Mcb_EReqStatus
Mcb_Read(Mcb_TInst* ptInst, Mcb_TMsg* mcbMsg, uint32_t u32Timeout);

/** Motion read/write functions */

/** Mapping functions */
void*
Mcb_TxMap(Mcb_TInst* ptInst, uint16_t u16Addr, uint16_t u16Sz);
void*
Mcb_RxMap(Mcb_TInst* ptInst, uint16_t u16Addr, uint16_t u16Sz);

/** Enabling cyclic mode.
 * Blocking function, while the config is written into driver. */
int32_t
Mcb_EnableCyclic(Mcb_TInst* ptInst);
/** Disable cyclic mode. */
int32_t
Mcb_DisableCyclic(Mcb_TInst* ptInst);

#endif
