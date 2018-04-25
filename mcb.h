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
    Mcb_EStatus eStatus;
} Mcb_TMsg;

/** Motion control bus instance */
typedef struct Mcb_TInst Mcb_TInst;

struct Mcb_TInst
{
    /** Indicates if mcb is in cyclic mode */
    bool isCyclic;
    /** Indicates the timeout applied for blocking transmissions */
    uint32_t u32Timeout;
    /** Linked Hsp module */
    Mcb_TIntf tIntf;
    /** Transmission mode */
    Mcb_EMode eMode;
    /** Config transmission Msg */
    Mcb_TMsg tConfig;
    /** Cyclic transmission buffer */
    uint16_t u16CyclicTx[MCB_FRM_MAX_CYCLIC_SZ];
    /** Cyclic reception buffer */
    uint16_t u16CyclicRx[MCB_FRM_MAX_CYCLIC_SZ];
    /** Cyclic transmission size */
    uint16_t u16CyclicSize;
    /** Callback to config over cyclic frame reception */
    void (*CfgOverCyclicEvnt)(Mcb_TInst* ptInst, Mcb_TMsg* pMcbMsg);
};

/** 
 * Initialization of a mcb instance 
 *
 * @param[in] ptInst
 *  Instance to be initialized
 * @param[in] eMode
 *  Indicates if blocking or non-blocking mode is applied
 * @param[in] u16Id
 *  Assigns an Id to the instance
 * @param[in] u32Timeout
 *  Indicates the applied timeout for blocking tranmissions in milliseconds
 */
void Mcb_Init(Mcb_TInst* ptInst, Mcb_EMode eMode, uint16_t u16Id, uint32_t u32Timeout);

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
Mcb_EStatus
Mcb_Write(Mcb_TInst* ptInst, Mcb_TMsg* mcbMsg);

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
Mcb_EStatus
Mcb_Read(Mcb_TInst* ptInst, Mcb_TMsg* mcbMsg);

/**
 * Attach an user callback to the reception event of a config frame over
 * Cyclic mode
 *
 * @param[in] ptInst
 *  Instance where callback is going to be linked
 * @param[in] Evnt
 *  User callback to be linked
 */
void
Mcb_AttachCfgOverCyclicCB(Mcb_TInst* ptInst, void (*Evnt)(Mcb_TInst* ptInst, Mcb_TMsg* pMcbMsg));

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

void
Mcb_CyclicProcess(Mcb_TInst* ptInst);

#endif
