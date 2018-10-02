/**
 * @file mcb.h
 * @brief This file contains API functions of the
 *        motion control bus (MCB)
 *
 * @author  Firmware department
 * @copyright Ingenia Motion Control (c) 2018. All rights reserved.
 */

 /**
 * \addtogroup MainAPI Main API
 *
 * @{
 *
 *  Main API containing all structs, variables and functions required
 *  to use Motion Control Bus.
 */
 
#ifndef MCB_H
#define MCB_H

#include "mcb_intf.h"

/** Maximum data size of the buffers */
#define MCB_MAX_DATA_SZ 128

/** Default timeout for blocking mode */
#define MCB_DFLT_TIMEOUT 500

/** Maximum number of mapped registers simultaneously */
#define MAX_MAPPED_REG (uint8_t)8U

/** Motion control bus mode of operation */
typedef enum
{
    /** Blocking mode, each request block until response */
    MCB_BLOCKING = 0,
    /** Non Blocking mode, if not ready, return state */
    MCB_NON_BLOCKING
} Mcb_EMode;

/** Frame data struct */
typedef struct
{
    /** Destination / source node */
    uint16_t u16Node;
    /** Target register address */
    uint16_t u16Addr;
    /** Master / slave command */
    uint16_t u16Cmd;
    /** Message total size (words) */
    uint16_t u16Size;
    /** Static data */
    uint16_t u16Data[MCB_MAX_DATA_SZ];
    /** Message status */
    Mcb_EStatus eStatus;
} Mcb_TMsg;

/** List struct to store mapped registers */
typedef struct
{
    /** Number of available register on the list */
    uint8_t u8Mapped;
    /** Word size of mapped registers */
    uint16_t u16MappedSize;
    /** Array containing key of mapped registers */
    uint16_t u16Addr[MAX_MAPPED_REG];
    /** Array containing size of mapped registers, in bytes */
    uint16_t u16Sz[MAX_MAPPED_REG];
} Mcb_TMappingList;

/** Motion control bus instance */
typedef struct Mcb_TInst Mcb_TInst;

/** Main motion control instance */
struct Mcb_TInst
{
    /** Indicates if mcb is in cyclic mode */
    volatile bool isCyclic;
    /** Indicates if a Cyclic to Config request is active */
    volatile bool isCyclic2Cfg;
    /** Indicates the timeout applied for blocking transmissions */
    uint32_t u32Timeout;
    /** Linked mcb module */
    Mcb_TIntf tIntf;
    /** Transmission mode */
    Mcb_EMode eMode;
    /** Config transmission Msg */
    Mcb_TMsg tConfig;
    /** Cyclic transmission (from MCB master point of view) buffer */
    uint16_t u16CyclicTx[MCB_FRM_MAX_CYCLIC_SZ];
    /** Cyclic reception (from MCB master point of view) buffer */
    uint16_t u16CyclicRx[MCB_FRM_MAX_CYCLIC_SZ];
    /** Cyclic transmission size */
    uint16_t u16CyclicSize;
    /** RX mapping (from MCB slave point of view) list */
    Mcb_TMappingList tCyclicRxList;
    /** TX mapping (from MCB slave point of view) list */
    Mcb_TMappingList tCyclicTxList;
    /** Callback to config over cyclic frame reception */
    void (*CfgOverCyclicEvnt)(Mcb_TInst* ptInst, Mcb_TMsg* pMcbMsg);
};

/** Motion control bus frame types*/
typedef enum
{
    /** Cyclic mode without sync */
    MCB_CYC_NON_SYNC = 0,
    /** Cyclic mode with sync enabled */
    MCB_CYC_SYNC
} Mcb_ECyclicMode;

/** 
 * Initialization of a mcb instance 
 *
 * @param[in] ptInst
 *  Instance to be initialized
 * @param[in] eMode
 *  Indicates if blocking or non-blocking mode is applied
 * @param[in] u16Id
 *  Assigns an Id to the instance
 * @param[in] bCalcCrc
 *  If true, @ref Mcb_IntfComputeCrc is called. This function has a built-in CRC,
 *  or it can be replaced with user-specific implementation.
 *  If false, no CRC function is called. Used when the CRC is automatically
 *  computed by hardware.
 * @param[in] u32Timeout
 *  Indicates the applied timeout for blocking tranmissions in milliseconds
 */
void Mcb_Init(Mcb_TInst* ptInst, Mcb_EMode eMode, uint16_t u16Id, bool bCalcCrc, uint32_t u32Timeout);

/**
 * Deinitializes a mcb instance
 *
 * @param[in] ptInst
 *  Instance to be deinitialized
 */
void Mcb_Deinit(Mcb_TInst* ptInst);

/**
 * Generic write function
 *
 * @param[in] ptInst
 *  Specifies the target instance
 * @param[in,out] mcbMsg
 *  Request to be send and load with reply
 *
 * @retval Status of the motion control instance
 */
Mcb_EStatus
Mcb_Write(Mcb_TInst* ptInst, Mcb_TMsg* mcbMsg);

/**
 * Generic read function
 *
 * @param[in] ptInst
 *  Specifies the target instance
 * @param[in,out] mcbMsg
 *  Request to be send and load with reply
 *
 * @retval Status of the motion control instance
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

/**
 * Map a Tx cyclic register into the cyclic buffer
 *
 * @note blocking function
 *
 * @param[in] ptInst
 *  Mcb instance where register is mapped
 * @param[in] u16Addr
 *  Key address of the register to be mapped
 * @param[in] u16Sz
 *  Size (bytes) of the register to be mapped
 *
 * @retval Pointer to cyclic buffer where data is located
 */
void*
Mcb_TxMap(Mcb_TInst* ptInst, uint16_t u16Addr, uint16_t u16Sz);

/**
 * Map a Rx cyclic register into the cyclic buffer
 *
 * @note blocking function
 *
 * @param[in] ptInst
 *  Mcb instance where register is mapped
 * @param[in] u16Addr
 *  Key address of the register to be mapped
 * @param[in] u16Sz
 *  Size (bytes) of the register to be mapped
 *
 * @retval Pointer to cyclic buffer where data is located
 */
void*
Mcb_RxMap(Mcb_TInst* ptInst, uint16_t u16Addr, uint16_t u16Sz);

/**
 * Unmap the last Tx mapped register
 *
 * @note blocking function
 *
 * @param[in] ptInst
 *  Mcb instance where register is mapped
 *
 * @retval Number of remaining mapped registers
 */
uint8_t
Mcb_TxUnmap(Mcb_TInst* ptInst);

/**
 * Unmap the last Rx mapped register
 *
 * @note blocking function
 *
 * @param[in] ptInst
 *  Mcb instance where register is mapped
 *
 * @retval Number of remaining mapped registers
 */
uint8_t
Mcb_RxUnmap(Mcb_TInst* ptInst);

/**
 * Unmap All the mapped register
 *
 * @note blocking function
 *
 * @param[in] ptInst
 *  Mcb instance
 */
void
Mcb_UnmapAll(Mcb_TInst* ptInst);

/**
 * Enables cyclic mode.
 *
 * @note Blocking function, while the config is written into driver.
 *
 * @param[in] ptInst
 *  Mcb instance
 *
 *  @retval 0 if we are already in cyclic mode.
 *          > 0 if transition successful, indicating the cyclic size.
 *          < 0 indicates an errorcode.
 */
int32_t
Mcb_EnableCyclic(Mcb_TInst* ptInst);

/**
 * Disables cyclic mode.
 *
 * @note This function only sets up the config frame, but
 *       data is not transmitted because cyclic mode is enabled and
 *       config frames must be transmitted through cyclic transfers.
 *
 * @param[in] ptInst
 *  Mcb instance
 *
 *  @retval MCB_CYCLIC_SUCCESS is disable is executed,
 *          MCB_CYCLIC_ERROR otherwise
 */
Mcb_EStatus
Mcb_DisableCyclic(Mcb_TInst* ptInst);

/**
 * Sets the desired cyclic mode.
 *
 * @note Blocking function, while the config is written into driver.
 *
 * @param[in] ptInst
 *  Mcb instance
 * @param[in] eNewCycMode Desired cyclic mode.
 */
int32_t
Mcb_SetCyclicMode(Mcb_TInst* ptInst, Mcb_ECyclicMode eNewCycMode);

/**
 * Function to be called cyclically when cyclic mode is enabled
 *
 * @param[in] ptInst
 *  Mcb instance
 *
 * @retval true if a cyclic transmission is done
 *         false otherwise
 */
bool
Mcb_CyclicProcess(Mcb_TInst* ptInst);

#endif

/** @} */
