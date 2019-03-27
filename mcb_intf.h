/**
 * @file mcb_intf.h
 * @brief This file contains API for accessing to low level interface
 * 		  of the motion control bus (MCB)
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
 
#ifndef MCB_INTF_H
#define MCB_INTF_H

#include "mcb_usr.h"

/**
 * Initialize a Motion Control Bus interface
 *
 * @param[in]  ptInst
 *  Instace to be initialized
 */
void
Mcb_IntfInit(Mcb_TIntf* ptInst);

/**
 * Deinitialize Motion Control Bus interface
 *
 * @param[in] ptInst
 *  Instance to be deinitialized
 */
void
Mcb_IntfDeinit(Mcb_TIntf* ptInst);

/**
 * Resets the Motion Control Bus interface
 *
 * @param[in]  ptInst
 *  Instace to be reinitialized
 */
void
Mcb_IntfReset(Mcb_TIntf* ptInst);

/**
 * Execute a complete config write sequence through MCB
 *
 * @param[in] ptInst
 *  Target instance
 * @param[in] u16Node
 *  Target slave
 * @param[in] u16Addr
 *  Register address to be written
 * @param[in] pu16Data
 *  Data to be written
 * @param[in] pu16Sz
 *  Size to be written
 *
 * @retval Mcb_EStatus
 */
Mcb_EStatus
Mcb_IntfWrite(Mcb_TIntf* ptInst, uint16_t u16Node, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz);

/**
 * Execute a complete config read sequence through MCB
 *
 * @param[in] ptInst
 *  Target instance
 * @param[in] u16Node
 *  Target slave
 * @param[in] u16Addr
 *  Register address to be read
 * @param[out] pu16Data
 *  Data to be read
 * @param[out] pu16Sz
 *  Size of read words
 *
 * @retval Mcb_EStatus
 */
Mcb_EStatus
Mcb_IntfRead(Mcb_TIntf* ptInst, uint16_t u16Node, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz);

/**
 * Execute a complete config getinfo sequence through MCB
 *
 * @param[in] ptInst
 *  Target instance
 * @param[in] u16Node
 *  Target slave
 * @param[in] u16Addr
 *  Register address to be read
 * @param[out] pu16Data
 *  Data to be read
 * @param[out] pu16Sz
 *  Size of read words
 *
 * @retval Mcb_EStatus
 */
Mcb_EStatus
Mcb_IntfGetInfo(Mcb_TIntf* ptInst, uint16_t u16Node, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz);

/**
 * Process config data inside cyclic frames
 *
 * @param[in] ptInst
 *  Target instance
 * @param[in] u16Node
 *  Target slave
 * @param[in] u16Addr
 *  Register address to be read / write through config
 * @param[in, out] pu16Cmd
 *  Command to be sent
 * @param[in, out] pu16Data
 *  Data to be read / write through config
 * @param[in, out] pu16CfgSz
 *  Size of the configuration transmission
 * @param[in] pisNewData
 *  Indicates if a new config data must be added into cyclic frame
 *
 * @retval Mcb_EStatus
 */
Mcb_EStatus
Mcb_IntfCfgOverCyclic(Mcb_TIntf* ptInst, uint16_t u16Node, uint16_t u16Addr, uint16_t* pu16Cmd, uint16_t* pu16Data,
                      uint16_t* pu16CfgSz, bool* pisNewData);

/**
 * Execute a cyclic transfer through MCB
 *
 * @param[in] ptInst
 *  Target instance
 * @param[in] ptInBuf
 *  Cyclic data to be sent
 * @param[out] ptOutBuf
 *  Received Cyclic data
 * @param[in] u16CyclicSz
 *  Cyclic transmission size
 * @param[in] isNewData
 *  Indicates if a new config data must be added into cyclic frame
 *
 * @retval Mcb_EStatus
 */
void
Mcb_IntfCyclic(Mcb_TIntf* ptInst, uint16_t *ptInBuf, uint16_t *ptOutBuf, uint16_t u16CyclicSz, bool isNewData);
#endif /* MCB_INTF_H */

/** @} */
