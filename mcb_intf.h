/**
 * @file mcb_intf.f
 * @brief This file contains API for accessing to low level interface
 * 		  of the motion control bus (MCB)
 *
 * @author  Firmware department
 * @copyright Ingenia Motion Control (c) 2018. All rights reserved.
 */

#ifndef MCB_INTF_H
#define MCB_INTF_H

#include "mcb_usr.h"

/** Initialize a High speed protocol interface */
void
Mcb_IntfInit(Mcb_TIntf* ptInst);

/** Deinitialize a high speed protocol interface */
void
Mcb_IntfDeinit(Mcb_TIntf* ptInst);

/** Write frame */
Mcb_EStatus
Mcb_IntfWrite(Mcb_TIntf* ptInst, uint16_t u16Node, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz);
/** Read frame */
Mcb_EStatus
Mcb_IntfRead(Mcb_TIntf* ptInst, uint16_t u16Node, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz);

Mcb_EStatus
Mcb_IntfCyclicTranfer(Mcb_TIntf* ptInst, uint16_t u16Node, uint16_t u16Addr, uint16_t u16Cmd, uint16_t* pu16Data,
        uint16_t* pu16Sz, uint16_t *ptInBuf, uint16_t *ptOutBuf);
#endif /* MCB_INTF_H */
