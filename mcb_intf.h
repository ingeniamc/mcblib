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

#include <stdint.h>
#include <stdbool.h>
#include "mcb_frame.h"

/** McbIntf communication states */
typedef enum
{
    /** Transmission successful */
	MCB_SUCCESS = 0,
    /** Bus in stand by */
	MCB_STANDBY,
    /** Sending a write request */
	MCB_WRITE_REQUEST,
    /** Waiting for write request ack */
	MCB_WRITE_REQUEST_ACK,
    /** Processing answer from write request */
	MCB_WRITE_ANSWER,
    /** Processing write */
	MCB_WRITE_ANSWER_PENDING,
    /** Sending a read request */
	MCB_READ_REQUEST,
    /** Waiting for read request ack */
	MCB_READ_REQUEST_ACK,
    /** Processing answer from read request */
	MCB_READ_ANSWER,
    /** Processing request */
	MCB_READ_REQUEST_PENDING,
    /** Waiting and processing slave cyclic frame */
	MCB_CYCLIC_ANSWER,
    /** Cancel transaction */
	MCB_CANCEL,
    /** Transaction error */
	MCB_ERROR
} EMcbStatus;

typedef struct McbIntf McbIntf;

struct McbIntf
{
    /** Indicates the state of the communication bus */
	EMcbStatus eState;
    /** IRQ Event signal */
    bool isIrqEvnt;
    /** Frame pool for holding tx data */
	TMcbFrame tTxfrm;
    /** Frame pool for holding rx data */
	TMcbFrame tRxfrm;
    /** Pending data size to be transmitted/received */
    uint16_t u16Sz;
    /** Pending bits flag */
    bool isPending;
};

/** Initialize a High speed protocol interface */
void
McbIntfInit(McbIntf* ptInst);

/** Deinitialize a high speed protocol interface */
void
McbIntfDeinit(McbIntf* ptInst);

/** Write frame */
EMcbStatus McbIntfWrite(McbIntf* ptInst, uint16_t* ptNode, uint16_t* ptAddr, uint16_t* ptCmd, uint16_t* ptData,
        uint16_t* ptSz);
/** Read frame */
EMcbStatus McbIntfRead(McbIntf* ptInst, uint16_t* ptNode, uint16_t* ptAddr, uint16_t* ptCmd, uint16_t* ptData);
#endif /* MCB_INTF_H */
