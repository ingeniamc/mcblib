/**
 * @file mcb_usr.h
 * @brief This file contains functions to be implemented by user to
 *        migrate platform dependencies
 *
 * @author  Firmware department
 * @copyright Ingenia Motion Control (c) 2018. All rights reserved.
 */

#ifndef MCB_USR_H
#define MCB_USR_H

#include <stdint.h>
#include <stdbool.h>
#include "mcb_frame.h"

/** McbIntf Pin status */
typedef enum
{
    /** Indicates that the value on the ping is GND */
    MCB_LOW = 0,
    /** Indicates that the value on the pin is VDD */
    MCB_HIGH = 1
} Mcb_EPinVal;

/** McbIntf communication states */
typedef enum
{
    /** Transmission successful */
    MCB_SUCCESS = 0,
    /** Bus in stand by */
    MCB_STANDBY,
    /** Sending a write request */
    MCB_WRITE_REQUEST,
    /** Processing answer from write request */
    MCB_WRITE_ANSWER,
    /** Sending a read request */
    MCB_READ_REQUEST,
    /** Processing answer from read request */
    MCB_READ_ANSWER,
    /** Transaction error */
    MCB_ERROR
} Mcb_EStatus;

typedef struct
{
    /** Identification used for multiple instances */
    uint16_t u16Id;
    /** Indicates the state of the communication bus */
    Mcb_EStatus eState;
    /** Indicates if a config request has been requested over cyclic frames */
    bool isCfgOverCyclic;
    /** IRQ Event signal */
    volatile bool isIrqEvnt;
    /** Frame pool for holding tx data */
    Mcb_TFrame tTxfrm;
    /** Frame pool for holding rx data */
    Mcb_TFrame tRxfrm;
    /** Pending data size to be transmitted/received */
    uint16_t u16Sz;
    /** Pending bits flag */
    bool isPending;
} Mcb_TIntf;

/**
 * Sets the IRQ event flag used internally for protocol synchronization
 *
 * @note This function must be included into an external rising edge interrupt
 *       of the GPI working as IRQ.
 *
 * @param[in] ptInst
 *  Pointer to McbIntf instace linked to this interrupt
 */
void
Mcb_IntfIRQEvent(Mcb_TIntf* ptInst);

/**
 * Checks if the SPI interface is ready for a new
 * transmission
 *
 * @param[in] u16Id
 *  Id of the McbIntf used to identify multiple instances
 *
 * @retval true if it is ready,
 *         false otherwise
 */
bool
Mcb_IntfIsReady(uint16_t u16Id);

/**
 * Computes the CRC of the incoming data.
 * This protocol uses CRC-CCITT (XModem).
 *
 * @param[in] u16Id
 *  Id of the McbIntf used to identify multiple instances
 *
 * @note If this function is not overriden, it
 *       implements a SW version of CRC
 */
bool
Mcb_IntfCheckCrc(uint16_t u16Id);

/**
 * Gets the number of milliseconds since system was started
 *
 * @param[in] u16Id
 *  Id of the McbIntf used to identify multiple instances
 *
 * @retval milliseconds
 */
uint32_t
Mcb_GetMillis(void);

/**
 * Executes a SPI transfer
 *
 * @note Chip select must be managed on this function
 *
 * @note Blocking and non-blocking modes are supported
 *
 * @param[in] u16Id
 *  Id of the McbIntf used to identify multiple instances
 * @param[in] pu16In
 *  Pointer to input buffer to be sent
 * @param[out] pu16Out
 *  Pointer to output buffer to be recevied
 * @param[in] u16Sz
 *  Size of the transmission in words (16 bit)
 */
void
Mcb_IntfSPITransfer(uint16_t u16Id, uint16_t* pu16In, uint16_t* pu16Out, uint16_t u16Sz);

#endif /* MCB_USR_H */
