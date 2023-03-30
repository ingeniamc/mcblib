/**
 * @file mcb_usr.h
 * @brief This file contains functions to be implemented by user to
 *        migrate platform dependencies
 *
 * @author  Firmware department
 * @copyright Ingenia Motion Control (c) 2018. All rights reserved.
 */

/**
 * \addtogroup UserAPI Functions implemented by user
 * @{
 *
 *  This functions must to be implemented by the user to adapt
 *  its HAL / BSP.
 */
 
#ifndef MCB_USR_H
#define MCB_USR_H

#include <stdint.h>
#include <stdbool.h>
#include "mcb_frame.h"

/** Number of resources instances */
#define MCB_NUMBER_RESOURCES (uint16_t)1U


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
    /** Write request success */
    MCB_WRITE_SUCCESS = 0,
    /** Read request success */
    MCB_READ_SUCCESS,
    /** Get info success */
    MCB_GETINFO_SUCCESS,
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
    /** Sending a get info request */
    MCB_GETINFO_REQUEST,
    /** Processing answer from get info request */
    MCB_GETINFO_ANSWER,
    /** Write config request error */
    MCB_WRITE_ERROR,
    /** Read config request error */
    MCB_READ_ERROR,
    /** Get info config request error */
    MCB_GETINFO_ERROR,
} Mcb_EStatus;

/** Motion control communication interface instance */
typedef struct
{
    /** Identification used for multiple instances */
    uint16_t u16Id;
    /** Indicates if the interface needs to call a function to calculate the CRC.
    If true, @ref Mcb_IntfComputeCrc is called. This function has a built-in CRC,
    or it can be replaced with user-specific implementation.
    If false, no CRC function is called. Used when the CRC is automatically
    computed by hardware. */
    bool bCalcCrc;
    /** Indicates the state of the communication bus */
    Mcb_EStatus eState;
    /** Indicates if there is a new config request over cyclic state */
    volatile bool isNewCfgOverCyclic;
    /** Indicates if a config request has been requested over cyclic state */
    volatile bool isCfgOverCyclic;
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
 * Reads the value of the IRQ signal
 *
 * @note This function is used to see if a mcb slave is available
 *
 * @param[in] u16Id
 *  Id of the McbIntf used to identify multiple instances
 *
 * @retval 0 if pin is at LOW level, 1 if it at HIGH level
 */
uint8_t
Mcb_IntfReadIRQ(uint16_t u16Id);

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
 * @param[in] pu16Buf
 *  Pointer to target buffer to compute CRC
 * @param[in] u16Sz
 *  Size of the buffer in words
 * @note If this function is not overriden, it
 *       implements a SW version of CRC
 * @retval Result of the CRC
 */
uint16_t
Mcb_IntfComputeCrc(const uint16_t* pu16Buf, uint16_t u16Sz);

/**
 * Checks the CRC of the incoming data.
 * This protocol uses CRC-CCITT (XModem).
 *
 * @param[in] u16Id
 *  Id of the McbIntf used to identify multiple instances
 * @param[in] pu16Buf
 *  Pointer to target buffer to compute CRC
 * @param[in] u16Sz
 *  Size of the buffer in words
 * @retval TRUE if the CRC is valid, FALSE otherwise
 */
bool
Mcb_IntfCheckCrc(uint16_t u16Id, const uint16_t* pu16Buf, uint16_t u16Sz);

/**
 * Gets the number of milliseconds since system was started
 *
 * @retval milliseconds
 */
uint32_t
Mcb_GetMillis(void);

/**
 * Relinquish (give up / allow others to execute) CPU.
 * This is especially useful on RTOS systems which rely on cooperative
 * threads in order share the CPU time.
 * This routine will be invoked on every iteration if a blocking method
 * uses busy waiting (i.e. timeouts).
 *
 * This should be a no-op on non RTOS systems.
 */
void
Mcb_RelinquishCPU(void);

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

/**
 * Generate a pulse on the Sync0 signal for synchronization purpose
 *
 * @note Slave is triggering the rising edge of the Sync0 signal, so there
 *       is not a limitation on the maximum pulse width.
 *       The minimum pulse width is 20 ns
 * @param[in] u16Id
 *  Id of the McbIntf used to identify multiple instances
 *
 */
void
Mcb_IntfSyncSignal(uint16_t u16Id);

/**
 * Initialize resource instance
 *
 * @note The resource instance has to be initialized in release state
 *
 * @param[in] u16Id
 *  Instance Id to be initialized
 */
void
Mcb_IntfInitResource(uint16_t u16Id);

/**
 * Delete resource instance
 *
 * @param[in] u16Id
 *  Instance Id to be deleted
 */
void
Mcb_IntfDeinitResource(uint16_t u16Id);

/**
 * Try to take resource instance
 *
 * @note Non blocking function
 *
 * @param[in] u16Id
 *  Instance Id to be taken
 *
 * @retval TRUE if resource taken, FALSE otherwise
 */
bool
Mcb_IntfTryTakeResource(uint16_t u16Id);

/**
 * Take resource instance
 *
 * @note Blocking function
 *
 * @param[in] u16Id
 *  Instance Id to be taken
 *
 * @retval TRUE if resource taken, FALSE otherwise
 */
bool
Mcb_IntfTakeResource(uint16_t u16Id);

/**
 * Unlock resource instance
 *
 * @param[in] u16Id
 *  Instance Id to be unlocked
 */
void
Mcb_IntfReleaseResource(uint16_t u16Id);

#endif /* MCB_USR_H */

/** @} */
