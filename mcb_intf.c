/**
 * @file mcb_intf.c
 * @brief This file contains API for accessing to low level interface
 * 		  of the motion control bus (MCB)
 *
 * @author  Firmware department
 * @copyright Ingenia Motion Control (c) 2018. All rights reserved.
 */

#include "mcb_intf.h"
#include <stddef.h>

#define DFLT_TIMEOUT  100
#define SIZE_WORDS    2

/**
 * Execute a Spi transfer
 *
 * @param[in] ptInst
 *  Target instance
 * @param[in] tInFrame
 *  Input frame
 * @param[out] tOutFrame
 *  Output frame
 */
static void
Mcb_IntfTransfer(const Mcb_TIntf* ptInst, Mcb_TFrame* tInFrame, Mcb_TFrame* tOutFrame);

/**
 * Process a write command
 *
 * @note Mcb in non cyclic status
 *
 * @param[in] ptInst
 *  Target instance
 * @param[in] u16Addr
 *  Register address
 * @param[in] pu16Data
 *  Data to be read
 * @param[out] pu16Sz
 *  Size of read words
 *
 * @retval If a new transfer is ready
 */
static bool
Mcb_IntfWriteCfg(Mcb_TIntf* ptInst, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz);

/**
 * Process a read command
 *
 * @note Mcb in non cyclic status
 *
 * @param[in] ptInst
 *  Target instance
 * @param[in] u16Addr
 *  Register address
 * @param[out] pu16Data
 *  Data to be write
 * @param[in] pu16Sz
 *  Size of read words
 *
 * @retval If a new transfer is ready
 */
static bool
Mcb_IntfReadCfg(Mcb_TIntf* ptInst, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz);

/**
 * Process a get info command
 *
 * @note Mcb in non cyclic status
 *
 * @param[in] ptInst
 *  Target instance
 * @param[in] u16Addr
 *  Register address
 * @param[out] pu16Data
 *  Data to be read
 * @param[in] pu16Sz
 *  Size of read words
 *
 * @retval If a new transfer is ready
 */
static bool
Mcb_IntfGetInfoCfg(Mcb_TIntf* ptInst, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz);

/**
 * Process a write command
 *
 * @note Mcb in cyclic status
 *
 * @param[in] ptInst
 *  Target instance
 * @param[in] u16Addr
 *  Register address
 * @param[in] pu16Data
 *  Data to be read
 * @param[out] pu16Sz
 *  Size of read words
 *
 * @retval If a new transfer is ready
 */
static bool
Mcb_IntfWriteCfgOverCyclic(Mcb_TIntf* ptInst, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz);

/**
 * Process a read command
 *
 * @note Mcb in non cyclic status
 *
 * @param[in] ptInst
 *  Target instance
 * @param[in] u16Addr
 *  Register address
 * @param[out] pu16Data
 *  Data to be write
 * @param[in] pu16Sz
 *  Size of read words
 *
 * @retval If a new transfer is ready
 */
static bool
Mcb_IntfReadCfgOverCyclic(Mcb_TIntf* ptInst, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz);

/**
 * Process a get info command
 *
 * @note Mcb non cyclic status
 *
 * @param[in] ptInst
 *  Target instance
 * @param[in] u16Addr
 *  Register address
 * @param[out] pu16Data
 *  Data to be read
 * @param[in] pu16Sz
 *  Size of read words
 *
 * @retval If a new transfer is ready
 */
static bool
Mcb_IntfGetInfoCfgOverCyclic(Mcb_TIntf* ptInst, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz);

void Mcb_IntfInit(Mcb_TIntf* ptInst)
{
    ptInst->eState = MCB_STANDBY;
    Mcb_IntfInitSem(SEMAPHORE_IRQ_RESOURCE);
    ptInst->isCfgOverCyclic = false;
}

void Mcb_IntfDeinit(Mcb_TIntf* ptInst)
{
    ptInst->eState = MCB_STANDBY;
    Mcb_IntfDeinitSem(SEMAPHORE_IRQ_RESOURCE);
    ptInst->isCfgOverCyclic = false;
}

void Mcb_IntfReset(Mcb_TIntf* ptInst)
{
    ptInst->eState = MCB_STANDBY;
    Mcb_IntfDeinitSem(SEMAPHORE_IRQ_RESOURCE);
    Mcb_IntfInitSem(SEMAPHORE_IRQ_RESOURCE);
}

Mcb_EStatus Mcb_IntfWrite(Mcb_TIntf* ptInst, uint16_t u16Node, uint16_t u16Addr, uint16_t* pu16Data,
        uint16_t* pu16Sz)
{
    bool isNewData = false;

    /** Check if data is already available (IRQ) & SPI is ready for transmission */
    if ((Mcb_IntfIsReady(ptInst->u16Id) != false) && (Mcb_IntfTryLockSem(SEMAPHORE_IRQ_RESOURCE) != false))
    {
        if ((ptInst->eState == MCB_WRITE_ANSWER) && (Mcb_IntfCheckCrc(ptInst->u16Id, ptInst->tRxfrm.u16Buf, ptInst->tTxfrm.u16Sz) == false))
        {
            ptInst->eState = MCB_WRITE_ERROR;
        }
        else
        {
            isNewData = Mcb_IntfWriteCfg(ptInst, u16Addr, pu16Data, pu16Sz);
        }

        /** Set up a new frame if an error is detected */
        if (ptInst->eState == MCB_WRITE_ERROR)
        {
            Mcb_FrameCreateConfig(&(ptInst->tTxfrm), 0, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, ptInst->bCalcCrc);
            isNewData = true;
        }

        /** Use node to choose the chip select */
        if (isNewData != false)
        {
            Mcb_IntfTransfer(ptInst, &(ptInst->tTxfrm), &(ptInst->tRxfrm));
        }
        else
        {
            Mcb_IntfUnlockSem(SEMAPHORE_IRQ_RESOURCE);
        }
    }

    return ptInst->eState;
}

Mcb_EStatus Mcb_IntfRead(Mcb_TIntf* ptInst, uint16_t u16Node, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz)
{
    bool isNewData = false;

    /** Check if data is already available (IRQ) & SPI is ready for transmission */
    if ((Mcb_IntfIsReady(ptInst->u16Id) != false) && (Mcb_IntfTryLockSem(SEMAPHORE_IRQ_RESOURCE) != false))
    {
        if ((ptInst->eState == MCB_READ_ANSWER) && (Mcb_IntfCheckCrc(ptInst->u16Id, ptInst->tRxfrm.u16Buf, ptInst->tTxfrm.u16Sz) == false))
        {
            ptInst->eState = MCB_READ_ERROR;
        }
        else
        {
            isNewData = Mcb_IntfReadCfg(ptInst, u16Addr, pu16Data, pu16Sz);
        }

        /** Set up a new frame if an error is detected */
        if (ptInst->eState == MCB_READ_ERROR)
        {
            Mcb_FrameCreateConfig(&(ptInst->tTxfrm), 0, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, ptInst->bCalcCrc);
            isNewData = true;
        }

        /** Use node to choose the chip select */
        if (isNewData != false)
        {
            Mcb_IntfTransfer(ptInst, &(ptInst->tTxfrm), &(ptInst->tRxfrm));
        }
        else
        {
            Mcb_IntfUnlockSem(SEMAPHORE_IRQ_RESOURCE);
        }
    }

    return ptInst->eState;
}

Mcb_EStatus Mcb_IntfGetInfo(Mcb_TIntf* ptInst, uint16_t u16Node, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz)
{
    bool isNewData = false;

    /** Check if data is already available (IRQ) & SPI is ready for transmission */
    if ((Mcb_IntfIsReady(ptInst->u16Id) != false) && (Mcb_IntfTryLockSem(SEMAPHORE_IRQ_RESOURCE) != false))
    {
        if ((ptInst->eState == MCB_GETINFO_ANSWER) && (Mcb_IntfCheckCrc(ptInst->u16Id, ptInst->tRxfrm.u16Buf, ptInst->tTxfrm.u16Sz) == false))
        {
            ptInst->eState = MCB_GETINFO_ERROR;
        }
        else
        {
            isNewData = Mcb_IntfGetInfoCfg(ptInst, u16Addr, pu16Data, pu16Sz);
        }

        /** Set up a new frame if an error is detected */
        if (ptInst->eState == MCB_GETINFO_ERROR)
        {
            Mcb_FrameCreateConfig(&(ptInst->tTxfrm), 0, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, ptInst->bCalcCrc);
            isNewData = true;
        }

        /** Use node to choose the chip select */
        if (isNewData != false)
        {
            Mcb_IntfTransfer(ptInst, &(ptInst->tTxfrm), &(ptInst->tRxfrm));
        }
        else
        {
            Mcb_IntfUnlockSem(SEMAPHORE_IRQ_RESOURCE);
        }
    }

    return ptInst->eState;
}

void Mcb_IntfIRQEvent(Mcb_TIntf* ptInst)
{
    Mcb_IntfUnlockSem(SEMAPHORE_IRQ_RESOURCE);
}

void Mcb_IntfTransfer(const Mcb_TIntf* ptInst, Mcb_TFrame* ptInFrame, Mcb_TFrame* ptOutFrame)
{
    Mcb_IntfSPITransfer(ptInst->u16Id, ptInFrame->u16Buf, ptOutFrame->u16Buf, ptInFrame->u16Sz);
}

Mcb_EStatus Mcb_IntfCfgOverCyclic(Mcb_TIntf* ptInst, uint16_t u16Node, uint16_t u16Addr, uint16_t* pu16Cmd,
                                  uint16_t* pu16Data, uint16_t* pu16CfgSz, bool* pisNewData)
{
    Mcb_EStatus eCyclicState = MCB_STANDBY;
    static uint16_t u16CurrentCmd = MCB_REQ_IDLE;

    *pisNewData = false;

    if (ptInst->isCfgOverCyclic == false)
    {
        if (ptInst->isNewCfgOverCyclic != false)
        {
            /** If a config command is requested, add it into cyclic frame */
            u16CurrentCmd = *pu16Cmd;
            switch (u16CurrentCmd)
            {
                case MCB_REQ_GETINFO:
                    /** Generate initial frame */
                    *pisNewData = Mcb_IntfGetInfoCfgOverCyclic(ptInst, u16Addr, pu16Data, pu16CfgSz);
                    ptInst->isCfgOverCyclic = true;
                    break;
                case MCB_REQ_READ:
                    /** Generate initial frame */
                    *pisNewData = Mcb_IntfReadCfgOverCyclic(ptInst, u16Addr, pu16Data, pu16CfgSz);
                    ptInst->isCfgOverCyclic = true;
                    break;
                case MCB_REQ_WRITE:
                    /** Generate initial frame */
                    *pisNewData = Mcb_IntfWriteCfgOverCyclic(ptInst, u16Addr, pu16Data, pu16CfgSz);
                    ptInst->isCfgOverCyclic = true;
                    break;
                default:
                    /** Nothing */
                    break;
            }
            ptInst->isNewCfgOverCyclic = false;
        }
    }
    else
    {
        /** Keep on processing the config request */
        switch (u16CurrentCmd)
        {
            case MCB_REQ_GETINFO:
                *pisNewData = Mcb_IntfGetInfoCfgOverCyclic(ptInst, u16Addr, pu16Data, pu16CfgSz);
                break;
            case MCB_REQ_READ:
                *pisNewData = Mcb_IntfReadCfgOverCyclic(ptInst, u16Addr, pu16Data, pu16CfgSz);
                break;
            case MCB_REQ_WRITE:
                *pisNewData = Mcb_IntfWriteCfgOverCyclic(ptInst, u16Addr, pu16Data, pu16CfgSz);
                break;
            default:
                /** Nothing */
                break;
        }

        switch (ptInst->eState)
        {
            case MCB_WRITE_SUCCESS:
                *pu16Cmd = MCB_REP_ACK;
                *pu16CfgSz = ptInst->u16Sz;
                ptInst->isCfgOverCyclic = false;
                eCyclicState = MCB_WRITE_SUCCESS;
                break;
            case MCB_READ_SUCCESS:
                *pu16Cmd = MCB_REP_ACK;
                *pu16CfgSz = ptInst->u16Sz;
                ptInst->isCfgOverCyclic = false;
                eCyclicState = MCB_READ_SUCCESS;
                break;
            case MCB_GETINFO_SUCCESS:
                *pu16Cmd = MCB_REP_ACK;
                *pu16CfgSz = ptInst->u16Sz;
                ptInst->isCfgOverCyclic = false;
                eCyclicState = MCB_GETINFO_SUCCESS;
                break;
            case MCB_WRITE_ERROR:
                *pu16Cmd = MCB_REP_WRITE_ERROR;
                ptInst->isCfgOverCyclic = false;
                eCyclicState = MCB_WRITE_ERROR;
                break;
            case MCB_READ_ERROR:
                *pu16Cmd = MCB_REP_READ_ERROR;
                ptInst->isCfgOverCyclic = false;
                eCyclicState = MCB_READ_ERROR;
                break;
            case MCB_GETINFO_ERROR:
                *pu16Cmd = MCB_REP_GETINFO_ERROR;
                ptInst->isCfgOverCyclic = false;
                eCyclicState = MCB_GETINFO_ERROR;
                break;
            default:
                /* Nothing */
                break;
        }
    }

    return eCyclicState;
}

void Mcb_IntfCyclic(Mcb_TIntf* ptInst, uint16_t *ptInBuf, uint16_t *ptOutBuf, uint16_t u16CyclicSz, bool isNewData)
{
    if (isNewData == false)
    {
        /** The CRC can only be appended by the AppendCyclic() */
        Mcb_FrameCreateConfig(&(ptInst->tTxfrm), 0, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, false);
        Mcb_FrameAppendCyclic(&(ptInst->tTxfrm), ptInBuf, u16CyclicSz, ptInst->bCalcCrc);
    }
    else
    {
        Mcb_FrameAppendCyclic(&(ptInst->tTxfrm), ptInBuf, u16CyclicSz, ptInst->bCalcCrc);
    }

    Mcb_IntfTransfer(ptInst, &(ptInst->tTxfrm), &(ptInst->tRxfrm));

    /** Get cyclic data from last transmission */
    if (Mcb_IntfCheckCrc(ptInst->u16Id, ptInst->tRxfrm.u16Buf, ptInst->tTxfrm.u16Sz) != false)
    {
        Mcb_FrameGetCyclicData(&ptInst->tRxfrm, ptOutBuf, u16CyclicSz);
    }
}

static bool Mcb_IntfWriteCfg(Mcb_TIntf* ptInst, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz)
{
    bool isNewData = false;

    if ((ptInst->eState != MCB_READ_REQUEST) && (ptInst->eState != MCB_READ_ANSWER) &&
        (ptInst->eState != MCB_WRITE_REQUEST) && (ptInst->eState != MCB_WRITE_ANSWER) &&
        (ptInst->eState != MCB_GETINFO_REQUEST) && (ptInst->eState != MCB_GETINFO_ANSWER))
    {
        ptInst->u16Sz = *pu16Sz;
        ptInst->isPending = true;
        ptInst->eState = MCB_WRITE_REQUEST;
    }

    switch (ptInst->eState)
    {
        case MCB_WRITE_REQUEST:
            if (ptInst->u16Sz > MCB_FRM_CONFIG_SZ)
            {
                Mcb_FrameCreateConfig(&(ptInst->tTxfrm), u16Addr, MCB_REQ_WRITE, MCB_FRM_SEG,
                        &pu16Data[*pu16Sz - ptInst->u16Sz], ptInst->bCalcCrc);
                ptInst->u16Sz -= MCB_FRM_CONFIG_SZ;
            }
            else if (ptInst->u16Sz == 0)
            {
                Mcb_FrameCreateConfig(&(ptInst->tTxfrm), u16Addr, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, ptInst->bCalcCrc);
                ptInst->isPending = false;
            }
            else
            {
                Mcb_FrameCreateConfig(&(ptInst->tTxfrm), u16Addr, MCB_REQ_WRITE, MCB_FRM_NOTSEG,
                        &pu16Data[*pu16Sz - ptInst->u16Sz], ptInst->bCalcCrc);
                ptInst->u16Sz = 0;
            }

            isNewData = true;
            ptInst->eState = MCB_WRITE_ANSWER;
            break;
        case MCB_WRITE_ANSWER:
            /** Check reception */
            switch (Mcb_FrameGetCmd(&(ptInst->tRxfrm)))
            {
                case MCB_REP_ACK:
                    /* Copy read data to buffer - Also copy it in case of error msg */
                    Mcb_FrameGetConfigData(&(ptInst->tRxfrm), &pu16Data[(uint16_t)0U]);

                    if (Mcb_FrameGetAddr(&(ptInst->tRxfrm)) == u16Addr)
                    {
                        if (ptInst->isPending != false)
                        {
                            ptInst->eState = MCB_WRITE_REQUEST;
                        }
                        else
                        {
                            ptInst->eState = MCB_WRITE_SUCCESS;
                        }
                    }
                    else
                    {
                        ptInst->eState = MCB_WRITE_ERROR;
                    }
                    break;
                case MCB_REP_WRITE_ERROR:
                    /* Copy read data to buffer - Also copy it in case of error msg */
                    Mcb_FrameGetConfigData(&(ptInst->tRxfrm), &pu16Data[(uint16_t)0U]);

                    if (Mcb_FrameGetAddr(&(ptInst->tRxfrm)) == u16Addr)
                    {
                        ptInst->eState = MCB_WRITE_ERROR;
                    }
                    else
                    {
                        ptInst->eState = MCB_WRITE_ERROR;
                    }
                    break;
                case MCB_REQ_IDLE:
                    ptInst->eState = MCB_WRITE_REQUEST;
                    break;
                default:
                    ptInst->eState = MCB_WRITE_ERROR;
                    break;
            }
            break;
        default:
            ptInst->eState = MCB_STANDBY;
            break;
    }

    return isNewData;
}

static bool Mcb_IntfReadCfg(Mcb_TIntf* ptInst, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz)
{
    bool isNewData = false;

    if ((ptInst->eState != MCB_READ_REQUEST) && (ptInst->eState != MCB_READ_ANSWER) &&
        (ptInst->eState != MCB_WRITE_REQUEST) && (ptInst->eState != MCB_WRITE_ANSWER) &&
        (ptInst->eState != MCB_GETINFO_REQUEST) && (ptInst->eState != MCB_GETINFO_ANSWER))
    {
        ptInst->isPending = true;
        ptInst->eState = MCB_READ_REQUEST;
        ptInst->u16Sz = 0;
    }

    switch (ptInst->eState)
    {
        case MCB_READ_REQUEST:
            /* Send read request */
            if (ptInst->isPending != false)
            {
                Mcb_FrameCreateConfig(&(ptInst->tTxfrm), u16Addr, MCB_REQ_READ, MCB_FRM_NOTSEG, NULL, ptInst->bCalcCrc);
                /** Read is requested once, then IDLE are sent until the complete read is reached */
                ptInst->isPending = false;
            }
            else
            {
                Mcb_FrameCreateConfig(&(ptInst->tTxfrm), u16Addr, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, ptInst->bCalcCrc);
            }

            isNewData = true;
            ptInst->eState = MCB_READ_ANSWER;
            break;
        case MCB_READ_ANSWER:
            /** Check reception */
            switch (Mcb_FrameGetCmd(&(ptInst->tRxfrm)))
            {
                case MCB_REP_ACK:
                    /* Copy read data to buffer - Also copy it in case of error msg */
                    ptInst->u16Sz += Mcb_FrameGetConfigData(&(ptInst->tRxfrm), &pu16Data[ptInst->u16Sz]);

                    if (Mcb_FrameGetAddr(&(ptInst->tRxfrm)) == u16Addr)
                    {
                        if (Mcb_FrameGetSegmented(&(ptInst->tRxfrm)) != false)
                        {
                            ptInst->eState = MCB_READ_REQUEST;
                        }
                        else
                        {
                            *pu16Sz = ptInst->u16Sz;
                            ptInst->eState = MCB_READ_SUCCESS;
                        }
                    }
                    else
                    {
                        ptInst->eState = MCB_READ_ERROR;
                    }
                    break;
                case MCB_REP_READ_ERROR:
                    /* Copy read data to buffer - Also copy it in case of error msg */
                    ptInst->u16Sz += Mcb_FrameGetConfigData(&(ptInst->tRxfrm), &pu16Data[ptInst->u16Sz]);

                    if (Mcb_FrameGetAddr(&(ptInst->tRxfrm)) == u16Addr)
                    {
                        ptInst->u16Sz = MCB_FRM_CONFIG_SZ;
                        ptInst->eState = MCB_READ_ERROR;
                    }
                    else
                    {
                        ptInst->eState = MCB_READ_ERROR;
                    }
                    break;
                case MCB_REQ_IDLE:
                    ptInst->eState = MCB_READ_REQUEST;
                    break;
                default:
                    ptInst->eState = MCB_READ_ERROR;
                    break;
            }
            break;
        default:
            ptInst->eState = MCB_STANDBY;
            break;
    }

    return isNewData;
}

static bool Mcb_IntfGetInfoCfg(Mcb_TIntf* ptInst, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz)
{
    bool isNewData = false;

    if ((ptInst->eState != MCB_READ_REQUEST) && (ptInst->eState != MCB_READ_ANSWER) &&
        (ptInst->eState != MCB_WRITE_REQUEST) && (ptInst->eState != MCB_WRITE_ANSWER) &&
        (ptInst->eState != MCB_GETINFO_REQUEST) && (ptInst->eState != MCB_GETINFO_ANSWER))
    {
        ptInst->isPending = true;
        ptInst->eState = MCB_GETINFO_REQUEST;
        ptInst->u16Sz = 0;
    }

    switch (ptInst->eState)
    {
        case MCB_GETINFO_REQUEST:
            /* Send getinfo request */
            if (ptInst->isPending != false)
            {
                Mcb_FrameCreateConfig(&(ptInst->tTxfrm), u16Addr, MCB_REQ_GETINFO, MCB_FRM_NOTSEG, NULL, ptInst->bCalcCrc);
                /** Read is requested once, then IDLE are sent until the complete read is reached */
                ptInst->isPending = false;
            }
            else
            {
                Mcb_FrameCreateConfig(&(ptInst->tTxfrm), u16Addr, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, ptInst->bCalcCrc);
            }

            isNewData = true;
            ptInst->eState = MCB_GETINFO_ANSWER;
            break;
        case MCB_GETINFO_ANSWER:
            /** Check reception */
            switch (Mcb_FrameGetCmd(&(ptInst->tRxfrm)))
            {
                case MCB_REP_ACK:
                    /* Copy getinfo data to buffer - Also copy it in case of error msg */
                    ptInst->u16Sz += Mcb_FrameGetConfigData(&(ptInst->tRxfrm), &pu16Data[ptInst->u16Sz]);

                    if (Mcb_FrameGetAddr(&(ptInst->tRxfrm)) == u16Addr)
                    {
                        if (Mcb_FrameGetSegmented(&(ptInst->tRxfrm)) != false)
                        {
                            ptInst->eState = MCB_GETINFO_REQUEST;
                        }
                        else
                        {
                            *pu16Sz = ptInst->u16Sz;
                            ptInst->eState = MCB_GETINFO_SUCCESS;
                        }
                    }
                    else
                    {
                        ptInst->eState = MCB_GETINFO_ERROR;
                    }
                    break;
                case MCB_REP_GETINFO_ERROR:
                    /* Copy read data to buffer - Also copy it in case of error msg */
                    ptInst->u16Sz += Mcb_FrameGetConfigData(&(ptInst->tRxfrm), &pu16Data[ptInst->u16Sz]);

                    if (Mcb_FrameGetAddr(&(ptInst->tRxfrm)) == u16Addr)
                    {
                        ptInst->u16Sz = MCB_FRM_CONFIG_SZ;
                        ptInst->eState = MCB_GETINFO_ERROR;
                    }
                    else
                    {
                        ptInst->eState = MCB_GETINFO_ERROR;
                    }
                    break;
                case MCB_REQ_IDLE:
                    ptInst->eState = MCB_GETINFO_REQUEST;
                    break;
                default:
                    ptInst->eState = MCB_GETINFO_ERROR;
                    break;
            }
            break;
        default:
            ptInst->eState = MCB_STANDBY;
            break;
    }

    return isNewData;
}

static bool Mcb_IntfWriteCfgOverCyclic(Mcb_TIntf* ptInst, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz)
{
    bool isNewData = false;

    if ((ptInst->eState != MCB_READ_REQUEST) && (ptInst->eState != MCB_READ_ANSWER) &&
        (ptInst->eState != MCB_WRITE_REQUEST) && (ptInst->eState != MCB_WRITE_ANSWER) &&
        (ptInst->eState != MCB_GETINFO_REQUEST) && (ptInst->eState != MCB_GETINFO_ANSWER))
    {
        ptInst->u16Sz = *pu16Sz;
        ptInst->eState = MCB_WRITE_REQUEST;
    }

    switch (ptInst->eState)
    {
        case MCB_WRITE_REQUEST:
            if (ptInst->u16Sz > MCB_FRM_CONFIG_SZ)
            {
                Mcb_FrameCreateConfig(&(ptInst->tTxfrm), u16Addr, MCB_REQ_WRITE, MCB_FRM_SEG,
                        &pu16Data[*pu16Sz - ptInst->u16Sz], false);
                ptInst->u16Sz -= MCB_FRM_CONFIG_SZ;
            }
            else
            {
                Mcb_FrameCreateConfig(&(ptInst->tTxfrm), u16Addr, MCB_REQ_WRITE, MCB_FRM_NOTSEG,
                        &pu16Data[*pu16Sz - ptInst->u16Sz], false);
                ptInst->u16Sz = 0;
            }

            isNewData = true;
            ptInst->eState = MCB_WRITE_ANSWER;
            break;
        case MCB_WRITE_ANSWER:
            /** Check reception */
            switch (Mcb_FrameGetCmd(&(ptInst->tRxfrm)))
            {
                case MCB_REP_ACK:
                    /* Copy read data to buffer - Also copy it in case of error msg */
                    Mcb_FrameGetConfigData(&(ptInst->tRxfrm), &pu16Data[(uint16_t)0U]);
                    if (Mcb_FrameGetAddr(&(ptInst->tRxfrm)) == u16Addr)
                    {
                        if (ptInst->isPending != false)
                        {
                            ptInst->eState = MCB_WRITE_REQUEST;
                        }
                        else
                        {
                            ptInst->u16Sz = MCB_FRM_CONFIG_SZ;
                            ptInst->eState = MCB_WRITE_SUCCESS;
                        }
                    }
                    else
                    {
                        ptInst->eState = MCB_WRITE_ERROR;
                    }
                    break;
                case MCB_REP_WRITE_ERROR:
                    /* Copy read data to buffer - Also copy it in case of error msg */
                    Mcb_FrameGetConfigData(&(ptInst->tRxfrm), &pu16Data[(uint16_t)0U]);

                    if (Mcb_FrameGetAddr(&(ptInst->tRxfrm)) == u16Addr)
                    {
                        ptInst->u16Sz = MCB_FRM_CONFIG_SZ;
                        ptInst->eState = MCB_WRITE_ERROR;
                    }
                    else
                    {
                        ptInst->eState = MCB_WRITE_ERROR;
                    }
                    break;
                case MCB_REQ_IDLE:
                    /** Waiting for reply */
                    break;
                default:
                    ptInst->eState = MCB_WRITE_ERROR;
                    break;
            }
            break;
        default:
            ptInst->eState = MCB_STANDBY;
            break;
    }

    return isNewData;
}

static bool Mcb_IntfReadCfgOverCyclic(Mcb_TIntf* ptInst, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz)
{
    bool isNewData = false;

    if ((ptInst->eState != MCB_READ_REQUEST) && (ptInst->eState != MCB_READ_ANSWER) &&
        (ptInst->eState != MCB_WRITE_REQUEST) && (ptInst->eState != MCB_WRITE_ANSWER) &&
        (ptInst->eState != MCB_GETINFO_REQUEST) && (ptInst->eState != MCB_GETINFO_ANSWER))
    {
        ptInst->eState = MCB_READ_REQUEST;
        ptInst->u16Sz = 0;
    }

    switch (ptInst->eState)
    {
        case MCB_READ_REQUEST:
            /* Send read request */
            Mcb_FrameCreateConfig(&(ptInst->tTxfrm), u16Addr, MCB_REQ_READ, MCB_FRM_NOTSEG, NULL, false);
            isNewData = true;
            ptInst->eState = MCB_READ_ANSWER;
            break;
        case MCB_READ_ANSWER:
            /** Check reception */
            switch (Mcb_FrameGetCmd(&(ptInst->tRxfrm)))
            {
                case MCB_REP_ACK:
                    /* Copy read data to buffer - Also copy it in case of error msg */
                    ptInst->u16Sz += Mcb_FrameGetConfigData(&(ptInst->tRxfrm), &pu16Data[ptInst->u16Sz]);

                    if (Mcb_FrameGetAddr(&(ptInst->tRxfrm)) == u16Addr)
                    {
                        if (Mcb_FrameGetSegmented(&(ptInst->tRxfrm)) != false)
                        {
                            ptInst->eState = MCB_READ_ANSWER;
                        }
                        else
                        {
                            *pu16Sz = ptInst->u16Sz;
                            ptInst->eState = MCB_READ_SUCCESS;
                        }
                    }
                    else
                    {
                        ptInst->eState = MCB_READ_ERROR;
                    }
                    break;
                case MCB_REP_READ_ERROR:
                    /* Copy read data to buffer - Also copy it in case of error msg */
                    ptInst->u16Sz += Mcb_FrameGetConfigData(&(ptInst->tRxfrm), &pu16Data[ptInst->u16Sz]);

                    if (Mcb_FrameGetAddr(&(ptInst->tRxfrm)) == u16Addr)
                    {
                        ptInst->u16Sz = MCB_FRM_CONFIG_SZ;
                        ptInst->eState = MCB_READ_ERROR;
                    }
                    else
                    {
                        ptInst->eState = MCB_READ_ERROR;
                    }
                    break;
                case MCB_REQ_IDLE:
                    /** Waiting for new data */
                    break;
                default:
                    ptInst->eState = MCB_READ_ERROR;
                    break;
            }
            break;
        default:
            ptInst->eState = MCB_STANDBY;
            break;
    }

    return isNewData;
}

static bool Mcb_IntfGetInfoCfgOverCyclic(Mcb_TIntf* ptInst, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz)
{
    bool isNewData = false;

    if ((ptInst->eState != MCB_READ_REQUEST) && (ptInst->eState != MCB_READ_ANSWER) &&
        (ptInst->eState != MCB_WRITE_REQUEST) && (ptInst->eState != MCB_WRITE_ANSWER) &&
        (ptInst->eState != MCB_GETINFO_REQUEST) && (ptInst->eState != MCB_GETINFO_ANSWER))
    {
        ptInst->eState = MCB_GETINFO_REQUEST;
        ptInst->u16Sz = 0;
    }

    switch (ptInst->eState)
    {
        case MCB_GETINFO_REQUEST:
            /* Send read request */
            Mcb_FrameCreateConfig(&(ptInst->tTxfrm), u16Addr, MCB_REQ_GETINFO, MCB_FRM_NOTSEG, NULL, false);
            isNewData = true;
            ptInst->eState = MCB_GETINFO_ANSWER;
            break;
        case MCB_GETINFO_ANSWER:
            /** Check reception */
            switch (Mcb_FrameGetCmd(&(ptInst->tRxfrm)))
            {
                case MCB_REP_ACK:
                    /* Copy read data to buffer - Also copy it in case of error msg */
                    ptInst->u16Sz += Mcb_FrameGetConfigData(&(ptInst->tRxfrm), &pu16Data[ptInst->u16Sz]);

                    if (Mcb_FrameGetAddr(&(ptInst->tRxfrm)) == u16Addr)
                    {
                        if (Mcb_FrameGetSegmented(&(ptInst->tRxfrm)) != false)
                        {
                            ptInst->eState = MCB_GETINFO_ANSWER;
                        }
                        else
                        {
                            *pu16Sz = ptInst->u16Sz;
                            ptInst->eState = MCB_GETINFO_SUCCESS;
                        }
                    }
                    else
                    {
                        ptInst->eState = MCB_GETINFO_ERROR;
                    }
                    break;
                case MCB_REP_GETINFO_ERROR:
                    /* Copy read data to buffer - Also copy it in case of error msg */
                    ptInst->u16Sz += Mcb_FrameGetConfigData(&(ptInst->tRxfrm), &pu16Data[ptInst->u16Sz]);

                    if (Mcb_FrameGetAddr(&(ptInst->tRxfrm)) == u16Addr)
                    {
                        ptInst->u16Sz = MCB_FRM_CONFIG_SZ;
                        ptInst->eState = MCB_GETINFO_ERROR;
                    }
                    else
                    {
                        ptInst->eState = MCB_GETINFO_ERROR;
                    }
                    break;
                case MCB_REQ_IDLE:
                    /** Waiting for new data */
                    break;
                default:
                    ptInst->eState = MCB_GETINFO_ERROR;
                    break;
            }
            break;
        default:
            ptInst->eState = MCB_STANDBY;
            break;
    }

    return isNewData;
}
