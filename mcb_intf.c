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

static void
Mcb_IntfTransfer(const Mcb_TIntf* ptInst, Mcb_TFrame* tInFrame, Mcb_TFrame* tOutFrame);

static bool
Mcb_IntfWriteProcess(Mcb_TIntf* ptInst, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz);

static bool
Mcb_IntfReadProcess(Mcb_TIntf* ptInst, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz);

void Mcb_IntfInit(Mcb_TIntf* ptInst)
{
    ptInst->eState = MCB_STANDBY;
    ptInst->isIrqEvnt = true;
    ptInst->isCfgOverCyclic = false;
}

void Mcb_IntfDeinit(Mcb_TIntf* ptInst)
{
    ptInst->eState = MCB_STANDBY;
    ptInst->isIrqEvnt = false;
    ptInst->isCfgOverCyclic = false;
}

Mcb_EStatus Mcb_IntfWrite(Mcb_TIntf* ptInst, uint16_t u16Node, uint16_t u16Addr, uint16_t* pu16Data,
        uint16_t* pu16Sz)
{
    /** Check if data is already available (IRQ) & SPI is ready for transmission */
    if ((Mcb_IntfIsReady(ptInst->u16Id) != false) && (ptInst->isIrqEvnt != false))
    {
        bool isNewData = Mcb_IntfWriteProcess(ptInst, u16Addr, pu16Data, pu16Sz);

        /** Use node to choose the chip select */
        if (isNewData != false)
        {
            Mcb_IntfTransfer(ptInst, &(ptInst->tTxfrm), &(ptInst->tRxfrm));
        }
    }

    return ptInst->eState;
}

Mcb_EStatus Mcb_IntfRead(Mcb_TIntf* ptInst, uint16_t u16Node, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz)
{
    /** Check if data is already available (IRQ) & SPI is ready for transmission */
    if ((Mcb_IntfIsReady(ptInst->u16Id) != false) && (ptInst->isIrqEvnt != false))
    {
        bool isNewData = Mcb_IntfReadProcess(ptInst, u16Addr, pu16Data, pu16Sz);

        /** Use node to choose the chip select */

        if (isNewData != false)
        {
            Mcb_IntfTransfer(ptInst, &(ptInst->tTxfrm), &(ptInst->tRxfrm));
        }
    }

    return ptInst->eState;
}

void Mcb_IntfIRQEvent(Mcb_TIntf* ptInst)
{
    ptInst->isIrqEvnt = true;
}

void Mcb_IntfTransfer(const Mcb_TIntf* ptInst, Mcb_TFrame* ptInFrame, Mcb_TFrame* ptOutFrame)
{
    Mcb_IntfSPITransfer(ptInst->u16Id, ptInFrame->u16Buf, ptOutFrame->u16Buf, ptInFrame->u16Sz);
}

Mcb_EStatus Mcb_IntfCyclicTranfer(Mcb_TIntf* ptInst, uint16_t u16Node, uint16_t u16Addr, uint16_t* pu16Cmd,
        uint16_t* pu16Data, uint16_t* pu16CfgSz, uint16_t *ptInBuf, uint16_t *ptOutBuf, uint16_t u16CyclicSz)
{
    if ((Mcb_IntfIsReady(ptInst->u16Id) != false) && (ptInst->isIrqEvnt != false))
    {
        /** Get cyclic data from last transmission */
        Mcb_FrameGetCyclicData(&ptInst->tRxfrm, ptOutBuf, u16CyclicSz);

        bool isNewData = false;
        /** If a frame command is requested, add it into cyclic frame */
        if (ptInst->isCfgOverCyclic == false)
        {
            switch (*pu16Cmd)
            {
                case MCB_REQ_READ:
                    /** Generate initial frame */
                    isNewData = Mcb_IntfReadProcess(ptInst, u16Addr, pu16Data, pu16CfgSz);
                    ptInst->isCfgOverCyclic = true;
                    break;
                case MCB_REQ_WRITE:
                    /** Generate initial frame */
                    ptInst->isCfgOverCyclic = true;
                    isNewData = Mcb_IntfWriteProcess(ptInst, u16Addr, pu16Data, pu16CfgSz);
                    break;
                default:
                    /** Nothing */
                    break;
            }
        }
        else
        {
            /** Process config header */
            switch (Mcb_FrameGetCmd(&ptInst->tRxfrm))
            {
                case MCB_REP_ACK:
                    /** Keep on processing config frame */
                    switch (*pu16Cmd)
                    {
                        case MCB_REQ_READ:
                            isNewData = Mcb_IntfReadProcess(ptInst, u16Addr, pu16Data, pu16CfgSz);
                            break;
                        case MCB_REQ_WRITE:
                            isNewData = Mcb_IntfWriteProcess(ptInst, u16Addr, pu16Data, pu16CfgSz);
                            break;
                        default:
                            /** Nothing */
                            break;
                    }

                    if (ptInst->eState == MCB_SUCCESS)
                    {
                        *pu16Cmd = MCB_REP_ACK;
                        ptInst->isCfgOverCyclic = false;
                    }
                    else if (ptInst->eState == MCB_ERROR)
                    {
                        *pu16Cmd |= MCB_REP_ERROR;
                        ptInst->isCfgOverCyclic = false;
                    }
                    else
                    {
                        /** Nothing */
                    }
                    break;
                default:
                    /** Waiting for something */
                    break;
            }
        }

        if (isNewData == false)
        {
            Mcb_FrameCreateConfig(&(ptInst->tTxfrm), 0, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, false);
            Mcb_FrameAppendCyclic(&(ptInst->tTxfrm), ptInBuf, u16CyclicSz, false);
        }
        else
        {
            Mcb_FrameAppendCyclic(&(ptInst->tTxfrm), ptInBuf, u16CyclicSz, false);
        }

        Mcb_IntfTransfer(ptInst, &(ptInst->tTxfrm), &(ptInst->tRxfrm));
    }

    return ptInst->eState;
}

static bool Mcb_IntfWriteProcess(Mcb_TIntf* ptInst, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz)
{
    bool isNewData = false;

    if (ptInst->eState == MCB_STANDBY)
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
                        &pu16Data[*pu16Sz - ptInst->u16Sz], false);
                ptInst->u16Sz -= MCB_FRM_CONFIG_SZ;
            }
            else if (ptInst->u16Sz == 0)
            {
                Mcb_FrameCreateConfig(&(ptInst->tTxfrm), u16Addr, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, false);
                ptInst->isPending = false;
            }
            else
            {
                Mcb_FrameCreateConfig(&(ptInst->tTxfrm), u16Addr, MCB_REQ_WRITE, MCB_FRM_NOTSEG,
                        &pu16Data[*pu16Sz - ptInst->u16Sz], false);
                ptInst->u16Sz = 0;
            }

            ptInst->isIrqEvnt = false;
            isNewData = true;
            ptInst->eState = MCB_WRITE_ANSWER;
            break;
        case MCB_WRITE_ANSWER:
            /** Check reception */
            if (Mcb_IntfCheckCrc(ptInst->u16Id) != false)
            {
                switch (Mcb_FrameGetCmd(&(ptInst->tRxfrm)))
                {
                    case MCB_REP_ACK:
                        if (Mcb_FrameGetAddr(&(ptInst->tRxfrm)) == u16Addr)
                        {
                            if (ptInst->isPending != false)
                            {
                                ptInst->eState = MCB_WRITE_REQUEST;
                            }
                            else
                            {
                                ptInst->eState = MCB_SUCCESS;
                            }
                        }
                        break;
                    case MCB_REQ_IDLE:
                        ptInst->eState = MCB_WRITE_REQUEST;
                        break;
                    default:
                        ptInst->eState = MCB_ERROR;
                        break;
                }
            }
            else
            {
                ptInst->eState = MCB_ERROR;
            }

            if (ptInst->eState == MCB_ERROR)
            {
                Mcb_FrameCreateConfig(&(ptInst->tTxfrm), 0, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, false);
                isNewData = true;
            }
            break;
        default:
            ptInst->eState = MCB_STANDBY;
            break;
    }

    return isNewData;
}

static bool Mcb_IntfReadProcess(Mcb_TIntf* ptInst, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz)
{
    bool isNewData = false;

    if (ptInst->eState == MCB_STANDBY)
    {
        ptInst->isPending = true;
        ptInst->eState = MCB_READ_REQUEST;
    }

    switch (ptInst->eState)
    {
        case MCB_READ_REQUEST:
            /* Send read request */
            Mcb_FrameCreateConfig(&(ptInst->tTxfrm), u16Addr, MCB_REQ_READ, MCB_FRM_NOTSEG, pu16Data, false);

            ptInst->isIrqEvnt = false;
            isNewData = true;
            ptInst->eState = MCB_READ_ANSWER;
            break;
        case MCB_READ_ANSWER:
            /** Check reception */
            if (Mcb_IntfCheckCrc(ptInst->u16Id) != false)
            {
                switch (Mcb_FrameGetCmd(&(ptInst->tRxfrm)))
                {
                    case MCB_REP_ACK:
                        /* Copy read data to buffer - Also copy it in case of error msg */
                        ptInst->u16Sz += Mcb_FrameGetConfigData(&(ptInst->tRxfrm), pu16Data);

                        if (Mcb_FrameGetAddr(&(ptInst->tRxfrm)) == u16Addr)
                        {
                            if (Mcb_FrameGetSegmented(&(ptInst->tRxfrm)) != false)
                            {
                                ptInst->eState = MCB_READ_REQUEST;
                            }
                            else
                            {
                                ptInst->eState = MCB_SUCCESS;
                            }
                        }
                        break;
                    case MCB_REQ_IDLE:
                        ptInst->eState = MCB_READ_REQUEST;
                        break;
                    default:
                        ptInst->eState = MCB_ERROR;
                        break;
                }
            }
            else
            {
                ptInst->eState = MCB_ERROR;
            }

            if (ptInst->eState == MCB_ERROR)
            {
                Mcb_FrameCreateConfig(&(ptInst->tTxfrm), 0, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, false);
                isNewData = true;
            }
            break;
        default:
            ptInst->eState = MCB_STANDBY;
            break;
    }

    return isNewData;
}
