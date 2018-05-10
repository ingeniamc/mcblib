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
Mcb_IntfWriteCfg(Mcb_TIntf* ptInst, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz);

static bool
Mcb_IntfReadCfg(Mcb_TIntf* ptInst, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz);

static bool
Mcb_IntfWriteCfgOverCyclic(Mcb_TIntf* ptInst, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz);

static bool
Mcb_IntfReadCfgOverCyclic(Mcb_TIntf* ptInst, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz);

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
    bool isNewData = false;

    /** Check if data is already available (IRQ) & SPI is ready for transmission */
    if ((Mcb_IntfIsReady(ptInst->u16Id) != false) && (ptInst->isIrqEvnt != false))
    {
        if ((ptInst->eState == MCB_WRITE_ANSWER) && (Mcb_IntfCheckCrc(ptInst->u16Id, ptInst->tRxfrm.u16Buf, ptInst->tTxfrm.u16Sz) == false))
        {
            ptInst->eState = MCB_ERROR;
        }
        else
        {
            isNewData = Mcb_IntfWriteCfg(ptInst, u16Addr, pu16Data, pu16Sz);
        }

        /** Set up a new frame if an error is detected */
        if (ptInst->eState == MCB_ERROR)
        {
            Mcb_FrameCreateConfig(&(ptInst->tTxfrm), 0, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, ptInst->calcCrc);
            isNewData = true;
        }

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
    bool isNewData = false;

    /** Check if data is already available (IRQ) & SPI is ready for transmission */
    if ((Mcb_IntfIsReady(ptInst->u16Id) != false) && (ptInst->isIrqEvnt != false))
    {
        if ((ptInst->eState == MCB_READ_ANSWER) && (Mcb_IntfCheckCrc(ptInst->u16Id, ptInst->tRxfrm.u16Buf, ptInst->tTxfrm.u16Sz) == false))
        {
            ptInst->eState = MCB_ERROR;
        }
        else
        {
            isNewData = Mcb_IntfReadCfg(ptInst, u16Addr, pu16Data, pu16Sz);
        }

        /** Set up a new frame if an error is detected */
        if (ptInst->eState == MCB_ERROR)
        {
            Mcb_FrameCreateConfig(&(ptInst->tTxfrm), 0, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, ptInst->calcCrc);
            isNewData = true;
        }

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

Mcb_EStatus Mcb_IntfCyclicTransfer(Mcb_TIntf* ptInst, uint16_t u16Node, uint16_t u16Addr, uint16_t* pu16Cmd,
        uint16_t* pu16Data, uint16_t* pu16CfgSz, uint16_t *ptInBuf, uint16_t *ptOutBuf, uint16_t u16CyclicSz)
{
    Mcb_EStatus eCyclicState = MCB_STANDBY;

    if ((Mcb_IntfIsReady(ptInst->u16Id) != false) && (ptInst->isIrqEvnt != false))
    {
        eCyclicState = MCB_CYCLIC_REQUEST;
        /** Indicate that a cyclic message is transmitted */
        /** Get cyclic data from last transmission */
        if (Mcb_IntfCheckCrc(ptInst->u16Id, ptInst->tRxfrm.u16Buf, ptInst->tTxfrm.u16Sz) != false)
        {
            Mcb_FrameGetCyclicData(&ptInst->tRxfrm, ptOutBuf, u16CyclicSz);
        }

        bool isNewData = false;
        /** If a frame command is requested, add it into cyclic frame */
        if (ptInst->isCfgOverCyclic == false)
        {
            switch (*pu16Cmd)
            {
                case MCB_REQ_READ:
                    /** Generate initial frame */
                    isNewData = Mcb_IntfReadCfgOverCyclic(ptInst, u16Addr, pu16Data, pu16CfgSz);
                    ptInst->isCfgOverCyclic = true;
                    break;
                case MCB_REQ_WRITE:
                    /** Generate initial frame */
                    ptInst->isCfgOverCyclic = true;
                    isNewData = Mcb_IntfWriteCfgOverCyclic(ptInst, u16Addr, pu16Data, pu16CfgSz);
                    break;
                default:
                    /** Nothing */
                    break;
            }
        }
        else
        {
            /** Keep on processing config frame */
            switch (*pu16Cmd)
            {
                case MCB_REQ_READ:
                    isNewData = Mcb_IntfReadCfg(ptInst, u16Addr, pu16Data, pu16CfgSz);
                    break;
                case MCB_REQ_WRITE:
                    isNewData = Mcb_IntfWriteCfg(ptInst, u16Addr, pu16Data, pu16CfgSz);
                    break;
                default:
                    /** Nothing */
                    break;
            }

            if (ptInst->eState == MCB_SUCCESS)
            {
                *pu16Cmd = MCB_REP_ACK;
                ptInst->isCfgOverCyclic = false;
                eCyclicState = MCB_CYCLIC_SUCCESS;
            }
            else if (ptInst->eState == MCB_ERROR)
            {
                *pu16Cmd |= MCB_REP_ERROR;
                ptInst->isCfgOverCyclic = false;
                eCyclicState = MCB_CYCLIC_ERROR;
            }
            else
            {
                /** Nothing */
            }
        }

        if (isNewData == false)
        {
            Mcb_FrameCreateConfig(&(ptInst->tTxfrm), 0, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, ptInst->calcCrc);
            Mcb_FrameAppendCyclic(&(ptInst->tTxfrm), ptInBuf, u16CyclicSz, ptInst->calcCrc);
        }
        else
        {
            Mcb_FrameAppendCyclic(&(ptInst->tTxfrm), ptInBuf, u16CyclicSz, ptInst->calcCrc);
        }

        ptInst->isIrqEvnt = false;
        Mcb_IntfTransfer(ptInst, &(ptInst->tTxfrm), &(ptInst->tRxfrm));
    }

    return eCyclicState;
}

static bool Mcb_IntfWriteCfg(Mcb_TIntf* ptInst, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz)
{
    bool isNewData = false;

    if ((ptInst->eState == MCB_STANDBY) || (ptInst->eState == MCB_SUCCESS) || (ptInst->eState == MCB_ERROR))
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
                        &pu16Data[*pu16Sz - ptInst->u16Sz], ptInst->calcCrc);
                ptInst->u16Sz -= MCB_FRM_CONFIG_SZ;
            }
            else if (ptInst->u16Sz == 0)
            {
                Mcb_FrameCreateConfig(&(ptInst->tTxfrm), u16Addr, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, ptInst->calcCrc);
                ptInst->isPending = false;
            }
            else
            {
                Mcb_FrameCreateConfig(&(ptInst->tTxfrm), u16Addr, MCB_REQ_WRITE, MCB_FRM_NOTSEG,
                        &pu16Data[*pu16Sz - ptInst->u16Sz], ptInst->calcCrc);
                ptInst->u16Sz = 0;
            }

            ptInst->isIrqEvnt = false;
            isNewData = true;
            ptInst->eState = MCB_WRITE_ANSWER;
            break;
        case MCB_WRITE_ANSWER:
            /** Check reception */
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
                            ptInst->u16Sz = MCB_FRM_CONFIG_SZ;
                            ptInst->eState = MCB_SUCCESS;
                        }
                    }
                    else
                    {
                        ptInst->eState = MCB_ERROR;
                    }
                    break;
                case MCB_REQ_IDLE:
                    ptInst->eState = MCB_WRITE_REQUEST;
                    break;
                default:
                    ptInst->eState = MCB_ERROR;
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

    if ((ptInst->eState == MCB_STANDBY) || (ptInst->eState == MCB_SUCCESS) || (ptInst->eState == MCB_ERROR))
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
                Mcb_FrameCreateConfig(&(ptInst->tTxfrm), u16Addr, MCB_REQ_READ, MCB_FRM_NOTSEG, pu16Data, ptInst->calcCrc);
            }
            else
            {
                Mcb_FrameCreateConfig(&(ptInst->tTxfrm), u16Addr, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, ptInst->calcCrc);
            }

            ptInst->isIrqEvnt = false;
            isNewData = true;
            ptInst->eState = MCB_READ_ANSWER;
            break;
        case MCB_READ_ANSWER:
            /** Check reception */
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
                            *pu16Sz = ptInst->u16Sz;
                            ptInst->eState = MCB_SUCCESS;
                        }
                    }
                    else
                    {
                        ptInst->eState = MCB_ERROR;
                    }
                    break;
                case MCB_REQ_IDLE:
                    ptInst->eState = MCB_READ_REQUEST;
                    ptInst->isPending = false;
                    break;
                default:
                    ptInst->eState = MCB_ERROR;
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

    if ((ptInst->eState == MCB_STANDBY) || (ptInst->eState == MCB_SUCCESS) || (ptInst->eState == MCB_ERROR))
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
                        &pu16Data[*pu16Sz - ptInst->u16Sz], ptInst->calcCrc);
                ptInst->u16Sz -= MCB_FRM_CONFIG_SZ;
            }
            else
            {
                Mcb_FrameCreateConfig(&(ptInst->tTxfrm), u16Addr, MCB_REQ_WRITE, MCB_FRM_NOTSEG,
                        &pu16Data[*pu16Sz - ptInst->u16Sz], ptInst->calcCrc);
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
                    if (Mcb_FrameGetAddr(&(ptInst->tRxfrm)) == u16Addr)
                    {
                        if (ptInst->isPending != false)
                        {
                            ptInst->eState = MCB_WRITE_REQUEST;
                        }
                        else
                        {
                            ptInst->u16Sz = MCB_FRM_CONFIG_SZ;
                            ptInst->eState = MCB_SUCCESS;
                        }
                    }
                    else
                    {
                        ptInst->eState = MCB_ERROR;
                    }
                    break;
                case MCB_REQ_IDLE:
                    /** Waiting for reply */
                    break;
                default:
                    ptInst->eState = MCB_ERROR;
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

    if ((ptInst->eState == MCB_STANDBY) || (ptInst->eState == MCB_SUCCESS) || (ptInst->eState == MCB_ERROR))
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
                Mcb_FrameCreateConfig(&(ptInst->tTxfrm), u16Addr, MCB_REQ_READ, MCB_FRM_NOTSEG, pu16Data, ptInst->calcCrc);
            }
            else
            {
                Mcb_FrameCreateConfig(&(ptInst->tTxfrm), u16Addr, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, ptInst->calcCrc);
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
                    ptInst->u16Sz += Mcb_FrameGetConfigData(&(ptInst->tRxfrm), pu16Data);

                    if (Mcb_FrameGetAddr(&(ptInst->tRxfrm)) == u16Addr)
                    {
                        if (Mcb_FrameGetSegmented(&(ptInst->tRxfrm)) != false)
                        {
                            ptInst->eState = MCB_READ_REQUEST;
                        }
                        else
                        {
                            *pu16Sz = ptInst->u16Sz;
                            ptInst->eState = MCB_SUCCESS;
                        }
                    }
                    else
                    {
                        ptInst->eState = MCB_ERROR;
                    }
                    break;
                case MCB_REQ_IDLE:
                    /** Waiting for new data */
                    break;
                default:
                    ptInst->eState = MCB_ERROR;
                    break;
            }
            break;
        default:
            ptInst->eState = MCB_STANDBY;
            break;
    }

    return isNewData;
}
