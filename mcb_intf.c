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
}

void Mcb_IntfDeinit(Mcb_TIntf* ptInst)
{
    ptInst->eState = MCB_STANDBY;
    ptInst->isIrqEvnt = false;
}

Mcb_EStatus Mcb_IntfWrite(Mcb_TIntf* ptInst, uint16_t u16Node, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz)
{
    bool isNewData = Mcb_IntfWriteProcess(ptInst, u16Addr, pu16Data, pu16Sz);

    /** Use node to choose the chip select */

    if (isNewData != false)
    {
        Mcb_IntfTransfer(ptInst, &(ptInst->tTxfrm), &(ptInst->tRxfrm));
    }

    return ptInst->eState;
}

Mcb_EStatus Mcb_IntfRead(Mcb_TIntf* ptInst, uint16_t u16Node, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz)
{
    bool isNewData = Mcb_IntfReadProcess(ptInst, u16Addr, pu16Data, pu16Sz);

    /** Use node to choose the chip select */

    if (isNewData != false)
    {
        Mcb_IntfTransfer(ptInst, &(ptInst->tTxfrm), &(ptInst->tRxfrm));
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

Mcb_EStatus Mcb_IntfCyclicTranfer(Mcb_TIntf* ptInst, uint16_t u16Node, uint16_t u16Addr, uint16_t u16Cmd,
        uint16_t* pu16Data, uint16_t* pu16Sz, uint16_t *ptInBuf, uint16_t *ptOutBuf)
{
    bool isNewData;

    switch (u16Cmd)
    {
        case MCB_REQ_READ:
            isNewData = Mcb_IntfReadProcess(ptInst, u16Addr, pu16Data, pu16Sz);
            break;
        case MCB_REQ_WRITE:
            isNewData = Mcb_IntfWriteProcess(ptInst, u16Addr, pu16Data, pu16Sz);
            break;
        default:
            /* Nothing */
            break;
    }

    if ((Mcb_IntfIsReady(ptInst->u16Id) != false) && (ptInst->isIrqEvnt != false))
    {
        if (isNewData != false)
        {
            Mcb_FrameCreate(&(ptInst->tRxfrm), 0, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, ptInBuf, 3, false);
        }
        else
        {
            Mcb_FrameCreate(&(ptInst->tRxfrm), 0, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, ptInBuf, 3, false);
        }

        Mcb_IntfTransfer(ptInst, &(ptInst->tTxfrm), &(ptInst->tRxfrm));
    }

    return ptInst->eState;
}

static bool Mcb_IntfWriteProcess(Mcb_TIntf* ptInst, uint16_t u16Addr, uint16_t* pu16Data, uint16_t* pu16Sz)
{
    bool isNewData = false;

    switch (ptInst->eState)
    {
        case MCB_STANDBY:
            ptInst->u16Sz = *pu16Sz;
            ptInst->isPending = true;
            ptInst->eState = MCB_WRITE_REQUEST;
            break;
        case MCB_WRITE_REQUEST:
            /** Check if data is already available (IRQ) */
            if ((Mcb_IntfIsReady(ptInst->u16Id) != false) && (ptInst->isIrqEvnt != false))
            {
                if (ptInst->u16Sz > MCB_FRM_CONFIG_SZ)
                {
                    Mcb_FrameCreate(&(ptInst->tTxfrm), u16Addr, MCB_REQ_WRITE, MCB_FRM_SEG,
                            &pu16Data[*pu16Sz - ptInst->u16Sz], NULL, 0, false);
                    ptInst->u16Sz -= MCB_FRM_CONFIG_SZ;
                }
                else if (ptInst->u16Sz == 0)
                {
                    Mcb_FrameCreate(&(ptInst->tTxfrm), u16Addr, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, NULL, 0, false);
                    ptInst->isPending = false;
                }
                else
                {
                    Mcb_FrameCreate(&(ptInst->tTxfrm), u16Addr, MCB_REQ_WRITE, MCB_FRM_NOTSEG,
                            &pu16Data[*pu16Sz - ptInst->u16Sz], NULL, 0, false);
                    ptInst->u16Sz = 0;
                }

                ptInst->isIrqEvnt = false;
                isNewData = true;
                ptInst->eState = MCB_WRITE_ANSWER;
            }
            break;
        case MCB_WRITE_ANSWER:
            /** Wait until data is received */
            if ((Mcb_IntfIsReady(ptInst->u16Id) != false) && (ptInst->isIrqEvnt != false))
            {
                /** Check reception */
                if ((Mcb_IntfCheckCrc(ptInst->u16Id) != false) && (Mcb_FrameGetAddr(&(ptInst->tRxfrm)) == u16Addr)
                        && (Mcb_FrameGetCmd(&(ptInst->tRxfrm)) == MCB_REP_ACK))
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
                else
                {
                    ptInst->eState = MCB_CANCEL;
                }
            }
            break;
        case MCB_CANCEL:
            /* Cancel init transaction */
            if (Mcb_IntfIsReady(ptInst->u16Id) != false)
            {
                Mcb_FrameCreate(&(ptInst->tTxfrm), 0, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, NULL, 0, false);
                isNewData = true;
                ptInst->eState = MCB_ERROR;
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

    switch (ptInst->eState)
    {
        case MCB_STANDBY:
            ptInst->isPending = true;
            ptInst->eState = MCB_READ_REQUEST;
            break;
        case MCB_READ_REQUEST:
            /** Check if data is already available (IRQ) */
            if ((Mcb_IntfIsReady(ptInst->u16Id) != false) && (ptInst->isIrqEvnt != false))
            {
                /* Send read request */
                Mcb_FrameCreate(&(ptInst->tTxfrm), u16Addr, MCB_REQ_READ, MCB_FRM_NOTSEG, pu16Data, NULL, 0, false);

                ptInst->isIrqEvnt = false;
                isNewData = true;
                ptInst->eState = MCB_READ_ANSWER;
            }
            break;
        case MCB_READ_ANSWER:
            /** Wait until data is received */
            if ((Mcb_IntfIsReady(ptInst->u16Id) != false) && (ptInst->isIrqEvnt != false))
            {
                /** Check reception */
                if ((Mcb_IntfCheckCrc(ptInst->u16Id) != false) && (Mcb_FrameGetAddr(&(ptInst->tRxfrm)) == u16Addr)
                        && (Mcb_FrameGetCmd(&(ptInst->tRxfrm)) == MCB_REP_ACK))
                {
                    /* Copy read data to buffer - Also copy it in case of error msg */
                    ptInst->u16Sz += Mcb_FrameGetConfigData(&(ptInst->tRxfrm), pu16Data);

                    if (Mcb_FrameGetSegmented(&(ptInst->tRxfrm)) != false)
                    {
                        ptInst->eState = MCB_READ_REQUEST;
                    }
                    else
                    {
                        ptInst->eState = MCB_SUCCESS;
                    }
                }
                else
                {
                    ptInst->eState = MCB_CANCEL;
                }
            }
            break;
        case MCB_CANCEL:
            /* Cancel init transaction */
            if (Mcb_IntfIsReady(ptInst->u16Id) != false)
            {
                Mcb_FrameCreate(&(ptInst->tTxfrm), 0, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, NULL, 0, false);
                isNewData = true;
                ptInst->eState = MCB_ERROR;
            }
            break;
        default:
            ptInst->eState = MCB_STANDBY;
            break;
    }

    return isNewData;
}
