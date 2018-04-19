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

static Mcb_EStatus
Mcb_IntfCyclicTranfer(Mcb_TIntf* ptInst, uint16_t* ptInBuf, uint16_t* ptOutBuf);

void Mcb_IntfInit(Mcb_TIntf* ptInst)
{
    ptInst->eState = MCB_STANDBY;
    ptInst->isIrqEvnt = false;
}

void Mcb_IntfDeinit(Mcb_TIntf* ptInst)
{
    ptInst->eState = MCB_STANDBY;
    ptInst->isIrqEvnt = false;
}

Mcb_EStatus Mcb_IntfWrite(Mcb_TIntf* ptInst, uint16_t* pu16Node, uint16_t* pu16Addr, uint16_t* pu16Cmd,
        uint16_t* pu16Data, uint16_t* pu16Sz)
{
    switch (ptInst->eState)
    {
        case MCB_STANDBY:
            ptInst->u16Sz = *pu16Sz;
            ptInst->isPending = false;
            ptInst->eState = MCB_WRITE_REQUEST;
            break;
        case MCB_WRITE_REQUEST:
            if (Mcb_IntfIRQRead(ptInst->u16Id) == MCB_HIGH)
            {
                /* Check if static transmission should be segmented */
                if (ptInst->u16Sz > MCB_FRM_CONFIG_SZ)
                {
                    Mcb_FrameCreate(&(ptInst->tTxfrm), *pu16Addr, *pu16Cmd, MCB_FRM_SEG,
                            &pu16Data[*pu16Sz - ptInst->u16Sz], NULL, 0, false);
                    ptInst->isPending = true;
                }
                else
                {
                    Mcb_FrameCreate(&(ptInst->tTxfrm), *pu16Addr, *pu16Cmd, MCB_FRM_NOTSEG,
                            &pu16Data[*pu16Sz - ptInst->u16Sz], NULL, 0, false);
                }

                if (Mcb_IntfIsReady(ptInst->u16Id) != false)
                {
                    if ((ptInst->u16Sz - MCB_FRM_HEAD_SZ) >= MCB_FRM_CONFIG_SZ)
                    {
                        ptInst->u16Sz -= MCB_FRM_CONFIG_SZ;
                    }

                    ptInst->isIrqEvnt = false;

                    Mcb_IntfTransfer(ptInst, &(ptInst->tTxfrm), &(ptInst->tRxfrm));

                    /* Note: We prepare the next frame before the activation of the IRQ to
                     * improve the timing of the system */
                    if ((ptInst->u16Sz - MCB_FRM_HEAD_SZ) >= MCB_FRM_CONFIG_SZ)
                    {
                        Mcb_FrameCreate(&(ptInst->tTxfrm), *pu16Addr, *pu16Cmd, MCB_FRM_SEG,
                                &pu16Data[*pu16Sz - ptInst->u16Sz], NULL, 0, false);
                    }
                    else
                    {
                        Mcb_FrameCreate(&(ptInst->tTxfrm), *pu16Addr, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, NULL, 0,
                                false);
                    }
                    ptInst->eState = MCB_WRITE_REQUEST_ACK;
                }
            }
            break;
        case MCB_WRITE_REQUEST_ACK:
            /** Check if data is already available (IRQ) */
            if ((Mcb_IntfIsReady(ptInst->u16Id) != false) && (ptInst->isIrqEvnt != false))
            {
                ptInst->eState = MCB_WRITE_ANSWER;
                ptInst->u16Sz -= MCB_FRM_CONFIG_SZ;
                ptInst->isIrqEvnt = false;
                /* Now we just need to send the already built frame */
                Mcb_IntfTransfer(ptInst, &(ptInst->tTxfrm), &(ptInst->tRxfrm));
            }
            break;
        case MCB_WRITE_ANSWER:
            /** Wait until data is received */
            if ((Mcb_IntfIsReady(ptInst->u16Id) != false) && (ptInst->isIrqEvnt != false))
            {
                /** Check reception */
                if ((Mcb_IntfCheckCrc(ptInst->u16Id) != false) && (Mcb_FrameGetAddr(&(ptInst->tRxfrm)) == *pu16Addr))
                {
                    if (Mcb_FrameGetCmd(&(ptInst->tRxfrm)) == MCB_REP_WRITE_ERROR)
                    {
                        *pu16Cmd = Mcb_FrameGetCmd(&(ptInst->tRxfrm));
                        ptInst->eState = MCB_CANCEL;
                    }
                    else if (Mcb_FrameGetCmd(&(ptInst->tRxfrm)) == MCB_REP_ACK)
                    {
                        if (ptInst->isPending != false)
                        {
                            /* Prepare next frame */
                            if (ptInst->u16Sz > MCB_FRM_CONFIG_SZ)
                            {
                                Mcb_FrameCreate(&(ptInst->tTxfrm), *pu16Addr, *pu16Cmd, MCB_FRM_SEG,
                                        &pu16Data[*pu16Sz - ptInst->u16Sz], NULL, 0, false);
                            }
                            else if (ptInst->u16Sz == MCB_FRM_CONFIG_SZ)
                            {
                                Mcb_FrameCreate(&(ptInst->tTxfrm), *pu16Addr, *pu16Cmd, MCB_FRM_NOTSEG,
                                        &pu16Data[*pu16Sz - ptInst->u16Sz], NULL, 0, false);
                            }
                            else
                            {
                                /* Dummy message, allow reception of last frame CRC */
                                ptInst->isPending = false;
                                Mcb_FrameCreate(&(ptInst->tTxfrm), *pu16Addr, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, NULL,
                                        0,
                                        false);
                            }
                            ptInst->eState = MCB_WRITE_REQUEST_ACK;
                        }
                        else
                        {
                            *pu16Cmd = Mcb_FrameGetCmd(&(ptInst->tRxfrm));
                            ptInst->eState = MCB_SUCCESS;
                            *pu16Sz = MCB_FRM_CONFIG_SZ;
                        }
                    }
                    else
                    {
                        ptInst->eState = MCB_CANCEL;
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
                Mcb_IntfTransfer(ptInst, &(ptInst->tTxfrm), &(ptInst->tRxfrm));
                ptInst->eState = MCB_ERROR;
            }
            break;
        default:
            ptInst->eState = MCB_STANDBY;
            break;
    }

    return ptInst->eState;
}

Mcb_EStatus Mcb_IntfRead(Mcb_TIntf* ptInst, uint16_t* pu16Node, uint16_t* pu16Addr, uint16_t* pu16Cmd,
        uint16_t* pu16Data)
{
    switch (ptInst->eState)
    {
        case MCB_STANDBY:
            ptInst->isIrqEvnt = false;
            ptInst->eState = MCB_READ_REQUEST;
            break;
        case MCB_READ_REQUEST:
            if (Mcb_IntfIRQRead(ptInst->u16Id) == MCB_HIGH)
            {
                /* Send read request */
                Mcb_FrameCreate(&(ptInst->tTxfrm), *pu16Addr, MCB_REQ_READ, MCB_FRM_NOTSEG, pu16Data, NULL, 0, false);

                ptInst->isIrqEvnt = false;

                Mcb_IntfTransfer(ptInst, &(ptInst->tTxfrm), &(ptInst->tRxfrm));

                ptInst->u16Sz = 0;
                /* Note: We prepare the next frame before checking the IRQ to improve
                 * the timing of the system */
                Mcb_FrameCreate(&(ptInst->tTxfrm), 0, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, NULL, 0, false);
                ptInst->eState = MCB_READ_REQUEST_ACK;
            }
            break;
        case MCB_READ_REQUEST_ACK:
            /* Check if data is already available (IRQ) */
            if ((Mcb_IntfIsReady(ptInst->u16Id) == true) && (ptInst->isIrqEvnt == true))
            {
                /* Now we just need to send the already built frame */
                ptInst->isIrqEvnt = false;

                Mcb_IntfTransfer(ptInst, &(ptInst->tTxfrm), &(ptInst->tRxfrm));

                ptInst->eState = MCB_READ_ANSWER;
            }
            break;
        case MCB_READ_ANSWER:
            /** Wait until data is received */
            if (ptInst->isIrqEvnt == true)
            {
                /* Check reception */
                if ((Mcb_IntfCheckCrc(ptInst->u16Id) == true) && (Mcb_FrameGetAddr(&(ptInst->tRxfrm)) == *pu16Addr))
                {
                    /* Copy read data to buffer - Also copy it in case of error msg */
                    uint16_t u16SzRead = Mcb_FrameGetConfigData(&(ptInst->tRxfrm), pu16Data);
                    pu16Data += u16SzRead;
                    ptInst->u16Sz += u16SzRead;
                    *pu16Cmd = Mcb_FrameGetCmd(&(ptInst->tRxfrm));
                    if (Mcb_FrameGetCmd(&(ptInst->tRxfrm)) == MCB_REP_READ_ERROR)
                    {
                        ptInst->eState = MCB_CANCEL;
                    }
                    else if (Mcb_FrameGetCmd(&(ptInst->tRxfrm)) == MCB_REP_ACK)
                    {
                        if (Mcb_FrameGetSegmented(&(ptInst->tRxfrm)) != false)
                        {
                            ptInst->eState = MCB_READ_REQUEST_ACK;
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
                else
                {
                    ptInst->eState = MCB_CANCEL;
                }
            }
            break;
        case MCB_CANCEL:
            /* Cancel init transaction */
            if (Mcb_IntfIsReady(ptInst->u16Id) == true)
            {
                Mcb_FrameCreate(&(ptInst->tTxfrm), 0, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, NULL, 0, false);
                Mcb_IntfTransfer(ptInst, &(ptInst->tTxfrm), &(ptInst->tRxfrm));
                ptInst->eState = MCB_ERROR;
            }
            break;
        default:
            ptInst->eState = MCB_STANDBY;
            break;
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

Mcb_EStatus Mcb_IntfCyclicSpiTranfer(Mcb_TIntf* ptInst, uint16_t *ptInBuf, uint16_t *ptOutBuf)
{
    switch (ptInst->eState)
    {
        case MCB_STANDBY:
            /* Wait fall IRQ indicating received data */
            if (Mcb_IntfIRQRead(ptInst->u16Id) == MCB_HIGH)
            {
                Mcb_FrameCreate(&(ptInst->tRxfrm), 0, MCB_REQ_IDLE, MCB_FRM_NOTSEG, ptInBuf, ptOutBuf, 3, false);
                ptInst->eState = MCB_CYCLIC_ANSWER;
            }
            break;
        case MCB_CYCLIC_ANSWER:
            /** Wait until data is received */
            if (Mcb_IntfIsReady(ptInst->u16Id) == true)
            {
                ptInst->eState = MCB_STANDBY;
            }
            break;
        default:
            break;
    }

    return ptInst->eState;
}
