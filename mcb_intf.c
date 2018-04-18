/**
 * @file mcb_intf.c
 * @brief This file contains API for accessing to low level interface
 * 		  of the motion control bus (MCB)
 *
 * @author  Firmware department
 * @copyright Ingenia Motion Control (c) 2018. All rights reserved.
 */

#include "mcb_intf.h"
#include "mcb_frame.h"
#include <stdint.h>
#include <stdio.h>

#define DFLT_TIMEOUT  100
#define SIZE_WORDS    2

static void
McbIntfTransfer(const McbIntf* ptInst, TMcbFrame* tInFrame, TMcbFrame* tOutFrame);

static uint8_t
McbIntfIRQRead(const McbIntf* ptInst);

static bool
McbIntfIsReady(const McbIntf* ptInst);

static bool
McbIntfCheckCrc(const McbIntf* ptInst);

static EMcbStatus
McbIntfCyclicTranfer(McbIntf* ptInst, uint16_t* ptInBuf, uint16_t* ptOutBuf);

static void McbIntfTransfer(const McbIntf* ptInst, TMcbFrame* ptInFrame, TMcbFrame* ptOutFrame)
{
    /** Set to low Chip select pin */

    /** Launch a SPI transfer */
}

void McbIntfInit(McbIntf* ptInst)
{
    ptInst->eState = MCB_STANDBY;

    /** Linkar IRQ, pin IRQ, SYNC0 y SYNC1, and CS */
    ptInst->isIrqEvnt = false;
    /** SPI no es necesario creo */
}

void McbIntfDeinit(McbIntf* ptInst)
{

}

EMcbStatus McbIntfWrite(McbIntf* ptInst, uint16_t* ptNode, uint16_t* ptAddr, uint16_t* ptCmd, uint16_t* ptData,
        uint16_t* ptSz)
{
    switch (ptInst->eState)
    {
        case MCB_STANDBY:
            ptInst->u16Sz = *ptSz;
            ptInst->isPending = false;
            ptInst->eState = MCB_WRITE_REQUEST;
            break;
        case MCB_WRITE_REQUEST:
            if (McbIntfIRQRead(ptInst) == 1)
            {
                /* Check if static transmission should be segmented */
                if (ptInst->u16Sz > MCB_FRM_CONFIG_SZ)
                {
                    McbFrameCreate(&(ptInst->tTxfrm), *ptAddr, *ptCmd, MCB_FRM_SEG,
                            &ptData[*ptSz - ptInst->u16Sz], NULL, 0, false);
                    ptInst->isPending = true;
                }
                else
                {
                    McbFrameCreate(&(ptInst->tTxfrm), *ptAddr, *ptCmd, MCB_FRM_NOTSEG,
                            &ptData[*ptSz - ptInst->u16Sz], NULL, 0, false);
                }

                if (McbIntfIsReady(ptInst) == true)
                {
                    if ((ptInst->u16Sz - MCB_FRM_HEAD_SZ) >= MCB_FRM_CONFIG_SZ)
                    {
                        ptInst->u16Sz -= MCB_FRM_CONFIG_SZ;
                    }

                    ptInst->isIrqEvnt = false;

                    McbIntfTransfer(ptInst, &(ptInst->tTxfrm), &(ptInst->tRxfrm));

                    if ((ptInst->u16Sz - MCB_FRM_HEAD_SZ) >= MCB_FRM_CONFIG_SZ)
                    {
                        McbFrameCreate(&(ptInst->tTxfrm), *ptAddr, *ptCmd, MCB_FRM_SEG,
                                &ptData[*ptSz - ptInst->u16Sz], NULL, 0, false);
                    }
                    else
                    {
                        /* Note: We prepare the next frame before the activation of the IRQ to
                         * improve the timing of the system */
                        McbFrameCreate(&(ptInst->tTxfrm), *ptAddr, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, NULL, 0,
                                false);
                    }
                    ptInst->eState = MCB_WRITE_REQUEST_ACK;
                }
            }
            break;
        case MCB_WRITE_REQUEST_ACK:
            /** Check if data is already available (IRQ) */
            if ((McbIntfIsReady(ptInst) == true) && (ptInst->isIrqEvnt == true))
            {
                ptInst->eState = MCB_WRITE_ANSWER;
                ptInst->u16Sz -= MCB_FRM_CONFIG_SZ;
                ptInst->isIrqEvnt = false;
                /* Now we just need to send the already built frame */
                McbIntfTransfer(ptInst, &(ptInst->tTxfrm), &(ptInst->tRxfrm));
            }
            break;
        case MCB_WRITE_ANSWER:
            /** Wait until data is received */
            if ((McbIntfIsReady(ptInst) == true) && (ptInst->isIrqEvnt == true))
            {
                /** Check reception */
                if ((McbIntfCheckCrc(ptInst) == true) && (McbFrameGetAddr(&(ptInst->tRxfrm)) == *ptAddr))
                {
                    if (McbFrameGetCmd(&(ptInst->tRxfrm)) == MCB_REP_WRITE_ERROR)
                    {
                        *ptCmd = McbFrameGetCmd(&(ptInst->tRxfrm));
                        ptInst->eState = MCB_CANCEL;
                    }
                    else if (McbFrameGetCmd(&(ptInst->tRxfrm)) == MCB_REP_ACK)
                    {
                        if (ptInst->isPending == true)
                        {
                            /* Prepare next frame */
                            if (ptInst->u16Sz > MCB_FRM_CONFIG_SZ)
                            {
                                McbFrameCreate(&(ptInst->tTxfrm), *ptAddr, *ptCmd, MCB_FRM_SEG,
                                        &ptData[*ptSz - ptInst->u16Sz], NULL, 0,
                                        false);
                            }
                            else if (ptInst->u16Sz == MCB_FRM_CONFIG_SZ)
                            {
                                McbFrameCreate(&(ptInst->tTxfrm), *ptAddr, *ptCmd, MCB_FRM_NOTSEG,
                                        &ptData[*ptSz - ptInst->u16Sz], NULL, 0,
                                        false);
                            }
                            else
                            {
                                /* Dummy message, allow reception of last frame CRC */
                                ptInst->isPending = false;
                                McbFrameCreate(&(ptInst->tTxfrm), *ptAddr, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL,
                                        NULL, 0, false);
                            }
                            ptInst->eState = MCB_WRITE_REQUEST_ACK;
                        }
                        else
                        {
                            *ptCmd = McbFrameGetCmd(&(ptInst->tRxfrm));
                            ptInst->eState = MCB_SUCCESS;
                            *ptSz = MCB_FRM_CONFIG_SZ;
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
            if (McbIntfIsReady(ptInst) == true)
            {
                McbFrameCreate(&(ptInst->tTxfrm), 0, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, NULL, 0, false);
                McbIntfTransfer(ptInst, &(ptInst->tTxfrm), &(ptInst->tRxfrm));
                ptInst->eState = MCB_ERROR;
            }
            break;
        default:
            ptInst->eState = MCB_STANDBY;
            break;
    }

    return ptInst->eState;
}

EMcbStatus McbIntfRead(McbIntf* ptInst, uint16_t* ptNode, uint16_t* ptAddr, uint16_t* ptCmd, uint16_t* ptData)
{
    switch (ptInst->eState)
    {
        case MCB_STANDBY:
            ptInst->isIrqEvnt = false;
            ptInst->eState = MCB_READ_REQUEST;
            break;
        case MCB_READ_REQUEST:
            if (McbIntfIRQRead(ptInst) == 1)
            {
                /* Send read request */
                McbFrameCreate(&(ptInst->tTxfrm), *ptAddr, MCB_REQ_READ, MCB_FRM_NOTSEG, ptData, NULL, 0, false);

                ptInst->isIrqEvnt = false;

                McbIntfTransfer(ptInst, &(ptInst->tTxfrm), &(ptInst->tRxfrm));

                ptInst->u16Sz = 0;
                /* Note: We prepare the next frame before checking the IRQ to improve
                 * the timing of the system */
                McbFrameCreate(&(ptInst->tTxfrm), 0, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, NULL, 0, false);
                ptInst->eState = MCB_READ_REQUEST_ACK;
            }
            break;
        case MCB_READ_REQUEST_ACK:
            /* Check if data is already available (IRQ) */
            if ((McbIntfIsReady(ptInst) == true) && (ptInst->isIrqEvnt == true))
            {
                /* Now we just need to send the already built frame */
                ptInst->isIrqEvnt = false;

                McbIntfTransfer(ptInst, &(ptInst->tTxfrm), &(ptInst->tRxfrm));

                ptInst->eState = MCB_READ_ANSWER;
            }
            break;
        case MCB_READ_ANSWER:
            /** Wait until data is received */
            if (ptInst->isIrqEvnt == true)
            {
                /* Check reception */
                if ((McbIntfCheckCrc(ptInst) == true) && (McbFrameGetAddr(&(ptInst->tRxfrm)) == *ptAddr))
                {
                    /* Copy read data to buffer - Also copy it in case of error msg */
                    size_t szReaded = McbFrameGetConfigData(&(ptInst->tRxfrm), ptData);
                    ptData += szReaded;
                    ptInst->u16Sz += szReaded;
                    *ptCmd = McbFrameGetCmd(&(ptInst->tRxfrm));
                    if (McbFrameGetCmd(&(ptInst->tRxfrm)) == MCB_REP_READ_ERROR)
                    {
                        ptInst->eState = MCB_CANCEL;
                    }
                    else if (McbFrameGetCmd(&(ptInst->tRxfrm)) == MCB_REP_ACK)
                    {
                        if (McbFrameGetSegmented(&(ptInst->tRxfrm)) != false)
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
            if (McbIntfIsReady(ptInst) == true)
            {
                McbFrameCreate(&(ptInst->tTxfrm), 0, MCB_REQ_IDLE, MCB_FRM_NOTSEG, NULL, NULL, 0, false);
                McbIntfTransfer(ptInst, &(ptInst->tTxfrm), &(ptInst->tRxfrm));
                ptInst->eState = MCB_ERROR;
            }
            break;
        default:
            ptInst->eState = MCB_STANDBY;
            break;
    }

    return ptInst->eState;
}

EMcbStatus McbIntfCyclicSpiTranfer(McbIntf* ptInst, uint16_t *ptInBuf, uint16_t *ptOutBuf)
{
    switch (ptInst->eState)
    {
        case MCB_STANDBY:
            /* Wait fall IRQ indicating received data */
            if (McbIntfIRQRead(ptInst) == 1)
            {
                McbFrameCreate(&(ptInst->tRxfrm), 0, MCB_REQ_IDLE, MCB_FRM_NOTSEG, ptInBuf, ptOutBuf, 3,
                        false);
                ptInst->eState = MCB_CYCLIC_ANSWER;
            }
            break;
        case MCB_CYCLIC_ANSWER:
            /** Wait until data is received */
            if (McbIntfIsReady(ptInst) == true)
            {
                ptInst->eState = MCB_STANDBY;
            }
            break;
        default:
            break;
    }

    return ptInst->eState;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
        default:
            //u16Irq = ENABLE;
            break;
    }
}

static uint8_t McbIntfIRQRead(const McbIntf* ptInst)
{
    return 0;
}

static bool McbIntfIsReady(const McbIntf* ptInst)
{
    return false;
}

static bool McbIntfCheckCrc(const McbIntf* ptInst)
{
    return false;
}
