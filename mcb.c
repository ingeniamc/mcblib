/**
 * @file mcb.c
 * @brief This file contains API functions of the
 *        motion control bus (MCB)
 *
 * @author  Firmware department
 * @copyright Ingenia Motion Control (c) 2018. All rights reserved.
 */

#include "mcb.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

void McbInit(McbInst* ptInst, EMcbMode eMode)
{
    ptInst->isCyclic = false;
    ptInst->eMode = eMode;

    McbIntfInit(&ptInst->tIntf);
}

void McbDeinit(McbInst* ptInst)
{    
    ptInst->isCyclic = false;    
    ptInst->eMode = MCB_BLOCKING;
    McbIntfDeinit(&ptInst->tIntf);
}

EMcbReqStatus McbWrite(McbInst* ptInst, McbMsg* mcbMsg, uint32_t u32Timeout)
{
    EMcbReqStatus eResult = MCB_MESSAGE_ERROR;
    EMcbStatus eStatus;
    uint16_t u16sz;

    if (ptInst->isCyclic == false)
    {
        if (ptInst->eMode == MCB_BLOCKING)
        {
            uint32_t u32Millis = HAL_GetTick();
            do
            {
                eStatus = McbIntfWrite(&ptInst->tIntf, &mcbMsg->u16Node, &mcbMsg->u16Addr, &mcbMsg->u16Cmd,
                        &mcbMsg->u16Data[0], &mcbMsg->u16Size);

            } while ((eStatus != MCB_ERROR) && (eStatus != MCB_SUCCESS)
                    && ((HAL_GetTick() - u32Millis) < u32Timeout));
        }
        else
        {
            /** No blocking mode */
            eStatus = McbIntfWrite(&ptInst->tIntf, &mcbMsg->u16Node, &mcbMsg->u16Addr, &mcbMsg->u16Cmd,
                    &mcbMsg->u16Data[0], &u16sz);
        }
        if (eStatus == MCB_SUCCESS)
        {
            eResult = MCB_MESSAGE_SUCCESS;
        }
        else
        {
            eResult = MCB_MESSAGE_ERROR;
        }
    }
    else
    {
        /* Cyclic mode */
    }

    return eResult;
}

EMcbReqStatus McbRead(McbInst* ptInst, McbMsg* mcbMsg, uint32_t u32Timeout)
{
    EMcbReqStatus eResult = 0;
    EMcbStatus eStatus = MCB_ERROR;

    if (ptInst->isCyclic == false)
    {
        if (ptInst->eMode == MCB_BLOCKING)
        {
            uint32_t u32Millis = HAL_GetTick();
            do
            {
                eStatus = McbIntfRead(&ptInst->tIntf, &mcbMsg->u16Node, &mcbMsg->u16Addr, &mcbMsg->u16Cmd,
                        &mcbMsg->u16Data[0]);

            } while ((eStatus != MCB_ERROR) && (eStatus != MCB_SUCCESS)
                    && ((HAL_GetTick() - u32Millis) < u32Timeout));

            mcbMsg->u16Size = ptInst->tIntf.u16Sz;
        }
        else
        {
            /** No blocking mode */
            eStatus = McbIntfRead(&ptInst->tIntf, &mcbMsg->u16Node, &mcbMsg->u16Addr, &mcbMsg->u16Cmd,
                    &mcbMsg->u16Data[0]);
        }

        if (eStatus == MCB_SUCCESS)
        {
            eResult = MCB_MESSAGE_SUCCESS;
        }
        else
        {
            eResult = MCB_MESSAGE_ERROR;
        }
    }
    else
    {
        /* Cyclic mode */
    }

    if (eStatus == MCB_SUCCESS)
    {
        eResult = MCB_MESSAGE_SUCCESS;
    }
    else
    {
        eResult = MCB_MESSAGE_ERROR;
    }

    return eResult;
}

void* McbTxMap(McbInst* ptInst, uint16_t addr, uint16_t u16Sz)
{
    return NULL;
}

void* McbRxMap(McbInst* ptInst, uint16_t addr, uint16_t u16Sz)
{
    return NULL;
}

int32_t McbEnableCyclic(McbInst* ptInst)
{
    int32_t i32Result = 0;
    EMcbStatus eStatus;

    if (ptInst->isCyclic == false)
    {
        uint16_t u16Sz;
        McbMsg mcbMsg;
        mcbMsg.u16Addr = 0x640;
        mcbMsg.u16Cmd = 1;
        mcbMsg.u16Data[0] = 2;
        mcbMsg.u16Data[1] = 0;
        mcbMsg.u16Data[2] = 0;
        mcbMsg.u16Data[3] = 0;
        u16Sz = 4;
        do
        {
            eStatus = McbIntfWrite(&ptInst->tIntf, &mcbMsg.u16Node, &mcbMsg.u16Addr, &mcbMsg.u16Cmd, &mcbMsg.u16Data[0],
                    &u16Sz);

        } while ((eStatus != MCB_ERROR) && (eStatus != MCB_SUCCESS));
    }

    return i32Result;
}

int32_t McbDisableCyclic(McbInst* ptInst)
{
    if (ptInst->isCyclic != false)
    {

    }

    return 0;
}

