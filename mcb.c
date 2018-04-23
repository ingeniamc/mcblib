/**
 * @file mcb.c
 * @brief This file contains API functions of the
 *        motion control bus (MCB)
 *
 * @author  Firmware department
 * @copyright Ingenia Motion Control (c) 2018. All rights reserved.
 */

#include "mcb.h"
#include <string.h>

void Mcb_Init(Mcb_TInst* ptInst, Mcb_EMode eMode, uint16_t u16Id)
{
    ptInst->isCyclic = false;
    ptInst->eMode = eMode;

    ptInst->tIntf.u16Id = u16Id;
    Mcb_IntfInit(&ptInst->tIntf);
}

void Mcb_Deinit(Mcb_TInst* ptInst)
{    
    ptInst->isCyclic = false;    
    ptInst->eMode = MCB_BLOCKING;
    Mcb_IntfDeinit(&ptInst->tIntf);
}

Mcb_EReqStatus Mcb_Write(Mcb_TInst* ptInst, Mcb_TMsg* mcbMsg, uint32_t u32Timeout)
{
    Mcb_EReqStatus eResult = MCB_MESSAGE_ERROR;
    Mcb_EStatus eStatus;
    uint16_t u16sz;

    if (ptInst->isCyclic == false)
    {
        if (ptInst->eMode == MCB_BLOCKING)
        {
            uint32_t u32Millis = Mcb_GetMillis();

            do
            {
                eStatus = Mcb_IntfWrite(&ptInst->tIntf, &mcbMsg->u16Node, &mcbMsg->u16Addr, &mcbMsg->u16Cmd,
                        &mcbMsg->u16Data[0], &mcbMsg->u16Size);

            } while ((eStatus != MCB_ERROR) && (eStatus != MCB_SUCCESS) && ((Mcb_GetMillis() - u32Millis) < u32Timeout));
        }
        else
        {
            /** No blocking mode */
            eStatus = Mcb_IntfWrite(&ptInst->tIntf, &mcbMsg->u16Node, &mcbMsg->u16Addr, &mcbMsg->u16Cmd,
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

Mcb_EReqStatus Mcb_Read(Mcb_TInst* ptInst, Mcb_TMsg* mcbMsg, uint32_t u32Timeout)
{
    Mcb_EReqStatus eResult = 0;
    Mcb_EStatus eStatus = MCB_ERROR;

    if (ptInst->isCyclic == false)
    {
        if (ptInst->eMode == MCB_BLOCKING)
        {
            uint32_t u32Millis = Mcb_GetMillis();
            do
            {
                eStatus = Mcb_IntfRead(&ptInst->tIntf, &mcbMsg->u16Node, &mcbMsg->u16Addr, &mcbMsg->u16Cmd,
                        &mcbMsg->u16Data[0]);

            } while ((eStatus != MCB_ERROR) && (eStatus != MCB_SUCCESS)
                    && ((Mcb_GetMillis() - u32Millis) < u32Timeout));

            mcbMsg->u16Size = ptInst->tIntf.u16Sz;
        }
        else
        {
            /** No blocking mode */
            eStatus = Mcb_IntfRead(&ptInst->tIntf, &mcbMsg->u16Node, &mcbMsg->u16Addr, &mcbMsg->u16Cmd,
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

void* Mcb_TxMap(Mcb_TInst* ptInst, uint16_t addr, uint16_t u16Sz)
{
    return NULL;
}

void* Mcb_RxMap(Mcb_TInst* ptInst, uint16_t addr, uint16_t u16Sz)
{
    return NULL;
}

int32_t Mcb_EnableCyclic(Mcb_TInst* ptInst)
{
    int32_t i32Result = 0;
    Mcb_EStatus eStatus;

    if (ptInst->isCyclic == false)
    {
        uint16_t u16Sz;
        Mcb_TMsg mcbMsg;
        mcbMsg.u16Addr = 0x640;
        mcbMsg.u16Cmd = 1;
        mcbMsg.u16Data[0] = 2;
        mcbMsg.u16Data[1] = 0;
        mcbMsg.u16Data[2] = 0;
        mcbMsg.u16Data[3] = 0;
        u16Sz = 4;
        do
        {
            eStatus = Mcb_IntfWrite(&ptInst->tIntf, &mcbMsg.u16Node, &mcbMsg.u16Addr, &mcbMsg.u16Cmd,
                    &mcbMsg.u16Data[0],
                    &u16Sz);

        } while ((eStatus != MCB_ERROR) && (eStatus != MCB_SUCCESS));
    }

    return i32Result;
}

int32_t Mcb_DisableCyclic(Mcb_TInst* ptInst)
{
    if (ptInst->isCyclic != false)
    {

    }

    return 0;
}

