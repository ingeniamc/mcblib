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

void Mcb_Init(Mcb_TInst* ptInst, Mcb_EMode eMode, uint16_t u16Id, uint32_t u32Timeout)
{
    ptInst->isCyclic = false;
    ptInst->eMode = eMode;
    ptInst->u32Timeout = u32Timeout;

    ptInst->tIntf.u16Id = u16Id;
    Mcb_IntfInit(&ptInst->tIntf);
}

void Mcb_Deinit(Mcb_TInst* ptInst)
{    
    ptInst->isCyclic = false;    
    ptInst->eMode = MCB_BLOCKING;
    Mcb_IntfDeinit(&ptInst->tIntf);
}

Mcb_EStatus Mcb_Write(Mcb_TInst* ptInst, Mcb_TMsg* pMcbMsg)
{
    Mcb_EStatus eStatus = MCB_ERROR;
    uint16_t u16sz;

    if (ptInst->isCyclic == false)
    {
        if (ptInst->eMode == MCB_BLOCKING)
        {
            uint32_t u32Millis = Mcb_GetMillis();

            do
            {
                eStatus = Mcb_IntfWrite(&ptInst->tIntf, pMcbMsg->u16Node, pMcbMsg->u16Addr, &pMcbMsg->u16Data[0],
                        &pMcbMsg->u16Size);

                if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
                {
                    eStatus = MCB_ERROR;
                    break;
                }

            } while ((eStatus != MCB_ERROR) && (eStatus != MCB_SUCCESS));
        }
        else
        {
            /** Non blocking mode */
            eStatus = Mcb_IntfWrite(&ptInst->tIntf, pMcbMsg->u16Node, pMcbMsg->u16Addr, &pMcbMsg->u16Data[0], &u16sz);
        }
    }
    else
    {
        /* Cyclic mode */
        memcpy(&ptInst->tConfigTx, &pMcbMsg, sizeof(Mcb_TMsg));
    }

    return eStatus;
}

Mcb_EStatus Mcb_Read(Mcb_TInst* ptInst, Mcb_TMsg* pMcbMsg)
{
    Mcb_EStatus eStatus = MCB_ERROR;

    if (ptInst->isCyclic == false)
    {
        if (ptInst->eMode == MCB_BLOCKING)
        {
            uint32_t u32Millis = Mcb_GetMillis();
            do
            {
                eStatus = Mcb_IntfRead(&ptInst->tIntf, pMcbMsg->u16Node, pMcbMsg->u16Addr, &pMcbMsg->u16Data[0],
                        &pMcbMsg->u16Size);

                if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
                {
                    eStatus = MCB_ERROR;
                    break;
                }

            } while ((eStatus != MCB_ERROR) && (eStatus != MCB_SUCCESS));

            pMcbMsg->u16Size = ptInst->tIntf.u16Sz;
        }
        else
        {
            /** Non blocking mode */
            eStatus = Mcb_IntfRead(&ptInst->tIntf, pMcbMsg->u16Node, pMcbMsg->u16Addr, &pMcbMsg->u16Data[0],
                    &pMcbMsg->u16Size);
        }
    }
    else
    {
        /* Cyclic mode */
        memcpy(&ptInst->tConfigTx, &pMcbMsg, sizeof(Mcb_TMsg));
    }

    return eStatus;
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
        Mcb_TMsg pMcbMsg;
        pMcbMsg.u16Addr = 0x640;
        pMcbMsg.u16Cmd = 1;
        pMcbMsg.u16Data[0] = 2;
        pMcbMsg.u16Data[1] = 0;
        pMcbMsg.u16Data[2] = 0;
        pMcbMsg.u16Data[3] = 0;
        u16Sz = 4;
        do
        {
            eStatus = Mcb_IntfWrite(&ptInst->tIntf, pMcbMsg.u16Node, pMcbMsg.u16Addr, &pMcbMsg.u16Data[0], &u16Sz);

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

void Mcb_CyclicProcess(Mcb_TInst* ptInst)
{
    if (ptInst->isCyclic != false)
    {
        Mcb_IntfCyclicTranfer(&ptInst->tIntf, ptInst->tConfigTx.u16Node, ptInst->tConfigTx.u16Addr,
                ptInst->tConfigTx.u16Cmd, ptInst->tConfigTx.u16Data, &ptInst->tConfigTx.u16Size, ptInst->u16CyclicTx,
                ptInst->u16CyclicRx);
    }
}

