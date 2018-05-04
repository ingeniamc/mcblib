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

#define CYCLIC_MODE (uint16_t)0x640
#define RX_MAP_BASE (uint16_t)0x650
#define TX_MAP_BASE (uint16_t)0x660

void Mcb_Init(Mcb_TInst* ptInst, Mcb_EMode eMode, uint16_t u16Id, uint32_t u32Timeout)
{
    ptInst->isCyclic = false;
    ptInst->eMode = eMode;
    ptInst->u32Timeout = u32Timeout;

    ptInst->tCyclicRxList.u8Mapped = 0;
    ptInst->tCyclicTxList.u8Mapped = 0;

    for (uint8_t u8Idx = 0; u8Idx < MAX_MAPPED_REG; u8Idx++)
    {
        ptInst->tCyclicRxList.u16Addr[u8Idx] = 0;
        ptInst->tCyclicRxList.u16Sz[u8Idx] = 0;
        ptInst->tCyclicTxList.u16Addr[u8Idx] = 0;
        ptInst->tCyclicTxList.u16Sz[u8Idx] = 0;
    }

    ptInst->tIntf.u16Id = u16Id;
    Mcb_IntfInit(&ptInst->tIntf);
}

void Mcb_Deinit(Mcb_TInst* ptInst)
{    
    ptInst->isCyclic = false;    
    ptInst->eMode = MCB_BLOCKING;
    Mcb_IntfDeinit(&ptInst->tIntf);

    ptInst->tCyclicRxList.u8Mapped = 0;
    ptInst->tCyclicTxList.u8Mapped = 0;

    for (uint8_t u8Idx = 0; u8Idx < MAX_MAPPED_REG; u8Idx++)
    {
        ptInst->tCyclicRxList.u16Addr[u8Idx] = 0;
        ptInst->tCyclicRxList.u16Sz[u8Idx] = 0;
        ptInst->tCyclicTxList.u16Addr[u8Idx] = 0;
        ptInst->tCyclicTxList.u16Sz[u8Idx] = 0;
    }
}

Mcb_EStatus Mcb_Write(Mcb_TInst* ptInst, Mcb_TMsg* pMcbMsg)
{
    uint16_t u16sz;

    pMcbMsg->eStatus = MCB_ERROR;

    if (ptInst->isCyclic == false)
    {
        if (ptInst->eMode == MCB_BLOCKING)
        {
            uint32_t u32Millis = Mcb_GetMillis();

            do
            {
                pMcbMsg->eStatus = Mcb_IntfWrite(&ptInst->tIntf, pMcbMsg->u16Node, pMcbMsg->u16Addr,
                        &pMcbMsg->u16Data[0], &pMcbMsg->u16Size);

                if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
                {
                    pMcbMsg->eStatus = MCB_ERROR;
                    break;
                }

            } while ((pMcbMsg->eStatus != MCB_ERROR) && (pMcbMsg->eStatus != MCB_SUCCESS));
        }
        else
        {
            /** Non blocking mode */
            pMcbMsg->eStatus = Mcb_IntfWrite(&ptInst->tIntf, pMcbMsg->u16Node, pMcbMsg->u16Addr, &pMcbMsg->u16Data[0],
                    &u16sz);
        }
    }
    else
    {
        /* Cyclic mode */
        memcpy(&ptInst->tConfig, pMcbMsg, sizeof(Mcb_TMsg));
    }

    return pMcbMsg->eStatus;
}

Mcb_EStatus Mcb_Read(Mcb_TInst* ptInst, Mcb_TMsg* pMcbMsg)
{
    pMcbMsg->eStatus = MCB_ERROR;

    if (ptInst->isCyclic == false)
    {
        if (ptInst->eMode == MCB_BLOCKING)
        {
            uint32_t u32Millis = Mcb_GetMillis();
            do
            {
                pMcbMsg->eStatus = Mcb_IntfRead(&ptInst->tIntf, pMcbMsg->u16Node, pMcbMsg->u16Addr,
                        &pMcbMsg->u16Data[0], &pMcbMsg->u16Size);

                if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
                {
                    pMcbMsg->eStatus = MCB_ERROR;
                    break;
                }

            } while ((pMcbMsg->eStatus != MCB_ERROR) && (pMcbMsg->eStatus != MCB_SUCCESS));

            pMcbMsg->u16Size = ptInst->tIntf.u16Sz;
        }
        else
        {
            /** Non blocking mode */
            pMcbMsg->eStatus = Mcb_IntfRead(&ptInst->tIntf, pMcbMsg->u16Node, pMcbMsg->u16Addr, &pMcbMsg->u16Data[0],
                    &pMcbMsg->u16Size);
        }
    }
    else
    {
        /* Cyclic mode */
        memcpy(&ptInst->tConfig, pMcbMsg, sizeof(Mcb_TMsg));
    }

    return pMcbMsg->eStatus;
}

void Mcb_AttachCfgOverCyclicCB(Mcb_TInst* ptInst, void (*Evnt)(Mcb_TInst* ptInst, Mcb_TMsg* pMcbMsg))
{
    if (Evnt != NULL)
    {
        ptInst->CfgOverCyclicEvnt = Evnt;
    }
    else
    {
        ptInst->CfgOverCyclicEvnt = NULL;
    }
}

void* Mcb_TxMap(Mcb_TInst* ptInst, uint16_t u16Addr, uint16_t u16Sz)
{
    Mcb_TMsg tMcbMsg;
    void* pRet = NULL;

    /** Set up internal struct and verify a proper configuration */
    if (ptInst->tCyclicTxList.u8Mapped < MAX_MAPPED_REG)
    {
        tMcbMsg.eStatus = MCB_STANDBY;
        tMcbMsg.u16Node = 2;
        tMcbMsg.u16Addr = TX_MAP_BASE + ptInst->tCyclicTxList.u8Mapped + 1;
        tMcbMsg.u16Cmd = MCB_REQ_WRITE;
        tMcbMsg.u16Size = sizeof(uint32_t);
        tMcbMsg.u16Data[0] = u16Addr;
        tMcbMsg.u16Data[1] = u16Sz;

        uint32_t u32Millis = Mcb_GetMillis();

        do
        {
            tMcbMsg.eStatus = Mcb_Write(ptInst, &tMcbMsg);

            if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
            {
                tMcbMsg.eStatus = MCB_ERROR;
                break;
            }

        } while ((tMcbMsg.eStatus != MCB_ERROR) && (tMcbMsg.eStatus != MCB_SUCCESS));

        switch (tMcbMsg.eStatus)
        {
            case MCB_SUCCESS:
                pRet = &ptInst->u16CyclicRx[ptInst->tCyclicTxList.u8Mapped];
                ptInst->tCyclicTxList.u16Addr[ptInst->tCyclicTxList.u8Mapped] = u16Addr;
                ptInst->tCyclicTxList.u16Sz[ptInst->tCyclicTxList.u8Mapped] = u16Sz;
                ptInst->tCyclicTxList.u8Mapped++;
                break;
            default:
                /** Nothing */
                break;
        }
    }

    return pRet;
}

void* Mcb_RxMap(Mcb_TInst* ptInst, uint16_t u16Addr, uint16_t u16Sz)
{
    Mcb_TMsg tMcbMsg;
    void* pRet = NULL;

    /** Set up internal struct and verify a proper configuration */
    if (ptInst->tCyclicRxList.u8Mapped < MAX_MAPPED_REG)
    {
        tMcbMsg.eStatus = MCB_STANDBY;
        tMcbMsg.u16Node = 2;
        tMcbMsg.u16Addr = RX_MAP_BASE + ptInst->tCyclicRxList.u8Mapped + 1;
        tMcbMsg.u16Cmd = MCB_REQ_WRITE;
        tMcbMsg.u16Size = sizeof(uint32_t);
        tMcbMsg.u16Data[0] = u16Addr;
        tMcbMsg.u16Data[1] = u16Sz;

        uint32_t u32Millis = Mcb_GetMillis();

        do
        {
            tMcbMsg.eStatus = Mcb_Write(ptInst, &tMcbMsg);

            if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
            {
                tMcbMsg.eStatus = MCB_ERROR;
                break;
            }

        } while ((tMcbMsg.eStatus != MCB_ERROR) && (tMcbMsg.eStatus != MCB_SUCCESS));

        switch (tMcbMsg.eStatus)
        {
            case MCB_SUCCESS:
                pRet = &ptInst->u16CyclicTx[ptInst->tCyclicRxList.u8Mapped];
                ptInst->tCyclicRxList.u16Addr[ptInst->tCyclicRxList.u8Mapped] = u16Addr;
                ptInst->tCyclicRxList.u16Sz[ptInst->tCyclicRxList.u8Mapped] = u16Sz;
                ptInst->tCyclicRxList.u8Mapped++;
                break;
            default:
                /** Nothing */
                break;
        }
    }

    return pRet;
}

uint8_t Mcb_TxUnmap(Mcb_TInst* ptInst)
{
    Mcb_TMsg tMcbMsg;

    /** Set up internal struct and verify a proper configuration */
    tMcbMsg.eStatus = MCB_STANDBY;
    tMcbMsg.u16Node = 2;
    tMcbMsg.u16Addr = TX_MAP_BASE + ptInst->tCyclicTxList.u8Mapped + 1;
    tMcbMsg.u16Cmd = MCB_REQ_WRITE;
    tMcbMsg.u16Size = sizeof(uint32_t);
    tMcbMsg.u16Data[0] = (uint16_t) 0U;
    tMcbMsg.u16Data[1] = (uint16_t) 0U;

    uint32_t u32Millis = Mcb_GetMillis();

    do
    {
        tMcbMsg.eStatus = Mcb_Write(ptInst, &tMcbMsg);

        if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
        {
            tMcbMsg.eStatus = MCB_ERROR;
            break;
        }

    } while ((tMcbMsg.eStatus != MCB_ERROR) && (tMcbMsg.eStatus != MCB_SUCCESS));

    switch (tMcbMsg.eStatus)
    {
        case MCB_SUCCESS:
            ptInst->tCyclicTxList.u16Addr[ptInst->tCyclicTxList.u8Mapped] = 0;
            ptInst->tCyclicTxList.u16Sz[ptInst->tCyclicTxList.u8Mapped] = 0;
            ptInst->tCyclicTxList.u8Mapped--;
            break;
        default:
            /** Nothing */
            break;
    }

    return ptInst->tCyclicTxList.u8Mapped;
}

uint8_t Mcb_RxUnmap(Mcb_TInst* ptInst)
{
    Mcb_TMsg tMcbMsg;

    /** Set up internal struct and verify a proper configuration */
    tMcbMsg.eStatus = MCB_STANDBY;
    tMcbMsg.u16Node = 2;
    tMcbMsg.u16Addr = RX_MAP_BASE + ptInst->tCyclicRxList.u8Mapped + 1;
    tMcbMsg.u16Cmd = MCB_REQ_WRITE;
    tMcbMsg.u16Size = sizeof(uint32_t);
    tMcbMsg.u16Data[0] = (uint16_t) 0U;
    tMcbMsg.u16Data[1] = (uint16_t) 0U;

    uint32_t u32Millis = Mcb_GetMillis();

    do
    {
        tMcbMsg.eStatus = Mcb_Write(ptInst, &tMcbMsg);

        if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
        {
            tMcbMsg.eStatus = MCB_ERROR;
            break;
        }

    } while ((tMcbMsg.eStatus != MCB_ERROR) && (tMcbMsg.eStatus != MCB_SUCCESS));

    switch (tMcbMsg.eStatus)
    {
        case MCB_SUCCESS:
            ptInst->tCyclicRxList.u16Addr[ptInst->tCyclicRxList.u8Mapped] = 0;
            ptInst->tCyclicRxList.u16Sz[ptInst->tCyclicRxList.u8Mapped] = 0;
            ptInst->tCyclicRxList.u8Mapped--;
            break;
        default:
            /** Nothing */
            break;
    }

    return ptInst->tCyclicRxList.u8Mapped;
}

void Mcb_UnmapAll(Mcb_TInst* ptInst)
{
    Mcb_TMsg tMcbMsg;

    /** Set up internal struct and verify a proper configuration */
    tMcbMsg.eStatus = MCB_STANDBY;
    tMcbMsg.u16Node = 2;
    tMcbMsg.u16Addr = RX_MAP_BASE;
    tMcbMsg.u16Cmd = MCB_REQ_WRITE;
    tMcbMsg.u16Size = sizeof(uint32_t);
    tMcbMsg.u16Data[0] = (uint16_t) 0U;

    uint32_t u32Millis = Mcb_GetMillis();

    do
    {
        tMcbMsg.eStatus = Mcb_Write(ptInst, &tMcbMsg);

        if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
        {
            tMcbMsg.eStatus = MCB_ERROR;
            break;
        }

    } while ((tMcbMsg.eStatus != MCB_ERROR) && (tMcbMsg.eStatus != MCB_SUCCESS));

    switch (tMcbMsg.eStatus)
    {
        case MCB_SUCCESS:
            ptInst->tCyclicRxList.u8Mapped = 0;
            break;
        default:
            /** Nothing */
            break;
    }

    /** Set up internal struct and verify a proper configuration */
    tMcbMsg.eStatus = MCB_STANDBY;
    tMcbMsg.u16Node = 2;
    tMcbMsg.u16Addr = TX_MAP_BASE;
    tMcbMsg.u16Cmd = MCB_REQ_WRITE;
    tMcbMsg.u16Size = sizeof(uint32_t);
    tMcbMsg.u16Data[0] = (uint16_t) 0U;

    u32Millis = Mcb_GetMillis();

    do
    {
        tMcbMsg.eStatus = Mcb_Write(ptInst, &tMcbMsg);

        if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
        {
            tMcbMsg.eStatus = MCB_ERROR;
            break;
        }

    } while ((tMcbMsg.eStatus != MCB_ERROR) && (tMcbMsg.eStatus != MCB_SUCCESS));

    switch (tMcbMsg.eStatus)
    {
        case MCB_SUCCESS:
            ptInst->tCyclicTxList.u8Mapped = 0;
            break;
        default:
            /** Nothing */
            break;
    }
}

int32_t Mcb_EnableCyclic(Mcb_TInst* ptInst)
{
    Mcb_TMsg tMcbMsg;
    uint16_t u16RxSz = 0;
    uint16_t u16TxSz = 0;
    int32_t i32Result = 0;

    if (ptInst->isCyclic == false)
    {
        /** Check and setup RX mapping */
        tMcbMsg.eStatus = MCB_STANDBY;
        tMcbMsg.u16Node = 2;
        tMcbMsg.u16Addr = RX_MAP_BASE;
        tMcbMsg.u16Cmd = MCB_REQ_WRITE;
        tMcbMsg.u16Size = sizeof(uint16_t);
        tMcbMsg.u16Data[0] = ptInst->tCyclicRxList.u8Mapped;

        uint32_t u32Millis = Mcb_GetMillis();

        do
        {
            tMcbMsg.eStatus = Mcb_Write(ptInst, &tMcbMsg);

            if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
            {
                tMcbMsg.eStatus = MCB_ERROR;
                break;
            }

        } while ((tMcbMsg.eStatus != MCB_ERROR) && (tMcbMsg.eStatus != MCB_SUCCESS));

        switch (tMcbMsg.eStatus)
        {
            case MCB_SUCCESS:
                for (uint16_t u16Idx = 0; u16Idx < ptInst->tCyclicRxList.u8Mapped; u16Idx++)
                {
                    u16RxSz += ptInst->tCyclicRxList.u16Sz[u16Idx];
                }
                break;
            default:
                i32Result = -1;
                break;
        }
        /** Check and setup TX mapping */
        tMcbMsg.eStatus = MCB_STANDBY;
        tMcbMsg.u16Node = 2;
        tMcbMsg.u16Addr = TX_MAP_BASE;
        tMcbMsg.u16Cmd = MCB_REQ_WRITE;
        tMcbMsg.u16Size = sizeof(uint16_t);
        tMcbMsg.u16Data[0] = ptInst->tCyclicTxList.u8Mapped;

        u32Millis = Mcb_GetMillis();

        do
        {
            tMcbMsg.eStatus = Mcb_Write(ptInst, &tMcbMsg);

            if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
            {
                tMcbMsg.eStatus = MCB_ERROR;
                break;
            }

        } while ((tMcbMsg.eStatus != MCB_ERROR) && (tMcbMsg.eStatus != MCB_SUCCESS));

        switch (tMcbMsg.eStatus)
        {
            case MCB_SUCCESS:
                for (uint16_t u16Idx = 0; u16Idx < ptInst->tCyclicTxList.u8Mapped; u16Idx++)
                {
                    u16TxSz += ptInst->tCyclicTxList.u16Sz[u16Idx];
                }
                break;
            default:
                i32Result = -2;
                break;
        }

        /** Enable cyclic mode */
        if ((u16RxSz > 0) || (u16TxSz > 0))
        {
            tMcbMsg.eStatus = MCB_STANDBY;
            tMcbMsg.u16Node = 2;
            tMcbMsg.u16Addr = CYCLIC_MODE;
            tMcbMsg.u16Cmd = MCB_REQ_WRITE;
            tMcbMsg.u16Size = sizeof(uint16_t);
            tMcbMsg.u16Data[0] = (uint16_t) 2U;

            uint32_t u32Millis = Mcb_GetMillis();

            do
            {
                tMcbMsg.eStatus = Mcb_Write(ptInst, &tMcbMsg);

                if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
                {
                    tMcbMsg.eStatus = MCB_ERROR;
                    break;
                }

            } while ((tMcbMsg.eStatus != MCB_ERROR) && (tMcbMsg.eStatus != MCB_SUCCESS));

            switch (tMcbMsg.eStatus)
            {
                case MCB_SUCCESS:
                    /* Nothing*/
                    break;
                default:
                    i32Result = -3;
                    break;
            }
        }

        /** Check bigger mapping and set up generated frame size */
        if (i32Result == 0)
        {
            /** Convert bytes into words */
            if (u16RxSz > u16TxSz)
            {
                ptInst->u16CyclicSize = u16RxSz >> 1;
            }
            else
            {
                ptInst->u16CyclicSize = u16TxSz >> 1;
            }

            ptInst->isCyclic = true;
        }

        i32Result = ptInst->u16CyclicSize;
    }

    return i32Result;
}

int32_t Mcb_DisableCyclic(Mcb_TInst* ptInst)
{
    Mcb_TMsg tMcbMsg;

    if (ptInst->isCyclic != false)
    {
        tMcbMsg.eStatus = MCB_STANDBY;
        tMcbMsg.u16Node = 2;
        tMcbMsg.u16Addr = CYCLIC_MODE;
        tMcbMsg.u16Cmd = MCB_REQ_WRITE;
        tMcbMsg.u16Size = sizeof(uint16_t);
        tMcbMsg.u16Data[0] = (uint16_t) 1U;

        /** Cyclic will be disabled through cyclic messages */
        tMcbMsg.eStatus = Mcb_Write(ptInst, &tMcbMsg);
    }

    return 0;
}

bool Mcb_CyclicProcess(Mcb_TInst* ptInst)
{
    bool isDataTransmitted = false;

    if (ptInst->isCyclic != false)
    {
        Mcb_EStatus eResult = Mcb_IntfCyclicTranfer(&ptInst->tIntf, ptInst->tConfig.u16Node, ptInst->tConfig.u16Addr,
                &ptInst->tConfig.u16Cmd, ptInst->tConfig.u16Data, &ptInst->tConfig.u16Size, ptInst->u16CyclicTx,
                ptInst->u16CyclicRx, ptInst->u16CyclicSize);

        if (eResult != MCB_STANDBY)
        {
            isDataTransmitted = true;

            if ((eResult == MCB_CYCLIC_SUCCESS) || (eResult == MCB_CYCLIC_ERROR))
            {
                if (ptInst->CfgOverCyclicEvnt != NULL)
                {
                    ptInst->CfgOverCyclicEvnt(ptInst, &ptInst->tConfig);
                }
            }
        }

    }

    return isDataTransmitted;
}

