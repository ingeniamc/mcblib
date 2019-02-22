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

#define ADDR_COMM_STATE     (uint16_t)0x640
#define ADDR_CYCLIC_MODE    (uint16_t)0x641
#define RX_MAP_BASE         (uint16_t)0x650
#define TX_MAP_BASE         (uint16_t)0x660

#define WORDSIZE_16BIT      1
#define WORDSIZE_32BIT      2

Mcb_EStatus Mcb_Init(Mcb_TInst* ptInst, Mcb_EMode eMode, uint16_t u16Id, bool bCalcCrc, uint32_t u32Timeout)
{
    Mcb_EStatus eRet = MCB_SUCCESS;
    ptInst->isCyclic = false;
    ptInst->isCyclic2Cfg = false;
    ptInst->eMode = eMode;
    ptInst->u32Timeout = u32Timeout;
    ptInst->eSyncMode = MCB_CYC_NON_SYNC;

    ptInst->CfgOverCyclicEvnt = NULL;

    ptInst->tCyclicRxList.u8Mapped = 0;
    ptInst->tCyclicTxList.u8Mapped = 0;
    ptInst->tCyclicRxList.u16MappedSize = 0;
    ptInst->tCyclicTxList.u16MappedSize = 0;

    for (uint8_t u8Idx = 0; u8Idx < MAX_MAPPED_REG; u8Idx++)
    {
        ptInst->tCyclicRxList.u16Addr[u8Idx] = 0;
        ptInst->tCyclicRxList.u16Sz[u8Idx] = 0;
        ptInst->tCyclicTxList.u16Addr[u8Idx] = 0;
        ptInst->tCyclicTxList.u16Sz[u8Idx] = 0;
    }

    ptInst->tIntf.u16Id = u16Id;
    ptInst->tIntf.bCalcCrc = bCalcCrc;
    Mcb_IntfInit(&ptInst->tIntf);

    if (Mcb_IntfReadIRQ(u16Id) == (uint8_t)0)
    {
        eRet = MCB_ERROR;
    }

    return eRet;
}

void Mcb_Deinit(Mcb_TInst* ptInst)
{    
    ptInst->isCyclic = false;    
    ptInst->eMode = MCB_BLOCKING;
    Mcb_IntfDeinit(&ptInst->tIntf);

    ptInst->tCyclicRxList.u8Mapped = 0;
    ptInst->tCyclicTxList.u8Mapped = 0;
    ptInst->tCyclicRxList.u16MappedSize = 0;
    ptInst->tCyclicTxList.u16MappedSize = 0;

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
                    Mcb_IntfReset(&ptInst->tIntf);
                    break;
                }

            } while ((pMcbMsg->eStatus != MCB_ERROR) && (pMcbMsg->eStatus != MCB_WRITE_ERROR) && (pMcbMsg->eStatus != MCB_SUCCESS));
        }
        else
        {
            /** Non blocking mode */
            pMcbMsg->eStatus = Mcb_IntfWrite(&ptInst->tIntf, pMcbMsg->u16Node, pMcbMsg->u16Addr, &pMcbMsg->u16Data[0],
                    &u16sz);
        }

        if (pMcbMsg->eStatus != MCB_SUCCESS)
        {
            pMcbMsg->u16Cmd |= MCB_REP_ERROR;
        }
        else
        {
            pMcbMsg->u16Cmd = MCB_REP_ACK;
        }
    }
    else
    {
        /* Cyclic mode */
        pMcbMsg->u16Cmd = MCB_REQ_WRITE;
        memcpy(&ptInst->tConfig, pMcbMsg, sizeof(Mcb_TMsg));
        pMcbMsg->eStatus = MCB_STANDBY;
        ptInst->tIntf.isNewCfgOverCyclic = true;
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
                    Mcb_IntfReset(&ptInst->tIntf);
                    break;
                }

            } while ((pMcbMsg->eStatus != MCB_ERROR) && (pMcbMsg->eStatus != MCB_READ_ERROR) && (pMcbMsg->eStatus != MCB_SUCCESS));

            pMcbMsg->u16Size = ptInst->tIntf.u16Sz;
        }
        else
        {
            /** Non blocking mode */
            pMcbMsg->eStatus = Mcb_IntfRead(&ptInst->tIntf, pMcbMsg->u16Node, pMcbMsg->u16Addr, &pMcbMsg->u16Data[0],
                    &pMcbMsg->u16Size);
        }

        if (pMcbMsg->eStatus == MCB_READ_ERROR)
        {
            pMcbMsg->u16Cmd = MCB_REP_READ_ERROR;
        }
    }
    else
    {
        /* Cyclic mode */
        pMcbMsg->u16Cmd = MCB_REQ_READ;
        memcpy(&ptInst->tConfig, pMcbMsg, sizeof(Mcb_TMsg));
        pMcbMsg->eStatus = MCB_STANDBY;
        ptInst->tIntf.isNewCfgOverCyclic = true;
    }

    return pMcbMsg->eStatus;
}

void Mcb_AttachCfgOverCyclicCB(Mcb_TInst* ptInst, void (*Evnt)(Mcb_TInst* ptInst, Mcb_TMsg* pMcbMsg))
{
    ptInst->CfgOverCyclicEvnt = Evnt;
}

void* Mcb_TxMap(Mcb_TInst* ptInst, uint16_t u16Addr, uint16_t u16Sz)
{
    Mcb_TMsg tMcbMsg;
    void* pRet = NULL;

    /** Set up internal struct and verify a proper configuration */
    if (ptInst->tCyclicTxList.u8Mapped < MAX_MAPPED_REG)
    {
        tMcbMsg.eStatus = MCB_STANDBY;
        tMcbMsg.u16Node = DEFAULT_MOCO_NODE;
        tMcbMsg.u16Addr = TX_MAP_BASE + ptInst->tCyclicTxList.u8Mapped + 1;
        tMcbMsg.u16Cmd = MCB_REQ_WRITE;
        tMcbMsg.u16Size = WORDSIZE_32BIT;
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
                pRet = &ptInst->u16CyclicRx[ptInst->tCyclicTxList.u16MappedSize];
                ptInst->tCyclicTxList.u16Addr[ptInst->tCyclicTxList.u8Mapped] = u16Addr;
                ptInst->tCyclicTxList.u16Sz[ptInst->tCyclicTxList.u8Mapped] = u16Sz;
                ptInst->tCyclicTxList.u8Mapped++;
                /** Ensure correct conversion from bytes to words */
                ptInst->tCyclicTxList.u16MappedSize += ((u16Sz + (u16Sz & 1)) >> 1);
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
        tMcbMsg.u16Node = DEFAULT_MOCO_NODE;
        tMcbMsg.u16Addr = RX_MAP_BASE + ptInst->tCyclicRxList.u8Mapped + 1;
        tMcbMsg.u16Cmd = MCB_REQ_WRITE;
        tMcbMsg.u16Size = WORDSIZE_32BIT;
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
                pRet = &ptInst->u16CyclicTx[ptInst->tCyclicRxList.u16MappedSize];
                ptInst->tCyclicRxList.u16Addr[ptInst->tCyclicRxList.u8Mapped] = u16Addr;
                ptInst->tCyclicRxList.u16Sz[ptInst->tCyclicRxList.u8Mapped] = u16Sz;
                ptInst->tCyclicRxList.u8Mapped++;
                /** Ensure correct conversion from bytes to words */
                ptInst->tCyclicRxList.u16MappedSize += ((u16Sz + (u16Sz & 1)) >> 1);
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
    uint16_t u16SizeBytes;

    /** Set up internal struct and verify a proper configuration */
    tMcbMsg.eStatus = MCB_STANDBY;
    tMcbMsg.u16Node = DEFAULT_MOCO_NODE;
    tMcbMsg.u16Addr = TX_MAP_BASE + ptInst->tCyclicTxList.u8Mapped + 1;
    tMcbMsg.u16Cmd = MCB_REQ_WRITE;
    tMcbMsg.u16Size = WORDSIZE_32BIT;
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
            /* Ensure correct conversion from bytes to words */
            u16SizeBytes = ptInst->tCyclicTxList.u16Sz[ptInst->tCyclicTxList.u8Mapped];
            ptInst->tCyclicTxList.u16MappedSize -= ((u16SizeBytes + (u16SizeBytes & 1)) >> 1);
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
    uint16_t u16SizeBytes;

    /** Set up internal struct and verify a proper configuration */
    tMcbMsg.eStatus = MCB_STANDBY;
    tMcbMsg.u16Node = DEFAULT_MOCO_NODE;
    tMcbMsg.u16Addr = RX_MAP_BASE + ptInst->tCyclicRxList.u8Mapped + 1;
    tMcbMsg.u16Cmd = MCB_REQ_WRITE;
    tMcbMsg.u16Size = WORDSIZE_32BIT;
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
            /* Ensure correct conversion from bytes to words */
            u16SizeBytes = ptInst->tCyclicRxList.u16Sz[ptInst->tCyclicRxList.u8Mapped];
            ptInst->tCyclicRxList.u16MappedSize -= ((u16SizeBytes + (u16SizeBytes & 1)) >> 1);
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
    tMcbMsg.u16Node = DEFAULT_MOCO_NODE;
    tMcbMsg.u16Addr = RX_MAP_BASE;
    tMcbMsg.u16Cmd = MCB_REQ_WRITE;
    tMcbMsg.u16Size = WORDSIZE_16BIT;
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
            ptInst->tCyclicRxList.u16MappedSize = 0;
            break;
        default:
            /** Nothing */
            break;
    }

    /** Set up internal struct and verify a proper configuration */
    tMcbMsg.eStatus = MCB_STANDBY;
    tMcbMsg.u16Node = DEFAULT_MOCO_NODE;
    tMcbMsg.u16Addr = TX_MAP_BASE;
    tMcbMsg.u16Cmd = MCB_REQ_WRITE;
    tMcbMsg.u16Size = WORDSIZE_16BIT;
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
            ptInst->tCyclicTxList.u16MappedSize = 0;
            break;
        default:
            /** Nothing */
            break;
    }
}

int32_t Mcb_EnableCyclic(Mcb_TInst* ptInst)
{
    Mcb_TMsg tMcbMsg;
    int32_t i32Result = CYCLIC_MODE_OK;

    if (ptInst->isCyclic == false)
    {
        tMcbMsg.eStatus = MCB_STANDBY;
        tMcbMsg.u16Node = DEFAULT_MOCO_NODE;
        tMcbMsg.u16Addr = ADDR_CYCLIC_MODE;
        tMcbMsg.u16Cmd = MCB_REQ_WRITE;
        tMcbMsg.u16Size = WORDSIZE_16BIT;
        tMcbMsg.u16Data[0] = (uint16_t)ptInst->eSyncMode;

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
                /** Do nothing */
                break;
            default:
                i32Result = CYCLIC_ERR_SYNC;
                break;
        }

        /** Check and setup RX mapping */
        tMcbMsg.eStatus = MCB_STANDBY;
        tMcbMsg.u16Node = DEFAULT_MOCO_NODE;
        tMcbMsg.u16Addr = RX_MAP_BASE;
        tMcbMsg.u16Cmd = MCB_REQ_WRITE;
        tMcbMsg.u16Size = WORDSIZE_16BIT;
        tMcbMsg.u16Data[0] = ptInst->tCyclicRxList.u8Mapped;

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
                /** Do nothing */
                break;
            default:
                i32Result = CYCLIC_ERR_RX_MAP;
                break;
        }

        if (i32Result == 0)
        {
            /** If RX mapping was OK, check and setup TX mapping */
            tMcbMsg.eStatus = MCB_STANDBY;
            tMcbMsg.u16Node = DEFAULT_MOCO_NODE;
            tMcbMsg.u16Addr = TX_MAP_BASE;
            tMcbMsg.u16Cmd = MCB_REQ_WRITE;
            tMcbMsg.u16Size = WORDSIZE_16BIT;
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
                    /** Do nothing */
                    break;
                default:
                    i32Result = CYCLIC_ERR_TX_MAP;
                    break;
            }
        }

        if (i32Result == 0)
        {
            /** If both mappings are OK, enable cyclic mode */
            tMcbMsg.eStatus = MCB_STANDBY;
            tMcbMsg.u16Node = DEFAULT_MOCO_NODE;
            tMcbMsg.u16Addr = ADDR_COMM_STATE;
            tMcbMsg.u16Cmd = MCB_REQ_WRITE;
            tMcbMsg.u16Size = WORDSIZE_16BIT;
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
                    /** Do nothing*/
                    break;
                default:
                    i32Result = CYCLIC_ERR_VALIDATION;
                    break;
            }
        }

        /** If cyclic mode is correctly enabled */
        if (i32Result == CYCLIC_MODE_OK)
        {
            /** Check bigger mapping and set up generated frame size */
            if (ptInst->tCyclicRxList.u16MappedSize > ptInst->tCyclicTxList.u16MappedSize)
            {
                ptInst->u16CyclicSize = ptInst->tCyclicRxList.u16MappedSize;
            }
            else
            {
                ptInst->u16CyclicSize = ptInst->tCyclicTxList.u16MappedSize;
            }

            ptInst->isCyclic = true;
            i32Result = ptInst->u16CyclicSize;
        }
    }

    return i32Result;
}

Mcb_EStatus  Mcb_DisableCyclic(Mcb_TInst* ptInst)
{
    Mcb_TMsg tMcbMsg;
    Mcb_EStatus  eRes = MCB_CYCLIC_SUCCESS;

    if (ptInst->isCyclic != false)
    {
        if ((ptInst->tIntf.eState == MCB_STANDBY) || (ptInst->tIntf.eState == MCB_SUCCESS) ||
            (ptInst->tIntf.eState == MCB_WRITE_ERROR) || (ptInst->tIntf.eState == MCB_READ_ERROR) ||
            (ptInst->tIntf.eState == MCB_ERROR))
        {
            tMcbMsg.eStatus = MCB_STANDBY;
            tMcbMsg.u16Node = DEFAULT_MOCO_NODE;
            tMcbMsg.u16Addr = ADDR_COMM_STATE;
            tMcbMsg.u16Cmd = MCB_REQ_WRITE;
            tMcbMsg.u16Size = WORDSIZE_16BIT;
            tMcbMsg.u16Data[0] = (uint16_t) 1U;

            /** Cyclic will be disabled through cyclic messages */
            tMcbMsg.eStatus = Mcb_Write(ptInst, &tMcbMsg);

            ptInst->isCyclic2Cfg = true;
        }
        else
        {
            eRes = MCB_CYCLIC_ERROR;
        }
    }

    return eRes;
}

void Mcb_SetCyclicMode(Mcb_TInst* ptInst, Mcb_ECyclicMode eNewCycMode)
{
    ptInst->eSyncMode = eNewCycMode;
}

Mcb_EStatus Mcb_CyclicProcess(Mcb_TInst* ptInst)
{
    Mcb_EStatus eState = MCB_STANDBY;
    bool isCfgData;

    if ((ptInst->isCyclic != false) && (Mcb_IntfIsReady(ptInst->tIntf.u16Id) != false) && (Mcb_IntfTryLockSem(SEMAPHORE_IRQ_RESOURCE) != false))
    {
        eState = MCB_CYCLIC_REQUEST;

        Mcb_EStatus eResult = Mcb_IntfCfgOverCyclic(&ptInst->tIntf, ptInst->tConfig.u16Node, ptInst->tConfig.u16Addr,
                                                    &ptInst->tConfig.u16Cmd, ptInst->tConfig.u16Data, &ptInst->tConfig.u16Size,
                                                    &isCfgData);

        if ((eResult == MCB_CYCLIC_SUCCESS) || (eResult == MCB_CYCLIC_ERROR))
        {
            if (ptInst->isCyclic2Cfg == false)
            {
                if (ptInst->CfgOverCyclicEvnt != NULL)
                {
                    ptInst->CfgOverCyclicEvnt(ptInst, &ptInst->tConfig);
                }
            }
            else
            {
                eState = eResult;
                ptInst->isCyclic2Cfg = false;
                ptInst->isCyclic = false;
            }
        }

        if (eState == MCB_CYCLIC_REQUEST)
        {
            Mcb_IntfCyclic(&ptInst->tIntf, ptInst->u16CyclicTx, ptInst->u16CyclicRx, ptInst->u16CyclicSize, isCfgData);
        }
        else
        {
            Mcb_IntfUnlockSem(SEMAPHORE_IRQ_RESOURCE);
        }
    }

    return eState;
}

