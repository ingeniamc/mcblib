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

/**
 * Handles a MCB config request received in cyclic mode.
 *
 * @param[in] ptInst
 *  Specifies the target MCB instance.
 * @param[in] pMcbMsg
 *  Pointer to the received config request.
 */
static void
Mcb_ConfigOverCyclicCompl(Mcb_TInst* ptInst, Mcb_TMsg* pMcbMsg);

/**
 * Generic blocking getinfo function
 *
 * @param[in] ptInst
 *  Specifies the target instance
 * @param[in,out] pMcbInfoMsg
 *  Request to be send and load with reply
 */
static void
Mcb_BlockingGetInfo(Mcb_TInst* ptInst, Mcb_TInfoMsg* pMcbInfoMsg);

/**
 * Generic blocking read function
 *
 * @param[in] ptInst
 *  Specifies the target instance
 * @param[in,out] pMcbMsg
 *  Request to be send and load with reply
 */
static void
Mcb_BlockingRead(Mcb_TInst* ptInst, Mcb_TMsg* pMcbMsg);

/**
 * Generic blocking write function
 *
 * @param[in] ptInst
 *  Specifies the target instance
 * @param[in,out] pMcbMsg
 *  Request to be send and load with reply
 */
static void
Mcb_BlockingWrite(Mcb_TInst* ptInst, Mcb_TMsg* pMcbMsg);

/**
 * Generic non blocking getinfo function
 *
 * @param[in] ptInst
 *  Specifies the target instance
 * @param[in,out] pMcbInfoMsg
 *  Request to be send and load with reply
 */
static void
Mcb_NonBlockingGetInfo(Mcb_TInst* ptInst, Mcb_TInfoMsg* pMcbInfoMsg);

/**
 * Generic non blocking read function
 *
 * @param[in] ptInst
 *  Specifies the target instance
 * @param[in,out] pMcbMsg
 *  Request to be send and load with reply
 */
static void
Mcb_NonBlockingRead(Mcb_TInst* ptInst, Mcb_TMsg* pMcbMsg);

/**
 * Generic non blocking write function
 *
 * @param[in] ptInst
 *  Specifies the target instance
 * @param[in,out] pMcbMsg
 *  Request to be send and load with reply
 */
static void
Mcb_NonBlockingWrite(Mcb_TInst* ptInst, Mcb_TMsg* pMcbMsg);


int32_t Mcb_Init(Mcb_TInst* ptInst, Mcb_EMode eMode, uint16_t u16Id, bool bCalcCrc, uint32_t u32Timeout)
{
    int32_t i32Ret = MCB_INIT_OK;

    ptInst->isCyclic = false;
    ptInst->eMode = eMode;
    if (eMode == MCB_BLOCKING)
    {
        ptInst->Mcb_GetInfo = Mcb_BlockingGetInfo;
        ptInst->Mcb_Read = Mcb_BlockingRead;
        ptInst->Mcb_Write = Mcb_BlockingWrite;
        ptInst->CfgOverCyclicEvnt = Mcb_ConfigOverCyclicCompl;
    }
    else
    {
        ptInst->Mcb_GetInfo = Mcb_NonBlockingGetInfo;
        ptInst->Mcb_Read = Mcb_NonBlockingRead;
        ptInst->Mcb_Write = Mcb_NonBlockingWrite;
        ptInst->CfgOverCyclicEvnt = NULL;
    }
    ptInst->u32Timeout = u32Timeout;
    ptInst->eSyncMode = MCB_CYC_NON_SYNC;

    ptInst->tCyclicRxList.u8Mapped = (uint8_t)0;
    ptInst->tCyclicTxList.u8Mapped = (uint8_t)0;
    ptInst->tCyclicRxList.u16MappedSize = (uint16_t)0U;
    ptInst->tCyclicTxList.u16MappedSize = (uint16_t)0U;

    for (uint8_t u8Idx = (uint8_t)0; u8Idx < MAX_MAPPED_REG; u8Idx++)
    {
        ptInst->tCyclicRxList.u16Addr[u8Idx] = (uint16_t)0U;
        ptInst->tCyclicRxList.u16Sz[u8Idx] = (uint16_t)0U;
        ptInst->tCyclicTxList.u16Addr[u8Idx] = (uint16_t)0U;
        ptInst->tCyclicTxList.u16Sz[u8Idx] = (uint16_t)0U;
    }

    ptInst->tIntf.u16Id = u16Id;
    ptInst->tIntf.bCalcCrc = bCalcCrc;
    Mcb_IntfInit(&ptInst->tIntf);

    if (Mcb_IntfReadIRQ(u16Id) == (uint8_t)0)
    {
        i32Ret = MCB_INIT_KO;
    }

    return i32Ret;
}

void Mcb_Deinit(Mcb_TInst* ptInst)
{    
    ptInst->isCyclic = false;    
    ptInst->eMode = MCB_BLOCKING;
    Mcb_IntfDeinit(&ptInst->tIntf);

    ptInst->Mcb_GetInfo = NULL;
    ptInst->Mcb_Read = NULL;
    ptInst->Mcb_Write = NULL;
    ptInst->CfgOverCyclicEvnt = NULL;

    ptInst->tCyclicRxList.u8Mapped = (uint8_t)0;
    ptInst->tCyclicTxList.u8Mapped = (uint8_t)0;
    ptInst->tCyclicRxList.u16MappedSize = (uint16_t)0U;
    ptInst->tCyclicTxList.u16MappedSize = (uint16_t)0U;

    for (uint8_t u8Idx = (uint8_t)0; u8Idx < MAX_MAPPED_REG; u8Idx++)
    {
        ptInst->tCyclicRxList.u16Addr[u8Idx] = (uint16_t)0U;
        ptInst->tCyclicRxList.u16Sz[u8Idx] = (uint16_t)0U;
        ptInst->tCyclicTxList.u16Addr[u8Idx] = (uint16_t)0U;
        ptInst->tCyclicTxList.u16Sz[u8Idx] = (uint16_t)0U;
    }
}

static void Mcb_BlockingGetInfo(Mcb_TInst* ptInst, Mcb_TInfoMsg* pMcbInfoMsg)
{
    uint32_t u32Millis = Mcb_GetMillis();
    pMcbInfoMsg->u16Cmd = MCB_REQ_GETINFO;

    if (ptInst->isCyclic == false)
    {
        do
        {
            pMcbInfoMsg->eStatus = Mcb_IntfGetInfo(&ptInst->tIntf, pMcbInfoMsg->u16Node, pMcbInfoMsg->u16Addr,
                                                   (uint16_t*)&pMcbInfoMsg->tInfoMsgData, &pMcbInfoMsg->u16Size);

            if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
            {
                pMcbInfoMsg->eStatus = MCB_GETINFO_ERROR;
                Mcb_IntfReset(&ptInst->tIntf);
                break;
            }
        } while ((pMcbInfoMsg->eStatus != MCB_GETINFO_ERROR)
                && (pMcbInfoMsg->eStatus != MCB_GETINFO_SUCCESS));
    }
    else
    {
        memcpy((void*)&ptInst->tConfigReq, (const void*)pMcbInfoMsg, sizeof(Mcb_TMsg));
        memcpy((void*)&ptInst->tConfigRpy, (const void*)pMcbInfoMsg, sizeof(Mcb_TMsg));
        ptInst->ptUsrConfig = (Mcb_TMsg*)pMcbInfoMsg;
        ptInst->tIntf.isNewCfgOverCyclic = true;

        do
        {
            if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
            {
                pMcbInfoMsg->eStatus = MCB_GETINFO_ERROR;
                Mcb_IntfReset(&ptInst->tIntf);
                break;
            }
        } while ((ptInst->tIntf.isNewCfgOverCyclic == true)
                || (ptInst->tIntf.isCfgOverCyclic == true));
    }

    if (pMcbInfoMsg->eStatus == MCB_GETINFO_ERROR)
    {
        pMcbInfoMsg->u16Cmd |= MCB_REP_ERROR;
    }
    else if (pMcbInfoMsg->eStatus == MCB_GETINFO_SUCCESS)
    {
        pMcbInfoMsg->u16Cmd = MCB_REP_ACK;
    }
}

static void Mcb_BlockingRead(Mcb_TInst* ptInst, Mcb_TMsg* pMcbMsg)
{
    uint32_t u32Millis = Mcb_GetMillis();
    pMcbMsg->u16Cmd = MCB_REQ_READ;

    if (ptInst->isCyclic == false)
    {
        do
        {
            pMcbMsg->eStatus = Mcb_IntfRead(&ptInst->tIntf, pMcbMsg->u16Node, pMcbMsg->u16Addr,
                                            &pMcbMsg->u16Data[0], &pMcbMsg->u16Size);

            if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
            {
                pMcbMsg->eStatus = MCB_READ_ERROR;
                Mcb_IntfReset(&ptInst->tIntf);
                break;
            }
        } while((pMcbMsg->eStatus != MCB_READ_ERROR)
                && (pMcbMsg->eStatus != MCB_READ_SUCCESS));
    }
    else
    {
        memcpy((void*)&ptInst->tConfigReq, (const void*)pMcbMsg, sizeof(Mcb_TMsg));
        memcpy((void*)&ptInst->tConfigRpy, (const void*)pMcbMsg, sizeof(Mcb_TMsg));
        ptInst->ptUsrConfig = pMcbMsg;
        ptInst->tIntf.isNewCfgOverCyclic = true;

        do
        {
            if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
            {
                pMcbMsg->eStatus = MCB_READ_ERROR;
                Mcb_IntfReset(&ptInst->tIntf);
                break;
            }
        }while ((ptInst->tIntf.isNewCfgOverCyclic == true)
                || (ptInst->tIntf.isCfgOverCyclic == true));
    }

    if (pMcbMsg->eStatus == MCB_READ_ERROR)
    {
        pMcbMsg->u16Cmd |= MCB_REP_ERROR;
    }
    else if (pMcbMsg->eStatus == MCB_READ_SUCCESS)
    {
        pMcbMsg->u16Cmd = MCB_REP_ACK;
    }
}

static void Mcb_BlockingWrite(Mcb_TInst* ptInst, Mcb_TMsg* pMcbMsg)
{
    uint32_t u32Millis = Mcb_GetMillis();
    pMcbMsg->u16Cmd = MCB_REQ_WRITE;

    if (ptInst->isCyclic == false)
    {
        do
        {
            pMcbMsg->eStatus = Mcb_IntfWrite(&ptInst->tIntf, pMcbMsg->u16Node, pMcbMsg->u16Addr,
                                             &pMcbMsg->u16Data[0], &pMcbMsg->u16Size);

            if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
            {
                pMcbMsg->eStatus = MCB_WRITE_ERROR;
                Mcb_IntfReset(&ptInst->tIntf);
                break;
            }
        }while ((pMcbMsg->eStatus != MCB_WRITE_ERROR)
                && (pMcbMsg->eStatus != MCB_WRITE_SUCCESS));
    }
    else
    {
        memcpy((void*)&ptInst->tConfigReq, (const void*)pMcbMsg, sizeof(Mcb_TMsg));
        memcpy((void*)&ptInst->tConfigRpy, (const void*)pMcbMsg, sizeof(Mcb_TMsg));
        ptInst->ptUsrConfig = pMcbMsg;
        ptInst->tIntf.isNewCfgOverCyclic = true;

        do
        {
            if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
            {
                pMcbMsg->eStatus = MCB_WRITE_ERROR;
                Mcb_IntfReset(&ptInst->tIntf);
                break;
            }
        } while((ptInst->tIntf.isNewCfgOverCyclic == true)
                || (ptInst->tIntf.isCfgOverCyclic == true));
    }

    if (pMcbMsg->eStatus == MCB_WRITE_ERROR)
    {
        pMcbMsg->u16Cmd |= MCB_REP_ERROR;
    }
    else if (pMcbMsg->eStatus == MCB_WRITE_SUCCESS)
    {
        pMcbMsg->u16Cmd = MCB_REP_ACK;
    }

}

static void Mcb_NonBlockingGetInfo(Mcb_TInst* ptInst, Mcb_TInfoMsg* pMcbInfoMsg)
{
    pMcbInfoMsg->u16Cmd = MCB_REQ_GETINFO;

    if (ptInst->isCyclic == false)
    {
        pMcbInfoMsg->eStatus = Mcb_IntfGetInfo(&ptInst->tIntf, pMcbInfoMsg->u16Node, pMcbInfoMsg->u16Addr,
                                               (uint16_t*)&pMcbInfoMsg->tInfoMsgData, &pMcbInfoMsg->u16Size);
    }
    else
    {
        pMcbInfoMsg->eStatus = MCB_STANDBY;
        memcpy((void*)&ptInst->tConfigReq, (const void*)pMcbInfoMsg, sizeof(Mcb_TMsg));
        memcpy((void*)&ptInst->tConfigRpy, (const void*)pMcbInfoMsg, sizeof(Mcb_TMsg));
        ptInst->tIntf.isNewCfgOverCyclic = true;
    }

    if (pMcbInfoMsg->eStatus == MCB_GETINFO_ERROR)
    {
        pMcbInfoMsg->u16Cmd |= MCB_REP_ERROR;
    }
    else if (pMcbInfoMsg->eStatus == MCB_GETINFO_SUCCESS)
    {
        pMcbInfoMsg->u16Cmd = MCB_REP_ACK;
    }
}

static void Mcb_NonBlockingRead(Mcb_TInst* ptInst, Mcb_TMsg* pMcbMsg)
{
    pMcbMsg->u16Cmd = MCB_REQ_READ;

    if (ptInst->isCyclic == false)
    {
        pMcbMsg->eStatus = Mcb_IntfRead(&ptInst->tIntf, pMcbMsg->u16Node, pMcbMsg->u16Addr,
                                        &pMcbMsg->u16Data[0], &pMcbMsg->u16Size);
    }
    else
    {
        pMcbMsg->eStatus = MCB_STANDBY;
        memcpy((void*)&ptInst->tConfigReq, (const void*)pMcbMsg, sizeof(Mcb_TMsg));
        memcpy((void*)&ptInst->tConfigRpy, (const void*)pMcbMsg, sizeof(Mcb_TMsg));
        ptInst->tIntf.isNewCfgOverCyclic = true;
    }

    if (pMcbMsg->eStatus == MCB_READ_ERROR)
    {
        pMcbMsg->u16Cmd |= MCB_REP_ERROR;
    }
    else if (pMcbMsg->eStatus == MCB_READ_SUCCESS)
    {
        pMcbMsg->u16Cmd = MCB_REP_ACK;
    }
}

static void Mcb_NonBlockingWrite(Mcb_TInst* ptInst, Mcb_TMsg* pMcbMsg)
{
    pMcbMsg->u16Cmd = MCB_REQ_WRITE;

    if (ptInst->isCyclic == false)
    {
        pMcbMsg->eStatus = Mcb_IntfWrite(&ptInst->tIntf, pMcbMsg->u16Node, pMcbMsg->u16Addr,
                                         &pMcbMsg->u16Data[0], &pMcbMsg->u16Size);
    }
    else
    {
        pMcbMsg->eStatus = MCB_STANDBY;
        memcpy((void*)&ptInst->tConfigReq, (const void*)pMcbMsg, sizeof(Mcb_TMsg));
        memcpy((void*)&ptInst->tConfigRpy, (const void*)pMcbMsg, sizeof(Mcb_TMsg));
        ptInst->tIntf.isNewCfgOverCyclic = true;
    }

    if (pMcbMsg->eStatus == MCB_WRITE_ERROR)
    {
        pMcbMsg->u16Cmd |= MCB_REP_ERROR;
    }
    else if (pMcbMsg->eStatus == MCB_WRITE_SUCCESS)
    {
        pMcbMsg->u16Cmd = MCB_REP_ACK;
    }
}

void Mcb_AttachCfgOverCyclicCB(Mcb_TInst* ptInst, void (*Evnt)(Mcb_TInst* ptInst, Mcb_TMsg* pMcbMsg))
{
    if (ptInst->eMode != MCB_BLOCKING)
    {
        ptInst->CfgOverCyclicEvnt = Evnt;
    }
}

void* Mcb_TxMap(Mcb_TInst* ptInst, uint16_t u16Addr, uint16_t u16Sz)
{
    Mcb_TMsg tMcbMsg;
    void* pRet = NULL;

    do
    {
        /** Check if the register is already mapped into mcb */
        uint8_t *pu8ByOffset = (uint8_t*)ptInst->u16CyclicRx;
        for (uint8_t u8TxMapCnt = (uint8_t)0; u8TxMapCnt < ptInst->tCyclicTxList.u8Mapped; ++u8TxMapCnt)
        {
            if (ptInst->tCyclicTxList.u16Addr[u8TxMapCnt] == u16Addr)
            {
                pRet = pu8ByOffset;
                break;
            }
            pu8ByOffset += ptInst->tCyclicTxList.u16Sz[u8TxMapCnt];
        }

        /** Set up internal struct and verify a proper configuration */
        if ((pRet != NULL) || (ptInst->tCyclicTxList.u8Mapped >= MAX_MAPPED_REG))
        {
            break;
        }

        tMcbMsg.u16Node = DEFAULT_MOCO_NODE;
        tMcbMsg.u16Addr = TX_MAP_BASE + ptInst->tCyclicTxList.u8Mapped + (uint16_t)1U;
        tMcbMsg.u16Cmd = MCB_REQ_WRITE;
        tMcbMsg.u16Size = WORDSIZE_32BIT;
        tMcbMsg.u16Data[0] = u16Addr;
        tMcbMsg.u16Data[1] = u16Sz;

        uint32_t u32Millis = Mcb_GetMillis();

        do
        {
            ptInst->Mcb_Write(ptInst, &tMcbMsg);

            if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
            {
                tMcbMsg.eStatus = MCB_WRITE_ERROR;
                break;
            }

        } while ((tMcbMsg.eStatus != MCB_WRITE_ERROR)
                 && (tMcbMsg.eStatus != MCB_WRITE_SUCCESS));

        switch (tMcbMsg.eStatus)
        {
            case MCB_WRITE_SUCCESS:
                pRet = &ptInst->u16CyclicRx[ptInst->tCyclicTxList.u16MappedSize];
                ptInst->tCyclicTxList.u16Addr[ptInst->tCyclicTxList.u8Mapped] = u16Addr;
                ptInst->tCyclicTxList.u16Sz[ptInst->tCyclicTxList.u8Mapped] = u16Sz;
                ptInst->tCyclicTxList.u8Mapped++;
                /** Ensure correct conversion from bytes to words */
                ptInst->tCyclicTxList.u16MappedSize += ((u16Sz + (u16Sz & (uint16_t)1U)) >> (uint16_t)1U);
                break;
            default:
                /** Nothing */
                break;
        }
    } while (false);

    return pRet;
}

void* Mcb_RxMap(Mcb_TInst* ptInst, uint16_t u16Addr, uint16_t u16Sz)
{
    Mcb_TMsg tMcbMsg;
    void* pRet = NULL;

    /** Set up internal struct and verify a proper configuration */
    if (ptInst->tCyclicRxList.u8Mapped < MAX_MAPPED_REG)
    {
        tMcbMsg.u16Node = DEFAULT_MOCO_NODE;
        tMcbMsg.u16Addr = RX_MAP_BASE + ptInst->tCyclicRxList.u8Mapped + (uint16_t)1U;
        tMcbMsg.u16Cmd = MCB_REQ_WRITE;
        tMcbMsg.u16Size = WORDSIZE_32BIT;
        tMcbMsg.u16Data[0] = u16Addr;
        tMcbMsg.u16Data[1] = u16Sz;

        uint32_t u32Millis = Mcb_GetMillis();

        do
        {
            ptInst->Mcb_Write(ptInst, &tMcbMsg);

            if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
            {
                tMcbMsg.eStatus = MCB_WRITE_ERROR;
                break;
            }

        } while ((tMcbMsg.eStatus != MCB_WRITE_ERROR)
                 && (tMcbMsg.eStatus != MCB_WRITE_SUCCESS));

        switch (tMcbMsg.eStatus)
        {
            case MCB_WRITE_SUCCESS:
                pRet = &ptInst->u16CyclicTx[ptInst->tCyclicRxList.u16MappedSize];
                ptInst->tCyclicRxList.u16Addr[ptInst->tCyclicRxList.u8Mapped] = u16Addr;
                ptInst->tCyclicRxList.u16Sz[ptInst->tCyclicRxList.u8Mapped] = u16Sz;
                ptInst->tCyclicRxList.u8Mapped++;
                /** Ensure correct conversion from bytes to words */
                ptInst->tCyclicRxList.u16MappedSize += ((u16Sz + (u16Sz & (uint16_t)1U)) >> (uint16_t)1U);
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
    tMcbMsg.u16Node = DEFAULT_MOCO_NODE;
    tMcbMsg.u16Addr = TX_MAP_BASE + ptInst->tCyclicTxList.u8Mapped + (uint16_t)1U;
    tMcbMsg.u16Cmd = MCB_REQ_WRITE;
    tMcbMsg.u16Size = WORDSIZE_32BIT;
    tMcbMsg.u16Data[0] = (uint16_t)0U;
    tMcbMsg.u16Data[1] = (uint16_t)0U;

    uint32_t u32Millis = Mcb_GetMillis();

    do
    {
        ptInst->Mcb_Write(ptInst, &tMcbMsg);

        if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
        {
            tMcbMsg.eStatus = MCB_WRITE_ERROR;
            break;
        }

    } while ((tMcbMsg.eStatus != MCB_WRITE_ERROR)
             && (tMcbMsg.eStatus != MCB_WRITE_SUCCESS));

    switch (tMcbMsg.eStatus)
    {
        case MCB_WRITE_SUCCESS:
            /* Ensure correct conversion from bytes to words */
            u16SizeBytes = ptInst->tCyclicTxList.u16Sz[ptInst->tCyclicTxList.u8Mapped];
            ptInst->tCyclicTxList.u16MappedSize -= ((u16SizeBytes + (u16SizeBytes & (uint16_t)1U)) >> (uint16_t)1U);
            ptInst->tCyclicTxList.u16Addr[ptInst->tCyclicTxList.u8Mapped] = (uint16_t)0U;
            ptInst->tCyclicTxList.u16Sz[ptInst->tCyclicTxList.u8Mapped] = (uint16_t)0U;
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
    tMcbMsg.u16Node = DEFAULT_MOCO_NODE;
    tMcbMsg.u16Addr = RX_MAP_BASE + ptInst->tCyclicRxList.u8Mapped + 1;
    tMcbMsg.u16Size = WORDSIZE_32BIT;
    tMcbMsg.u16Data[0] = (uint16_t)0U;
    tMcbMsg.u16Data[1] = (uint16_t)0U;

    uint32_t u32Millis = Mcb_GetMillis();

    do
    {
        ptInst->Mcb_Write(ptInst, &tMcbMsg);

        if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
        {
            tMcbMsg.eStatus = MCB_WRITE_ERROR;
            break;
        }

    } while ((tMcbMsg.eStatus != MCB_WRITE_ERROR)
             && (tMcbMsg.eStatus != MCB_WRITE_SUCCESS));

    switch (tMcbMsg.eStatus)
    {
        case MCB_WRITE_SUCCESS:
            /* Ensure correct conversion from bytes to words */
            u16SizeBytes = ptInst->tCyclicRxList.u16Sz[ptInst->tCyclicRxList.u8Mapped];
            ptInst->tCyclicRxList.u16MappedSize -= ((u16SizeBytes + (u16SizeBytes & (uint16_t)1U)) >> (uint16_t)1U);
            ptInst->tCyclicRxList.u16Addr[ptInst->tCyclicRxList.u8Mapped] = (uint16_t)0U;
            ptInst->tCyclicRxList.u16Sz[ptInst->tCyclicRxList.u8Mapped] = (uint16_t)0U;
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
    tMcbMsg.u16Node = DEFAULT_MOCO_NODE;
    tMcbMsg.u16Addr = RX_MAP_BASE;
    tMcbMsg.u16Size = WORDSIZE_16BIT;
    tMcbMsg.u16Data[0] = (uint16_t)0U;

    uint32_t u32Millis = Mcb_GetMillis();

    do
    {
        ptInst->Mcb_Write(ptInst, &tMcbMsg);

        if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
        {
            tMcbMsg.eStatus = MCB_WRITE_ERROR;
            break;
        }

    } while ((tMcbMsg.eStatus != MCB_WRITE_ERROR)
             && (tMcbMsg.eStatus != MCB_WRITE_SUCCESS));

    switch (tMcbMsg.eStatus)
    {
        case MCB_WRITE_SUCCESS:
            ptInst->tCyclicRxList.u8Mapped = (uint8_t)0;
            ptInst->tCyclicRxList.u16MappedSize = (uint16_t)0U;
            break;
        default:
            /** Nothing */
            break;
    }

    /** Set up internal struct and verify a proper configuration */
    tMcbMsg.u16Node = DEFAULT_MOCO_NODE;
    tMcbMsg.u16Addr = TX_MAP_BASE;
    tMcbMsg.u16Size = WORDSIZE_16BIT;
    tMcbMsg.u16Data[0] = (uint16_t)0U;

    u32Millis = Mcb_GetMillis();

    do
    {
        ptInst->Mcb_Write(ptInst, &tMcbMsg);

        if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
        {
            tMcbMsg.eStatus = MCB_WRITE_ERROR;
            break;
        }

    } while ((tMcbMsg.eStatus != MCB_WRITE_ERROR)
             && (tMcbMsg.eStatus != MCB_WRITE_SUCCESS));

    switch (tMcbMsg.eStatus)
    {
        case MCB_WRITE_SUCCESS:
            ptInst->tCyclicTxList.u8Mapped = (uint8_t)0;
            ptInst->tCyclicTxList.u16MappedSize = (uint16_t)0U;
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
        uint32_t u32Millis = Mcb_GetMillis();

        /** Check and setup RX mapping */
        tMcbMsg.u16Node = DEFAULT_MOCO_NODE;
        tMcbMsg.u16Addr = RX_MAP_BASE;
        tMcbMsg.u16Size = WORDSIZE_16BIT;
        tMcbMsg.u16Data[0] = ptInst->tCyclicRxList.u8Mapped;

        do
        {
            ptInst->Mcb_Write(ptInst, &tMcbMsg);

            if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
            {
                tMcbMsg.eStatus = MCB_WRITE_ERROR;
                break;
            }

        } while ((tMcbMsg.eStatus != MCB_WRITE_ERROR)
                 && (tMcbMsg.eStatus != MCB_WRITE_SUCCESS));

        switch (tMcbMsg.eStatus)
        {
            case MCB_WRITE_SUCCESS:
                /** Do nothing */
                break;
            default:
                i32Result = CYCLIC_ERR_RX_MAP;
                break;
        }

        if (i32Result == CYCLIC_MODE_OK)
        {
            /** If RX mapping was OK, check and setup TX mapping */
            tMcbMsg.u16Node = DEFAULT_MOCO_NODE;
            tMcbMsg.u16Addr = TX_MAP_BASE;
            tMcbMsg.u16Size = WORDSIZE_16BIT;
            tMcbMsg.u16Data[0] = ptInst->tCyclicTxList.u8Mapped;

            u32Millis = Mcb_GetMillis();

            do
            {
                ptInst->Mcb_Write(ptInst, &tMcbMsg);

                if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
                {
                    tMcbMsg.eStatus = MCB_WRITE_ERROR;
                    break;
                }

            } while ((tMcbMsg.eStatus != MCB_WRITE_ERROR)
                     && (tMcbMsg.eStatus != MCB_WRITE_SUCCESS));

            switch (tMcbMsg.eStatus)
            {
                case MCB_WRITE_SUCCESS:
                    /** Do nothing */
                    break;
                default:
                    i32Result = CYCLIC_ERR_TX_MAP;
                    break;
            }
        }

        if (i32Result == CYCLIC_MODE_OK)
        {
            /** If both mappings are OK, enable cyclic mode */
            tMcbMsg.u16Node = DEFAULT_MOCO_NODE;
            tMcbMsg.u16Addr = ADDR_COMM_STATE;
            tMcbMsg.u16Size = WORDSIZE_16BIT;
            tMcbMsg.u16Data[0] = (uint16_t)2U;

            uint32_t u32Millis = Mcb_GetMillis();

            do
            {
                ptInst->Mcb_Write(ptInst, &tMcbMsg);

                if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
                {
                    tMcbMsg.eStatus = MCB_WRITE_ERROR;
                    break;
                }

            } while ((tMcbMsg.eStatus != MCB_WRITE_ERROR)
                     && (tMcbMsg.eStatus != MCB_WRITE_SUCCESS));

            switch (tMcbMsg.eStatus)
            {
                case MCB_WRITE_SUCCESS:
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
    tMcbMsg.eStatus = MCB_STANDBY;

    if (ptInst->isCyclic != false)
    {
        if ((ptInst->tIntf.isCfgOverCyclic == false)
            && (ptInst->tIntf.isNewCfgOverCyclic == false))
        {
            tMcbMsg.u16Node = DEFAULT_MOCO_NODE;
            tMcbMsg.u16Addr = ADDR_COMM_STATE;
            tMcbMsg.u16Size = WORDSIZE_16BIT;
            tMcbMsg.u16Data[0] = (uint16_t)1U;
            /** Cyclic will be disabled through cyclic messages */

            ptInst->Mcb_Write(ptInst, &tMcbMsg);
        }
        else
        {
            tMcbMsg.eStatus = MCB_STANDBY;
        }
    }
    else
    {
        tMcbMsg.eStatus = MCB_WRITE_SUCCESS;
    }

    return tMcbMsg.eStatus;
}

Mcb_ECyclicMode Mcb_GetCyclicMode(Mcb_TInst* ptInst)
{
    Mcb_TMsg tMcbMsg;
    uint32_t u32Millis = Mcb_GetMillis();

    tMcbMsg.u16Node = DEFAULT_MOCO_NODE;
    tMcbMsg.u16Addr = ADDR_CYCLIC_MODE;
    tMcbMsg.u16Size = WORDSIZE_16BIT;

    do
    {
        ptInst->Mcb_Read(ptInst, &tMcbMsg);

        if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
        {
            tMcbMsg.eStatus = MCB_READ_ERROR;
            break;
        }

    } while ((tMcbMsg.eStatus != MCB_READ_ERROR)
             && (tMcbMsg.eStatus != MCB_READ_SUCCESS));

    switch (tMcbMsg.eStatus)
    {
        case MCB_READ_SUCCESS:
            ptInst->eSyncMode = tMcbMsg.u16Data[(uint16_t)0U];
            break;
        default:
            /** Do nothing */
            break;
    }

    return ptInst->eSyncMode;
}

Mcb_ECyclicMode Mcb_SetCyclicMode(Mcb_TInst* ptInst, Mcb_ECyclicMode eNewCycMode)
{
    Mcb_TMsg tMcbMsg;
    uint32_t u32Millis = Mcb_GetMillis();

    tMcbMsg.u16Node = DEFAULT_MOCO_NODE;
    tMcbMsg.u16Addr = ADDR_CYCLIC_MODE;
    tMcbMsg.u16Size = WORDSIZE_16BIT;
    tMcbMsg.u16Data[0] = (uint16_t)eNewCycMode;

    do
    {
        ptInst->Mcb_Write(ptInst, &tMcbMsg);

        if ((Mcb_GetMillis() - u32Millis) > ptInst->u32Timeout)
        {
            tMcbMsg.eStatus = MCB_WRITE_ERROR;
            break;
        }

    } while ((tMcbMsg.eStatus != MCB_WRITE_ERROR)
             && (tMcbMsg.eStatus != MCB_WRITE_SUCCESS));

    switch (tMcbMsg.eStatus)
    {
        case MCB_WRITE_SUCCESS:
            ptInst->eSyncMode = eNewCycMode;
            break;
        default:
            /** Do nothing */
            break;
    }

    return ptInst->eSyncMode;
}

bool Mcb_CyclicProcessLatch(Mcb_TInst* ptInst, Mcb_EStatus* peCfgStat)
{
    Mcb_EStatus eState = MCB_STANDBY;
    bool isTransfer = false;
    bool isCfgData;

    if ((ptInst->isCyclic != false) && (Mcb_IntfIsReady(ptInst->tIntf.u16Id) != false)
        && (Mcb_IntfTryTakeResource(ptInst->tIntf.u16Id) != false))
    {
        isTransfer = true;

        eState = Mcb_IntfCfgOverCyclic(&ptInst->tIntf, ptInst->tConfigRpy.u16Node, ptInst->tConfigRpy.u16Addr,
                                       &ptInst->tConfigRpy.u16Cmd, ptInst->tConfigRpy.u16Data,
                                       &ptInst->tConfigRpy.u16Size, &isCfgData);

        if ((eState == MCB_WRITE_SUCCESS) || (eState == MCB_WRITE_ERROR) ||
            (eState == MCB_READ_SUCCESS) || (eState == MCB_READ_ERROR) ||
            (eState == MCB_GETINFO_SUCCESS) || (eState == MCB_GETINFO_ERROR))
        {
            ptInst->tConfigRpy.eStatus = eState;

            if (ptInst->CfgOverCyclicEvnt != NULL)
            {
                ptInst->CfgOverCyclicEvnt(ptInst, &ptInst->tConfigRpy);
            }

            /* If the communication state has been written succesfully with the stop command,
             * set the interface as non-cyclic */
            if ((ptInst->tConfigReq.u16Addr == ADDR_COMM_STATE) &&
                (eState == MCB_WRITE_SUCCESS) &&
                (ptInst->tConfigReq.u16Data[0] == (uint16_t)1U))
            {
                isTransfer = false;
                ptInst->isCyclic = false;
            }
        }

        if (isTransfer != false)
        {
            Mcb_IntfCyclicLatch(&ptInst->tIntf, ptInst->u16CyclicTx,
                            ptInst->u16CyclicSize, isCfgData);
        }
        else
        {
            Mcb_IntfReleaseResource(ptInst->tIntf.u16Id);
        }
    }

    *peCfgStat = eState;
    return isTransfer;
}

void Mcb_CyclicFrameProcess(Mcb_TInst* ptInst)
{
    if (ptInst->isCyclic != false)
    {
        Mcb_IntfProcessCyclic(&ptInst->tIntf, ptInst->u16CyclicRx, ptInst->u16CyclicSize);
    }
}

static void Mcb_ConfigOverCyclicCompl(Mcb_TInst* ptInst, Mcb_TMsg* pMcbMsg)
{
    if ((ptInst->ptUsrConfig != NULL) && (pMcbMsg != NULL))
    {
        memcpy((void*)ptInst->ptUsrConfig, (const void*)pMcbMsg, sizeof(Mcb_TMsg));
    }
}
