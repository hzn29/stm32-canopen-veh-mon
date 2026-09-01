/*
 * CAN module object for generic microcontroller.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @author      Janez Paternoster
 * @copyright   2004 - 2020 Janez Paternoster
 *
 * This file is part of <https://github.com/CANopenNode/CANopenNode>, a CANopen Stack.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this
 * file except in compliance with the License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License is
 * distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 */

#include "301/CO_driver.h"

void
CO_CANsetConfigurationMode(void* CANptr) {
}

void
CO_CANsetNormalMode(CO_CANmodule_t* CANmodule) {

    CANmodule->CANnormal = true;
}

CO_ReturnError_t
CO_CANmodule_init(CO_CANmodule_t* CANmodule, void* CANptr, CO_CANrx_t rxArray[], uint16_t rxSize, CO_CANtx_t txArray[],
                  uint16_t txSize, uint16_t CANbitRate) {
    uint16_t i;

    if (CANmodule == NULL || rxArray == NULL || txArray == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    CANmodule->CANptr = CANptr;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANerrorStatus = 0;
    CANmodule->CANnormal = false;
    CANmodule->useCANrxFilters = (rxSize <= 32U) ? true : false; /* microcontroller dependent */
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount = 0U;
    CANmodule->errOld = 0U;

    for (i = 0U; i < rxSize; i++) {
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFU;
        rxArray[i].object = NULL;
        rxArray[i].CANrx_callback = NULL;
    }
    for (i = 0U; i < txSize; i++) {
        txArray[i].bufferFull = false;
    }



    if (CANmodule->useCANrxFilters) {
    } else {
    }


    return CO_ERROR_NO;
}

void
CO_CANmodule_disable(CO_CANmodule_t* CANmodule) {
    if (CANmodule != NULL) {
    }
}

CO_ReturnError_t
CO_CANrxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, uint16_t mask, bool_t rtr, void* object,
                   void (*CANrx_callback)(void* object, void* message)) {
    CO_ReturnError_t ret = CO_ERROR_NO;

    if ((CANmodule != NULL) && (object != NULL) && (CANrx_callback != NULL) && (index < CANmodule->rxSize)) {
        CO_CANrx_t* buffer = &CANmodule->rxArray[index];

        buffer->object = object;
        buffer->CANrx_callback = CANrx_callback;

        buffer->ident = ident & 0x07FFU;
        if (rtr) {
            buffer->ident |= 0x0800U;
        }
        buffer->mask = (mask & 0x07FFU) | 0x0800U;

        if (CANmodule->useCANrxFilters) {}
    } else {
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}

CO_CANtx_t*
CO_CANtxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, bool_t rtr, uint8_t noOfBytes,
                   bool_t syncFlag) {
    CO_CANtx_t* buffer = NULL;

    if ((CANmodule != NULL) && (index < CANmodule->txSize)) {
        buffer = &CANmodule->txArray[index];

        buffer->ident = ((uint32_t)ident & 0x07FFU) | ((uint32_t)(((uint32_t)noOfBytes & 0xFU) << 11U))
                        | ((uint32_t)(rtr ? 0x8000U : 0U));

        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }

    return buffer;
}

CO_ReturnError_t
CO_CANsend(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer) {
    FDCAN_HandleTypeDef* hfdcan = (FDCAN_HandleTypeDef*)CANmodule->CANptr;
    FDCAN_TxHeaderTypeDef txHeader = {0};
    txHeader.Identifier = buffer->ident & 0x07FFU;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    uint8_t dataLength = (uint8_t)((buffer->ident >> 11U) & 0x0FU);
    txHeader.DataLength = (dataLength == 0U) ? FDCAN_DLC_BYTES_0 : ((dataLength == 1U) ? FDCAN_DLC_BYTES_1 :
                          ((dataLength == 2U) ? FDCAN_DLC_BYTES_2 : ((dataLength == 3U) ? FDCAN_DLC_BYTES_3 :
                          ((dataLength == 4U) ? FDCAN_DLC_BYTES_4 : ((dataLength == 5U) ? FDCAN_DLC_BYTES_5 :
                          ((dataLength == 6U) ? FDCAN_DLC_BYTES_6 : ((dataLength == 7U) ? FDCAN_DLC_BYTES_7 : FDCAN_DLC_BYTES_8)))))));
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0U;
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, buffer->data) != HAL_OK)
    {
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        return CO_ERROR_TX_OVERFLOW;
    }
    buffer->bufferFull = false;
    CANmodule->firstCANtxMessage = false;
    return CO_ERROR_NO;
}

void
CO_CANclearPendingSyncPDOs(CO_CANmodule_t* CANmodule) {
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND(CANmodule);
    /* Abort frame from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    if (/* CAN frameIsOnCanBuffer && */ CANmodule->bufferInhibitFlag) {
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
    if (CANmodule->CANtxCount != 0U) {
        uint16_t i;
        CO_CANtx_t* buffer = &CANmodule->txArray[0];
        for (i = CANmodule->txSize; i > 0U; i--) {
            if (buffer->bufferFull) {
                if (buffer->syncFlag) {
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
            buffer++;
        }
    }
    CO_UNLOCK_CAN_SEND(CANmodule);

    if (tpdoDeleted != 0U) {
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
}

static uint16_t rxErrors = 0, txErrors = 0, overflow = 0;

void
CO_CANmodule_process(CO_CANmodule_t* CANmodule) {
    uint32_t err;

    err = ((uint32_t)txErrors << 16) | ((uint32_t)rxErrors << 8) | overflow;

    if (CANmodule->errOld != err) {
        uint16_t status = CANmodule->CANerrorStatus;

        CANmodule->errOld = err;

        if (txErrors >= 256U) {
            status |= CO_CAN_ERRTX_BUS_OFF;
        } else {
            status &= 0xFFFF
                      ^ (CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_WARNING
                         | CO_CAN_ERRTX_PASSIVE);

            if (rxErrors >= 128) {
                status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE;
            } else if (rxErrors >= 96) {
                status |= CO_CAN_ERRRX_WARNING;
            }

            if (txErrors >= 128) {
                status |= CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE;
            } else if (txErrors >= 96) {
                status |= CO_CAN_ERRTX_WARNING;
            }

            if ((status & CO_CAN_ERRTX_PASSIVE) == 0) {
                status &= 0xFFFF ^ CO_CAN_ERRTX_OVERFLOW;
            }
        }

        if (overflow != 0) {
            status |= CO_CAN_ERRRX_OVERFLOW;
        }

        CANmodule->CANerrorStatus = status;
    }
}

void
CO_CANinterrupt(CO_CANmodule_t* CANmodule) {

    if (1) {
        CO_CANrxMsg_t* rcvMsg;     /* pointer to received CAN frame in CAN module */
        uint16_t index;            /* index of received CAN frame */
        uint32_t rcvMsgIdent;      /* identifier of the received CAN frame */
        CO_CANrx_t* buffer = NULL; /* receive CAN frame buffer from CO_CANmodule_t object. */
        bool_t msgMatched = false;

        rcvMsg = 0; /* get CAN frame from module here */
        rcvMsgIdent = rcvMsg->ident;
        if (CANmodule->useCANrxFilters) {
            index = 0; /* get index of the received CAN frame here. Or something similar */
            if (index < CANmodule->rxSize) {
                buffer = &CANmodule->rxArray[index];
                if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U) {
                    msgMatched = true;
                }
            }
        } else {
            buffer = &CANmodule->rxArray[0];
            for (index = CANmodule->rxSize; index > 0U; index--) {
                if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U) {
                    msgMatched = true;
                    break;
                }
                buffer++;
            }
        }

        if (msgMatched && (buffer != NULL) && (buffer->CANrx_callback != NULL)) {
            buffer->CANrx_callback(buffer->object, (void*)rcvMsg);
        }

    }

    else if (0) {

        CANmodule->firstCANtxMessage = false;
        CANmodule->bufferInhibitFlag = false;
        if (CANmodule->CANtxCount > 0U) {
            uint16_t i; /* index of transmitting CAN frame */

            CO_CANtx_t* buffer = &CANmodule->txArray[0];
            for (i = CANmodule->txSize; i > 0U; i--) {
                if (buffer->bufferFull) {
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;

                    CANmodule->bufferInhibitFlag = buffer->syncFlag;
                    break; /* exit for loop */
                }
                buffer++;
            } /* end of for loop */

            if (i == 0U) {
                CANmodule->CANtxCount = 0U;
            }
        }
    } else {
    }
}
