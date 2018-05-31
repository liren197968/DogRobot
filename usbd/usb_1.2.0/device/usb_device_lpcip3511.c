/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"
#include "fsl_device_registers.h"

#if (((defined(USB_DEVICE_CONFIG_LPCIP3511FS)) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U)) || \
     ((defined(USB_DEVICE_CONFIG_LPCIP3511HS)) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U)))
#include "usb_device_dci.h"
#include "usb_device_lpcip3511.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* on Aruba IP3511 (USB0 FS), there are 8 physical EPs, on IP3511 HS (USB1 FS), there are 10 physical EPs. */
#define USB_LPC3511IP_MAX_PHY_ENDPOINT_MASK (0xFFFFu)

/*! @brief endpoint command status, buffer address offset */
#define USB_LPC3511IPHS_ENDPOINT_BUFFER_ADDRESS_OFFSET_MASK (0x000007FFu)
#define USB_LPC3511IPHS_ENDPOINT_BUFFER_NBYTES_SHIFT (11)
#define USB_LPC3511IPHS_ENDPOINT_BUFFER_NBYTES_MASK (0x03FFF800u)
#define USB_LPC3511IPFS_ENDPOINT_BUFFER_ADDRESS_OFFSET_MASK (0x0000FFFFu)
#define USB_LPC3511IPFS_ENDPOINT_BUFFER_NBYTES_SHIFT (16)
#define USB_LPC3511IPFS_ENDPOINT_BUFFER_NBYTES_MASK (0x03FF0000u)

#define USB_LPC3511IP_ENDPOINT_ENDPOINT_TYPE_MASK (0x01U << 26)
#define USB_LPC3511IP_ENDPOINT_RFTV_MASK (0x01U << 27)
#define USB_LPC3511IP_ENDPOINT_TOGGLE_RESET_MASK (0x01U << 28)
#define USB_LPC3511IP_ENDPOINT_STALL_MASK (0x01U << 29)
#define USB_LPC3511IP_ENDPOINT_DISABLE_MASK (0x01U << 30)
#define USB_LPC3511IP_ENDPOINT_ACTIVE_MASK (0x01U << 31)
#define USB_LPC3511IP_ENDPOINT_CONFIGURE_BITS_SHIFT (26)

#define USB_LPC3511IP_DEVCMDSTAT_INTERRUPT_WC_MASK (0x0F000000u)

#define USB_LPC3511IP_ENDPOINT_SET_ENDPOINT_AND(lpcState, index, odd, value)                         \
    *((volatile uint32_t *)(((uint32_t)(lpcState->epCommandStatusList)) | ((uint32_t)(index) << 3) | \
                            (((uint32_t)(odd)&1U) << 2U))) &= (value)

/*! @brief Set endpoint command/status value */
#if ((defined(USB_DEVICE_CONFIG_LPCIP3511HS)) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
#define USB_LPC3511IP_ENDPOINT_SET_ENDPOINT(lpcState, index, odd, value, NBytes, address)                \
                                                                                                         \
    *((volatile uint32_t *)(((uint32_t)(lpcState->epCommandStatusList)) | ((uint32_t)(index) << 3) |     \
                            (((uint32_t)(odd & 1U)) << 2U))) =                                           \
        ((lpc3511IpState->controllerSpeed) ?                                                             \
                                                                                                         \
             ((uint32_t)(value) | ((uint32_t)(NBytes) << USB_LPC3511IPHS_ENDPOINT_BUFFER_NBYTES_SHIFT) | \
              (((uint32_t)(address) >> 6) & USB_LPC3511IPHS_ENDPOINT_BUFFER_ADDRESS_OFFSET_MASK)) :      \
                                                                                                         \
             ((uint32_t)(value) | ((uint32_t)(NBytes) << USB_LPC3511IPFS_ENDPOINT_BUFFER_NBYTES_SHIFT) | \
              (((uint32_t)(address) >> 6) & USB_LPC3511IPFS_ENDPOINT_BUFFER_ADDRESS_OFFSET_MASK)))
#else
#define USB_LPC3511IP_ENDPOINT_SET_ENDPOINT(lpcState, index, odd, value, NBytes, address)            \
                                                                                                     \
    *((volatile uint32_t *)(((uint32_t)(lpcState->epCommandStatusList)) | ((uint32_t)(index) << 3) | \
                            (((uint32_t)(odd & 1U)) << 2U))) =                                       \
        ((uint32_t)(value) | ((uint32_t)(NBytes) << USB_LPC3511IPFS_ENDPOINT_BUFFER_NBYTES_SHIFT) |  \
         (((uint32_t)(address) >> 6) & USB_LPC3511IPFS_ENDPOINT_BUFFER_ADDRESS_OFFSET_MASK))
#endif

#define USB_LPC3511IP_ENDPOINT_DES_INDEX(endpoint) \
    (((((uint32_t)endpoint) & 0x0F) << 1) +        \
     ((((uint32_t)endpoint) & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) ? (1) : (0)))

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

extern usb_status_t USB_DeviceNotificationTrigger(void *handle, void *msg);
static usb_status_t USB_DeviceLpc3511IpTransaction(usb_device_lpc3511ip_state_struct_t *lpc3511IpState,
                                                   uint8_t endpointIndex);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* LPC3511IP controller driver instances */
#if ((USB_DEVICE_CONFIG_LPCIP3511FS + USB_DEVICE_CONFIG_LPCIP3511HS) == 1U)
USB_RAM_ADDRESS_ALIGNMENT_256 static usb_device_lpc3511ip_state_struct_t s_UsbDeviceLpc3511IpState1;
#define LPC_CONTROLLER_INSTANCE_ARRAY \
    {                                 \
        &s_UsbDeviceLpc3511IpState1   \
    }
#elif((USB_DEVICE_CONFIG_LPCIP3511FS + USB_DEVICE_CONFIG_LPCIP3511HS) == 2U)
USB_RAM_ADDRESS_ALIGNMENT_256 static usb_device_lpc3511ip_state_struct_t s_UsbDeviceLpc3511IpState1;
USB_RAM_ADDRESS_ALIGNMENT_256 static usb_device_lpc3511ip_state_struct_t s_UsbDeviceLpc3511IpState2;
#define LPC_CONTROLLER_INSTANCE_ARRAY                            \
    {                                                            \
        &s_UsbDeviceLpc3511IpState1, &s_UsbDeviceLpc3511IpState2 \
    }
#elif((USB_DEVICE_CONFIG_LPCIP3511FS + USB_DEVICE_CONFIG_LPCIP3511HS) == 3U)
USB_RAM_ADDRESS_ALIGNMENT_256 static usb_device_lpc3511ip_state_struct_t s_UsbDeviceLpc3511IpState1;
USB_RAM_ADDRESS_ALIGNMENT_256 static usb_device_lpc3511ip_state_struct_t s_UsbDeviceLpc3511IpState2;
USB_RAM_ADDRESS_ALIGNMENT_256 static usb_device_lpc3511ip_state_struct_t s_UsbDeviceLpc3511IpState3;
#define LPC_CONTROLLER_INSTANCE_ARRAY                                                         \
    {                                                                                         \
        &s_UsbDeviceLpc3511IpState1, &s_UsbDeviceLpc3511IpState2, &s_UsbDeviceLpc3511IpState3 \
    }
#else
#error "the max support ip3511 controller instance is 3."
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

static usb_device_lpc3511ip_endpoint_state_struct_t *USB_DeviceLpc3511IpGetEndpointStateStruct(
    usb_device_lpc3511ip_state_struct_t *lpc3511IpState, uint8_t endpointIndex)
{
    if (endpointIndex <= 2)
    {
        return &(lpc3511IpState->endpointState1[endpointIndex]);
    }
#if (USB_DEVICE_CONFIG_ENDPOINTS > 1)
    else if (endpointIndex <= (USB_DEVICE_CONFIG_ENDPOINTS * 2))
    {
        return &(lpc3511IpState->endpointState2[endpointIndex - 3]);
    }
#endif
    else
    {
    }

    return NULL;
}

/*!
 * @brief Write the command/status entry to start a transfer.
 *
 * The function is used to start a transfer by writing the command/status entry.
 *
 * @param lpc3511IpState      Pointer of the controller state structure.
 * @param endpoint         Endpoint number.
 * @param direction        The direction of the endpoint, 0U - USB_OUT, 1U - USB_IN.
 * @param buffer           The memory address to save the received data, or the memory address to hold the data need to
 * be sent.
 * @param length           The length of the data.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceLpc3511IpEndpointTransfer(usb_device_lpc3511ip_state_struct_t *lpc3511IpState,
                                                        uint8_t endpointIndex,
                                                        uint8_t *buffer,
                                                        uint32_t length)
{
    usb_device_lpc3511ip_endpoint_state_struct_t *epState =
        USB_DeviceLpc3511IpGetEndpointStateStruct(lpc3511IpState, endpointIndex);

    USB_OSA_SR_ALLOC();

    /* Enter critical */
    USB_OSA_ENTER_CRITICAL();

    /* Flag the endpoint is busy. */
    epState->stateUnion.stateBitField.transferring = 1U;

    /* update the command/status entry values to enable transfer */
    /* if control endpoint, the value is always zero */
    epState->stateUnion.stateBitField.transactionLength = length;
    if (lpc3511IpState->registerBase->EPINUSE & (0x1U << endpointIndex))
    {
        epState->stateUnion.stateBitField.lastOdd = 1U;
    }
    else
    {
        epState->stateUnion.stateBitField.lastOdd = 0U;
    }

    /* when receive the zero length packet, the controller will set 4 bytes buffer as 0x00 */
    if (buffer == NULL)
    {
        buffer = lpc3511IpState->zeroTransactionData;
    }

    USB_LPC3511IP_ENDPOINT_SET_ENDPOINT(
        lpc3511IpState, endpointIndex, (epState->stateUnion.stateBitField.lastOdd),
        (epState->stateUnion.stateBitField.epControlDefault << USB_LPC3511IP_ENDPOINT_CONFIGURE_BITS_SHIFT) |
            USB_LPC3511IP_ENDPOINT_ACTIVE_MASK,
        length, (uint32_t)buffer);

    epState->stateUnion.stateBitField.epControlDefault &=
        (~((USB_LPC3511IP_ENDPOINT_TOGGLE_RESET_MASK) >> USB_LPC3511IP_ENDPOINT_CONFIGURE_BITS_SHIFT));

    /* Exit critical */
    USB_OSA_EXIT_CRITICAL();
    return kStatus_USB_Success;
}

#if 0
/*!
 * @brief Prime a next setup transfer.
 *
 * The function is used to prime a buffer in control out pipe to wait for receiving the host's setup packet.
 *
 * @param lpc3511IpState       Pointer of the controller state structure.
 *
 */
static void USB_DeviceLpc3511IpPrimeNextSetup(usb_device_lpc3511ip_state_struct_t *lpc3511IpState)
{
    USB_LPC3511IP_ENDPOINT_SET_ENDPOINT(lpc3511IpState, 0, 1, 0, 8, lpc3511IpState->setupData);
}
#endif

/*!
 * @brief reset ip3511.
 *
 * @param lpc3511IpState       Pointer of the controller state structure.
 *
 */
static void USB_DeviceLpc3511IpSetDefaultState(usb_device_lpc3511ip_state_struct_t *lpc3511IpState)
{
    uint32_t index = 0;
    uint8_t usbAddress;

    lpc3511IpState->controlAlignBufferUsed = 0U;

    /* zero the command/status list buffer and disable all endpoints */
    for (index = 0; index < 4; ++index)
    {
        lpc3511IpState->epCommandStatusList[index] = 0x00000000U;
    }
    for (index = 4; index < USB_DEVICE_CONFIG_ENDPOINTS * 4; ++index)
    {
        lpc3511IpState->epCommandStatusList[index] = USB_LPC3511IP_ENDPOINT_DISABLE_MASK;
    }

    /* set address as 0 */
    usbAddress = 0U;
    USB_DeviceLpc3511IpControl(lpc3511IpState, kUSB_DeviceControlSetDeviceAddress, &usbAddress);

    lpc3511IpState->registerBase->EPLISTSTART = (uint32_t)lpc3511IpState->epCommandStatusList;
    /* all data buffer is in the same 4M range with this setup data buffer */
    lpc3511IpState->registerBase->DATABUFSTART = (uint32_t)lpc3511IpState->setupData;
    /* reset registers */
    lpc3511IpState->registerBase->EPINUSE = 0x0;
    lpc3511IpState->registerBase->EPSKIP = 0x0;
    /* enable all double-buffer */
    lpc3511IpState->registerBase->EPBUFCFG = USB_LPC3511IP_MAX_PHY_ENDPOINT_MASK;
    /* clear interrupts */
    lpc3511IpState->registerBase->INTSTAT =
        (USB_INTSTAT_DEV_INT_MASK | USB_INTSTAT_FRAME_INT_MASK | USB_LPC3511IP_MAX_PHY_ENDPOINT_MASK);
    /* enable interrupts */
    lpc3511IpState->registerBase->INTEN = USB_INTSTAT_DEV_INT_MASK | USB_LPC3511IP_MAX_PHY_ENDPOINT_MASK;

    /* Clear reset flag */
    lpc3511IpState->isResetting = 0U;
}

/* Config and Enable endpoint */
static usb_status_t USB_DeviceLpc3511IpEndpointInit(usb_device_lpc3511ip_state_struct_t *lpc3511IpState,
                                                    usb_device_endpoint_init_struct_t *epInit)
{
    uint8_t endpointIndex = USB_LPC3511IP_ENDPOINT_DES_INDEX(epInit->endpointAddress);
    usb_device_lpc3511ip_endpoint_state_struct_t *epState =
        USB_DeviceLpc3511IpGetEndpointStateStruct(lpc3511IpState, endpointIndex);
    uint16_t maxPacketSize = epInit->maxPacketSize;

    /* Make the endpoint max packet size align with USB Specification 2.0. */
    if (USB_ENDPOINT_ISOCHRONOUS == epInit->transferType)
    {
        if (maxPacketSize > USB_DEVICE_MAX_FS_ISO_MAX_PACKET_SIZE)
        {
            maxPacketSize = USB_DEVICE_MAX_FS_ISO_MAX_PACKET_SIZE;
        }
    }
    else
    {
        if (maxPacketSize > USB_DEVICE_MAX_FS_NONE_ISO_MAX_PACKET_SIZE)
        {
            maxPacketSize = USB_DEVICE_MAX_FS_NONE_ISO_MAX_PACKET_SIZE;
        }
    }

    /* update the endpoint structure fields */
    /* Set the endpoint idle */
    epState->stateUnion.stateBitField.transferring = 0U;
    /* Save the max packet size of the endpoint */
    epState->stateUnion.stateBitField.maxPacketSize = maxPacketSize;
    /* Clear the endpoint stalled state */
    epState->stateUnion.stateBitField.stalled = 0U;
    /* Set the ZLT field */
    epState->stateUnion.stateBitField.zlt = epInit->zlt;

    /* get the endpoint default control value */
    if (USB_ENDPOINT_ISOCHRONOUS == epInit->transferType)
    {
        epState->stateUnion.stateBitField.epControlDefault =
            (USB_LPC3511IP_ENDPOINT_ENDPOINT_TYPE_MASK >> USB_LPC3511IP_ENDPOINT_CONFIGURE_BITS_SHIFT);
    }
    else
    {
#if ((defined(USB_DEVICE_CONFIG_LPCIP3511HS)) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
        /* high-speed */
        if ((lpc3511IpState->controllerSpeed) && (USB_ENDPOINT_INTERRUPT == epInit->transferType))
        {
            epState->stateUnion.stateBitField.epControlDefault =
                ((USB_LPC3511IP_ENDPOINT_ENDPOINT_TYPE_MASK | USB_LPC3511IP_ENDPOINT_RFTV_MASK) >>
                 USB_LPC3511IP_ENDPOINT_CONFIGURE_BITS_SHIFT);
        }
        else
#endif
        {
            epState->stateUnion.stateBitField.epControlDefault = 0x00U;
        }
    }
    /* set the command/status value */
    USB_LPC3511IP_ENDPOINT_SET_ENDPOINT(
        lpc3511IpState, endpointIndex, 0,
        (epState->stateUnion.stateBitField.epControlDefault << USB_LPC3511IP_ENDPOINT_CONFIGURE_BITS_SHIFT), 0, 0);
    if ((epInit->endpointAddress & USB_ENDPOINT_NUMBER_MASK) == USB_CONTROL_ENDPOINT)
    {
        if (!(epInit->endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK))
        {
            /* Prime setup packet when the endpoint is control out endpoint. */
            USB_LPC3511IP_ENDPOINT_SET_ENDPOINT(lpc3511IpState, 0, 1, 0, 0, (uint32_t)lpc3511IpState->setupData);
        }
    }
    else
    {
        USB_LPC3511IP_ENDPOINT_SET_ENDPOINT(
            lpc3511IpState, endpointIndex, 1,
            (epState->stateUnion.stateBitField.epControlDefault << USB_LPC3511IP_ENDPOINT_CONFIGURE_BITS_SHIFT), 0, 0);
    }

    return kStatus_USB_Success;
}

/*!
 * @brief De-initialize a specified endpoint.
 *
 * The function is used to de-initialize a specified endpoint.
 * Current transfer of the endpoint will be canceled and the specified endpoint will be disabled.
 *
 * @param lpc3511IpState      Pointer of the controller state structure.
 * @param ep               The endpoint address, Bit7, 0U - USB_OUT, 1U - USB_IN.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceLpc3511IpEndpointDeinit(usb_device_lpc3511ip_state_struct_t *lpc3511IpState, uint8_t ep)
{
    uint8_t endpointIndex = USB_LPC3511IP_ENDPOINT_DES_INDEX(ep);
    usb_device_lpc3511ip_endpoint_state_struct_t *epState =
        USB_DeviceLpc3511IpGetEndpointStateStruct(lpc3511IpState, endpointIndex);

    /* Cancel the transfer of the endpoint */
    USB_DeviceLpc3511IpCancel(lpc3511IpState, ep);

    /* Disable the endpoint */
    USB_LPC3511IP_ENDPOINT_SET_ENDPOINT(lpc3511IpState, endpointIndex, 0, USB_LPC3511IP_ENDPOINT_DISABLE_MASK, 0, 0);
    /* Clear the max packet size */
    epState->stateUnion.stateBitField.maxPacketSize = 0U;

    return kStatus_USB_Success;
}

/*!
 * @brief Stall a specified endpoint.
 *
 * The function is used to stall a specified endpoint.
 * Current transfer of the endpoint will be canceled and the specified endpoint will be stalled.
 *
 * @param lpc3511IpState      Pointer of the controller state structure.
 * @param ep               The endpoint address, Bit7, 0U - USB_OUT, 1U - USB_IN.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceLpc3511IpEndpointStall(usb_device_lpc3511ip_state_struct_t *lpc3511IpState, uint8_t ep)
{
    uint8_t endpointIndex = USB_LPC3511IP_ENDPOINT_DES_INDEX(ep);
    usb_device_lpc3511ip_endpoint_state_struct_t *epState =
        USB_DeviceLpc3511IpGetEndpointStateStruct(lpc3511IpState, endpointIndex);

    /* cancel the transfer in the endpoint */
    USB_DeviceLpc3511IpCancel(lpc3511IpState, ep);

    /* Set endpoint stall flag. */
    epState->stateUnion.stateBitField.stalled = 1U;

    /* stall the endpoint */
    USB_LPC3511IP_ENDPOINT_SET_ENDPOINT(lpc3511IpState, endpointIndex, 0, USB_LPC3511IP_ENDPOINT_STALL_MASK, 0, 0);
    if ((ep & USB_ENDPOINT_NUMBER_MASK) != USB_CONTROL_ENDPOINT)
    {
        USB_LPC3511IP_ENDPOINT_SET_ENDPOINT(lpc3511IpState, endpointIndex, 1, USB_LPC3511IP_ENDPOINT_STALL_MASK, 0, 0);
    }

    return kStatus_USB_Success;
}

/*!
 * @brief Un-stall a specified endpoint.
 *
 * The function is used to un-stall a specified endpoint.
 * Current transfer of the endpoint will be canceled and the specified endpoint will be un-stalled.
 *
 * @param lpc3511IpState      Pointer of the controller state structure.
 * @param ep               The endpoint address, Bit7, 0U - USB_OUT, 1U - USB_IN.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceLpc3511IpEndpointUnstall(usb_device_lpc3511ip_state_struct_t *lpc3511IpState, uint8_t ep)
{
    uint8_t endpointIndex = USB_LPC3511IP_ENDPOINT_DES_INDEX(ep);
    usb_device_lpc3511ip_endpoint_state_struct_t *epState =
        USB_DeviceLpc3511IpGetEndpointStateStruct(lpc3511IpState, endpointIndex);

    /* Clear the endpoint stall state, the hardware resets the endpoint
     * toggle to one for both directions when a setup token is received */
    epState->stateUnion.stateBitField.stalled = 0U;

    /* unstall the endpoint for double buffers */
    USB_LPC3511IP_ENDPOINT_SET_ENDPOINT_AND(lpc3511IpState, endpointIndex, 0, (~USB_LPC3511IP_ENDPOINT_STALL_MASK));
    if ((ep & USB_ENDPOINT_NUMBER_MASK) != USB_CONTROL_ENDPOINT)
    {
        USB_LPC3511IP_ENDPOINT_SET_ENDPOINT_AND(lpc3511IpState, endpointIndex, 1, (~USB_LPC3511IP_ENDPOINT_STALL_MASK));

        /* toggle reset for the toggle */
        epState->stateUnion.stateBitField.epControlDefault |=
            ((USB_LPC3511IP_ENDPOINT_TOGGLE_RESET_MASK) >> USB_LPC3511IP_ENDPOINT_CONFIGURE_BITS_SHIFT);
    }

    return kStatus_USB_Success;
}

static void USB_DeviceLpc3511IpInterruptToken(usb_device_lpc3511ip_state_struct_t *lpc3511IpState,
                                              uint8_t endpointIndex,
                                              uint8_t isSetup,
                                              uint32_t errorStatus)
{
    usb_device_callback_message_struct_t message;
    uint32_t length;
    uint32_t remainLength;
    usb_setup_struct_t *setupPacket;
    uint32_t index;
    usb_device_lpc3511ip_endpoint_state_struct_t *epState =
        USB_DeviceLpc3511IpGetEndpointStateStruct(lpc3511IpState, endpointIndex);

    if ((!isSetup) && (0U == epState->stateUnion.stateBitField.transferring))
    {
        return;
    }

    if (isSetup)
    {
        message.length = 8U;
        message.buffer = (lpc3511IpState->setupData);
    }
    else
    {
        /* get the transaction length */
        if (1U == epState->stateUnion.stateBitField.lastOdd)
        {
            length = *(((uint32_t *)lpc3511IpState->registerBase->EPLISTSTART) + endpointIndex * 2 + 1);
        }
        else
        {
            length = *(((uint32_t *)lpc3511IpState->registerBase->EPLISTSTART) + endpointIndex * 2);
        }

#if ((defined(USB_DEVICE_CONFIG_LPCIP3511HS)) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
        if (lpc3511IpState->controllerSpeed)
        {
            remainLength =
                (length & USB_LPC3511IPHS_ENDPOINT_BUFFER_NBYTES_MASK) >> USB_LPC3511IPHS_ENDPOINT_BUFFER_NBYTES_SHIFT;
        }
        else
#endif
        {
            remainLength =
                (length & USB_LPC3511IPFS_ENDPOINT_BUFFER_NBYTES_MASK) >> USB_LPC3511IPFS_ENDPOINT_BUFFER_NBYTES_SHIFT;
        }

        length = epState->stateUnion.stateBitField.transactionLength - remainLength;

        /* control data buffer align is used */
        if (((endpointIndex >> 1U) == USB_CONTROL_ENDPOINT) && (lpc3511IpState->controlAlignBufferUsed == 1U) &&
            (length > 0U))
        {
            lpc3511IpState->controlAlignBufferUsed = 0U;
            if (endpointIndex == 0U) /* USB_OUT */
            {
                /* remainLength is used for buffer index */
                remainLength = epState->transferDone;
                for (index = 0; index < length; ++index)
                {
                    epState->transferBuffer[remainLength + index] = lpc3511IpState->controlData[index];
                }
            }
        }

        /* update the transferred length */
        epState->transferDone += length;

        /* update remaining length */
        remainLength = epState->transferLength - epState->transferDone;

        /* Whether the transfer is completed or not.
         * The transfer is completed when one of the following conditions meet:
         * 1. The remaining length is zero.
         * 2. The length of current transcation is not the multiple of max packet size.
         */
        if ((length > 0U) && (!(length % epState->stateUnion.stateBitField.maxPacketSize)) && (remainLength > 0U))
        {
            /* process the remaining data */
            if (endpointIndex & 0x01U) /* IN */
            {
                (void)USB_DeviceLpc3511IpTransaction(lpc3511IpState, endpointIndex);
            }
            else /* OUT */
            {
                USB_DeviceLpc3511IpTransaction(lpc3511IpState, endpointIndex);
            }
            return;
        }
        else
        {
            epState->stateUnion.stateBitField.transferring = 0U;
            message.length = epState->transferDone;
            message.buffer = epState->transferBuffer;

            /* process ZLT
             * 1. IN endpoint;
             * 2. transfer length is the multiple of max packet size.
            */
            if ((endpointIndex & 0x01U) && (length) && (!(length % epState->stateUnion.stateBitField.maxPacketSize)))
            {
                if ((endpointIndex >> 1U) == USB_CONTROL_ENDPOINT)
                {
                    setupPacket = (usb_setup_struct_t *)(&(lpc3511IpState->setupData[0]));
                    /*
                     * Send ZLT transaction if setup transfer and the required length is longer than actual length
                     */
                    if (USB_SHORT_FROM_LITTLE_ENDIAN(setupPacket->wLength) > epState->transferLength)
                    {
                        (void)USB_DeviceLpc3511IpEndpointTransfer(lpc3511IpState, 1U, NULL, 0U);
                        return;
                    }
                }
                else if ((epState->stateUnion.stateBitField.zlt))
                {
                    (void)USB_DeviceLpc3511IpEndpointTransfer(lpc3511IpState, endpointIndex, NULL, 0U);
                    return;
                }
                else
                {
                }
            }
        }
    }

    message.isSetup = isSetup;
    message.code = ((uint8_t)(endpointIndex >> 1) | (uint8_t)(((uint32_t)(endpointIndex & 0x01U) << 0x07U)));

    /* Notify the up layer the controller status changed. */
    USB_DeviceNotificationTrigger(lpc3511IpState->deviceHandle, &message);
}

/*!
 * @brief Handle the USB bus reset interrupt.
 *
 * The function is used to handle the USB bus reset interrupt.
 *
 * @param lpc3511IpState       Pointer of the controller state structure.
 *
 */
static void USB_DeviceLpc3511IpInterruptReset(usb_device_lpc3511ip_state_struct_t *lpc3511IpState)
{
    usb_device_callback_message_struct_t message;

    /* Set reset flag */
    lpc3511IpState->isResetting = 1U;

#if ((defined(USB_DEVICE_CONFIG_LPCIP3511HS)) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
    if (lpc3511IpState->controllerSpeed)
    {
        if (((lpc3511IpState->registerBase->DEVCMDSTAT & USB_DEVCMDSTAT_SPEED_MASK) >> USB_DEVCMDSTAT_SPEED_SHIFT) ==
            0x02U)
        {
            lpc3511IpState->deviceSpeed = USB_SPEED_HIGH;
        }
        else if (((lpc3511IpState->registerBase->DEVCMDSTAT & USB_DEVCMDSTAT_SPEED_MASK) >>
                  USB_DEVCMDSTAT_SPEED_SHIFT) == 0x01U)
        {
            lpc3511IpState->deviceSpeed = USB_SPEED_FULL;
        }
        else
        {
        }
    }
    else
#endif
    {
        lpc3511IpState->deviceSpeed = USB_SPEED_FULL;
    }

    message.buffer = (uint8_t *)NULL;
    message.code = kUSB_DeviceNotifyBusReset;
    message.length = 0U;
    message.isSetup = 0U;
    /* Notify up layer the USB bus reset signal detected. */
    USB_DeviceNotificationTrigger(lpc3511IpState->deviceHandle, &message);
}

#if (defined(USB_DEVICE_CONFIG_DETACH_ENABLE) && (USB_DEVICE_CONFIG_DETACH_ENABLE))
/*!
 * @brief Handle detach interrupt.
 *
 * The function is used to handle the detach interrupt.
 *
 * @param lpc3511IpState       Pointer of the controller state structure.
 *
 */
static void USB_DeviceLpc3511IpInterruptDetach(usb_device_lpc3511ip_state_struct_t *lpc3511IpState)
{
    usb_device_callback_message_struct_t message;

    message.buffer = (uint8_t *)NULL;
    message.code = kUSB_DeviceNotifyDetach;
    message.length = 0U;
    message.isSetup = 0U;

    /* Notify up layer the USB VBUS falling signal detected. */
    USB_DeviceNotificationTrigger(lpc3511IpState->deviceHandle, &message);
}
#endif

usb_status_t USB_DeviceLpc3511IpInit(uint8_t controllerId,
                                     usb_device_handle handle,
                                     usb_device_controller_handle *controllerHandle)
{
    usb_device_lpc3511ip_state_struct_t *lpc3511IpState;
    uint32_t ip3511Bases[] = USB_BASE_ADDRS;
    usb_device_lpc3511ip_state_struct_t *usbDeviceLpcIp3511StateArray[] = LPC_CONTROLLER_INSTANCE_ARRAY;

    /* get the controller instance */
    if ( (controllerId < kUSB_ControllerLpcIp3511Fs0) ||
        ((controllerId - kUSB_ControllerLpcIp3511Fs0) >= (uint8_t)USB_DEVICE_CONFIG_LPCIP3511FS) ||
        ((uint32_t)(controllerId - kUSB_ControllerLpcIp3511Fs0) >= (sizeof(ip3511Bases) / sizeof(uint32_t))))
    {
        return kStatus_USB_ControllerNotFound;
    }
    lpc3511IpState = usbDeviceLpcIp3511StateArray[controllerId - kUSB_ControllerLpcIp3511Fs0];
    lpc3511IpState->controllerId = controllerId;
    lpc3511IpState->controlAlignBufferUsed = 0U;
    /* get the ip base address */
    lpc3511IpState->registerBase = (USB_Type *)ip3511Bases[controllerId - kUSB_ControllerLpcIp3511Fs0];
#if ((defined(USB_DEVICE_CONFIG_LPCIP3511HS)) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
    if ((lpc3511IpState->controllerId >= kUSB_ControllerLpcIp3511Hs0) &&
        (lpc3511IpState->controllerId <= kUSB_ControllerLpcIp3511Hs1))
    {
        lpc3511IpState->controllerSpeed = 1U;
    }
    else
    {
        lpc3511IpState->controllerSpeed = 0U;
    }
#endif

    /* disable the controller */
    lpc3511IpState->registerBase->DEVCMDSTAT &=
        (~(USB_DEVCMDSTAT_DCON_MASK | USB_DEVCMDSTAT_DEV_EN_MASK | USB_DEVCMDSTAT_LPM_SUP_MASK));
    /* reset and enalbe the controller */
    USB_DeviceLpc3511IpSetDefaultState(lpc3511IpState);
    /* enable USB */
    lpc3511IpState->registerBase->DEVCMDSTAT |= (USB_DEVCMDSTAT_DEV_EN_MASK | USB_DEVCMDSTAT_FORCE_NEEDCLK_MASK);

    lpc3511IpState->deviceHandle = handle;
    *controllerHandle = lpc3511IpState;

    return kStatus_USB_Success;
}

usb_status_t USB_DeviceLpc3511IpDeinit(usb_device_controller_handle controllerHandle)
{
    usb_device_lpc3511ip_state_struct_t *lpc3511IpState = (usb_device_lpc3511ip_state_struct_t *)controllerHandle;
    uint32_t usbAddress;

    if (controllerHandle == NULL)
    {
        return kStatus_USB_InvalidHandle;
    }
    /* Clear all interrupt flags. */
    lpc3511IpState->registerBase->INTSTAT =
        (USB_INTSTAT_DEV_INT_MASK | USB_INTSTAT_FRAME_INT_MASK | USB_LPC3511IP_MAX_PHY_ENDPOINT_MASK);
    /* Disable all interrupts. */
    lpc3511IpState->registerBase->INTEN = 0U;
    /* Clear device address. */
    usbAddress = 0U;
    USB_DeviceLpc3511IpControl(lpc3511IpState, kUSB_DeviceControlSetDeviceAddress, &usbAddress);

    /* disable the controller */
    lpc3511IpState->registerBase->DEVCMDSTAT &= (~(USB_DEVCMDSTAT_DCON_MASK | USB_DEVCMDSTAT_DEV_EN_MASK));

    return kStatus_USB_Success;
}

static usb_status_t USB_DeviceLpc3511IpTransaction(usb_device_lpc3511ip_state_struct_t *lpc3511IpState,
                                                   uint8_t endpointIndex)
{
    usb_status_t error = kStatus_USB_Error;
    uint8_t *actualBuffer;
    uint32_t index;
    uint32_t length;
    usb_device_lpc3511ip_endpoint_state_struct_t *epState =
        USB_DeviceLpc3511IpGetEndpointStateStruct(lpc3511IpState, endpointIndex);

    /* Data length needs to less than max packet size in each call. */
    length = epState->transferLength - epState->transferDone;
    if (length > epState->stateUnion.stateBitField.maxPacketSize)
    {
        length = epState->stateUnion.stateBitField.maxPacketSize;
    }

    actualBuffer = epState->transferBuffer + epState->transferDone;
    /* align the buffer for control transfer */
    if (((endpointIndex >> 1U) == USB_CONTROL_ENDPOINT) && (((uint32_t)actualBuffer & 0x0000003FU) != 0U) &&
        (length > 0U))
    {
        lpc3511IpState->controlAlignBufferUsed = 1U;
        if (endpointIndex == 1U) /* USB_IN */
        {
            for (index = 0; index < length; ++index)
            {
                lpc3511IpState->controlData[index] = actualBuffer[index];
            }
        }
        actualBuffer = lpc3511IpState->controlData;
    }

    /* Send/Receive data when the device is not resetting. */
    if (0U == lpc3511IpState->isResetting)
    {
        error = USB_DeviceLpc3511IpEndpointTransfer(lpc3511IpState, endpointIndex, actualBuffer, length);
    }

    return error;
}

usb_status_t USB_DeviceLpc3511IpSend(usb_device_controller_handle controllerHandle,
                                     uint8_t endpointAddress,
                                     uint8_t *buffer,
                                     uint32_t length)
{
    usb_device_lpc3511ip_state_struct_t *lpc3511IpState = (usb_device_lpc3511ip_state_struct_t *)controllerHandle;
    uint8_t endpointIndex = USB_LPC3511IP_ENDPOINT_DES_INDEX(endpointAddress);
    usb_device_lpc3511ip_endpoint_state_struct_t *epState =
        USB_DeviceLpc3511IpGetEndpointStateStruct(lpc3511IpState, endpointIndex);

    if (((buffer != NULL) && (((uint32_t)buffer & 0xFFC00000U) != lpc3511IpState->registerBase->DATABUFSTART)) ||
        (1U == epState->stateUnion.stateBitField.transferring))
    {
        return kStatus_USB_Error;
    }

    /* Save the tansfer information */
    epState->transferDone = 0U;
    epState->transferBuffer = buffer;
    epState->transferLength = length;

    return USB_DeviceLpc3511IpTransaction(lpc3511IpState, endpointIndex);

/* prime the control setup transfer if it is control in endpoint and data length is zero
 * For IP3511 there is no need to prime, the buffer is always in the command/status list
*/
#if 0
    if ((0U == length) && (USB_CONTROL_ENDPOINT == (endpointAddress & USB_ENDPOINT_NUMBER_MASK)))
    {
        USB_DeviceLpc3511IpPrimeNextSetup(lpc3511IpState);
    }
#endif
}

usb_status_t USB_DeviceLpc3511IpRecv(usb_device_controller_handle controllerHandle,
                                     uint8_t endpointAddress,
                                     uint8_t *buffer,
                                     uint32_t length)
{
    return USB_DeviceLpc3511IpSend(controllerHandle, endpointAddress, buffer, length);
}

usb_status_t USB_DeviceLpc3511IpCancel(usb_device_controller_handle controllerHandle, uint8_t ep)
{
    usb_device_lpc3511ip_state_struct_t *lpc3511IpState = (usb_device_lpc3511ip_state_struct_t *)controllerHandle;
    usb_device_callback_message_struct_t message;
    uint8_t endpointIndex = USB_LPC3511IP_ENDPOINT_DES_INDEX(ep);
    usb_device_lpc3511ip_endpoint_state_struct_t *epState =
        USB_DeviceLpc3511IpGetEndpointStateStruct(lpc3511IpState, endpointIndex);

    /* Cancel the transfer and notify the up layer when the endpoint is busy. */
    if (epState->stateUnion.stateBitField.transferring)
    {
        /* cancel the transfer in the endpoint command/status */
        lpc3511IpState->registerBase->EPSKIP |= (0x01U << endpointIndex);
        while (lpc3511IpState->registerBase->EPSKIP & (0x01U << endpointIndex))
        {
        }
        /* clear the cancel interrupt */
        lpc3511IpState->registerBase->INTSTAT = (0x01U << endpointIndex);

        message.length = USB_UNINITIALIZED_VAL_32;
        message.buffer = epState->transferBuffer;
        message.code = ep;
        message.isSetup = 0U;
        epState->stateUnion.stateBitField.transferring = 0U;
        USB_DeviceNotificationTrigger(lpc3511IpState->deviceHandle, &message);
    }
    return kStatus_USB_Success;
}

usb_status_t USB_DeviceLpc3511IpControl(usb_device_controller_handle controllerHandle,
                                        usb_device_control_type_t type,
                                        void *param)
{
    usb_device_lpc3511ip_state_struct_t *lpc3511IpState = (usb_device_lpc3511ip_state_struct_t *)controllerHandle;
    usb_status_t error = kStatus_USB_Error;
    uint32_t tmp32Value;
    uint8_t tmp8Value;
#if ((defined(USB_DEVICE_CONFIG_REMOTE_WAKEUP)) && (USB_DEVICE_CONFIG_REMOTE_WAKEUP > 0U))
    usb_device_struct_t *deviceHandle;
#endif
    usb_device_lpc3511ip_endpoint_state_struct_t *epState;

    if (controllerHandle == NULL)
    {
        return kStatus_USB_InvalidHandle;
    }

#if ((defined(USB_DEVICE_CONFIG_REMOTE_WAKEUP)) && (USB_DEVICE_CONFIG_REMOTE_WAKEUP > 0U))
    deviceHandle = (usb_device_struct_t *)lpc3511IpState->deviceHandle;
#endif

    switch (type)
    {
        case kUSB_DeviceControlRun:
            lpc3511IpState->registerBase->DEVCMDSTAT |= (USB_DEVCMDSTAT_DCON_MASK);
            break;

        case kUSB_DeviceControlStop:
            lpc3511IpState->registerBase->DEVCMDSTAT &= (~USB_DEVCMDSTAT_DCON_MASK);
            break;

        case kUSB_DeviceControlEndpointInit:
            if (param)
            {
                error = USB_DeviceLpc3511IpEndpointInit(lpc3511IpState, (usb_device_endpoint_init_struct_t *)param);
            }
            break;

        case kUSB_DeviceControlEndpointDeinit:
            if (param)
            {
                tmp8Value = *((uint8_t *)param);
                error = USB_DeviceLpc3511IpEndpointDeinit(lpc3511IpState, tmp8Value);
            }
            break;

        case kUSB_DeviceControlEndpointStall:
            if (param)
            {
                tmp8Value = *((uint8_t *)param);
                error = USB_DeviceLpc3511IpEndpointStall(lpc3511IpState, tmp8Value);
            }
            break;

        case kUSB_DeviceControlEndpointUnstall:
            if (param)
            {
                tmp8Value = *((uint8_t *)param);
                error = USB_DeviceLpc3511IpEndpointUnstall(lpc3511IpState, tmp8Value);
            }
            break;

        case kUSB_DeviceControlGetDeviceStatus:
            if (param)
            {
                *((uint16_t *)param) =
                    (USB_DEVICE_CONFIG_SELF_POWER << (USB_REQUEST_STANDARD_GET_STATUS_DEVICE_SELF_POWERED_SHIFT))
#if ((defined(USB_DEVICE_CONFIG_REMOTE_WAKEUP)) && (USB_DEVICE_CONFIG_REMOTE_WAKEUP > 0U))
                    | (deviceHandle->remotewakeup << (USB_REQUEST_STANDARD_GET_STATUS_DEVICE_REMOTE_WARKUP_SHIFT))
#endif
                    ;
                error = kStatus_USB_Success;
            }
            break;

        case kUSB_DeviceControlGetEndpointStatus:
            if (param)
            {
                usb_device_endpoint_status_struct_t *endpointStatus = (usb_device_endpoint_status_struct_t *)param;

                if (((endpointStatus->endpointAddress) & USB_ENDPOINT_NUMBER_MASK) < USB_DEVICE_CONFIG_ENDPOINTS)
                {
                    epState = USB_DeviceLpc3511IpGetEndpointStateStruct(
                        lpc3511IpState, USB_LPC3511IP_ENDPOINT_DES_INDEX(endpointStatus->endpointAddress));
                    endpointStatus->endpointStatus = (uint16_t)(epState->stateUnion.stateBitField.stalled == 1U) ?
                                                         kUSB_DeviceEndpointStateStalled :
                                                         kUSB_DeviceEndpointStateIdle;
                    error = kStatus_USB_Success;
                }
            }
            break;

        case kUSB_DeviceControlSetDeviceAddress:
            if (param)
            {
                tmp8Value = *((uint8_t *)param);
                tmp32Value = lpc3511IpState->registerBase->DEVCMDSTAT;
                tmp32Value &= (~USB_DEVCMDSTAT_DEV_ADDR_MASK);
                tmp32Value |= (tmp8Value & USB_DEVCMDSTAT_DEV_ADDR_MASK);
                lpc3511IpState->registerBase->DEVCMDSTAT = tmp32Value;
                error = kStatus_USB_Success;
            }
            break;

        case kUSB_DeviceControlGetSynchFrame:
            break;

        case kUSB_DeviceControlResume:
#if defined(USB_DEVICE_CONFIG_REMOTE_WAKEUP) && (USB_DEVICE_CONFIG_REMOTE_WAKEUP > 0U)
            /* todo: turn on USB clock and enable the USB clock source */
            lpc3511IpState->registerBase->DEVCMDSTAT |= USB_DEVCMDSTAT_FORCE_NEEDCLK_MASK;
            lpc3511IpState->registerBase->DEVCMDSTAT &= USB_DEVCMDSTAT_DSUS_MASK;
            while (lpc3511IpState->registerBase->DEVCMDSTAT & USB_DEVCMDSTAT_DSUS_MASK)
            {
            }
            /* 0xF0FFFEFFu exclude the WC bits */
            lpc3511IpState->registerBase->DEVCMDSTAT &= ((~USB_DEVCMDSTAT_FORCE_NEEDCLK_MASK) & 0xF0FFFEFFu);
            error = kStatus_USB_Success;
#endif /* USB_DEVICE_CONFIG_REMOTE_WAKEUP */
            break;

        case kUSB_DeviceControlSetDefaultStatus:
            for (tmp32Value = 0U; tmp32Value < USB_DEVICE_CONFIG_ENDPOINTS; tmp32Value++)
            {
                USB_DeviceLpc3511IpEndpointDeinit(lpc3511IpState, (tmp32Value | (USB_IN << 0x07U)));
                USB_DeviceLpc3511IpEndpointDeinit(lpc3511IpState, (tmp32Value | (USB_OUT << 0x07U)));
            }
            USB_DeviceLpc3511IpSetDefaultState(lpc3511IpState);
            error = kStatus_USB_Success;
            break;

        case kUSB_DeviceControlGetSpeed:
            if (param)
            {
                *((uint8_t *)param) = lpc3511IpState->deviceSpeed;
                error = kStatus_USB_Success;
            }
            break;

        case kUSB_DeviceControlGetOtgStatus:
            break;
        case kUSB_DeviceControlSetOtgStatus:
            break;
        case kUSB_DeviceControlSetTestMode:
            break;
        default:
            break;
    }

    return error;
}

void USB_DeviceLpcIp3511IsrFunction(void *deviceHandle)
{
    usb_device_struct_t *handle = (usb_device_struct_t *)deviceHandle;
    usb_device_lpc3511ip_state_struct_t *lpc3511IpState;
    uint32_t interruptStatus;
    uint32_t usbErrorCode;
    uint32_t devState;

    if (NULL == deviceHandle)
    {
        return;
    }

    lpc3511IpState = (usb_device_lpc3511ip_state_struct_t *)(handle->controllerHandle);
    /* get and clear interrupt status */
    interruptStatus = lpc3511IpState->registerBase->INTSTAT;
    lpc3511IpState->registerBase->INTSTAT = interruptStatus;
    interruptStatus &= lpc3511IpState->registerBase->INTEN;

    usbErrorCode = (lpc3511IpState->registerBase->INFO & USB_INFO_ERR_CODE_MASK);

    /* device status change interrupt */
    if (interruptStatus & USB_INTSTAT_DEV_INT_MASK)
    {
        /* get and clear device state change status */
        devState = lpc3511IpState->registerBase->DEVCMDSTAT;
        lpc3511IpState->registerBase->DEVCMDSTAT |= USB_LPC3511IP_DEVCMDSTAT_INTERRUPT_WC_MASK;

        /* reset change */
        if (devState & USB_DEVCMDSTAT_DRES_C_MASK)
        {
            USB_DeviceLpc3511IpInterruptReset(lpc3511IpState);
        }

#if (defined(USB_DEVICE_CONFIG_DETACH_ENABLE) && (USB_DEVICE_CONFIG_DETACH_ENABLE))
        /* dis-connect change */
        if (devState & USB_DEVCMDSTAT_DCON_C_MASK)
        {
            USB_DeviceLpc3511IpInterruptDetach(lpc3511IpState);
        }
#endif

        /* Suspend/Resume */
        if (devState & USB_DEVCMDSTAT_DSUS_C_MASK)
        {
        }

#if 0U /* some soc don't support this bit, need check according to the feature macro */
        /* OTG Status change */
        if (lpc3511IpState->registerBase->DEVCMDSTAT & USB_DEVCMDSTAT_OTG_C_MASK)
        {
        }
#endif
    }

    /* endpoint transfers interrupt */
    if (interruptStatus & USB_LPC3511IP_MAX_PHY_ENDPOINT_MASK)
    {
        devState = 0U; /* devState means index here */
        /* receive the setup transaction */
        if (lpc3511IpState->registerBase->DEVCMDSTAT & USB_DEVCMDSTAT_SETUP_MASK)
        {
            devState = 2;
            if ((lpc3511IpState->endpointState1[0].stateUnion.stateBitField.stalled == 1U) ||
                (lpc3511IpState->endpointState1[1].stateUnion.stateBitField.stalled == 1U))
            {
                USB_LPC3511IP_ENDPOINT_SET_ENDPOINT_AND(
                    lpc3511IpState, USB_LPC3511IP_ENDPOINT_DES_INDEX(0x0), 0,
                    (~(USB_LPC3511IP_ENDPOINT_STALL_MASK | USB_LPC3511IP_ENDPOINT_ACTIVE_MASK)));
                USB_LPC3511IP_ENDPOINT_SET_ENDPOINT_AND(
                    lpc3511IpState, USB_LPC3511IP_ENDPOINT_DES_INDEX(0x80), 0,
                    (~(USB_LPC3511IP_ENDPOINT_STALL_MASK | USB_LPC3511IP_ENDPOINT_ACTIVE_MASK)));
                lpc3511IpState->endpointState1[0].stateUnion.stateBitField.stalled = 0U;
                lpc3511IpState->endpointState1[1].stateUnion.stateBitField.stalled = 0U;
            }
            /* W1 to clear the setup flag */
            lpc3511IpState->registerBase->DEVCMDSTAT |= USB_DEVCMDSTAT_SETUP_MASK;

            /* todo: setup token interrupt */
            USB_DeviceLpc3511IpInterruptToken(lpc3511IpState, 0U, 1, usbErrorCode);
        }
        for (; devState < (USB_DEVICE_CONFIG_ENDPOINTS * 2); ++devState)
        {
            /* check the endpoint interrupt */
            if (interruptStatus & (0x01U << devState))
            {
                USB_DeviceLpc3511IpInterruptToken(lpc3511IpState, devState, 0, usbErrorCode);
            }
        }
    }

#if 0U
    if (interruptStatus & USB_INTSTAT_FRAME_INT_MASK)
    {
    }
#endif
}

#endif
