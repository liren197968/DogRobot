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

#ifndef __USB_DEVICE_LPC3511IP_H__
#define __USB_DEVICE_LPC3511IP_H__

/*!
 * @addtogroup usb_device_controller_lpcip3511_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief The maximum value of ISO maximum packet size for FS in USB specification 2.0 */
#define USB_DEVICE_MAX_FS_ISO_MAX_PACKET_SIZE (1023U)

/*! @brief The maximum value of non-ISO maximum packet size for FS in USB specification 2.0 */
#define USB_DEVICE_MAX_FS_NONE_ISO_MAX_PACKET_SIZE (64U)

#define USB_DEVCMDSTAT_SPEED_MASK (0x00C00000U)
#define USB_DEVCMDSTAT_SPEED_SHIFT (22)

/*! @brief Endpoint state structure */
typedef struct _usb_device_lpc3511ip_endpoint_state_struct
{
    uint8_t *transferBuffer; /*!< Address of buffer containing the data to be transmitted */
    uint32_t transferLength; /*!< Length of data to transmit. */
    uint32_t transferDone;   /*!< The data length has been transferred*/
    union
    {
        uint32_t state; /*!< The state of the endpoint */
        struct
        {
            uint32_t maxPacketSize : 11U;     /*!< The maximum packet size of the endpoint */
            uint32_t stalled : 1U;            /*!< The endpoint is stalled or not */
            uint32_t transferring : 1U;       /*!< The endpoint is transferring */
            uint32_t zlt : 1U;                /*!< zlt flag */
            uint32_t lastOdd : 1U;            /*!< The last used double buffers */
            uint32_t resersed1 : 1U;          /*!< Reversed */
            uint32_t transactionLength : 11U; /*!< The last prime data length */
            uint32_t epControlDefault : 5u;   /*!< The EP command/status 26~30 bits */
        } stateBitField;
    } stateUnion;
} usb_device_lpc3511ip_endpoint_state_struct_t;

/*! @brief LPC USB controller (IP3511) state structure */
typedef struct _usb_device_lpc3511ip_state_struct
{
    /*!< LPC IP3511 endpoint command/status list, EPLISTSTART's value is the buffer pointer. */
    uint32_t epCommandStatusList[((USB_DEVICE_CONFIG_ENDPOINTS + 0x03U) & 0xFFFFFFFCU) * 4];
    /*!< control data buffer, must align with 64 */
    uint8_t controlData[64U];
    /*!< 8 bytes' setup data, must align with 64 */
    uint8_t setupData[8];
    /* there is 56 bytes idle in order to align with 64 */
    usb_device_handle deviceHandle; /*!< (4 bytes) Device handle used to identify the device object belongs to */
    USB_Type *registerBase;         /*!< (4 bytes) ip base address */
    /*!< (16 * 3 bytes)Endpoint state structures */
    usb_device_lpc3511ip_endpoint_state_struct_t endpointState1[3];
    /*!< 4 bytes for zero length transaction, must align with 64 */
    uint8_t zeroTransactionData[4];
#if (USB_DEVICE_CONFIG_ENDPOINTS > 1)
    usb_device_lpc3511ip_endpoint_state_struct_t endpointState2[(USB_DEVICE_CONFIG_ENDPOINTS * 2) - 3];
#endif
    uint8_t controllerId;           /*!< Controller ID */
    uint8_t isResetting;            /*!< Is doing device reset or not */
    uint8_t deviceSpeed;            /*!< some controller support the HS */
    uint8_t controlAlignBufferUsed; /*!< whether the control buffer is used for align */
#if ((defined(USB_DEVICE_CONFIG_LPCIP3511HS)) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
    uint8_t controllerSpeed;
#endif
} usb_device_lpc3511ip_state_struct_t;

/*!
 * @name USB device controller (IP3511) functions
 * @{
 */

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the USB device controller instance.
 *
 * This function initializes the USB device controller module specified by the controllerId.
 *
 * @param[in] controllerId      The controller ID of the USB IP. See the enumeration type usb_controller_index_t.
 * @param[in] handle            Pointer of the device handle used to identify the device object belongs to.
 * @param[out] controllerHandle An out parameter used to return the pointer of the device controller handle to the
 * caller.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceLpc3511IpInit(uint8_t controllerId,
                                     usb_device_handle handle,
                                     usb_device_controller_handle *controllerHandle);

/*!
 * @brief Deinitializes the USB device controller instance.
 *
 * This function deinitializes the USB device controller module.
 *
 * @param[in] controllerHandle   Pointer of the device controller handle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceLpc3511IpDeinit(usb_device_controller_handle controllerHandle);

/*!
 * @brief Sends data through a specified endpoint.
 *
 * This function sends data through a specified endpoint.
 *
 * @param[in] controllerHandle Pointer of the device controller handle.
 * @param[in] endpointAddress  Endpoint index.
 * @param[in] buffer           The memory address to hold the data need to be sent.
 * @param[in] length           The data length need to be sent.
 *
 * @return A USB error code or kStatus_USB_Success.
 *
 * @note The return value indicates whether the sending request is successful or not. The transfer completion is
 * notified by the
 * corresponding callback function.
 * Currently, only one transfer request can be supported for a specific endpoint.
 * If there is a specific requirement to support multiple transfer requests for a specific endpoint, the application
 * should implement a queue in the application level.
 * The subsequent transfer can begin only when the previous transfer is done (a notification is obtained through the
 * endpoint
 * callback).
 */
usb_status_t USB_DeviceLpc3511IpSend(usb_device_controller_handle controllerHandle,
                                     uint8_t endpointAddress,
                                     uint8_t *buffer,
                                     uint32_t length);

/*!
 * @brief Receives data through a specified endpoint.
 *
 * This function receives data through a specified endpoint.
 *
 * @param[in] controllerHandle Pointer of the device controller handle.
 * @param[in] endpointAddress  Endpoint index.
 * @param[in] buffer           The memory address to save the received data.
 * @param[in] length           The data length to be received.
 *
 * @return A USB error code or kStatus_USB_Success.
 *
 * @note The return value indicates whether the receiving request is successful or not. The transfer completion is
 * notified by the
 * corresponding callback function.
 * Currently, only one transfer request can be supported for a specific endpoint.
 * If there is a specific requirement to support multiple transfer requests for a specific endpoint, the application
 * should implement a queue in the application level.
 * The subsequent transfer can begin only when the previous transfer is done (a notification is obtained through the
 * endpoint
 * callback).
 */
usb_status_t USB_DeviceLpc3511IpRecv(usb_device_controller_handle controllerHandle,
                                     uint8_t endpointAddress,
                                     uint8_t *buffer,
                                     uint32_t length);

/*!
 * @brief Cancels the pending transfer in a specified endpoint.
 *
 * The function is used to cancel the pending transfer in a specified endpoint.
 *
 * @param[in] controllerHandle  ointer of the device controller handle.
 * @param[in] ep                Endpoint address, bit7 is the direction of endpoint, 1U - IN, abd 0U - OUT.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceLpc3511IpCancel(usb_device_controller_handle controllerHandle, uint8_t ep);

/*!
 * @brief Controls the status of the selected item.
 *
 * The function is used to control the status of the selected item.
 *
 * @param[in] controllerHandle      Pointer of the device controller handle.
 * @param[in] type             The selected item. Please refer to enumeration type usb_device_control_type_t.
 * @param[in,out] param            The parameter type is determined by the selected item.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceLpc3511IpControl(usb_device_controller_handle controllerHandle,
                                        usb_device_control_type_t type,
                                        void *param);

/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* __USB_DEVICE_LPC3511IP_H__ */
