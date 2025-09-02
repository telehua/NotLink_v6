/*
 * Copyright (c) 2013-2021 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ----------------------------------------------------------------------
 *
 * $Date:        16. June 2021
 * $Revision:    V2.1.0
 *
 * Project:      CMSIS-DAP Configuration
 * Title:        dap_config.h CMSIS-DAP Configuration File (Template)
 *
 *---------------------------------------------------------------------------*/

#ifndef __DAP_CONFIG_H__
#define __DAP_CONFIG_H__

/*
Provides definitions about the hardware and configuration of the Debug Unit.

This information includes:
 - Definition of Cortex-M processor parameters used in CMSIS-DAP Debug Unit.
 - Debug Unit Identification strings (Vendor, Product, Serial Number).
 - Debug Unit communication packet size.
 - Debug Access Port supported modes and settings (JTAG/SWD and SWO).
 - Optional information about a connected Target Device (for Evaluation Boards).
*/

#include "bsp.h" // Debug Unit Cortex-M Processor Header File

// Indicate that Serial Wire Debug (SWD) communication mode is available at the Debug Access Port.
// This information is returned by the command DAP_Info.Capabilities.
#define DAP_SWD 1 /* SWD Mode:  1 = available, 0 = not available. */

// Indicate that JTAG communication mode is available at the Debug Port.
// This information is returned by the command DAP_Info.Capabilities.
#define DAP_JTAG 1 /* JTAG Mode: 1 = available, 0 = not available. */

// Configure maximum number of JTAG devices on the scan chain connected to the Debug Access Port.
// This setting impacts the RAM requirements of the Debug Unit. Valid range is 1 .. 255.
#define DAP_JTAG_DEV_CNT 8U /* Maximum number of JTAG devices on scan chain. */

// Default communication mode on the Debug Access Port.
// Used for the command \ref DAP_Connect when Port Default mode is selected.
#define DAP_DEFAULT_PORT 1U /* Default JTAG/SWJ Port Mode: 1 = SWD, 2 = JTAG. */

// Default communication speed on the Debug Access Port for SWD and JTAG mode.
// Used to initialize the default SWD/JTAG clock frequency.
// The command \ref DAP_SWJ_Clock can be used to overwrite this default setting.
#define DAP_DEFAULT_SWJ_CLOCK 10000000U /* Default SWD/JTAG clock frequency in Hz. */

// Maximum Package Size for Command and Response data.
// This configuration settings is used to optimize the communication performance with the
// debugger and depends on the USB peripheral. Typical vales are 64 for Full-speed USB HID or WinUSB,
// 1024 for High-speed USB HID and 512 for High-speed USB WinUSB.
#define DAP_PACKET_SIZE 512U /* Specifies Packet Size in bytes. */

// Maximum Package Buffers for Command and Response data.
// This configuration settings is used to optimize the communication performance with the
// debugger and depends on the USB peripheral. For devices with limited RAM or USB buffer the
// setting can be reduced (valid range is 1 .. 255).
#define DAP_PACKET_COUNT 16U /* Specifies number of packets buffered. */

// Indicate that UART Serial Wire Output (SWO) trace is available.
// This information is returned by the command DAP_Info.Capabilities.
#define SWO_UART 0 /* SWO UART:  1 = available, 0 = not available. */

// USART Driver instance number for the UART SWO.
#define SWO_UART_DRIVER 0 /* USART Driver instance number (Driver_USART#). */

// Maximum SWO UART Baudrate.
#define SWO_UART_MAX_BAUDRATE 10000000U /* SWO UART Maximum Baudrate in Hz. */

// Indicate that Manchester Serial Wire Output (SWO) trace is available.
// This information is returned by the command DAP_Info.Capabilities.
#define SWO_MANCHESTER 0 /* SWO Manchester:  1 = available, 0 = not available. */

// SWO Trace Buffer Size.
#define SWO_BUFFER_SIZE 4096U /* SWO Trace Buffer Size in bytes (must be 2^n). */

// SWO Streaming Trace.
#define SWO_STREAM 0 /* SWO Streaming Trace: 1 = available, 0 = not available. */

// Clock frequency of the Test Domain Timer. Timer value is returned with \ref TIMESTAMP_GET.
#define TIMESTAMP_CLOCK 10000000U /* Timestamp clock in Hz (0 = timestamps not supported). */

// Indicate that UART Communication Port is available.
// This information is returned by the command DAP_Info.Capabilities.
#define DAP_UART 0 /* DAP UART:  1 = available, 0 = not available. */

// UART Receive Packet Count.
#define DAP_UART_PACKET_COUNT 32U

// UART Receive Packet Size.
#define DAP_UART_PACKET_SIZE 512U

// Indicate that UART Communication via USB COM Port is available.
// This information is returned by the command DAP_Info.Capabilities.
#define DAP_UART_USB_COM_PORT 1 /* USB COM Port:  1 = available, 0 = not available. */

// Debug Unit is connected to fixed Target Device.
// The Debug Unit may be part of an evaluation board and always connected to a fixed
// known device. In this case a Device Vendor, Device Name, Board Vendor and Board Name strings
// are stored and may be used by the debugger or IDE to configure device parameters.
#define TARGET_FIXED 0 /* Target: 1 = known, 0 = unknown. */

#define TARGET_DEVICE_VENDOR "Arm"    ///< String indicating the Silicon Vendor
#define TARGET_DEVICE_NAME "Cortex-M" ///< String indicating the Target Device
#define TARGET_BOARD_VENDOR "Arm"     ///< String indicating the Board Vendor
#define TARGET_BOARD_NAME "Arm board" ///< String indicating the Board Name

#if TARGET_FIXED != 0
#include <string.h>
static const char TargetDeviceVendor[] = TARGET_DEVICE_VENDOR;
static const char TargetDeviceName[] = TARGET_DEVICE_NAME;
static const char TargetBoardVendor[] = TARGET_BOARD_VENDOR;
static const char TargetBoardName[] = TARGET_BOARD_NAME;
#endif

#ifndef DAP_VENDOR_NAME
#define DAP_VENDOR_NAME "Arm"
#endif

#ifndef DAP_PORDUCT_NAME
#define DAP_PORDUCT_NAME "NotLink CMSIS-DAP"
#endif

#endif /* __DAP_CONFIG_H__ */
