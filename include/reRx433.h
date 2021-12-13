/* 
   EN: Module for receiving data from wireless sensors 433MHz. Based on https://github.com/sui77/rc-switch
   RU: Модуль для приема данных с беспроводных датчиков 433MHz. Основан на https://github.com/sui77/rc-switch
   --------------------------
   (с) 2021 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
   --------------------------
   Project home: https://github.com/kotyara12/consts/reRx433

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef __RE_RX433_H__
#define __RE_RX433_H__

#include <stdbool.h>
#include "reLed.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/** 
 * Number of maximum high/Low changes per packet.
 * We can handle up to (unsigned long) => 32 bit * 2 H/L changes per bit + 2 for sync
 */
#define RX433_SWITCH_MIN_CHANGES 7
#define RX433_SWITCH_MAX_CHANGES 67 

#ifdef __cplusplus
extern "C" {
#endif

void rx433_Init(const uint8_t gpioRx, QueueHandle_t queueProc);
void rx433_Enable();
void rx433_Disable();

bool rx433_IsAvailable();
void rx433_ResetAvailable();
uint32_t rx433_GetReceivedValue();
uint16_t rx433_GetReceivedBitLength();
uint16_t rx433_GetReceivedDelay();
uint16_t rx433_GetReceivedProtocol();

#ifdef __cplusplus
}
#endif

#endif // __RE_RX433_H__
