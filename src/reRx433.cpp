#include "reRx433.h"
#include <stdint.h>
#include <stdio.h>
#include "time.h"
#include "esp_err.h"
#include <driver/gpio.h>
#include "rLog.h"
#include "rTypes.h"

static const char* logTAG = "RX433";

TaskHandle_t _rxTask;
QueueHandle_t _rxQueue = nullptr;

#define ERR_CHECK(err, str) if (err != ESP_OK) rlog_e(logTAG, "%s: #%d %s", str, err, esp_err_to_name(err));
#define ERR_GPIO_SET_MODE "Failed to set GPIO mode"
#define ERR_GPIO_SET_ISR  "Failed to set ISR handler"

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Protocols ------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

/**
 * Description of a single pule, which consists of a high signal
 * whose duration is "high" times the base pulse length, followed
 * by a low signal lasting "low" times the base pulse length.
 * Thus, the pulse overall lasts (high+low)*pulseLength
 */
typedef struct rxHighLow_t{
    uint8_t high;
    uint8_t low;
} rxHighLow_t;

/**
 * A "protocol" describes how zero and one bits are encoded into high/low pulses.
 * 
 * Format for protocol definitions:
 * {pulselength, Sync bit, "0" bit, "1" bit, invertedSignal}
 * 
 * pulselength: pulse length in microseconds, e.g. 350
 * Sync bit: {1, 31} means 1 high pulse and 31 low pulses
 *     (perceived as a 31*pulselength long pulse, total length of sync bit is
 *     32*pulselength microseconds), i.e:
 *      _
 *     | |_______________________________ (don't count the vertical bars)
 * "0" bit: waveform for a data bit of value "0", {1, 3} means 1 high pulse
 *     and 3 low pulses, total length (1+3)*pulselength, i.e:
 *      _
 *     | |___
 * "1" bit: waveform for a data bit of value "1", e.g. {3,1}:
 *      ___
 *     |   |_
 *
 * These are combined to form Tri-State bits when sending or receiving codes.
 */
typedef struct rxProtocol_t {
  /** base pulse length in microseconds, e.g. 350 */
  uint16_t pulseLength;

  rxHighLow_t syncFactor;
  rxHighLow_t zero;
  rxHighLow_t one;

  /**
   * If true, interchange high and low logic levels in all transmissions.
   *
   * By default, RCSwitch assumes that any signals it sends or receives
   * can be broken down into pulses which start with a high signal level,
   * followed by a a low signal level. This is e.g. the case for the
   * popular PT 2260 encoder chip, and thus many switches out there.
   *
   * But some devices do it the other way around, and start with a low
   * signal level, followed by a high signal level, e.g. the HT6P20B. To
   * accommodate this, one can set invertedSignal to true, which causes
   * RCSwitch to change how it interprets any HighLow struct FOO: It will
   * then assume transmissions start with a low signal lasting
   * FOO.high*pulseLength microseconds, followed by a high signal lasting
   * FOO.low*pulseLength microseconds.
   */
  bool invertedSignal;
} rxProtocol_t; 

static const DRAM_ATTR rxProtocol_t rxProtocols[] = {
  { 350, {  1, 31 }, {  1,  3 }, {  3,  1 }, false },    // protocol 1 (Generic PIR)
  { 650, {  1, 10 }, {  1,  2 }, {  2,  1 }, false },    // protocol 2
  { 100, { 30, 71 }, {  4, 11 }, {  9,  6 }, false },    // protocol 3
  { 380, {  1,  6 }, {  1,  3 }, {  3,  1 }, false },    // protocol 4
  { 500, {  6, 14 }, {  1,  2 }, {  2,  1 }, false },    // protocol 5
  { 450, { 23,  1 }, {  1,  2 }, {  2,  1 }, true },     // protocol 6 (HT6P20B)
  { 150, {  2, 62 }, {  1,  6 }, {  6,  1 }, false },    // protocol 7 (HS2303-PT, i. e. used in AUKEY Remote)
  { 200, {  3, 130}, {  7, 16 }, {  3,  16}, false},     // protocol 8 Conrad RS-200 RX
  { 200, { 130, 7 }, {  16, 7 }, { 16,  3 }, true},      // protocol 9 Conrad RS-200 TX
  { 365, { 18,  1 }, {  3,  1 }, {  1,  3 }, true },     // protocol 10 (1ByOne Doorbell)
  { 270, { 36,  1 }, {  1,  2 }, {  2,  1 }, true },     // protocol 11 (HT12E)
  { 320, { 36,  1 }, {  1,  2 }, {  2,  1 }, true }      // protocol 12 (SM5212)
};

enum {
   numProtocols = sizeof(rxProtocols) / sizeof(rxProtocols[0])
};

static inline uint16_t diff(uint16_t A, uint16_t B) 
{
  return abs(A - B);
} 

/**
 * nSeparationLimit: minimum microseconds between received codes, closer codes are ignored.
 * according to discussion on issue #14 it might be more suitable to set the separation
 * limit to the same time as the 'low' part of the sync signal for the current protocol.
 */
static const uint16_t IRAM_ATTR nSeparationLimit = 4300;
static const uint8_t  IRAM_ATTR nReceiveTolerance = 60;

static volatile uint32_t _receivedValue = 0;
static volatile uint16_t _receivedBitlength = 0;
static volatile uint16_t _receivedDelay = 0;
static volatile uint16_t _receivedProtocol = 0;

static uint16_t _timings[RX433_SWITCH_MAX_CHANGES];

bool IRAM_ATTR rxDetectProtocol(const uint8_t p, uint16_t changeCount) 
{
  const rxProtocol_t &pro = rxProtocols[p-1];
  uint32_t code = 0;
  
  // Assuming the longer pulse length is the pulse captured in _timings[0]
  const uint16_t syncLengthInPulses = ((pro.syncFactor.low) > (pro.syncFactor.high)) ? (pro.syncFactor.low) : (pro.syncFactor.high);
  const uint16_t delay = _timings[0] / syncLengthInPulses;
  const uint16_t delayTolerance = delay * nReceiveTolerance / 100;

  /* For protocols that start low, the sync period looks like
    *               _________
    * _____________|         |XXXXXXXXXXXX|
    *
    * |--1st dur--|-2nd dur-|-Start data-|
    *
    * The 3rd saved duration starts the data.
    *
    * For protocols that start high, the sync period looks like
    *
    *  ______________
    * |              |____________|XXXXXXXXXXXXX|
    *
    * |-filtered out-|--1st dur--|--Start data--|
    *
    * The 2nd saved duration starts the data
    */
  const unsigned int firstDataTiming = (pro.invertedSignal) ? (2) : (1);

  for (unsigned int i = firstDataTiming; i < changeCount - 1; i += 2) {
    code <<= 1;
    if (diff(_timings[i], delay * pro.zero.high) < delayTolerance &&
        diff(_timings[i + 1], delay * pro.zero.low) < delayTolerance) {
      // zero
    } 
    else if (diff(_timings[i], delay * pro.one.high) < delayTolerance &&
             diff(_timings[i + 1], delay * pro.one.low) < delayTolerance) {
      // one
      code |= 1;
    } 
    else {
      // failed
      return false;
    }
  }

  // Ignore very short transmissions: no device sends them, so this must be noise
  if (changeCount > RX433_SWITCH_MIN_CHANGES) {    
    _receivedValue = code;
    _receivedBitlength = (changeCount - 1) / 2;
    _receivedDelay = delay;
    _receivedProtocol = p;
    return true;
  };

  return false;
}

// -----------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------- ISR handler -----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static gpio_num_t _gpioRx = GPIO_NUM_MAX;

static void IRAM_ATTR rxIsrHandler(void* arg)
{
  static uint64_t usTimePrev = 0;
  static uint64_t usTimeCurr = 0;
  static uint16_t cntChanges = 0;
  static uint16_t cntRepeats = 0;

  usTimeCurr = esp_timer_get_time();
  uint16_t usDuration = usTimeCurr - usTimePrev;
  if (usDuration > nSeparationLimit) {
    // A long stretch without signal level change occurred. 
    // This could be the gap between two transmission.
    if ((cntRepeats == 0) || (diff(usDuration, _timings[0]) < 200)) {
      // This long signal is close in length to the long signal which
      // started the previously recorded _timings; this suggests that
      // it may indeed by a a gap between two transmissions (we assume
      // here that a sender will send the signal multiple times,
      // with roughly the same gap between them).
      cntRepeats++;
      if (cntRepeats == 2) {
        for(uint8_t i = 1; i <= numProtocols; i++) {
          if (rxDetectProtocol(i, cntChanges)) {
            // receive succeeded for protocol i, post data to external queue
            QueueHandle_t queueProc = (QueueHandle_t)arg;
            if (queueProc) {
              reciever_data_t data;
              data.source = RTM_RX433;
              data.address = i;
              data.value = rx433_GetReceivedValue();
              data.count = rx433_GetReceivedBitLength();
              // reset recieved value
              rx433_ResetAvailable();
              // we have not woken a task at the start of the ISR
              BaseType_t xHigherPriorityTaskWoken = pdFALSE;
              // post data
              xQueueSendFromISR(queueProc, &data, &xHigherPriorityTaskWoken);
              // now the buffer is empty we can switch context if necessary.
              if (xHigherPriorityTaskWoken) {
                portYIELD_FROM_ISR();
              };
            } else {
              // reset recieved value
              rx433_ResetAvailable();
            };
            break;
          };
        };
        cntRepeats = 0;
      };
    };
    cntChanges = 0;
  };

  // Detect overflow
  if (cntChanges >= RX433_SWITCH_MAX_CHANGES) {
    cntChanges = 0;
    cntRepeats = 0;
  };
  _timings[cntChanges++] = usDuration;
  usTimePrev = usTimeCurr;
}

void rx433_Init(const uint8_t gpioRx, QueueHandle_t queueProc)
{
  _gpioRx = static_cast<gpio_num_t>(gpioRx);
  
  rlog_i(logTAG, "Initialization of 433MHz receiver on gpio #%d", _gpioRx);

  ERR_CHECK(gpio_install_isr_service(0), "Failed to install ISR service");

  gpio_pad_select_gpio(_gpioRx);
  ERR_CHECK(gpio_set_direction(_gpioRx, GPIO_MODE_INPUT), ERR_GPIO_SET_MODE);
  ERR_CHECK(gpio_set_pull_mode(_gpioRx, GPIO_FLOATING), ERR_GPIO_SET_MODE);
  ERR_CHECK(gpio_set_intr_type(_gpioRx, GPIO_INTR_ANYEDGE), ERR_GPIO_SET_ISR);
  ERR_CHECK(gpio_isr_handler_add(_gpioRx, rxIsrHandler, queueProc), ERR_GPIO_SET_ISR);
}

void rx433_Enable()
{
  esp_err_t err = gpio_intr_enable(_gpioRx);
  if (err == ESP_OK) {
    rlog_i(logTAG, "Receiver 433MHz started");
  } else {
    rlog_e(logTAG, "Failed to start 433MHz receiver");
  };
}

void rx433_Disable()
{
  esp_err_t err = gpio_intr_disable(_gpioRx);
  if (err == ESP_OK) {
    rlog_i(logTAG, "Receiver 433MHz stopped");
  } else {
    rlog_e(logTAG, "Failed to stop 433MHz receiver");
  };
}

// -----------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------- Public functions --------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool rx433_IsAvailable()
{
  return _receivedValue != 0;
}

void rx433_ResetAvailable()
{
  _receivedValue = 0;
}

uint32_t rx433_GetReceivedValue()
{
  return _receivedValue;
}

uint16_t rx433_GetReceivedBitLength()
{
  return _receivedBitlength;
}

uint16_t rx433_GetReceivedDelay()
{
  return _receivedDelay;
}

uint16_t rx433_GetReceivedProtocol()
{
  return _receivedProtocol;
}
