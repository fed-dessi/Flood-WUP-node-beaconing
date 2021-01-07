/***************************************************************************//**
 * @file main.c
 * @brief main.c
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

// -----------------------------------------------------------------------------
//                                   Includes
// -----------------------------------------------------------------------------
#include "sl_component_catalog.h"
#include "sl_system_init.h"
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
  #include "sl_power_manager.h"
#endif
#include "app_init.h"
#include "app_process.h"
#if defined(SL_CATALOG_KERNEL_PRESENT)
  #include "sl_system_kernel.h"
#else // SL_CATALOG_KERNEL_PRESENT
  #include "sl_system_process_action.h"
#endif // SL_CATALOG_KERNEL_PRESENT

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>
#include <queue.h>
#include <sl_sleeptimer.h>

#include "string.h"
#include "strings.h"
#include "stdio.h"

#include <sl_uartdrv_usart_vcom_config.h>
#include <sl_uartdrv_instances.h>
#include <sl_board_control_config.h>
#include <sl_power_manager_config.h>
// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------
#define QUEUE_DEFAULT_LENGTH 16
#define RETRANSMISSION_BUFFER_DEFAULT_LENGTH 10
#define STACK_SIZE (configMINIMAL_STACK_SIZE * 2)
#define SLEEPTIMER_DELAY_MS 1000

enum wupSequence{
  Wb,
  Wd,
  Wr
};

#pragma pack(push,1)
typedef struct
{
  uint16_t wupSeq;
  uint16_t hopCount;
  uint16_t pktSeq; //Packet Sequence #
} pkt_header_t;

typedef struct
{
  pkt_header_t header;
  uint8_t payload[10];
} pkt_t;
#pragma pack(pop)
// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------
///Idle task and Timer task definitions
static StaticTask_t xTimerTaskTCB;
static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

///Receiver Task
static StaticTask_t receiverTaskTCB;
static StackType_t receiverTaskStack[STACK_SIZE];
static void receiverTaskFunction (void*);
static TaskHandle_t receiverTaskHandle;

///Delayer Task
static StaticTask_t delayerTaskTCB;
static StackType_t delayerTaskStack[configMINIMAL_STACK_SIZE];
static void delayerTaskFunction (void*);
static TaskHandle_t delayerTaskHandle;

///RFSense callback
static void rfSenseCb(void);

///Callback Function
static void timerCallback(sl_sleeptimer_timer_handle_t *handle, void *data);
// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                                Static Variables
// -----------------------------------------------------------------------------
/// A static handle of a RAIL instance
static RAIL_Handle_t rail_handle;

/// RAIL tx and rx queue
static uint8_t railRxFifo[sizeof(pkt_t) * QUEUE_DEFAULT_LENGTH];
static uint16_t rxFifoSize = sizeof(pkt_t) * QUEUE_DEFAULT_LENGTH;

///Dummy packet structs
static pkt_t rxPacket;

///Rx Packet handle, details and info
static RAIL_RxPacketHandle_t packet_handle;
static RAIL_RxPacketInfo_t packet_info;

/// Pointer used to force context switch from ISR
static BaseType_t xHigherPriorityTaskWoken;

///VCOM Serial print buffer
static uint8_t transmitterBuffer[100];

///Sleeptimer handles
static sl_sleeptimer_timer_handle_t delayerSleeptimerHandle;

static uint16_t hopCount = 0;
static uint32_t pktSequenceNumber = 0;

static volatile bool wait, isTimerRunning;
// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------
/******************************************************************************
 * Main function
 *****************************************************************************/
int main(void)
{
  // Initialize Silicon Labs device, system, service(s) and protocol stack(s).
  // Note that if the kernel is present, processing task(s) will be created by
  // this call.
  sl_system_init();

  // Initialize the application. For example, create periodic timer(s) or
  // task(s) if the kernel is present.
  rail_handle = app_init();

  //Receiver Task
  receiverTaskHandle = xTaskCreateStatic (receiverTaskFunction, "receiverTask", STACK_SIZE, NULL, 4, receiverTaskStack, &receiverTaskTCB);
  if (receiverTaskHandle == NULL)
   {
     return(0);
   }

  //Delayer Task
  //This task prevents going into sleep mode again after waking up. "Sleep mode" is just the Idle Task running,
  //when we wake up with RFSense we would immediately fall back into it after the callback.
  delayerTaskHandle = xTaskCreateStatic (delayerTaskFunction, "delayerTask", configMINIMAL_STACK_SIZE, NULL, 1, delayerTaskStack, &delayerTaskTCB);
  if (delayerTaskHandle == NULL)
   {
     return(0);
   }

  //enabling vcom
  GPIO_PinOutSet (SL_BOARD_ENABLE_VCOM_PORT, SL_BOARD_ENABLE_VCOM_PIN);

#if defined(SL_CATALOG_KERNEL_PRESENT)
  // Start the kernel. Task(s) created in app_init() will start running.
  sl_system_kernel_start();
#else // SL_CATALOG_KERNEL_PRESENT
  while (1) {
    // Do not remove this call: Silicon Labs components process action routine
    // must be called from the super loop.
    sl_system_process_action();

    // Application process.
    app_process_action(rail_handle);

#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
    // Let the CPU go to sleep if the system allows it.
    sl_power_manager_sleep();
#endif
  }
#endif // SL_CATALOG_KERNEL_PRESENT
}

// -----------------------------------------------------------------------------
//                          Static Function Definitions
// -----------------------------------------------------------------------------
///Idle and Timer Task definition
void vApplicationGetIdleTaskMemory (StaticTask_t **ppxIdleTaskTCBBuffer,
                               StackType_t **ppxIdleTaskStackBuffer,
                               uint32_t *pulIdleTaskStackSize)
{
  //Declare static variables so they're not placed on the stack

  *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory (StaticTask_t **ppxTimerTaskTCBBuffer,
                                StackType_t **ppxTimerTaskStackBuffer,
                                uint32_t *pulTimerTaskStackSize)
{

  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

///Receiver Task
void receiverTaskFunction (void *rt){
  while (true)
    {
      ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
      packet_handle = RAIL_GetRxPacketInfo (rail_handle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);

      if (packet_handle != RAIL_RX_PACKET_HANDLE_INVALID){
          sl_sleeptimer_is_timer_running(&delayerSleeptimerHandle, &isTimerRunning);
          if(isTimerRunning){
              sl_sleeptimer_restart_timer_ms(&delayerSleeptimerHandle, SLEEPTIMER_DELAY_MS, timerCallback, (void*)&wait, 0, 0);
          }

          RAIL_CopyRxPacket (&rxPacket, &packet_info);
          RAIL_ReleaseRxPacket (rail_handle, packet_handle);

          if(rxPacket.header.wupSeq == Wb){
              hopCount = rxPacket.header.hopCount + 1;

              snprintf (&transmitterBuffer, 100, "\r\nBeacon Packet received:\r\nWUP Sequence: %u\r\nNew Hop Count: %lu\r\n", rxPacket.header.wupSeq, hopCount);
              while (ECODE_OK != UARTDRV_TransmitB (sl_uartdrv_usart_vcom_handle, &transmitterBuffer[0], strlen (transmitterBuffer)));
          }

      }
    }
}

///Delayer task, avoids we immediately go to sleep after waking up with RFSense
void delayerTaskFunction (void *dt)
{
  while (1)
    {
      ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
      sl_sleeptimer_is_timer_running(&delayerSleeptimerHandle, &isTimerRunning);
      if(!isTimerRunning){
          wait = true;
          sl_sleeptimer_start_timer_ms(&delayerSleeptimerHandle, SLEEPTIMER_DELAY_MS, timerCallback, (void*)&wait, 0, 0);
          while(wait);
      }
      //TODO: remove after finishing the debugging phase
      if(true)
        __asm__("nop");
    }
}

void timerCallback(sl_sleeptimer_timer_handle_t *handle, void *data){
  volatile bool *wait_flag = (bool*)data;

  *wait_flag = false;
}


///Idle Task Hook, we turn off the radio and start the RFSense peripheral on the Sub GHZ freq before entering "sleep mode"
void vApplicationIdleHook ()
{
  // Starting RFSENSE before going to sleep
  RAIL_Idle (rail_handle, RAIL_IDLE, true);
  RAIL_StartRfSense (rail_handle, RAIL_RFSENSE_SUBGHZ_LOW_SENSITIVITY, 50, rfSenseCb);
}

///RFSense Callback function
void rfSenseCb ()
{
  //We've woken up with RFSense, now we schedule a receiving window of 1s
  //and notify the delayer task so we don't immediately go to sleep
  if (RAIL_StartRx (rail_handle, 0 , NULL) == RAIL_STATUS_NO_ERROR)
    {
      xHigherPriorityTaskWoken = pdFALSE;
      vTaskNotifyGiveFromISR(delayerTaskHandle, &xHigherPriorityTaskWoken);
    }
}


///RAIL event handler
void
sl_rail_app_on_event (RAIL_Handle_t rail_handle, RAIL_Events_t events)
{
  if (events & RAIL_EVENT_CAL_NEEDED)
    {
      RAIL_Calibrate (rail_handle, NULL, RAIL_CAL_ALL_PENDING);
    }
  if (events & RAIL_EVENT_RX_PACKET_RECEIVED)
    {
      //new rx -> deferred handler architecture
      RAIL_HoldRxPacket (rail_handle);
      vTaskNotifyGiveFromISR(receiverTaskHandle, &xHigherPriorityTaskWoken);

    }
}



RAIL_Status_t RAILCb_SetupRxFifo (RAIL_Handle_t railHandle)
{
  RAIL_Status_t status = RAIL_SetRxFifo (railHandle, &railRxFifo[0], &rxFifoSize);
  if (rxFifoSize != sizeof(pkt_t) * QUEUE_DEFAULT_LENGTH)
    {
      // We set up an incorrect FIFO size
      return RAIL_STATUS_INVALID_PARAMETER;
    }
  if (status == RAIL_STATUS_INVALID_STATE)
    {
      // Allow failures due to multiprotocol
      return RAIL_STATUS_NO_ERROR;
    }
  return status;
}
