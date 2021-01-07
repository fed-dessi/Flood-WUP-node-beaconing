#include "sl_event_handler.h"

#include "em_chip.h"
#include "sl_board_init.h"
#include "sl_device_init_dcdc.h"
#include "sl_device_init_hfxo.h"
#include "sl_device_init_lfxo.h"
#include "sl_device_init_clocks.h"
#include "sl_device_init_emu.h"
#include "pa_conversions_efr32.h"
#include "sl_rail_util_init.h"
#include "sl_sleeptimer.h"
#include "sl_mpu.h"
#include "sl_uartdrv_instances.h"
#include "freertos_sl_system.h"
#include "sl_power_manager.h"

void sl_platform_init(void)
{
  CHIP_Init();
  sl_board_preinit();
  sl_device_init_dcdc();
  sl_device_init_hfxo();
  sl_device_init_lfxo();
  sl_device_init_clocks();
  sl_device_init_emu();
  sl_board_init();
  sl_power_manager_init();
}

void sl_kernel_start(void)
{
  vTaskStartScheduler();
}

void sl_driver_init(void)
{
  sl_uartdrv_init_instances();
}

void sl_service_init(void)
{
  sl_sleeptimer_init();
  sl_mpu_disable_execute_from_ram();
}

void sl_stack_init(void)
{
  sl_rail_util_pa_init();
  sl_rail_util_init();
}

void sl_internal_app_init(void)
{
}

