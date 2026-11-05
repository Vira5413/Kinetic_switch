/***************************************************************************//**
 * @file
 * @brief main() function.
 *******************************************************************************
 * Hand-written for the bt_rail_soc_energy_harvesting_kinetic_switch example,
 * which does not ship a main.c even though its .slcp lists ../src/main.c.
 *
 * Differs from the stock Simplicity SDK sl_main skeleton in one respect:
 * this example's app_init() returns a RAIL_Handle_t, and app_process_action()
 * takes that handle as an argument. See inc/app.h.
 ******************************************************************************/
#include "sl_component_catalog.h"
#include "sl_main_init.h"
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
#include "sl_power_manager.h"
#endif
#if defined(SL_CATALOG_KERNEL_PRESENT)
#include "sl_main_kernel.h"
#else // SL_CATALOG_KERNEL_PRESENT
#include "sl_main_process_action.h"
#endif // SL_CATALOG_KERNEL_PRESENT

#include "rail.h"
#include "app.h"

int main(void)
{

  // Initialize Silicon Labs device, system, service(s) and protocol stack(s).
  // Note that if the kernel is present, the start task will be started and
  // software component initialization will take place there.
  sl_main_init();

#if defined(SL_CATALOG_KERNEL_PRESENT)
  sl_main_kernel_start();
#else // SL_CATALOG_KERNEL_PRESENT

  // User provided code.
  // app_init() returns the RAIL handle; app_process_action() needs it.
 app_init();

  while (1) {
    // Silicon Labs components process action routine
    // must be called from the super loop.
    sl_main_process_action();

    // User provided code. Application process.
    app_process_action();

#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
    // Let the CPU go to sleep if the system allows it.
    sl_power_manager_sleep();
#endif
  }
#endif // SL_CATALOG_KERNEL_PRESENT
}
