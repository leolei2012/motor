
#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

// <<< Use Configuration Wizard in Context Menu >>>

// <h>Debug Configuration
// <o>Debug UART output
//   <0=> Disabled
//   <1=> Enabled
// <i>Enable debug messages through the debug UART.
#define CONFIG_DEBUG_UART_ENABLE    1

// <o>Assert checks
//   <0=> Disabled
//   <1=> Enabled
// <i>Enable runtime assert checks.
#define CONFIG_ASSERT_ENABLE        1
// </h>

// <h>Firmware Version
// <o>Major version <0-255>
// <i>Firmware major version number.
#define FW_VERSION_MAJOR            0

// <o>Minor version <0-255>
// <i>Firmware minor version number.
#define FW_VERSION_MINOR            1

// <o>Patch version <0-255>
// <i>Firmware patch version number.
#define FW_VERSION_PATCH            0
// </h>

// <h>Clock Configuration
// <o>System clock frequency (Hz)
//   <170000000=> 170 MHz
//   <72000000=> 72 MHz
// <i>Keep this value aligned with the actual system clock setup.
#define CONFIG_SYSCLK_HZ            170000000UL
// </h>

// <h>RTOS Configuration
// <o>RT-Thread support
//   <0=> Disabled
//   <1=> Enabled
// <i>Select whether this project runs with RT-Thread.
#define CONFIG_RTOS_RTTHREAD        1
// </h>

// <h>Feature Modules
// <o>Watchdog module
//   <0=> Disabled
//   <1=> Enabled
// <i>Enable the watchdog feature module.
#define CONFIG_MODULE_WATCHDOG      1

// <o>WWDG counter value <0x40-0x7F>
// <i>WWDG reload counter value.
#define CONFIG_WWDG_COUNTER         0x7FU

// <o>WWDG window value <0x41-0x7F>
// <i>Refresh is only allowed after the counter drops below this value.
// <i>Current window: refresh when counter is 0x4F..0x40, about 24.6ms..32.8ms after reload at 64MHz PCLK.
#define CONFIG_WWDG_WINDOW          0x50U

// <o>WWDG feed task period (ms)
// <i>The main loop polls the hardware counter and refreshes only inside the valid window.
#define CONFIG_WWDG_FEED_PERIOD_MS  1U
// </h>

// <<< end of configuration section >>>

#endif /* PROJECT_CONFIG_H */
