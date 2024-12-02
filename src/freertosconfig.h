// FreeRTOSConfig.h

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

// The following definitions are required by FreeRTOS
#define configUSE_PREEMPTION        1   // Use preemption (1 = enabled)
#define configUSE_IDLE_HOOK         0   // Don't use the idle hook
#define configUSE_TICK_HOOK         0   // Don't use the tick hook
#define configUSE_16_BIT_TICKS      0   // Use 32-bit ticks (common for ESP32)
#define configMAX_PRIORITIES        5   // Max priority level for tasks (adjust as needed)
#define configMINIMAL_STACK_SIZE    128 // Minimum stack size for the idle task (in words)
#define configSUPPORT_STATIC_ALLOCATION    1
#define CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ 240
// Define the system tick rate (Hz). For 1 ms ticks, use 1000 Hz.
#define configTICK_RATE_HZ          8000
// #define configGENERATE_RUN_TIME_STATS 1
// #define configUSE_TRACE_FACILITY 1
// #define configUSE_STATS_FORMATTING_FUNCTIONS 1

// // Define the heap memory management strategy (e.g., heap_4, heap_5, etc.)
// #define configFRTOS_MEMORY_SCHEME   4  // Dynamic memory allocation scheme

#endif // FREERTOS_CONFIG_H
