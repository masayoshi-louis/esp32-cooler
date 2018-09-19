#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_console.h"
#include "driver/uart.h"
#include "linenoise/linenoise.h"
#include "cmd_decl.h"
#include "../config.h"

#ifdef CONFIG_ENABLE_CLI

void setupCli()
{
    const uart_config_t uart_config = {
        baud_rate : CONFIG_CONSOLE_UART_BAUDRATE,
        data_bits : UART_DATA_8_BITS,
        parity : UART_PARITY_DISABLE,
        stop_bits : UART_STOP_BITS_1,
        flow_ctrl : UART_HW_FLOWCTRL_DISABLE,
        rx_flow_ctrl_thresh : 0,
        use_ref_tick : true
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));

    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));

    /* Initialize the console */
    const esp_console_config_t console_config = {
        max_cmdline_length : 256,
        max_cmdline_args : 8,
#if CONFIG_LOG_COLORS
        hint_color : atoi(LOG_COLOR_CYAN),
#else
        hint_color : 0,
#endif
        hint_bold : 0
    };
    ESP_ERROR_CHECK(esp_console_init(&console_config));

    /* Configure linenoise line completion library */
    /* Enable multiline editing. If not set, long commands will scroll within
     * single line.
     */
    linenoiseSetMultiLine(1);
    /* Tell linenoise where to get command completions and hints */
    linenoiseSetCompletionCallback(&esp_console_get_completion);
    linenoiseSetHintsCallback((linenoiseHintsCallback *)&esp_console_get_hint);
    /* Set command history size */
    linenoiseHistorySetMaxLen(100);

    esp_console_register_help_command();
    register_system();
    register_wifi();
}

#endif
