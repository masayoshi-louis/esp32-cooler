#include <sdkconfig.h>
#include <esp_system.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_console.h>
#include <driver/uart.h>
#include <linenoise/linenoise.h>
#include "cmd_decl.h"
#include "../config.h"

#ifdef CONFIG_ENABLE_CLI

void cliLoopTask(void *)
{
    /* Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */
    const char *prompt = LOG_COLOR_I "esp32> " LOG_RESET_COLOR;

    /* Figure out if the terminal supports escape sequences */
    int probe_status = linenoiseProbe();
    if (probe_status)
    { /* zero indicates success */
        printf("\n"
               "Your terminal application does not support escape sequences.\n"
               "Line editing and history features are disabled.\n"
               "On Windows, try using Putty instead.\n");
        linenoiseSetDumbMode(1);
#if CONFIG_LOG_COLORS
        /* Since the terminal doesn't support escape sequences,
         * don't use color codes in the prompt.
         */
        prompt = "esp32> ";
#endif //CONFIG_LOG_COLORS
    }

    /* Main loop */
    while (true)
    {
        /* Get a line using linenoise.
         * The line is returned when ENTER is pressed.
         */
        char *line = linenoise(prompt);
        if (line == NULL)
        { /* Ignore empty lines */
            continue;
        }
        /* Add the command to the history */
        linenoiseHistoryAdd(line);
#if CONFIG_STORE_HISTORY
        /* Save command history to filesystem */
        linenoiseHistorySave(HISTORY_PATH);
#endif

        /* Try to run the command */
        int ret;
        esp_err_t err = esp_console_run(line, &ret);
        if (err == ESP_ERR_NOT_FOUND)
        {
            printf("Unrecognized command\n");
        }
        else if (err == ESP_ERR_INVALID_ARG)
        {
            // command was empty
        }
        else if (err == ESP_OK && ret != ESP_OK)
        {
            printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(err));
        }
        else if (err != ESP_OK)
        {
            printf("Internal error: %s\n", esp_err_to_name(err));
        }
        /* linenoise allocates line buffer on the heap, so need to free it */
        linenoiseFree(line);
    }
}

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
    register_app();

    xTaskCreate(cliLoopTask, "cli", configMINIMAL_STACK_SIZE, NULL, 0, NULL);
}

#endif
