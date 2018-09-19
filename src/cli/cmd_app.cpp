#include <stdlib.h>
#include <Arduino.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <esp_console.h>
#include <argtable3/argtable3.h>
#include <linenoise/linenoise.h>
#include "cmd_decl.h"

extern double temperatureSetpoint;
extern double humiditySetpoint;

void printStatus();

static struct
{
    struct arg_dbl *value;
    struct arg_end *end;
} set_th_args;

int setTemperature(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&set_th_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, set_th_args.end, argv[0]);
        return 1;
    }
    temperatureSetpoint = set_th_args.value->dval[0];
    return 0;
}

int setHumidity(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&set_th_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, set_th_args.end, argv[0]);
        return 1;
    }
    humiditySetpoint = set_th_args.value->dval[0];
    return 0;
}

int showStatus(int argc, char **argv)
{
    size_t uartBufferLen = 0;
    while (true)
    {
        uart_flush_input(UART_NUM_0);
        linenoiseClearScreen();
        printStatus();
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_0, &uartBufferLen));
        if (uartBufferLen)
        {
            return 0;
        }
        delay(500);
    }
}

void register_app()
{
    set_th_args.value = arg_dbl1(NULL, NULL, "<value>", "value");
    set_th_args.end = arg_end(2);

    const esp_console_cmd_t set_t_cmd = {
        .command = "set-t",
        .help = "Set target temperature",
        .hint = NULL,
        .func = &setTemperature,
        .argtable = &set_th_args};

    ESP_ERROR_CHECK(esp_console_cmd_register(&set_t_cmd));

    const esp_console_cmd_t set_h_cmd = {
        .command = "set-h",
        .help = "Set target humidity",
        .hint = NULL,
        .func = &setHumidity,
        .argtable = &set_th_args};

    ESP_ERROR_CHECK(esp_console_cmd_register(&set_h_cmd));

    const esp_console_cmd_t show_status_cmd = {
        .command = "status",
        .help = "Watch status",
        .hint = NULL,
        .func = &showStatus,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&show_status_cmd));
}
