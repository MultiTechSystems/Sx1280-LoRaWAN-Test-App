#include "commands.h"

ConfigManager config_mng;
DeviceConfig_t device_config;

BufferedSerial pc(USBTX, USBRX);
char ser_buf[32] = {0};

void wait_for_command() {
    bool cmd_mode = false;

    // Timer tm;
    // tm.start();

    // printf("Press a key to enter command mode\r\n");

    // while (tm.read_ms() < 1000) {
    //     if (pc.readable()) {
    //         while (pc.readable()) {
    //             pc.getc();
    //         }
    //         cmd_mode = true;
    //         break;
    //     }
    // }

    cmd_mode = true;

    if (cmd_mode) {
        tinyshell_thread();
    }
}

/**
 * Entry point for application
 */
int main(void)
{
    // Sensor Inputs
    // Digital Pins D3, D5, D7, D11, D12, D13 used by the Radio
    // (A0 used for Radio Reset, A3 used for Antenna Switch)

    config_mng.Load(device_config);

    pc.set_baud(115200);

    printf("\r\nInitializing...\r\n");

    InitApplication();
    SetupRadio();

    printf("RTC Time: %lu seconds\r\n", time(NULL));

    wait_for_command();
    
    return 0;
}
