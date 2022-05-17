/**********************************************************************
* COPYRIGHT 2015 MULTI-TECH SYSTEMS, INC.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of MULTI-TECH SYSTEMS, INC. nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

/*

$ mbed compile -t GCC_ARM -m MAX32670_LORA

*/

#include "commands.h"
#include "lorawan_types.h"
#include <inttypes.h>

extern BufferedSerial pc;
extern char ser_buf;
extern DeviceConfig_t device_config;
extern ConfigManager config_mng;

bool exit_cmd_mode = false;

static char prompt[] = "$ ";
static char ok_str[] = "\r\nOK\r\n";
// static char error_str[] = "\r\nerror\r\n";
static char invalid_args_str[] = "\r\ninvalid args\r\n";

tinysh_cmd_t reset_cmd = { 0, "reset", "reset command", "", reset_func, 0, 0, 0 };

tinysh_cmd_t deveui_cmd = { 0, "deveui", "deveui command", "", deveui_func, 0, 0, 0 };
tinysh_cmd_t appeui_cmd = { 0, "appeui", "appeui command", "", appeui_func, 0, 0, 0 };
tinysh_cmd_t appkey_cmd = { 0, "appkey", "appkey command", "", appkey_func, 0, 0, 0 };

tinysh_cmd_t devaddr_cmd = { 0, "devaddr", "devaddr command", "", devaddr_func, 0, 0, 0 };
tinysh_cmd_t nwkskey_cmd = { 0, "nwkskey", "nwkskey command", "", nwkskey_func, 0, 0, 0 };
tinysh_cmd_t appskey_cmd = { 0, "appskey", "appskey command", "", appskey_func, 0, 0, 0 };

tinysh_cmd_t ucnt_cmd = { 0, "ucnt", "uplink count command", "", ucnt_func, 0, 0, 0 };
tinysh_cmd_t dcnt_cmd = { 0, "dcnt", "downlink count command", "", dcnt_func, 0, 0, 0 };

tinysh_cmd_t join_cmd = { 0, "join", "join network", "", join_func, 0, 0, 0 };
tinysh_cmd_t send_cmd = { 0, "send", "send packet", "<PAYLOAD>", send_func, 0, 0, 0 };
tinysh_cmd_t sendi_cmd = { 0, "sendi", "send packet", "<TIMEOUT> <PACKETS> <PAYLOAD>", sendi_func, 0, 0, 0 };
tinysh_cmd_t recv_cmd = { 0, "recv", "recv packet", "<TIMEOUT>", recv_func, 0, 0, 0 };

tinysh_cmd_t channels_cmd = { 0, "channels", "list channels", "", channels_func, 0, 0, 0 };
tinysh_cmd_t channel_index_cmd = { 0, "index", "current channel index", "0-2,-1", channel_index_func, 0, 0, 0 };
tinysh_cmd_t datarate_cmd = { 0, "datarate", "datarate setting", "0-15", datarate_func, 0, 0, 0 };
tinysh_cmd_t power_cmd = { 0, "power", "power setting", "0-15", power_func, 0, 0, 0 };

tinysh_cmd_t app_port_cmd = { 0, "port", "application port", "0-255", app_port_func, 0, 0, 0 };


#define RX_TIMEOUT_MS 500
#define RX_EARLY_MS 20
#define PKT_FCTRL 5
#define PKT_PORT 8
#define PKT_PAYLOAD 9
#define PKT_MIC_LEN 4

#define PKT_JOIN_ACCEPT_APP_NONCE 1
#define PKT_JOIN_ACCEPT_NET_ID 4
#define PKT_JOIN_ACCEPT_DEV_ADDR 7


bool joined = false;
bool rx_mac_cmds = false;
int tx_mac_cmd_size = 0;

int rx_1_time_out = 5000;
int payload_size = 3;

uint32_t uplink_frame_counter = 0;
uint8_t join_request[23];
uint8_t tx_mac_cmd_buffer[20];


/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( const uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( void );


/*!
 * \brief All the callbacks are stored in a structure
 */
radio_events_t Callbacks =
{
    .tx_done                = &OnTxDone,
    .tx_timeout             = &OnTxTimeout,      
    .rx_done                = &OnRxDone,           
    .rx_timeout             = &OnRxTimeout,    
    .rx_error               = &OnRxError,       
    .fhss_change_channel    = NULL,
    .cad_done               = NULL,
};

/*!
 * \brief Define IO and callbacks for radio
 * mosi, miso, sclk, nss, busy, dio1, dio2, dio3, rst, callbacks
 */
//SX1280Hal Radio( D11, D12, D13, D7, D3, D5, NC, NC, A0, &Callbacks );

SX126X_LoRaRadio Radio(MBED_CONF_SX126X_LORA_DRIVER_SPI_MOSI,
                       MBED_CONF_SX126X_LORA_DRIVER_SPI_MISO,
                       MBED_CONF_SX126X_LORA_DRIVER_SPI_SCLK,
                       MBED_CONF_SX126X_LORA_DRIVER_SPI_CS,
                       MBED_CONF_SX126X_LORA_DRIVER_RESET,
                       MBED_CONF_SX126X_LORA_DRIVER_DIO1,
                       MBED_CONF_SX126X_LORA_DRIVER_BUSY,
                       MBED_CONF_SX126X_LORA_DRIVER_FREQ_SELECT,
                       MBED_CONF_SX126X_LORA_DRIVER_DEVICE_SELECT,
                       MBED_CONF_SX126X_LORA_DRIVER_CRYSTAL_SELECT,
                       MBED_CONF_SX126X_LORA_DRIVER_ANT_SWITCH);

/*!
 * \brief Control the Antenna Diversity switch
 */
// DigitalOut ANT_SW( A3 );

/*!
 * \brief Tx LED toggling on transmition success
 */
bool TX_LED;

/*!
 * \brief Rx LED toggling on reception success
 */
bool RX_LED;

/*!
 * \brief Mask of IRQs
 */
uint16_t IrqMask = 0x0000;

/*!
 * \brief Locals parameters and status for radio API
 * NEED TO BE OPTIMIZED, COPY OF STUCTURE ALREADY EXISTING
 */
packet_params_t PacketParams;
packet_status_t PacketStatus;
modulation_params_t ModulationParams;

// void SetAntennaSwitch( int );
void LedBlink( void );

#define RX_TIMEOUT_TICK_SIZE            RADIO_TICK_SIZE_1000_US
#define BUFFER_SIZE                     255

lora::Crypto crypto;
uint8_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];
uint8_t RxBuffer[BUFFER_SIZE];
uint8_t RxBufferSize = 0;
uint8_t TxBufferSize = 0;
int16_t RxRssi = 0;
bool packet_rxd = false;

uint8_t rx_1_datarate = 0;
uint8_t rx_1_offset = 0;
uint8_t rx_2_datarate = 0;

void InitApplication( void )
{
    // RX_LED = 1;
    // TX_LED = 1;

    // SetAntennaSwitch( 1 );

    ThisThread::sleep_for( 500 ); // wait for on board DC/DC start-up time
    
    Radio.lock();
    Radio.init_radio( &Callbacks );
    Radio.standby();
    Radio.set_public_network(true);
    Radio.unlock();

    // Can also be set in LDO mode but consume more power
    // Radio Power Mode [0: LDO, 1:DC_DC]
    // Radio.SetRegulatorMode( ( RadioRegulatorModes_t ) 1 );
    
    memset( &Buffer, 0x00, BufferSize );

    RX_LED = 0;
    TX_LED = 0;
}

void LedBlink( void )
{
    if( ( TX_LED == 0 ) && ( RX_LED == 0 ) )
    {
        TX_LED = 1;
    }
    else if( ( TX_LED == 1 ) && ( RX_LED == 0 ) )
    {
        RX_LED = 1;
    }
    else if( ( TX_LED == 1 ) && ( RX_LED == 1 ) )
    {
        TX_LED = 0;
    }
    else
    {
        RX_LED = 0;
    }
}

// void SetAntennaSwitch( int on )
// {
//     if( on == 1 )
//     {
//         ANT_SW = 1; // ANT1
//     }
//     else
//     {
//         ANT_SW = 0; // ANT0
//     }
// }

void OnTxDone( void )
{
    TX_LED = 0;
    printf("TxDone\r\n");
}

void OnRxDone( const uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    printf("OnRxDone\r\n");
    RxRssi = rssi;
    RxBufferSize = size;
    for( uint8_t i = 0; i < size; i++ )
    {
        RxBuffer[i] = payload[i];
    }
    packet_rxd = true;
    RX_LED = 0;
}

void OnTxTimeout( void )
{
    printf("txtimeout\r\n");
    TX_LED = 0;
}

void OnRxTimeout( void )
{
    printf("OnRxTimeout\r\n");
    RX_LED = 0;
}

void OnRxError( void )
{
    printf("OnRxError\r\n");
    RX_LED = 0;
}

// void OnRangingDone( IrqRangingCode_t val )
// {
//     // printf("OnRangingDone\r\n");
// }

void OnCadDone( bool channelActivityDetected )
{
    // printf("OnCadDone\r\n");
}

int tx_channel_index = -1;

uint32_t tx_channels[] = { 902300000U, 
                            902500000U, 
                            902700000U,
                            902900000U, 
                            903100000U, 
                            903300000U, 
                            903500000U, 
                            903700000U};

uint32_t rx1_channels[] = { 923300000U,
                            923900000U,
                            924500000U,
                            925100000U,
                            925700000U,
                            926300000U,
                            926900000U,
                            927500000U };

uint32_t rx2_channel = 923300000U; // SF12 BW500


void SetupRadio() {
    // with how the mbed driver works, this section seems unnecessary
    ModulationParams.modem_type = MODEM_LORA;
    PacketParams.modem_type = MODEM_LORA;
    ModulationParams.params.lora.spreading_factor = LORA_SF12;
    ModulationParams.params.lora.bandwidth = LORA_BW_500; //LORA_BW_0800; <- original value, not sure what 0800 corresponds to.
    ModulationParams.params.lora.coding_rate = LORA_CR_4_8;
    PacketParams.params.lora.preamble_length = 8;
    PacketParams.params.lora.header_type = LORA_PACKET_EXPLICIT;
    PacketParams.params.lora.payload_length = 23;
    PacketParams.params.lora.crc_mode = LORA_CRC_ON;
    PacketParams.params.lora.invert_IQ = LORA_IQ_NORMAL;
    // These functions are private for SX126x driver.. 
    // Radio.set_modem( ModulationParams.modem_type );
    // Radio.set_modulation_params( &ModulationParams );
    // Radio.set_packet_params( &PacketParams );

    // set_tx_power would be a suitable replacement, but the function is private
    // Radio.SetTxParams( 10, RADIO_RAMP_20_US );
    // Don't want polling mode
    // Radio.SetPollingMode();

    memset(join_request, 0, 23);
    memset(tx_mac_cmd_buffer, 0, 20);
}

void FillJoinRequest(uint8_t* buffer, uint16_t nonce, uint8_t* key) {
    uint32_t mic = 0;

    buffer[0] = 0x00;   // MAC Header

    for (int i = 0; i < 8; ++i) {
        buffer[8-i] = device_config.settings.AppEUI[i];
        buffer[16-i] = device_config.provisioning.DeviceEUI[i];
    }

    buffer[17] = nonce;
    buffer[18] = nonce >> 8;

    crypto.JoinComputeMic(buffer, 19, key, &mic);
    crypto.CopyMicToArray(mic, buffer+19);
}

void ChooseRandomChannel() {
    int i = rand() % 8;

    // if (tx_channel_index != -1 && tx_channel_index < 8)
    //     i = tx_channel_index;

    tx_channel_index = i;

    printf("Using channel %u : %lu\r\n", i, tx_channels[i]);
    Radio.lock();
    Radio.set_channel( tx_channels[i] );
    Radio.unlock();
}

void SetTxMode() {
    Radio.lock();
    Radio.radio_reset();
    Radio.set_tx_config(MODEM_LORA,            // Modem
                        22,                    // TxPower (dbm)
                        0,                     // fdev
                        0,//LORA_BW_125,       // Bandwidth
                        0x08,//LORA_SF10,      // Datarate
                        0x04,//LORA_CR_4_8,    // Code Rate
                        8,                     // Preamble Len
                        false,                 // FixLen?
                        true,                  // CRC?
                        false,                 // FreqHop?
                        0,                     // Hop Period
                        LORA_IQ_NORMAL,        // IQ inverted?
                        6000);                 // Timeout
    Radio.set_public_network(true);
    Radio.unlock();
    // PacketParams.params.lora.crc_mode = LORA_CRC_ON;
    // PacketParams.params.lora.invert_IQ = LORA_IQ_NORMAL;
    // Radio.SetPacketParams( &PacketParams );

    // ModulationParams.params.lora.SpreadingFactor = static_cast<RadioLoRaSpreadingFactors_t>((12) << 4);
    // Radio.SetModulationParams( &ModulationParams );

    // Dio Irq is configured in Radio.send and Radio.receive
    //IrqMask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;
    //Radio.SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    TX_LED = 1;
    packet_rxd = false;
}

void SetRxMode(uint32_t freq, uint8_t dr) {
    RX_LED = 1;
    Radio.lock();
    Radio.set_channel( freq );
    Radio.set_rx_config(MODEM_LORA,     // Modem
                        2,              // Bandwidth    // 2 is 500kHz
                        LORA_SF8,       // DataRate     // should probably use dr
                        1,              // Coderate     //'1' is just copied from mbed loraphy //LORA_CR_4_8,
                        0,              // Bandwidth AFC
                        8,              // Preamble Length
                        500,            // RxSingle Timeout (symbol)
                        false,          // FixLen?
                        0,              // PayloadLen (when FixLen == True)
                        false,          // CrcOn?
                        false,          // FreqHopOn?
                        0,              // Hop Period
                        true,           // IQ inverted?
                        true);          // RxContinuous?

    Radio.receive();
    Radio.unlock();
}

void ProcessRadioEvents(int32_t timeout) {
    // Timer tm;
    // tm.start();

    // while (tm.read_ms() < timeout && !packet_rxd) {
    //     Radio.ProcessIrqs();
    //     ThisThread::sleep_for(10);
    // }
}

void GetJoinRxPacketInfo(uint8_t* key) {
    // Radio.GetPayload( RxBuffer, &RxBufferSize, BUFFER_SIZE );
    // Radio.GetPacketStatus( &PacketStatus );

    // while(RX_LED == 1);

    printf("Rx %d Bytes Rssi: %d\r\n", RxBufferSize, /*PacketStatus.LoRa.SnrPkt,*/ RxRssi);
    printf("Encrypted RxPayload: ");
    for (int i = 0; i < RxBufferSize; ++i) {
        printf("%02X", RxBuffer[i]);
    }
    printf("\r\n");

    crypto.JoinDecrypt(RxBuffer+1, RxBufferSize-1, key, RxBuffer+1);

    printf("Decrypted RxPayload: ");
    for (int i = 0; i < RxBufferSize; ++i) {
        printf("%02X", RxBuffer[i]);
    }
    printf("\r\n");
}

uint16_t GetRxPacketInfo() {

    // Radio.GetPayload( RxBuffer, &RxBufferSize, BUFFER_SIZE );
    // Radio.GetPacketStatus( &PacketStatus );

    while(RX_LED == 1);

    uint16_t fcnt = RxBuffer[7] << 8 | RxBuffer[6];

    printf("Rx %d Bytes Rssi: %d FCNT: %u\r\n", RxBufferSize, /*PacketStatus.LoRa.SnrPkt,*/ RxRssi, fcnt);
    printf("RxPayload: ");
    for (int i = 0; i < RxBufferSize; ++i) {
        printf("%02X", RxBuffer[i]);
    }
    printf("\r\n");

    return fcnt;
}

uint32_t read_u32(uint8_t* buff) {
    return buff[0] << 24 
        | buff[1] << 16 
        | buff[2] << 8 
        | buff[3] << 0;
}

uint32_t read_u32_r(uint8_t* buff) {
    return buff[3] << 24 
        | buff[2] << 16 
        | buff[1] << 8 
        | buff[0] << 0;
}

void copy_u32_r(uint8_t* dst, uint32_t src) {
    dst[0] = src & 0xFF;
    dst[1] = src >> 8;
    dst[2] = src >> 16;
    dst[3] = src >> 24;
}

bool CheckJoinMic(uint8_t* key) {
    uint32_t mic = 0;
    crypto.JoinComputeMic(RxBuffer, RxBufferSize-PKT_MIC_LEN, key, &mic);

    uint32_t rx_mic = read_u32_r(RxBuffer + RxBufferSize-PKT_MIC_LEN);

    printf("Join MIC: %lu RXMIC: %lu\r\n", mic, rx_mic);

    return mic == rx_mic;
}

bool CheckMic(uint8_t* key, uint32_t counter) {
    uint32_t mic = 0;
    // void Crypto::ComputeMic(uint8_t *buffer, uint16_t size, uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint32_t *mic)
    crypto.ComputeMic(RxBuffer, RxBufferSize-PKT_MIC_LEN, key, device_config.session.NetworkAddress, 1, counter, &mic);

    uint32_t rx_mic = read_u32_r(RxBuffer + RxBufferSize-PKT_MIC_LEN);

    printf("Pkt MIC: %lu RXMIC: %lu\r\n", mic, rx_mic);

    return mic == rx_mic;
}

int UnpackJoinAccept(uint8_t* key, uint16_t nonce) {

    int rx_1_time_out = 1000;

    // Join Accept 17 or 33 bytes
    //    ANONCE NETID  DEVADDR  DR RX1 MIC
    // 0  1 2 3  4 5 6  7 8 9 10 11 12  13    16
    // 20 6E0000 130000 0E000027 00 05  3530970C
    //
    //    ANONCE NETID  DEVADDR  DR RX1 ADDITIONAL CHANNELS              MIC
    // 0  1 2 3  4 5 6  7 8 9 10 11 12  13                            28 29    32
    // 20 6E0000 130000 0E000027 00 05  00000000000000000000000000000000 3530970C

    uint8_t* appNonce = RxBuffer+PKT_JOIN_ACCEPT_APP_NONCE;
    uint8_t* netID = RxBuffer+PKT_JOIN_ACCEPT_NET_ID;
    device_config.session.NetworkAddress = read_u32_r(RxBuffer +PKT_JOIN_ACCEPT_DEV_ADDR);

    crypto.DeriveSessionKeys(device_config.settings.AppKey, appNonce, netID, nonce, device_config.session.NetworkKey, device_config.session.DataKey);

    printf("Dev Addr: %08lX\r\n", device_config.session.NetworkAddress);

    printf("Nwk SKey: ");
    for (int i = 0; i < 16; ++i) {
        printf("%02X", device_config.session.NetworkKey[i]);
    }
    printf("\r\n");

    printf("App SKey: ");
    for (int i = 0; i < 16; ++i) {
        printf("%02X", device_config.session.DataKey[i]);
    }
    printf("\r\n");

    // Rx Parameters
    // Rx1 DR Offset
    rx_1_offset = RxBuffer[11] >> 4;
    // Rx2 DR
    rx_2_datarate = RxBuffer[11] & 0x0F;

    // Rx1 Delay
    rx_1_time_out = RxBuffer[12] * 1000U;
    if (rx_1_time_out == 0) rx_1_time_out = 1000U;
    printf("Rx1 Delay: %d\r\n", rx_1_time_out);

    if (RxBufferSize > 17) {
        // Additional Channels
    }

    return rx_1_time_out;
}


void InitTxPacket(bool confirmed, uint8_t port) {

    if(confirmed)
        Buffer[0] = 0x80; // Confirmed Uplink
    else
        Buffer[0] = 0x40; // Unconfirmed Uplink

    copy_u32_r(Buffer + 1, device_config.session.NetworkAddress);

    Buffer[5] = 0x00; // FCtrl
    Buffer[6] = 0;    // FCnt
    Buffer[7] = 0;    // FCnt

    Buffer[8] = port; // Port

}

void EncryptPacket(uint8_t payload_size, uint8_t tx_mac_cmd_size, uint8_t port, uint32_t counter) {
    uint8_t localPayloadSize = 13 + tx_mac_cmd_size + payload_size;
    uint32_t mic = 0;

    printf("TxPacket DEC: ");
    for (int i = 0; i < localPayloadSize; ++i) {
        printf("%02X", Buffer[i]);
    }
    printf("\r\n");

    if (port == 0) {
        crypto.PayloadEncrypt(Buffer + 9, payload_size, device_config.session.NetworkKey, device_config.session.NetworkAddress, 0, counter, Buffer + 9);
    } else {
        crypto.PayloadEncrypt(Buffer + 9 + tx_mac_cmd_size, payload_size, device_config.session.DataKey, device_config.session.NetworkAddress, 0, counter, Buffer + 9 + tx_mac_cmd_size);
    }

    crypto.ComputeMic(Buffer, localPayloadSize - 4, device_config.session.NetworkKey, device_config.session.NetworkAddress, 0, counter, &mic);
    crypto.CopyMicToArray(mic, Buffer + (localPayloadSize - 4));

    printf("TxPacket ENC: ");
    for (int i = 0; i < localPayloadSize; ++i) {
        printf("%02X", Buffer[i]);
    }
    printf("\r\n");

    //PacketParams.params.lora.PayloadLength = localPayloadSize;
    TxBufferSize = localPayloadSize;
}

uint8_t DecryptPayload(uint8_t* mac_cmd_buffer) {
    uint8_t mac_cmd_size = 0;

    if (RxBufferSize > 12 && ((RxBuffer[5] & 0x0F) == 0) && RxBuffer[8] == 0x00) {
        crypto.PayloadDecrypt(RxBuffer + 9, RxBufferSize - 13, device_config.session.NetworkKey, device_config.session.NetworkAddress, 1, RxBuffer[7] << 8 | RxBuffer[6], RxBuffer + 9);
        mac_cmd_size = RxBufferSize - 13;
        for (int i = 0; i < mac_cmd_size; ++i) {
            mac_cmd_buffer[i] = *(RxBuffer + 9 + i);
        }

    } else {

        if (RxBufferSize > 12 && ((RxBuffer[5] & 0x0F) > 0)) {
            mac_cmd_size = (RxBuffer[5] & 0x0F);
            for (int i = 0; i < mac_cmd_size; ++i) {
                mac_cmd_buffer[i] = *(RxBuffer + 8 + i);
            }
        }

        crypto.PayloadDecrypt(RxBuffer + 9, RxBufferSize - 13, device_config.session.DataKey, device_config.session.NetworkAddress, 1, RxBuffer[7] << 8 | RxBuffer[6], RxBuffer + 9);
    }

    return mac_cmd_size;
}





void reset_func(int argc, char **argv) {
    //HAL_NVIC_SystemReset();
}

void run_func(int argc, char **argv) {
    exit_cmd_mode = true;
}

int read_hex_str(const char* input, uint8_t* val, size_t length) {
    unsigned char tmp = 0;
    for (size_t i = 0, j = 0; i < length; ++i, j+=2) {
        if (sscanf(input+j, "%2hhx", &tmp)) {
            val[i] = tmp;
        } else {
            return -1;
        }
    }
    return 0;
}

void print_hex_str(const uint8_t* val, size_t length) {
    printf("\r\n");
    for (size_t i = 0; i < length; ++i) {
        printf("%02x", val[i]);
    }
    printf("\r\n");
}

void deveui_func(int argc, char **argv) {
    if (argc == 1) {
        print_hex_str(device_config.provisioning.DeviceEUI, 8);
        printf("\r\n");
    } else if (argc == 2 && (strlen(argv[1]) == 16)) {
        read_hex_str(argv[1], device_config.provisioning.DeviceEUI, 8);
        printf(ok_str);
    } else {
        printf(invalid_args_str);
    }
}

void appeui_func(int argc, char **argv) {
    if (argc == 1) {
        print_hex_str(device_config.settings.AppEUI, 8);
        printf("\r\n");
    } else if (argc == 2 && (strlen(argv[1]) == 16)) {
        read_hex_str(argv[1], device_config.settings.AppEUI, 8);
        printf(ok_str);
    } else {
        printf(invalid_args_str);
    }
}

void devaddr_func(int argc, char **argv) {
    if (argc == 1) {
        print_hex_str((uint8_t*)&device_config.session.NetworkAddress, 4);
        printf("\r\n");
    } else if (argc == 2 && (strlen(argv[1]) == 8)) {
        read_hex_str(argv[1], (uint8_t*)&device_config.session.NetworkAddress, 4);
        printf(ok_str);
    } else {
        printf(invalid_args_str);
    }
}

void appskey_func(int argc, char **argv) {
    if (argc == 1) {
        print_hex_str(device_config.session.DataKey, 16);
        printf("\r\n");
    } else if (argc == 2 && (strlen(argv[1]) == 32)) {
        read_hex_str(argv[1], device_config.session.DataKey, 16);
        printf(ok_str);
    } else {
        printf(invalid_args_str);
    }
}

void nwkskey_func(int argc, char **argv) {
    if (argc == 1) {
        print_hex_str(device_config.session.NetworkKey, 16);
        printf("\r\n");
    } else if (argc == 2 && (strlen(argv[1]) == 32)) {
        read_hex_str(argv[1], device_config.session.NetworkKey, 16);
        printf(ok_str);
    } else {
        printf(invalid_args_str);
    }
}

void appkey_func(int argc, char **argv) {
    if (argc == 1) {
        print_hex_str(device_config.settings.AppKey, 16);
        printf("\r\n");
    } else if (argc == 2 && (strlen(argv[1]) == 32)) {
        read_hex_str(argv[1], device_config.settings.AppKey, 16);
        printf(ok_str);
    } else {
        printf(invalid_args_str);
    }
}

void ack_retries_func(int argc, char **argv) {
    if (argc == 1) {
        printf("\r\n%u\r\n", device_config.settings.ACKAttempts);
    } else if (argc == 2 && (strlen(argv[1]) == 1)) {
        read_hex_str(argv[1], &device_config.settings.ACKAttempts, 1);
        printf(ok_str);
    } else {
        printf(invalid_args_str);
    }
}

void datarate_func(int argc, char **argv) {
    if (argc == 1) {
        printf("\r\nDR%u\r\n", device_config.settings.TxDataRate);
    } else if (argc == 2 && (strlen(argv[1]) == 1)) {
        int val = 1;
        if (sscanf(argv[1], "%d", &val) && val >= 0 && val <= 15) {
            device_config.settings.TxDataRate = val;
            printf(ok_str);
        } else {
            printf(invalid_args_str);
        }
    } else {
        printf(invalid_args_str);
    }
}

void app_port_func(int argc, char **argv) {
    if (argc == 1) {
        printf("\r\n%d\r\n", device_config.settings.Port);
    } else if (argc == 2) {
        int val = 1;
        if (sscanf(argv[1], "%d", &val) && val >= 0 && val <= 255) {
            device_config.settings.Port = val;
            printf(ok_str);
        } else {
            printf(invalid_args_str);
        }
    } else {
        printf(invalid_args_str);
    }
}

void ucnt_func(int argc, char **argv) {
    if (argc == 1) {
        printf("\r\n%lu\r\n", device_config.session.UplinkCounter);
    } else if (argc == 2) {
        uint32_t val = 0;
        if (sscanf(argv[1], "%" SCNu32, &val)) {
            device_config.session.UplinkCounter = val;
            printf(ok_str);
        } else {
            printf(invalid_args_str);
        }
    } else {
        printf(invalid_args_str);
    }
}

void dcnt_func(int argc, char **argv) {
    if (argc == 1) {
        printf("\r\n%lu\r\n", device_config.session.DownlinkCounter);
    } else if (argc == 2) {
        uint32_t val = 0;
        if (sscanf(argv[1], "%" SCNu32, &val)) {
            device_config.session.DownlinkCounter = val;
            printf(ok_str);
        } else {
            printf(invalid_args_str);
        }
    } else {
        printf(invalid_args_str);
    }
}

void power_func(int argc, char **argv) {
    if (argc == 1) {
        printf("\r\n%d\r\n", device_config.settings.TxPower);
    } else if (argc == 2) {
        int val = 1;
        if (sscanf(argv[1], "%d", &val)) {
            device_config.settings.TxPower = val;
            // Radio.SetTxParams( device_config.settings.TxPower, RADIO_RAMP_20_US );
            printf(ok_str);
        } else {
            printf(invalid_args_str);
        }
    } else {
        printf(invalid_args_str);
    }
}


void channels_func(int argc, char **argv) {
    if (argc == 1) {
        for (int i = 0; i < 3; i++) {
            printf("\r\nchan %d : %lu", i, tx_channels[i]);         
        }
        printf(ok_str);
    } else {
        printf(invalid_args_str);
    }
}


void channel_index_func(int argc, char **argv) {
    if (argc == 1) {
        printf("\r\n%d\r\n", tx_channel_index);        
    } else if (argc == 2) {
        int val = 1;
        if (sscanf(argv[1], "%d", &val) && val >= -1 && val < 3) {
            tx_channel_index = val;
            printf(ok_str);
        } else {
            printf(invalid_args_str);
        }
    } else {
        printf(invalid_args_str);
    }
}

void join_func(int argc, char **argv) {

    printf(ok_str);

    static uint32_t nonce = time(NULL) + 26500;

    rx_1_time_out = 5000;
    rx_2_datarate = 0;
    
    printf("DevNonce: %lu, 0x%02X\r\n",  nonce, nonce);
    
    FillJoinRequest(join_request, nonce, device_config.settings.AppKey);

    printf("JoinRequest: |");
    for (int i = 0; i < 23; i++) {
        if( i == 1 || i == 9 || i == 17 || i == 19) printf("|");
        printf("%02X", join_request[i]);
    }
    printf("|\r\n");

    printf("----MAC HDR:  ");
    printf("%02X", join_request[0]);

    printf("\r\n----JoinEUI:  ");
    for (int i = 1; i < 9; i++) {
        printf("%02X", join_request[i]);
    }

    printf("\r\n----DevEUI:   ");
    for (int i = 9; i < 17; i++) {
        printf("%02X", join_request[i]);
    }

    printf("\r\n----DevNonce: ");
    for (int i = 17; i < 19; i++) {
        printf("%02X", join_request[i]);
    }
    printf("\r\n");

    printf("----MIC:      ");
    for (int i = 19; i < 23; i++) {
        printf("%02X", join_request[i]);
    }
    printf("\r\n");

    SetTxMode();
    ChooseRandomChannel();

    printf("SendPayload\r\n");
    //Radio.SendPayload( join_request, 23, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, 2000 } );
    Radio.lock();
    Radio.set_max_payload_length(MODEM_LORA, 23);
    Radio.send( join_request, 23 );
    Radio.unlock();
    TX_LED = 1;

    // Wait for TxDone
    printf("Waiting for TxDone\r\n");
    while (TX_LED) {
        ThisThread::sleep_for(1); // wait for txdone event
        //ProcessRadioEvents(1);
    }
    //ProcessRadioEvents(rx_1_time_out - RX_EARLY_MS);

    // Join Delay
    ThisThread::sleep_for(4500);
    
    // RX 1
    printf("Open Rx1..\r\n");
    printf("Rx1 Channel = %lu\r\n", rx1_channels[tx_channel_index]);
    SetRxMode(rx1_channels[tx_channel_index], rx_1_datarate);    // may need to modify SetRxMode to include timeout.
    while(RX_LED) {
        ThisThread::sleep_for(50);
    }
    //Radio.SetRx(( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_MS });

    ProcessRadioEvents(995);

    if (!packet_rxd) {
        // RX 2
        printf("Open Rx2\r\n");
        SetRxMode(rx2_channel, rx_2_datarate);
        //Radio.SetRx(( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_MS });
        ProcessRadioEvents(1000);
    }

    if (packet_rxd) {
        GetJoinRxPacketInfo(device_config.settings.AppKey);

        if (CheckJoinMic(device_config.settings.AppKey)) {
            rx_1_time_out = UnpackJoinAccept(device_config.settings.AppKey, nonce);
            joined = true;            
        }
    }

    nonce++;
    
}

void recv_func(int argc, char **argv) {

    int timeout = 3000;
    int packets = 1;

    if (argc == 1) {
        // use default timeout
    } else if (argc >= 2) {
        if (sscanf(argv[1], "%d", &timeout)) {
            printf(ok_str);
        } else {
            printf(invalid_args_str);
        }
        if (argc > 2) {
            if (sscanf(argv[2], "%d", &packets)) {
                printf(ok_str);
            } else {
                printf(invalid_args_str);
            }
        }
    } else {
        printf(invalid_args_str);
        return;
    }

    uint32_t freq = rx2_channel;

    if (tx_channel_index != -1)
        freq = tx_channels[tx_channel_index];

    
    do {
        packet_rxd = false;

        printf("Open Rx Window %lu MHz DR%d\r\n", freq, device_config.settings.TxDataRate);
        SetRxMode(freq, device_config.settings.TxDataRate);

        uint16_t rx_timeout = (uint16_t)(timeout > 0 ? timeout : 30000);
        //Radio.SetRx(( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, rx_timeout });
        ProcessRadioEvents( rx_timeout );
        while( RX_LED == 1 );   // this is probably wrong

        if (packet_rxd) {
            uint8_t mac_cmd_buffer[20];
            int mac_cmd_size = 0;
            uint16_t fcnt = GetRxPacketInfo();

            // Check MIC
            // TODO: Handle 16-bit rollover
            if (CheckMic(device_config.session.NetworkKey, fcnt) 
                && device_config.session.DownlinkCounter < fcnt) {
                
                device_config.session.DownlinkCounter = fcnt;
            
                mac_cmd_size = DecryptPayload(mac_cmd_buffer);

                if (RxBufferSize - mac_cmd_size > 13) {
                    // Received Payload from Server
                    printf("Rx Packet on Port %d: ", RxBuffer[8+mac_cmd_size]);
                    for (int i = 9 + mac_cmd_size; i < RxBufferSize - 4; ++i) {
                        printf("%02X", RxBuffer[i]);
                    }
                    printf("\r\n");
                }

                if (mac_cmd_size > 0) {

                    printf("Rx MAC Commands: ");
                    for (int i = 0; i < mac_cmd_size; ++i) {
                        printf("%02X", mac_cmd_buffer[i]);
                    }
                    printf("\r\n");
                    // Process MAC Commands

                    for (int i = 0; i < mac_cmd_size; ++i) {
                        // Handle Rx Timing Req
                        if (mac_cmd_buffer[i] == 0x08) {
                            // 1 byte payload
                            i+=1;

                            // Unpack Rx1Delay
                            int delay = (mac_cmd_buffer[i] & 0xF);
                            if (delay > 1)
                                rx_1_time_out = delay * 1000U;
                            else
                                rx_1_time_out = 1000U;

                            tx_mac_cmd_buffer[tx_mac_cmd_size++] = 0x08; // Command ID
                        }

                        // Handle DevStatusReq
                        if (mac_cmd_buffer[i] == 0x06) {
                            // 0 byte payload
                            tx_mac_cmd_buffer[tx_mac_cmd_size++] = 0x06;                      // Command ID
                            tx_mac_cmd_buffer[tx_mac_cmd_size++] = RxRssi;  // Received Packet SNR
                            tx_mac_cmd_buffer[tx_mac_cmd_size++] = 0xFF;                      // Battery Level
                        }
                    }

                    printf("Tx MAC Commands: ");
                    for (int i = 0; i < tx_mac_cmd_size; ++i) {
                        printf("%02X", tx_mac_cmd_buffer[i]);
                    }
                    printf("\r\n");

                }
            } else {
                printf("Failed MIC verification\r\n");
            }
        }
    } while (timeout < 0 && --packets > 0);

}

void sendi_func(int argc, char **argv) {
    int timeout = 10000;
    int packets = 10;

    if (argc == 1) {
        // use default timeout
    } else if (argc >= 2) {
        if (sscanf(argv[1], "%d", &timeout)) {
            printf(ok_str);
        } else {
            printf(invalid_args_str);
        }
        if (argc > 2) {
            if (sscanf(argv[2], "%d", &packets)) {
                printf(ok_str);
            } else {
                printf(invalid_args_str);
            }
        }
    } else {
        printf(invalid_args_str);
        return;
    }

    for (int i = 0; i < packets; i++) {
        send_func(argc, argv);
        ThisThread::sleep_for(timeout);
    }
}

void send_func(int argc, char **argv) {
    TX_LED = 1;

    printf(ok_str);

    InitTxPacket(false, device_config.settings.Port);

    uint32_t event_time = 0;
    uint16_t event_val = 0;

    event_time = time(NULL);

    printf("The event value %u occurred at %lu\r\n", event_val, event_time);

    device_config.session.UplinkCounter++;
    Buffer[6] = device_config.session.UplinkCounter & 0xFF;
    Buffer[7] = device_config.session.UplinkCounter >> 8;

    Buffer[PKT_PORT] = device_config.settings.Port;

    int index = PKT_PAYLOAD;

    // TODO: accept args and replace packet payload

    // Add 5 Payload Bytes
    Buffer[index++] = event_time >> 16;
    Buffer[index++] = event_time >> 8;
    Buffer[index++] = event_time;
    Buffer[index++] = event_val >> 8;
    Buffer[index++] = event_val;

    payload_size = 6; // Port (1) + Payload Bytes (5)

    EncryptPacket(payload_size, tx_mac_cmd_size, device_config.settings.Port, device_config.session.UplinkCounter);

    SetTxMode();
    ChooseRandomChannel();

    printf("Sending packet FCNT: %lu\r\n", device_config.session.UplinkCounter);
    //Radio.SendPayload( Buffer, TxBufferSize, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, 10000 } );
  
    Radio.lock();
    Radio.set_max_payload_length(MODEM_LORA, TxBufferSize);
    Radio.send( Buffer, TxBufferSize );
    Radio.unlock();

    tx_mac_cmd_size = 0;

    // Wait for TxDone
    while (TX_LED) {
        ThisThread::sleep_for(100);

    }

    //ProcessRadioEvents(rx_1_time_out - RX_EARLY_MS);

    // RX 1
    printf("Open Rx1\r\n");
    printf("Rx1 Channel = %lu\r\n", rx1_channels[tx_channel_index]);
    
    // Rx1 Delay
    ThisThread::sleep_for(500);
    
    SetRxMode(rx1_channels[tx_channel_index], 0);

    //Radio.SetRx(( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_MS });
    while(RX_LED) {
        ThisThread::sleep_for(50);
    }
    // ProcessRadioEvents(995);

    if (!packet_rxd) {
        // RX 2
        printf("Open Rx2\r\n");
        SetRxMode(rx2_channel, rx_2_datarate);
        //Radio.SetRx(( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_MS });
        //ProcessRadioEvents(1000);
    }

    if (packet_rxd) {
        uint8_t mac_cmd_buffer[20];
        int mac_cmd_size = 0;
        uint16_t fcnt = GetRxPacketInfo();

        // Check MIC
        // TODO: Handle 16-bit rollover
        if (CheckMic(device_config.session.NetworkKey, fcnt) 
            && device_config.session.DownlinkCounter < fcnt) {
            
            device_config.session.DownlinkCounter = fcnt;
           
            mac_cmd_size = DecryptPayload(mac_cmd_buffer);

            if (RxBufferSize - mac_cmd_size > 13) {
                // Received Payload from Server
                printf("Rx Packet on Port %d: ", RxBuffer[8+mac_cmd_size]);
                for (int i = 9 + mac_cmd_size; i < RxBufferSize - 4; ++i) {
                    printf("%02X", RxBuffer[i]);
                }
                printf("\r\n");
            }

            if (mac_cmd_size > 0) {

                printf("Rx MAC Commands: ");
                for (int i = 0; i < mac_cmd_size; ++i) {
                    printf("%02X", mac_cmd_buffer[i]);
                }
                printf("\r\n");
                // Process MAC Commands

                for (int i = 0; i < mac_cmd_size; ++i) {
                    // Handle Rx Timing Req
                    if (mac_cmd_buffer[i] == 0x08) {
                        // 1 byte payload
                        i+=1;

                        // Unpack Rx1Delay
                        int delay = (mac_cmd_buffer[i] & 0xF);
                        if (delay > 1)
                            rx_1_time_out = delay * 1000U;
                        else
                            rx_1_time_out = 1000U;

                        tx_mac_cmd_buffer[tx_mac_cmd_size++] = 0x08; // Command ID
                    }

                    // Handle DevStatusReq
                    if (mac_cmd_buffer[i] == 0x06) {
                        // 0 byte payload
                        tx_mac_cmd_buffer[tx_mac_cmd_size++] = 0x06;                      // Command ID
                        tx_mac_cmd_buffer[tx_mac_cmd_size++] = RxRssi;  // Received Packet SNR
                        tx_mac_cmd_buffer[tx_mac_cmd_size++] = 0xFF;                      // Battery Level
                    }
                }

                printf("Tx MAC Commands: ");
                for (int i = 0; i < tx_mac_cmd_size; ++i) {
                    printf("%02X", tx_mac_cmd_buffer[i]);
                }
                printf("\r\n");

            }
        } else {
            printf("Failed MIC verification\r\n");
        }
    }
}


void tinysh_char_out(unsigned char c) {
    pc.write(&c, 1);
}

void tinyshell_thread() {
    pc.write("Multitech Sx1262 LoRaWAN shell\r\n", 32);

    tinysh_set_prompt(prompt);
    tinysh_add_command(&reset_cmd);
    
    tinysh_add_command(&deveui_cmd);
    tinysh_add_command(&appeui_cmd);
    tinysh_add_command(&appkey_cmd);
    
    tinysh_add_command(&join_cmd);
    tinysh_add_command(&send_cmd);
    tinysh_add_command(&sendi_cmd);
    tinysh_add_command(&recv_cmd);

    tinysh_add_command(&devaddr_cmd);
    tinysh_add_command(&nwkskey_cmd);
    tinysh_add_command(&appskey_cmd);
    tinysh_add_command(&ucnt_cmd);
    tinysh_add_command(&dcnt_cmd);
    
    tinysh_add_command(&channels_cmd);
    tinysh_add_command(&channel_index_cmd);
    tinysh_add_command(&power_cmd);
    tinysh_add_command(&datarate_cmd);

    tinysh_add_command(&app_port_cmd);
    
    while (!exit_cmd_mode) {
        if(pc.read(&ser_buf, 1) == 1) tinysh_char_in(ser_buf);
    }

}







