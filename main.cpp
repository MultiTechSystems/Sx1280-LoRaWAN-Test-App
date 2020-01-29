#include "mbed.h"
#include "sx1280-hal.h"
#include "Crypto.h"
#include "DeepSleepLock.h"

Serial dbg(USBTX, USBRX);


/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( void );

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
void OnRxError( IrqErrorCode_t );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRangingDone( IrqRangingCode_t );

/*!
 * \brief All the callbacks are stored in a structure
 */
RadioCallbacks_t Callbacks =
{
    &OnTxDone,        // txDone
    &OnRxDone,        // rxDone
    NULL,             // syncWordDone
    NULL,             // headerDone
    &OnTxTimeout,     // txTimeout
    &OnRxTimeout,     // rxTimeout
    &OnRxError,       // rxError
    &OnRangingDone,   // rangingDone
    NULL,             // cadDone
};

/*!
 * \brief Define IO and callbacks for radio
 * mosi, miso, sclk, nss, busy, dio1, dio2, dio3, rst, callbacks
 */
SX1280Hal Radio( D11, D12, D13, D7, D3, D5, NC, NC, A0, &Callbacks );

/*!
 * \brief Control the Antenna Diversity switch
 */
DigitalOut ANT_SW( A3 );

/*!
 * \brief Tx LED toggling on transmition success
 */
DigitalOut TX_LED( A4 );

/*!
 * \brief Rx LED toggling on reception success
 */
DigitalOut RX_LED( A5 );

/*!
 * \brief Mask of IRQs
 */
uint16_t IrqMask = 0x0000;

/*!
 * \brief Locals parameters and status for radio API
 * NEED TO BE OPTIMIZED, COPY OF STUCTURE ALREADY EXISTING
 */
PacketParams_t PacketParams;
PacketStatus_t PacketStatus;
ModulationParams_t ModulationParams;

void SetAntennaSwitch( int );
void LedBlink( void );

#define RX_TIMEOUT_TICK_SIZE            RADIO_TICK_SIZE_1000_US
#define BUFFER_SIZE                     255

lora::Crypto crypto;
uint8_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];
uint8_t RxBuffer[BUFFER_SIZE];
uint8_t RxBufferSize = 0;

bool packet_rxd = false;

void InitApplication( void )
{
    RX_LED = 1;
    TX_LED = 1;

    SetAntennaSwitch( 1 );

    wait_ms( 500 ); // wait for on board DC/DC start-up time

    Radio.Init( );

    // Can also be set in LDO mode but consume more power
    // Radio Power Mode [0: LDO, 1:DC_DC]
    Radio.SetRegulatorMode( ( RadioRegulatorModes_t ) 1 );
    Radio.SetStandby( STDBY_RC );

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

void SetAntennaSwitch( int on )
{
    if( on == 1 )
    {
        ANT_SW = 1; // ANT1
    }
    else
    {
        ANT_SW = 0; // ANT0
    }
}

void OnTxDone( void )
{
    printf("OnTxDone\r\n");
    TX_LED = 0;
}

void OnRxDone( void )
{
    printf("OnRxDone\r\n");
    packet_rxd = true;
    RX_LED = 0;
}

void OnTxTimeout( void )
{
    // printf("OnTxTimeout\r\n");
}

void OnRxTimeout( void )
{
    printf("OnRxTimeout\r\n");
    RX_LED = 0;
}

void OnRxError( IrqErrorCode_t errorCode )
{
    printf("OnRxError\r\n");
    RX_LED = 0;
}

void OnRangingDone( IrqRangingCode_t val )
{
    // printf("OnRangingDone\r\n");
}

void OnCadDone( bool channelActivityDetected )
{
    // printf("OnCadDone\r\n");
}

uint8_t devEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

uint8_t appKey[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

uint32_t tx_channels[] = { 2403000000U, 2479000000U, 2425000000U };
uint32_t rx2_channel = 2422000000U;

uint32_t devaddr = 0x00000000;
uint8_t appSKey[] = { 0xF2,0xA4,0x68,0xFD,0xD5,0x85,0xF9,0xC5,0x78,0x8E,0x1E,0x1C,0xAF,0xC5,0x7F,0x27 };
uint8_t nwkSKey[] = { 0x8A,0xDF,0xAB,0xAA,0xF2,0xA7,0x74,0xCC,0x68,0x6D,0x1C,0x13,0x9E,0x95,0x39,0x87 };

void SetupRadio() {
    ModulationParams.PacketType = PACKET_TYPE_LORA;
    PacketParams.PacketType = PACKET_TYPE_LORA;
    ModulationParams.Params.LoRa.SpreadingFactor = LORA_SF12;
    ModulationParams.Params.LoRa.Bandwidth = LORA_BW_0800;
    ModulationParams.Params.LoRa.CodingRate = LORA_CR_4_8;
    PacketParams.Params.LoRa.PreambleLength = 8;
    PacketParams.Params.LoRa.HeaderType = LORA_PACKET_EXPLICIT;
    PacketParams.Params.LoRa.PayloadLength = 23;
    PacketParams.Params.LoRa.Crc = LORA_CRC_ON;
    PacketParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
    Radio.SetPacketType( ModulationParams.PacketType );
    Radio.SetModulationParams( &ModulationParams );
    Radio.SetPacketParams( &PacketParams );

    Radio.SetTxParams( 10, RADIO_RAMP_20_US );
    Radio.SetPollingMode();
}

void FillJoinRequest(uint8_t* buffer, uint16_t nonce, uint8_t* key) {
    uint32_t mic = 0;

    buffer[0] = 0x00;

    for (int i = 0; i < 8; ++i) {
        buffer[8-i] = appEUI[i];
        buffer[16-i] = devEUI[i];
    }

    buffer[17] = nonce;
    buffer[18] = nonce >> 8;

    crypto.JoinComputeMic(buffer, 19, key, &mic);
    crypto.CopyMicToArray(mic, buffer+19);
}

void ChooseRandomChannel() {
    int i = rand() % 3;
    printf("Using channel %u : %lu\r\n", i, tx_channels[i]);
    Radio.SetRfFrequency( tx_channels[i] );
}

void SetTxMode() {
    PacketParams.Params.LoRa.Crc = LORA_CRC_ON;
    PacketParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
    Radio.SetPacketParams( &PacketParams );

    IrqMask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;
    Radio.SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );

    TX_LED = 1;
    packet_rxd = false;
}

void SetRxMode(uint32_t freq) {
    PacketParams.Params.LoRa.Crc = LORA_CRC_OFF;
    PacketParams.Params.LoRa.InvertIQ = LORA_IQ_INVERTED;
    Radio.SetPacketParams( &PacketParams );

    if (freq > 0) {
        Radio.SetRfFrequency( freq );
    }

    IrqMask = IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT;
    Radio.SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    RX_LED = 1;
}

void ProcessRadioEvents(int32_t timeout) {
    Timer tm;
    tm.start();

    while (tm.read_ms() < timeout) {
        Radio.ProcessIrqs();
        wait_ms(10);
    }
}

void GetJoinRxPacketInfo(uint8_t* key) {
    Radio.GetPayload( RxBuffer, &RxBufferSize, BUFFER_SIZE );
    Radio.GetPacketStatus( &PacketStatus );

    printf("Rx %d Bytes Snr: %d Rssi: %d\r\n", RxBufferSize, PacketStatus.LoRa.SnrPkt, PacketStatus.LoRa.RssiPkt);
    printf("RxPayload: ");
    for (int i = 0; i < RxBufferSize; ++i) {
        printf("%02X", RxBuffer[i]);
    }
    printf("\r\n");

    crypto.JoinDecrypt(RxBuffer+1, RxBufferSize-1, key, RxBuffer+1);

    printf("RxPayload: ");
    for (int i = 0; i < RxBufferSize; ++i) {
        printf("%02X", RxBuffer[i]);
    }
    printf("\r\n");
}

int GetRxPacketInfo() {

    Radio.GetPayload( RxBuffer, &RxBufferSize, BUFFER_SIZE );
    Radio.GetPacketStatus( &PacketStatus );

    uint16_t fcnt = RxBuffer[7] << 8 | RxBuffer[6];

    printf("Rx %d Bytes Snr: %d Rssi: %d FCNT: %u\r\n", RxBufferSize, PacketStatus.LoRa.SnrPkt, PacketStatus.LoRa.RssiPkt, fcnt);
    printf("RxPayload: ");
    for (int i = 0; i < RxBufferSize; ++i) {
        printf("%02X", RxBuffer[i]);
    }
    printf("\r\n");

    return fcnt;
}

bool CheckJoinMic(uint8_t* key) {
    uint32_t mic = 0;
    crypto.JoinComputeMic(RxBuffer, RxBufferSize-4, key, &mic);

    uint32_t rx_mic = RxBuffer[RxBufferSize-1] << 24 | RxBuffer[RxBufferSize-2] << 16 | RxBuffer[RxBufferSize-3] << 8 | RxBuffer[RxBufferSize-4] << 0;

    printf("Join MIC: %lu RXMIC: %lu\r\n", mic, rx_mic);

    return mic == rx_mic;
}

bool CheckMic(uint8_t* key, uint32_t counter) {
    uint32_t mic = 0;
    // void Crypto::ComputeMic(uint8_t *buffer, uint16_t size, uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint32_t *mic)
    crypto.ComputeMic(RxBuffer, RxBufferSize-4, key, devaddr, 1, counter, &mic);

    uint32_t rx_mic = RxBuffer[RxBufferSize-1] << 24 | RxBuffer[RxBufferSize-2] << 16 | RxBuffer[RxBufferSize-3] << 8 | RxBuffer[RxBufferSize-4] << 0;

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

    uint8_t* appNonce = RxBuffer+1;
    uint8_t* netID = RxBuffer+4;
    devaddr = RxBuffer[7] | RxBuffer[8] << 8 | RxBuffer[9] << 16 | RxBuffer[10] << 24;

    crypto.DeriveSessionKeys(appKey, appNonce, netID, nonce, nwkSKey, appSKey);

    printf("Dev Addr: %08X\r\n", devaddr);

    printf("Nwk SKey: ");
    for (int i = 0; i < 16; ++i) {
        printf("%02X", nwkSKey[i]);
    }
    printf("\r\n");

    printf("App SKey: ");
    for (int i = 0; i < 16; ++i) {
        printf("%02X", appSKey[i]);
    }
    printf("\r\n");

    // Rx Parameters
    // Rx1 DR Offset
    // rx_1_offset = RxBuffer[11] >> 4;
    // Rx2 DR
    // rx_2_datarate = RxBuffer[11] & 0x0F;

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

    Buffer[1] = devaddr & 0xFF;
    Buffer[2] = devaddr >> 8;
    Buffer[3] = devaddr >> 16;
    Buffer[4] = devaddr >> 24;
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
        crypto.PayloadEncrypt(Buffer + 9, payload_size, nwkSKey, devaddr, 0, counter, Buffer + 9);
    } else {
        crypto.PayloadEncrypt(Buffer + 9 + tx_mac_cmd_size, payload_size, appSKey, devaddr, 0, counter, Buffer + 9 + tx_mac_cmd_size);
    }

    crypto.ComputeMic(Buffer, localPayloadSize - 4, nwkSKey, devaddr, 0, counter, &mic);
    crypto.CopyMicToArray(mic, Buffer + (localPayloadSize - 4));

    printf("TxPacket ENC: ");
    for (int i = 0; i < localPayloadSize; ++i) {
        printf("%02X", Buffer[i]);
    }
    printf("\r\n");

    PacketParams.Params.LoRa.PayloadLength = localPayloadSize;
}

uint8_t DecryptPayload(uint8_t* mac_cmd_buffer) {
    uint8_t mac_cmd_size = 0;

    if (RxBufferSize > 12 && ((RxBuffer[5] & 0x0F) == 0) && RxBuffer[8] == 0x00) {
        crypto.PayloadDecrypt(RxBuffer + 9, RxBufferSize - 13, nwkSKey, devaddr, 1, RxBuffer[7] << 8 | RxBuffer[6], RxBuffer + 9);
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

        crypto.PayloadDecrypt(RxBuffer + 9, RxBufferSize - 13, appSKey, devaddr, 1, RxBuffer[7] << 8 | RxBuffer[6], RxBuffer + 9);
    }

    return mac_cmd_size;
}


#define RX_TIMEOUT_MS 500
#define RX_EARLY_MS 20
#define PKT_FCTRL 5
#define PKT_PORT 8
#define PKT_PAYLOAD 9


int main(int argc, char**argv) {

    // Sensor Inputs
    // Digital Pins D3, D5, D7, D11, D12, D13 used by the Radio
    // (A0 used for Radio Reset, A3 used for Antenna Switch)
    AnalogIn a1(A1);
    AnalogIn a2(A2);


    bool joined = false;
    bool rx_mac_cmds = false;
    int tx_mac_cmd_size = 0;

    int rx_1_time_out = 5000;
    int payload_size = 3;

    uint32_t uplink_frame_counter = 0;
    uint8_t app_port = 4;

    uint8_t join_request[23];
    uint8_t tx_mac_cmd_buffer[20];

    memset(join_request, 0, 23);
    memset(tx_mac_cmd_buffer, 0, 20);

    dbg.baud(115200);

    printf("Initializing...\r\n");

    InitApplication();
    SetupRadio();

    printf("RTC Time: %d seconds\r\n", time(NULL));

    uint16_t nonce = 0 + time(NULL);

    while (!joined) {
        printf("DevNonce: %d\r\n", nonce);

        FillJoinRequest(join_request, nonce, appKey);

        printf("JoinRequest: ");
        for (int i = 0; i < 23; ++i) {
            printf("%02X", join_request[i]);
        }
        printf("\r\n");

        ChooseRandomChannel();
        SetTxMode();

        printf("SendPayload\r\n");
        Radio.SendPayload( join_request, 23, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, 2000 } );

        // Wait for TxDone
        while (TX_LED) {
            ProcessRadioEvents(1);
        }

        ProcessRadioEvents(rx_1_time_out - RX_EARLY_MS);

        // RX 1
        printf("Open Rx1\r\n");
        SetRxMode(0);
        Radio.SetRx(( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_MS });

        ProcessRadioEvents(995);

        if (!packet_rxd) {
            // RX 2
            printf("Open Rx2\r\n");
            SetRxMode(rx2_channel);
            Radio.SetRx(( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_MS });
            ProcessRadioEvents(1000);
        }

        if (packet_rxd) {
            GetJoinRxPacketInfo(appKey);

            if (CheckJoinMic(appKey)) {

                rx_1_time_out = UnpackJoinAccept(appKey, nonce);

                joined = true;
                break;
            }
        }

        nonce++;
        wait_ms(5000);
    }


    InitTxPacket(false, app_port);

    uint32_t event_time = 0;
    uint16_t event_val = 0;

    while (true) {

        printf("Waiting for event\r\n");
        while (true) {
            uint16_t trigger = a1.read_u16();

            if (trigger > 16000) break;

            ThisThread::sleep_for(100);
        }

        event_time = time(NULL);
        event_val = a2.read_u16();

        printf("The event value %u occurred at %lu\r\n", event_val, event_time);

        uplink_frame_counter++;
        Buffer[6] = uplink_frame_counter & 0xFF;
        Buffer[7] = uplink_frame_counter >> 8;

        if (rx_mac_cmds) {
            app_port = 0;
            for (int i = 0; i < tx_mac_cmd_size; ++i) {
                Buffer[8+i] = tx_mac_cmd_buffer[i];
            }
        }

        app_port = 4;

        Buffer[PKT_FCTRL] = tx_mac_cmd_size;

        Buffer[PKT_PORT+tx_mac_cmd_size] = app_port; // Port

        int index = PKT_PAYLOAD + tx_mac_cmd_size;

        // Add 5 Payload Bytes
        Buffer[index++] = event_time >> 16;
        Buffer[index++] = event_time >> 8;
        Buffer[index++] = event_time;
        Buffer[index++] = event_val >> 8;
        Buffer[index++] = event_val;

        payload_size = 6; // Port (1) + Payload Bytes (5)

        EncryptPacket(payload_size, tx_mac_cmd_size, app_port, uplink_frame_counter);
        ChooseRandomChannel();
        SetTxMode();

        printf("Sending packet FCNT: %d\r\n", uplink_frame_counter);
        Radio.SendPayload( Buffer, PacketParams.Params.LoRa.PayloadLength, ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, 10000 } );

        tx_mac_cmd_size = 0;

        // Wait for TxDone
        while (TX_LED) {
            ProcessRadioEvents(1);
        }

        ProcessRadioEvents(rx_1_time_out - RX_EARLY_MS);

        // RX 1
        printf("Open Rx1\r\n");
        SetRxMode(0);
        Radio.SetRx(( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_MS });

        ProcessRadioEvents(995);

        if (!packet_rxd) {
            // RX 2
            printf("Open Rx2\r\n");
            SetRxMode(rx2_channel);
            Radio.SetRx(( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_MS });
            ProcessRadioEvents(1000);
        }

        if (packet_rxd) {
            uint8_t mac_cmd_buffer[20];
            int mac_cmd_size = 0;
            int fcnt = GetRxPacketInfo();

            // TODO: Handle 16-bit rollover

            rx_mac_cmds = false;

            // Check MIC
            if (CheckMic(nwkSKey, fcnt)) {

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

                    rx_mac_cmds = true;

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
                            tx_mac_cmd_buffer[tx_mac_cmd_size++] = PacketStatus.LoRa.SnrPkt;  // Received Packet SNR
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

    return 0;
}
