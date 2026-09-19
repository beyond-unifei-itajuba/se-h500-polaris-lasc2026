#include "LoRaWan_APP.h"
#include "Arduino.h"
// Headers gerados pelo Nanopb (proto do projeto H500)
#include "h500_avionic.pb.h"
#include "pb_common.h"
#include "pb.h"
#include "pb_decode.h"

#define RF_FREQUENCY                                915000000 // Hz
#define LORA_BANDWIDTH                              0         // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                              1        // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH                         8        // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                          0        // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                   false
#define LORA_IQ_INVERSION_ON                         false

static RadioEvents_t RadioEvents;
bool lora_idle = true;

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

    RadioEvents.RxDone = OnRxDone;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                       LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                       LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                       0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
}

void loop() {
    if (lora_idle) {
        lora_idle = false;
        Radio.Rx(0); // 0 = escuta contínua, sem timeout
    }

    // Processa os eventos de rádio em background
    Radio.IrqProcess();
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    Radio.Sleep();

    // 1. Instancia a mensagem Protobuf vazia
    h500_avionics packet = h500_avionics_init_zero;

    // 2. Cria a stream de entrada a partir do payload recebido pelo rádio
    pb_istream_t stream = pb_istream_from_buffer(payload, size);

    // 3. Faz o decode
    bool status = pb_decode(&stream, h500_avionics_fields, &packet);

    if (status) {
        Serial.printf("\r\nPacote recebido! RSSI: %d dBm | SNR: %d | Tamanho: %d bytes\r\n", rssi, snr, size);

        Serial.printf(
        "DATA,%.6f,%.6f,%d,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.2f,%.6f,%.6f\n",
        packet.latitude / 100000.0,
        packet.longitude / 100000.0,
        packet.sats,
        packet.humidity / 10.0,
        packet.temperatureC_dht / 10.0,
        packet.acceleration_x / 100.0,
        packet.acceleration_y / 100.0,
        packet.acceleration_z / 100.0,
        packet.gyro_x / 100.0,
        packet.gyro_y / 100.0,
        packet.gyro_z / 100.0,
        packet.pressure / 100.0,
        packet.altGY / 100.0,
        packet.altMaxGY / 100.0,
        packet.temperatureC_GY / 10.0,
        packet.airspeed_ms / 100.0,
        packet.expected_latitude / 100000.0,
        packet.expected_longitude / 100000.0
        );

        // Mesmos fatores de divisão usados no encode (buildTelemetryPacket) --- Usar se não for usar o script em python
        /*Serial.print("Latitude        : "); Serial.println(packet.latitude / 100000.0, 6);
        Serial.print("Longitude       : "); Serial.println(packet.longitude / 100000.0, 6);
        Serial.print("Satelites       : "); Serial.println(packet.sats);
        Serial.print("Umidade         : "); Serial.println(packet.humidity / 10.0, 1);
        Serial.print("Temp (DHT)      : "); Serial.println(packet.temperatureC_dht / 10.0, 1);
        Serial.print("Accel X         : "); Serial.println(packet.acceleration_x / 100.0, 2);
        Serial.print("Accel Y         : "); Serial.println(packet.acceleration_y / 100.0, 2);
        Serial.print("Accel Z         : "); Serial.println(packet.acceleration_z / 100.0, 2);
        Serial.print("Gyro X          : "); Serial.println(packet.gyro_x / 100.0, 2);
        Serial.print("Gyro Y          : "); Serial.println(packet.gyro_y / 100.0, 2);
        Serial.print("Gyro Z          : "); Serial.println(packet.gyro_z / 100.0, 2);
        Serial.print("Pressao         : "); Serial.println(packet.pressure / 100.0, 2);
        Serial.print("Altitude GY     : "); Serial.println(packet.altGY / 100.0, 2);
        Serial.print("Alt Max GY      : "); Serial.println(packet.altMaxGY / 100.0, 2);
        Serial.print("Temp (GY)       : "); Serial.println(packet.temperatureC_GY / 10.0, 1);
        Serial.print("Airspeed (m/s)  : "); Serial.println(packet.airspeed_ms / 100.0, 2);
        Serial.print("Expected lat    : "); Serial.println(packet.expected_latitude / 100000.0, 6);
        Serial.print("Expected lon    : "); Serial.println(packet.expected_longitude / 100000.0, 6);*/

    } else {
        Serial.println("Erro ao decodificar o pacote recebido!");
    }

    lora_idle = true;
}
