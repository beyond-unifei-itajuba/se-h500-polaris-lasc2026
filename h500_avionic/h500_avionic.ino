#include "Arduino.h"
#include "LoRaWan_APP.h"

#include <Wire.h>
#include <SPI.h>
#include <TinyGPS++.h>

#include "DHT.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BMP085.h>

#include <SD.h>
#include <FS.h>

#include "h500_avionic.pb.h"
#include "pb_common.h"
#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

#include "mariomusic.h"


static unsigned long int lastReadGY87 = 0;
static unsigned long int lastReadGPS = 0;
static unsigned long int lastReadDHTAndPitot = 0;
static unsigned long int lastSaveToSD = 0;
static unsigned long int lastLoraSend = 0;  // lastLoraSend
static unsigned long int lastCheckSystemHealth = 0; //lastCheckSystemHealth
static unsigned long int lastKalmanUpdate = 0;
static unsigned long int launchTime = 0;
static unsigned long int currentMillis = 0;


#define READ_GY87_INTERVAL 100
#define READ_GPS_INTERVAL 1000
#define READ_DHT_AND_PITOT_INTERVAL 300
#define SAVE_TO_SD_INTERVAL 500
#define LORA_SEND_INTERVAL 1000
#define CHECK_SYSTEM_HEALTH_INTERVAL 60000

#define DHTPIN 5
#define DHTTYPE DHT22
DHT dhtDriver(DHTPIN, DHTTYPE);

#define GPS_RX 6 // RX inverted with TX on proto
#define GPS_TX 7
TinyGPSPlus gpsDriver;

#define RF_FREQUENCY 915000000
#define TX_OUTPUT_POWER 5
#define LORA_BANDWIDTH 0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

static RadioEvents_t RadioEvents;
bool loraIdle = true;
unsigned long timer = 0;

#define I2C_SDA 42
#define I2C_SCL 41

struct ENUData {
    float east;
    float north;
    float up;
    bool referenceSet = false;
}; ENUData enu;

double refLatitude = 0.0;
double refLongitude = 0.0;
float refAltitude = 0.0;

const double EARTH_RADIUS = 6378137.0; // WGS84, em metros

// Ruído das medições (variância = desvio_padrão²) — ajustar depois com dados reais de testes
const float R_GPS_HORIZ = 4.5;
const float R_GPS_VERT  = 30;
const float R_BARO      = 1; 


void setEnuReference(double lat, double lon, float alt){
    refLatitude = lat;
    refLongitude = lon;
    refAltitude = alt;
    enu.referenceSet = true;
    Serial.println("[ENU] Reference definida (origem)");
}

void convertToEnu(double lat, double lon, float alt){
    if (!enu.referenceSet) return;

    double refLatRad = refLatitude * DEG_TO_RAD;
    double dLat = (lat - refLatitude) * DEG_TO_RAD;
    double dLon = (lon - refLongitude) * DEG_TO_RAD;

    enu.north = dLat * EARTH_RADIUS;
    enu.east = dLon * EARTH_RADIUS;
    enu.up = alt-refAltitude;
}

struct AxisKalman {
    float pos;      // posição nesse eixo (m)
    float vel;      // velocidade nesse eixo (m/s)
    float P[2][2];  // covariância: [ [Ppp, Ppv], [Pvp, Pvv] ]
};

AxisKalman kfE, kfN, kfU;

void initAxisKalman(AxisKalman &axis, float initialPosUncertainty, float initialVelUncertainty);

void initAxisKalman(AxisKalman &axis, float initialPosUncertainty, float initialVelUncertainty){
    axis.pos = 0;
    axis.vel = 0;
    axis.P[0][0] = initialPosUncertainty;
    axis.P[0][1] = 0;
    axis.P[1][0] = 0;
    axis.P[1][1] = initialVelUncertainty;
}

void initKalmanFilter(){
    initAxisKalman(kfE, 5.0, 100.0);
    initAxisKalman(kfN, 5.0, 100.0);
    initAxisKalman(kfU, 5.0, 100.0);
    lastKalmanUpdate = millis();
}

void kalmanPredictAxis(AxisKalman &axis, float dt, float q);

void kalmanPredictAxis(AxisKalman &axis, float dt, float q){
    axis.pos += axis.vel * dt;
    // vel não muda na predição

    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;

    axis.P[0][0] += q * dt4 / 4.0;
    axis.P[0][1] += q * dt3 / 2.0;
    axis.P[1][0] += q * dt3 / 2.0;
    axis.P[1][1] += q * dt2;
}

void kalmanPredict(float dt){
    float qHoriz = 1.0;
    float qVert  = 0.5;

    kalmanPredictAxis(kfE, dt, qHoriz);
    kalmanPredictAxis(kfN, dt, qHoriz);
    kalmanPredictAxis(kfU, dt, qVert);
}

void kalmanUpdateAxis(AxisKalman &axis, float measurement, float R);

void kalmanUpdateAxis(AxisKalman &axis, float measurement, float R){
    // Inovação: diferença entre o que o sensor mediu e o que o filtro previa
    float y = measurement - axis.pos;

    // Quão incerta é essa inovação (incerteza do filtro + incerteza do sensor)
    float S = axis.P[0][0] + R;

    // Ganho de Kalman: o quanto confiar na medição vs. na predição
    float K0 = axis.P[0][0] / S;
    float K1 = axis.P[1][0] / S;

    // Corrige o estado
    axis.pos += K0 * y;
    axis.vel += K1 * y;

    // Corrige a covariância (guarda os valores antigos antes de sobrescrever)
    float P00 = axis.P[0][0];
    float P01 = axis.P[0][1];
    float P10 = axis.P[1][0];
    float P11 = axis.P[1][1];

    axis.P[0][0] = P00 - K0 * P00;
    axis.P[0][1] = P01 - K0 * P01;
    axis.P[1][0] = P10 - K1 * P00;
    axis.P[1][1] = P11 - K1 * P01;
}

struct LandingPrediction {
    double latitude;
    double longitude;
    float timetoGroundSec;
    bool valid;
}; LandingPrediction landing;

Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_BMP085 bmp;

// --- Pitot Tube ---
#define PITOT_PIN 19
const float SENSOR_VS   = 5.0;
const float ESP32_VREF  = 3.3;
const float ADC_MAX     = 4095.0;

const float R1 = 10000.0;
const float R2 = 22000.0;
const float DIVIDER_FACTOR = R2 / (R1 + R2);

const float DEADZONE_PA    = 1.0;
const int   FILTER_SAMPLES = 5;
const int   TARE_SAMPLES   = 300;

const float R_AR = 287.05;

float sensorVZero = 2.5;
float filterReadings[FILTER_SAMPLES];

float filterTotalSum = 0;
int   filterIndex    = 0;

// --- SD Card ---
#define SD_MOSI 20
#define SD_MISO 47
#define SD_SCK  26
#define SD_CS   48
SPIClass spiSD(FSPI);
char dataRecoveryBuffer[256];
bool sdErrors = true;


#define BUZZER_PIN 4

bool launchDone = false;
bool recoveryMode = false;

#include <esp_task_wdt.h>

#define WDT_TIMEOUT_SECONDS 10


struct GPSData {
    float latitude;
    float longitude;
    float altitude;
    int   satelliteCount;
    bool  hasError = true;
};

GPSData gps;

struct DHTData {
    float humidity;
    float temperatureC;
    float temperatureF;
    float heatIndexC;
    float heatIndexF;
    int   readCount;
    bool  hasError = true;
};

DHTData dht;

struct GY87Data {
    float accelX;
    float accelY;
    float accelZ;

    float gyroX;
    float gyroY;
    float gyroZ;

    float magX;
    float magY;
    float magZ;

    float pressure;
    float altitude;
    float altitudeMax = 0;
    float altitudeRef;
    float temperature;

    bool hasError = true;
    bool bmpDataValid = false;
    int  referenceSampleCount = 0;
};

GY87Data gy87;

static float altitudeRefSum = 0;



struct PitotData {
    float rhoFallBack      = 1.146;  // Densidade do ar em kg/m³ (~25°C)
    float filteredPressure;
    float airspeedMps;
};

PitotData pitot;


uint8_t buffer[128];
h500_avionics packet = h500_avionics_init_zero;
h500_avionics received_packet = h500_avionics_init_zero; // test decoder
size_t packetSize = 0;


void OnTxDone(void){
    loraIdle = true;
}

void OnTxTimeout(void){
    Serial.println("TX Timeout");
    Radio.Sleep();
    loraIdle = true;
}

void initDHT(){
    dhtDriver.begin();
}

void initGPS(){
    Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    Serial.println("GPS Initialized");
}

void initLoRa(){
    Serial.println("LoRa Initialized");

    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(
        MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH,
        LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, LORA_IQ_INVERSION_ON, 3000
    );
}

void initGY87(){
    Wire.begin(I2C_SDA, I2C_SCL);

    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050!");
        gy87.hasError = true;
    } else {
        gy87.hasError = false;
    }

    if (!mag.begin()) {
        Serial.println("Failed to find HMC5883L!");
        gy87.hasError = true;
    } else {
        gy87.hasError = false;
    }

    if (!bmp.begin()){ 
        Serial.println("Failed to find BMP180!");
        gy87.hasError = true;
    } else {
        gy87.hasError = false;
    }
    
    if(!gy87.hasError){
        Serial.println("GY87 initialized");
    }
}

void calibrateAndInitPitot() {

  Serial.println("\nINICIANDO CALIBRAÇÃO DO PITOT (AUTO-ZERO)...");
  Serial.println("ATENÇÃO: O sensor já deve estar fixado na sua posição/ângulo definitivo.");
  Serial.println("Mantenha o tubo pitot estático, sem vento.");
  delay(2000);
  
  float tareSum = 0;
  
  for (int i = 0; i < TARE_SAMPLES; i++) {
    int adcRaw = analogRead(PITOT_PIN);
    float measuredVolts = adcRaw * (ESP32_VREF / ADC_MAX);
    float sensorVolts = measuredVolts / DIVIDER_FACTOR; 
    
    tareSum += sensorVolts;
    
    if (i % 50 == 0) Serial.print(".");
    delay(10);
  }
  
  sensorVZero = tareSum / TARE_SAMPLES;
  
  for (int i = 0; i < FILTER_SAMPLES; i++) filterReadings[i] = 0;

  Serial.println("\n[PITOT] Calibração concluída!");
  Serial.print("[PITOT] V_Zero Calibrado: "); Serial.print(sensorVZero, 4); Serial.println(" V");
}

void initSD(){
     Serial.println("Initializing SD Card...");
     spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
     if (!SD.begin(SD_CS, spiSD)) {
       Serial.println("Failed to mount SD Card! Check wiring and format.");
       sdErrors = true;
       return;
     }
     uint8_t cardType = SD.cardType();
     if (cardType == CARD_NONE) {
       Serial.println("No SD card attached.");
       sdErrors = true;
       return;
     }

     if (!SD.exists("/data_telemetry_h500.csv")) {       
         const char* header = "Lat;Lon;Alt;Sats;Temp;Umid;HeatIndex;AccX;AccY;AccZ;Pressure;AltMax;Altitude;Rho;P_Filtered;V_ms;V_kmh\n";
         writeFile(SD, "/data_telemetry_h500.csv", header);
     } else {
         appendFile(SD, "/data_telemetry_h500.csv", "\SYSTEM RESET\n");
     }

     sdErrors = false;
     Serial.println("SD Card mounted successfully!");
}

void initWatchdog() {
    Serial.println("Initializing Watchdog Timer...");

    #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
        esp_task_wdt_config_t twdt_config = {
            .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
            .idle_core_mask = (1 << configNUM_CORES) - 1,
            .trigger_panic = true
        };
        esp_task_wdt_init(&twdt_config);
    #else
        esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true); 
    #endif

    esp_task_wdt_add(NULL); 
    Serial.println("Watchdog initialized successfully!");
}

void updateAirDensity(){
    if(!gy87.bmpDataValid) return;

    float tempCelsius = gy87.temperature;          // Leituras BMP085
    float absolutePressurePa = gy87.pressure;       // ...
    
    float tempKelvin = tempCelsius + 273.15;        // Kelvin
    
    // Lei dos Gases Ideais: Rho = P / (R * T)
    if (tempKelvin > 0 && absolutePressurePa > 0) {
        pitot.rhoFallBack = absolutePressurePa / (R_AR * tempKelvin);
    }
}

void readDHT(){
    dht.humidity = dhtDriver.readHumidity();
    dht.temperatureC = dhtDriver.readTemperature();
    dht.temperatureF = dhtDriver.readTemperature(true);

    if (isnan(dht.humidity) || isnan(dht.temperatureC)) {
        dht.humidity = 0;
        dht.temperatureC = 0;
        dht.temperatureF = 0;
        dht.heatIndexF = 0;
        dht.heatIndexC = 0;

        dht.hasError = true;

        return;
    }

    dht.hasError = false;
    dht.heatIndexF = dhtDriver.computeHeatIndex(dht.temperatureF, dht.humidity);
    dht.heatIndexC = dhtDriver.computeHeatIndex(dht.temperatureC, dht.humidity, false);

 
}

void readGPS(){
      gps.hasError = false;
      gps.latitude = gpsDriver.location.lat();
      gps.longitude = gpsDriver.location.lng();
          if (gpsDriver.altitude.isValid()) gps.altitude = gpsDriver.altitude.meters();
          if (gpsDriver.satellites.isValid()) gps.satelliteCount = gpsDriver.satellites.value();

    if (!enu.referenceSet && gy87.referenceSampleCount >= 20) {
        setEnuReference(gps.latitude, gps.longitude, gps.altitude);
        initKalmanFilter();
    }
    convertToEnu(gps.latitude, gps.longitude, gps.altitude);

    kalmanUpdateAxis(kfE, enu.east,  R_GPS_HORIZ);
    kalmanUpdateAxis(kfN, enu.north, R_GPS_HORIZ);
    kalmanUpdateAxis(kfU, enu.up,    R_GPS_VERT);
    Serial.print("[GPS RAW] lat="); Serial.print(gps.latitude, 6);
    Serial.print(" lon="); Serial.print(gps.longitude, 6);
    Serial.print(" sats="); Serial.println(gps.satelliteCount);
}

void sendPacketByLoRa(){
  if (loraIdle){
    loraIdle = false;
  }

  Radio.Send(buffer, packetSize);

  if (!loraIdle){
      Radio.IrqProcess();
  }
}


void readGY87(){
    sensors_event_t accelEvent, gyroEvent, mpuTempEvent, magEvent;

    if (!mpu.getEvent(&accelEvent, &gyroEvent, &mpuTempEvent)) {
        gy87.hasError = true;
    } else {
        gy87.hasError = false;
        gy87.accelX = accelEvent.acceleration.x;
        gy87.accelY = accelEvent.acceleration.y;
        gy87.accelZ = accelEvent.acceleration.z;

        gy87.gyroX = gyroEvent.gyro.x;
        gy87.gyroY = gyroEvent.gyro.y;
        gy87.gyroZ = gyroEvent.gyro.z;
    }

    if (!mag.getEvent(&magEvent)) {
        gy87.hasError = true;
    } else {
        gy87.hasError = false;
        gy87.magX = magEvent.magnetic.x;
        gy87.magY = magEvent.magnetic.y;
        gy87.magZ = magEvent.magnetic.z;
    }

    float rawPressure = bmp.readPressure();
    float rawAltitude = bmp.readAltitude();
    float rawTemperature = bmp.readTemperature();
    float currentAltitude;

    if (isnan(rawPressure) || isnan(rawAltitude)) {
        gy87.hasError = true;
        gy87.bmpDataValid = false;
    } else {
        gy87.hasError = false;
        gy87.bmpDataValid = true;
        gy87.temperature = rawTemperature;
        gy87.pressure = rawPressure;
        gy87.altitude = rawAltitude;


        if(enu.referenceSet){
            kalmanUpdateAxis(kfU, gy87.altitude - gy87.altitudeRef, R_BARO);
        }

        if(gy87.referenceSampleCount < 100){              // seta a altitude de ref como a media das 100 primeiras leituras
            altitudeRefSum += gy87.altitude;              
            gy87.referenceSampleCount++;

            if(gy87.referenceSampleCount == 100){
                gy87.altitudeRef = altitudeRefSum /100;
            }
        }

        if(gy87.referenceSampleCount >= 100){
            currentAltitude = gy87.altitude - gy87.altitudeRef;

            if(currentAltitude > gy87.altitudeMax){
                gy87.altitudeMax = currentAltitude;

                    if(gy87.altitudeMax > 100){
                        if(!launchDone){
                            launchDone = true;
                            launchTime = millis();
                        }
                    }
            }
        }
    }
}

float readPressureByADCPitot(){
    int adcRaw = analogRead(PITOT_PIN);
  
    float measuredVoltage = adcRaw * (ESP32_VREF / ADC_MAX);
  
    float sensorVoltage = measuredVoltage / DIVIDER_FACTOR;

    float pressurePa = ((sensorVoltage - sensorVZero) / (0.2 * SENSOR_VS)) * 1000.0;
  
    return pressurePa;  
}

float applyPitotFilter(float newReading) {
  filterTotalSum -= filterReadings[filterIndex];
  filterReadings[filterIndex] = newReading;
  filterTotalSum += filterReadings[filterIndex];
  filterIndex = (filterIndex + 1) % FILTER_SAMPLES;
  return filterTotalSum / FILTER_SAMPLES;
}

void readPitot() {
  updateAirDensity();
  float rawPressure = readPressureByADCPitot();
  pitot.filteredPressure = applyPitotFilter(rawPressure);
  
  pitot.airspeedMps = 0;
  if (pitot.filteredPressure > DEADZONE_PA) {
    pitot.airspeedMps = sqrt((2.0 * pitot.filteredPressure) / pitot.rhoFallBack);
  }
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing to file: %s\n", path);
    File file = fs.open(path, FILE_WRITE);
    if(!file) {
        Serial.println("Failed to open file for writing.");
        return;
    }
    if(file.print(message)) {
        Serial.println("File written.");
    } else {
        Serial.println("Write failed.");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);
    File file = fs.open(path, FILE_APPEND);
    if(!file) {
        Serial.println("Failed to open file for appending.");
        return;
    }
    if(file.print(message)) {
        Serial.println("Message appended.");
    } else {
        Serial.println("Append failed.");
        sdErrors = true;
    }
    file.close();
}

void saveTelemetryToSD() {
    char logBuffer[512]; 

    snprintf(dataRecoveryBuffer, sizeof(dataRecoveryBuffer),
        "%.6f;%.6f;%.1f;%d;%.1f;%.1f;%.1f;%.2f;%.2f;%.2f;%.0f;%.1f;%.1f;%.3f;%.2f;%.2f;%.2f\n",
        gps.latitude, gps.longitude,
        gps.altitude, gps.satelliteCount,
        dht.temperatureC,
        dht.humidity,
        dht.heatIndexC,
        gy87.accelX,
        gy87.accelY,
        gy87.accelZ,
        gy87.pressure,
        gy87.altitudeMax,
        gy87.altitude,
        pitot.rhoFallBack,
        pitot.filteredPressure,
        pitot.airspeedMps,
        (pitot.airspeedMps * 3.6)
    );

    appendFile(SD, "/data_telemetry_h500.csv", dataRecoveryBuffer);
}

void applyNanopbEncoder(){
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    bool status = pb_encode(&stream, h500_avionics_fields, &packet);

    if (!status) {
          Serial.println("Falhou o encode do Nanopb!");
          return;
    }
    
    packetSize = stream.bytes_written;
}

void applyNanopbDecoder(){          // test only
    pb_istream_t decodeStream = pb_istream_from_buffer(buffer, packetSize);
    
    bool decodeStatus = pb_decode(&decodeStream, h500_avionics_fields, &received_packet);

    if(!decodeStatus){
        Serial.println("Falha na decodificação!");
        return;
    }

    Serial.println("Dados decodificados com sucesso:");
    Serial.print("Latitude        : "); Serial.println(received_packet.latitude / 100000.0, 5);
    Serial.print("Longitude       : "); Serial.println(received_packet.longitude / 100000.0, 5);
    Serial.print("Satélites       : "); Serial.println(received_packet.sats);
    Serial.print("Umidade         : "); Serial.println(received_packet.humidity / 10.0, 1);
    Serial.print("Temp (DHT)      : "); Serial.println(received_packet.temperatureC_dht / 10.0, 1);
    Serial.print("Accel X         : "); Serial.println(received_packet.acceleration_x / 10.0, 1);
    Serial.print("Accel Y         : "); Serial.println(received_packet.acceleration_y / 10.0, 1);
    Serial.print("Accel Z         : "); Serial.println(received_packet.acceleration_z / 10.0, 1);
    Serial.print("Gyro X          : "); Serial.println(received_packet.gyro_x / 10.0, 1);
    Serial.print("Gyro Y          : "); Serial.println(received_packet.gyro_y / 10.0, 1);
    Serial.print("Gyro Z          : "); Serial.println(received_packet.gyro_z / 10.0, 1);
    Serial.print("Pressão         : "); Serial.println(received_packet.pressure / 10.0, 1);
    Serial.print("Altitude GY     : "); Serial.println(received_packet.altGY / 10.0, 1);
    Serial.print("Alt Máx GY      : "); Serial.println(received_packet.altMaxGY / 10.0, 1);
    Serial.print("Temp (GY)       : "); Serial.println(received_packet.temperatureC_GY / 10.0, 1);
    Serial.print("Airspeed (m/s)  : "); Serial.println(received_packet.airspeed_ms / 100.0, 2);
    Serial.print("Expected lat    : "); Serial.println(received_packet.expected_latitude / 100000.0, 5);
    Serial.print("Expected long   : "); Serial.println(received_packet.expected_longitude / 100000.0, 5);
}


void buildTelemetryPacket(){   
    packet = h500_avionics_init_zero;

    packet.latitude =            (int32_t)(gps.latitude * 100000);
    packet.longitude =           (int32_t)(gps.longitude * 100000);
    packet.sats =                gps.satelliteCount;                  // já é int
    packet.humidity =            (int32_t)(dht.humidity * 10);
    packet.temperatureC_dht =    (int32_t)(dht.temperatureC * 10);
    packet.acceleration_x =      (int32_t)(gy87.accelX * 100);
    packet.acceleration_y =      (int32_t)(gy87.accelY * 100);
    packet.acceleration_z =      (int32_t)(gy87.accelZ * 100);
    packet.gyro_x =              (int32_t)(gy87.gyroX * 100);
    packet.gyro_y =              (int32_t)(gy87.gyroY * 100);
    packet.gyro_z =              (int32_t)(gy87.gyroZ * 100);
    packet.pressure =            (int32_t)(gy87.pressure * 100);
    packet.altGY =               (int32_t)(gy87.altitude * 100);
    packet.altMaxGY =            (int32_t)(gy87.altitudeMax * 100);
    packet.temperatureC_GY =     (int32_t)(gy87.temperature * 10);
    packet.airspeed_ms =         (int32_t)(pitot.airspeedMps * 100);
    packet.expected_latitude =   (int32_t)(landing.latitude * 100000);
    packet.expected_longitude =  (int32_t)(landing.longitude * 100000);
}

void printTelemetryToSerial(){
    Serial.println("\n--- TELEMETRY DATA ---");
    
    // GPS
    Serial.print("[GPS]   Lat: "); Serial.print(gps.latitude, 6);
    Serial.print(" | Lon: "); Serial.print(gps.longitude, 6);
    Serial.print(" | Alt: "); Serial.print(gps.altitude, 1);
    Serial.print("m | Sats: "); Serial.println(gps.satelliteCount);

    // DHT
    Serial.print("[DHT]   Temp: "); Serial.print(dht.temperatureC, 1);
    Serial.print(" °C | Hum: "); Serial.print(dht.humidity, 1);
    Serial.print(" % | Heat Index: "); Serial.print(dht.heatIndexC, 1);
    Serial.println(" °C");

    // GY-87
    Serial.print("[GY87]  Accel (m/s²): X: "); Serial.print(gy87.accelX, 2);
    Serial.print(" | Y: "); Serial.print(gy87.accelY, 2);
    Serial.print(" | Z: "); Serial.println(gy87.accelZ, 2);
    
    Serial.print("[GY87] Pressao: "); Serial.print(gy87.pressure, 0); Serial.print(" Pa");
    Serial.print(" | Altitude: ");    Serial.print(gy87.altitude, 1); Serial.print(" m");
    Serial.print(" | Alt Max: ");     Serial.print(gy87.altitudeMax, 1); Serial.println(" m");

    // PITOT
    Serial.print("[PITOT] RHO: "); Serial.print(pitot.rhoFallBack, 3);
    Serial.print(" | P_Pa: "); Serial.print(pitot.filteredPressure, 2);
    Serial.print(" | V_ms: "); Serial.print(pitot.airspeedMps, 2);
    Serial.print(" | V_kmh: "); Serial.println(pitot.airspeedMps * 3.6, 2);

    // PAYLOAD
    Serial.print("[PAYLOAD] Expected Latitude: "); Serial.print(landing.latitude, 2);
    Serial.print(" | Expected Longitude: "); Serial.println(landing.longitude, 2);

    
    Serial.println("------------------------");
}

void bip(int count, int time_on, int time_off){
    for (int i = 0; i < count; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(time_on);
        digitalWrite(BUZZER_PIN, LOW);
        delay(time_off);
    }
}

void checkSystemHealth(){
    bip(1, 1000, 100); 
    delay(500);

     esp_task_wdt_reset();


    if (dht.hasError) {
        bip(1, 100, 100);
        delay(500);
    }

     esp_task_wdt_reset();


    if (gy87.hasError) {
        bip(2, 100, 100);
        delay(500);
    }

    esp_task_wdt_reset();

    if (gps.hasError) {
        bip(3, 100, 100);
        delay(500);
    }

    esp_task_wdt_reset();


    if (sdErrors){
        bip(4, 100, 100);
        initSD();
        delay(500);
    }

    esp_task_wdt_reset();

}

void runRecoveryMode(){
    static bool buzzerState = false;
    
    buzzerState = !buzzerState;
    
    if (buzzerState) {
        digitalWrite(BUZZER_PIN, HIGH);
    } else {
        digitalWrite(BUZZER_PIN, LOW);
    }
}

void updateGPSPositionWhenIsAvailable(){
    while (Serial2.available()) {
        gpsDriver.encode(Serial2.read());
    }

    if (gpsDriver.location.isValid()) {
        gps.hasError = false;

        if (gpsDriver.location.isUpdated()) {
            readGPS();   // só roda quando tem fix novo de verdade
        }
    } else {
        gps.hasError = true;
    }
}

void predictLandingPoint(){
    landing.valid = false;

    if (!enu.referenceSet)  return;

    float timeToGround = -kfU.pos / kfU.vel;

    if (timeToGround <= 0 || timeToGround > 600) return; // sanity check:

    float eastLanding  = kfE.pos + kfE.vel * timeToGround;
    float northLanding = kfN.pos + kfN.vel * timeToGround;

    double refLatRad = refLatitude * DEG_TO_RAD;

    landing.latitude  = refLatitude  + (northLanding / EARTH_RADIUS) * RAD_TO_DEG;
    landing.longitude = refLongitude + (eastLanding / (EARTH_RADIUS * cos(refLatRad))) * RAD_TO_DEG;
    landing.timetoGroundSec = timeToGround;
    landing.valid = true;
}

#define TEST_MODE_SIMULATE_DESCENT true
const float TEST_SIMULATED_VERTICAL_VEL = -5.0;  // m/s, chute de velocidade sob paraquedas
const float TEST_SIMULATED_ALTITUDE     = 50.0;  // m, chute de altura pra ter "distância" até o solo

void predictLandingPointDebugger(){
    landing.valid = false;

    float verticalVel = kfU.vel;
    float verticalPos = kfU.pos;

    if (TEST_MODE_SIMULATE_DESCENT) {
        // Ignora o launchDone e força uma descida simulada,
        // só pra validar a matemática de projeção com E/N reais do GPS
        verticalVel = TEST_SIMULATED_VERTICAL_VEL;
        verticalPos = TEST_SIMULATED_ALTITUDE;
    } else {
        if (!enu.referenceSet) return;
        if (!launchDone) return;
        if (verticalVel >= -0.1) return;
    }

    float timeToGround = -verticalPos / verticalVel;
    if (timeToGround <= 0 || timeToGround > 600) return;

    float eastLanding  = kfE.pos + kfE.vel * timeToGround;
    float northLanding = kfN.pos + kfN.vel * timeToGround;

    double refLatRad = refLatitude * DEG_TO_RAD;
    landing.latitude  = refLatitude  + (northLanding / EARTH_RADIUS) * RAD_TO_DEG;
    landing.longitude = refLongitude + (eastLanding / (EARTH_RADIUS * cos(refLatRad))) * RAD_TO_DEG;
    landing.timetoGroundSec = timeToGround;
    landing.valid = true;
}

void printKalmanDebug(){
    Serial.println("--- KALMAN DEBUG ---");
    Serial.print("kfE: pos="); Serial.print(kfE.pos, 3);
    Serial.print(" vel="); Serial.println(kfE.vel, 3);

    Serial.print("kfN: pos="); Serial.print(kfN.pos, 3);
    Serial.print(" vel="); Serial.println(kfN.vel, 3);

    Serial.print("kfU: pos="); Serial.print(kfU.pos, 3);
    Serial.print(" vel="); Serial.println(kfU.vel, 3);

    if (landing.valid) {
        Serial.print("POSICAO ATUAL: lat= "); Serial.print(gps.latitude, 6);
        Serial.print("lon= "); Serial.println(gps.longitude, 6);


        Serial.print("POUSO PREVISTO: lat="); Serial.print(landing.latitude, 6);
        Serial.print(" lon="); Serial.print(landing.longitude, 6);
        Serial.print(" t="); Serial.println(landing.timetoGroundSec);
    } else {
        Serial.println("Previsão de pouso: ainda não disponível");
    }
}


void setup(){
    Serial.begin(115200);
    pinMode(BUZZER_PIN, OUTPUT);

    sing(1, BUZZER_PIN);

    initGY87();
    initDHT();
    initGPS();
    initLoRa();
    initSD();
    delay(1000);

    checkSystemHealth();
    calibrateAndInitPitot();
    initWatchdog();      // sempre a ultima função a iniciar, se iniciar antes das demais, pode matar o sistema
                         // pois não foi alimentada, check system health passa pois está sendo alimentada por dentro
}


void loop(){
    currentMillis = millis();
    esp_task_wdt_reset();

    updateGPSPositionWhenIsAvailable();

    if(currentMillis - lastReadGY87 >= READ_GY87_INTERVAL){
        lastReadGY87 = currentMillis;

        //readGY87();

        float dt = (currentMillis - lastKalmanUpdate) / 1000.0;
        kalmanPredict(dt);
        predictLandingPoint();
        //predictLandingPointDebugger(); // apenas debugger
        lastKalmanUpdate = currentMillis;
    }

    if(currentMillis - lastReadDHTAndPitot >= READ_DHT_AND_PITOT_INTERVAL){
        lastReadDHTAndPitot = currentMillis;

        //readDHT();
        readPitot();
    }

    if(currentMillis - lastLoraSend >= LORA_SEND_INTERVAL){
        lastLoraSend = currentMillis;

        //printKalmanDebug(); // debug
        printTelemetryToSerial(); // debug 
        applyNanopbEncoder();
        buildTelemetryPacket();
        // applyNanopbDecoder(); // debug
        sendPacketByLoRa();

        if(launchDone  && !recoveryMode) {
            if(currentMillis - launchTime >= 180000){ //DEFINIR TEMPO
                recoveryMode = true;
                runRecoveryMode();
            }
        }
    }

    if(currentMillis - lastSaveToSD >= SAVE_TO_SD_INTERVAL){
        lastSaveToSD = currentMillis;

        saveTelemetryToSD();
    }

      
    if(!launchDone && (currentMillis - lastCheckSystemHealth >= CHECK_SYSTEM_HEALTH_INTERVAL) && !recoveryMode) {
        lastCheckSystemHealth = currentMillis;
        checkSystemHealth();
    }
}
