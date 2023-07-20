#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <TTN_esp32.h>
#include "DHT.h"
#include "ByteArrayUtils.h"
/***************************************************************************
 *  Go to your TTN console register a device then the copy fields
 *  and replace the CHANGE_ME strings below
 ****************************************************************************/
// const char *devEui = "80B3D57ED004397E";                 // Change to TTN Device EUI
// const char *appEui = "1300000000000014";                 // Change to TTN Application EUI
// const char *appKey = "47521E11573093C237C7333983DD475C"; // Chaneg to TTN Application Key

double temp = 0;
double hum = 0;
double bat = 0;

TTN_esp32 ttn;

#define DHTPIN 10
#define DHTTYPE DHT22

#define EXT_WAKEUP_PIN 2
#define ADC_READ_PIN 1

#define C3

#define SLEEP_SECONDS 20

#ifdef C3

#define UNUSED_PIN 0xFF
#define SS 7
#define RST_LoRa 3
#define DIO0 18
#define DIO1 19
#define DIO2 0xFF
#define REG_3V3_EN 0
#else
#define UNUSED_PIN 0xFF
#define SS 5
#define RST_LoRa 15
#define DIO0 12
#define DIO1 14
#define DIO2 27
#endif
struct SensorData
{
    uint32_t temperature : 10;   // 10 bit sıcaklık alanı
    uint32_t batteryVoltage : 9; // 9 bit batarya voltajı alanı
    uint32_t humidity : 10;      // 7 bit nem alanı
    uint32_t motorStatus : 1;    // 1 bit motor çalışma bilgisi alanı
};

struct DateData
{
    uint32_t hour1 : 5; // 10 bit sıcaklık alanı
    uint32_t min1 : 6;  // 9 bit batarya voltajı alanı
    uint32_t hour2 : 5; // 7 bit nem alanı
    uint32_t min2 : 6;  // 1 bit motor çalışma bilgisi alanı
};

uint32_t unpackedData = 0;
struct DateData day;

DHT dht(DHTPIN, DHTTYPE);

void print_wakeup_reason()
{
    esp_sleep_wakeup_cause_t wakeup_reason;
    wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason)
    {
    case 1:
        Serial.println("Wakeup caused by external signal using RTC_IO");
        break;
    case 2:
        Serial.println("Wakeup caused by external signal using RTC_CNTL");
        break;
    case 3:
        Serial.println("Wakeup caused by timer");
        break;
    case 4:
        Serial.println("Wakeup caused by touchpad");

        break;
    case 5:
        Serial.println("Wakeup caused by ULP program");
        break;
    default:
        Serial.println("Wakeup was not caused by deep sleep");

        break;
    }
}

void waitForTransactions()
{
    Serial.println("Waiting for pending transactions... ");
    Serial.println("Waiting took " + String(ttn.waitForPendingTransactions()) + "ms");
}

void unPack(){
    day.min1 = unpackedData & (0x3F);
    day.hour1 = (unpackedData & ((0x1F) << 6)) >> 6;
    day.min2 = (unpackedData & ((0x3F) << 11)) >> 11;
    day.hour2 = (unpackedData & ((0x1F) << 17)) >> 17;

    Serial.print(day.hour1);
    Serial.println(day.min1);
    Serial.print(day.hour2);
    Serial.println(day.min2);

}
void message(const uint8_t *payload, size_t size, uint8_t port, int rssi)
{
    Serial.println("-- MESSAGE");
    Serial.printf("Received %d bytes on port %d (RSSI=%ddB) :", size, port, rssi);

    unpackedData = 0;

    for (int i = 0; i < size; i++)
    {
        Serial.printf(" %x", payload[i]);
        unpackedData |= (payload[i]) << ((size - 1 - i) * 8);
    }

    unpackedData = unpackedData & 0x3fffff;
    

    Serial.println();
    Serial.printf("%x ", unpackedData);
    Serial.println();
    unPack();

}

// 32 bit veriyi göndermek için fonksiyon
bool send32BitData(uint32_t data)
{
    uint8_t payload[4];
    payload[0] = (data >> 24) & 0xFF; // En üst 8 bit
    payload[1] = (data >> 16) & 0xFF; // Sonraki 8 bit
    payload[2] = (data >> 8) & 0xFF;  // Sonraki 8 bit
    payload[3] = data & 0xFF;         // En alt 8 bit

    // Gönderim işlemini yap
    bool sendStatus = ttn.sendBytes(payload, 4, 1, 0);

    return sendStatus;
}

// void Control(){

// while(digitalRead(2)==HIGH){
//     if(millis()>5000){
//         Serial.println("bluetooth starting...");
//         ttn.stop();
// 		ble.begin();

//     }
//     else Serial.println("TTN will work");

// }

// }
void setup()
{
    pinMode(REG_3V3_EN, OUTPUT);
    digitalWrite(REG_3V3_EN, HIGH);
    Serial.begin(115200);
    struct SensorData sensorData;

    // Control();

    // print_wakeup_reason();
    // if(wakeByButton == true){
    // Serial.println("wake By Button");

    // }

    dht.begin();
    delay(100);
    Serial.println("Starting");
    delay(900);

    bat = analogRead(ADC_READ_PIN) * (2.9 / 4095) * 4.4;

    sensorData.temperature = (unsigned int)((dht.readTemperature() * 100) / 10);
    sensorData.humidity = (unsigned int)((dht.readHumidity() * 100) / 10);
    sensorData.batteryVoltage = (unsigned int)((bat * 100));
    sensorData.motorStatus = 0;

    uint32_t packedData = *(uint32_t *)&sensorData;

    Serial.println(sensorData.temperature);
    Serial.println(dht.readTemperature());
    Serial.println();

    Serial.println(sensorData.humidity);
    Serial.println(dht.readHumidity());
    Serial.println();

    Serial.println(sensorData.batteryVoltage);
    Serial.println(bat);

    bat = analogRead(ADC_READ_PIN) * (2.9 / 4095) * 4.4;

    // Serial.println(temp);
    // Serial.println(hum);
    // Serial.println(bat);

    // setCpuFrequencyMhz(10); // reduce clock to consume low current

    // Print the wakeup reason for ESP32

    ttn.begin(SS, UNUSED_PIN, RST_LoRa, DIO0, DIO1, DIO2);
    // Declare callback function for handling downlink messages from server
    ttn.onMessage(message);
    // Join the network

    ttn.join(/*devEui, appEui, appKey*/);

    LMIC_setAdrMode(false);
    LMIC_setDrTxpow(DR_SF10, 14);

    Serial.print("Joining ChirpStack Server");
    while (!ttn.isJoined())
    {
        Serial.print(".");
        delay(500);
    }
    Serial.println("\njoined!");

    // Make sure any pending transactions are handled first
    waitForTransactions();

    // Send our data
    send32BitData(packedData);

    // Make sure our transactions is handled before going to sleep
    waitForTransactions();

    // Sleep time in micro seconds so multiply by 1000000
    esp_sleep_enable_timer_wakeup(SLEEP_SECONDS * 1000000);

    // Set wakeUp pin to wake the system up when button is pressed
    // esp_deep_sleep_enable_gpio_wakeup(1 << EXT_WAKEUP_PIN, ESP_GPIO_WAKEUP_GPIO_HIGH);

    // Go to sleep now
    Serial.println("Going to sleep!");
    esp_deep_sleep_start();
    // Everything beyond this point will never be called
}

void loop()
{
    // Never called
}