#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <TTN_esp32.h>
// #include "TTN_CayenneLPP.h"
#include <ArduinoJson.h>
#include "DHT.h"
/***************************************************************************
 *  Go to your TTN console register a device then the copy fields
 *  and replace the CHANGE_ME strings below
 ****************************************************************************/
const char *devEui = "70B3D57ED004397D";                 // Change to TTN Device EUI
const char *appEui = "1300000000000013";                 // Change to TTN Application EUI
const char *appKey = "47521E11573093C237C7333983DD475C"; // Chaneg to TTN Application Key

double temp = 12.543;
double hum = 30.543;

String state = "normal";

TTN_esp32 ttn;
StaticJsonDocument<96> doc;

#define DHTPIN 10
#define DHTTYPE DHT22

#define EXT_WAKEUP_PIN 2

#define C3

#define SLEEP_SECONDS 45

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

// Declerations
void parseJson(String gelen);
String formedAsJSON(double temp, double hum, String state);
double round2(double value);
String getSensorData();

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

/*
Önce mesaj node-red tarafında JSON String olarak gönderiliyor burada biz bunu byte byte alıyoruz.
Daha sonrasında byte byte alınan veri bir String JSON olarak birleştiriliyor daha sonrasında JSON
kütüphanesi deserialize fonksiyonu ile parse işlemi gerçekleştiriliyor.
*/

void message(const uint8_t *payload, size_t size, uint8_t port, int rssi)
{
    Serial.println("-- MESSAGE");
    Serial.printf("Received %d bytes on port %d (RSSI=%ddB) :", size, port, rssi);
    String asciiData;
    for (int i = 0; i < size; i++)
    {
        Serial.printf(" %c", payload[i]);
        asciiData += (char)payload[i];
    }
    Serial.println();
    Serial.println(asciiData);
    parseJson(asciiData);
}

void parseJson(String gelen)
{
    DeserializationError error = deserializeJson(doc, gelen);

    if (error)
    {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        return;
    }

    int no = doc["no"];                   // 12
    float temp = doc["temp"];             // 24.5
    const char *message = doc["message"]; // "selam"
    Serial.println(no);
    Serial.println(temp);
    Serial.println(message);
}

/*
Veri önce formedAsJSON fonksiyonu ile JSON String ifadesine dönüştürülüyor serialize fonksiyonu ile
sonrasunda veri byte byte bir char dizisi içerisine dolduruluyor ve gönderiliyor
*/
void sendData(const char *message)
{
    // Metni uint8_t türünden bir byte dizisine dönüştürme
    size_t length = strlen(message);
    uint8_t payload[length];
    for (size_t i = 0; i < length; i++)
    {
        payload[i] = static_cast<uint8_t>(message[i]);
    }
    ttn.sendBytes(payload, sizeof(payload), 1, 0);
}

String formedAsJSON(double tempp, double humm, String state)
{
    StaticJsonDocument<200> doc;
    // Serial.println(humm);
    // Serial.println(temp);

    doc["temp"] = round2(tempp);
    doc["hum"] = round2(humm);
    doc["state"] = state;
    // JSON verisini serileştirme
    String json;
    serializeJson(doc, json);
    Serial.println(json);
    // serializeJson(doc,Serial);

    return json;
}

double round2(double value)
{
    return (int)(value * 100 + 0.5) / 100.0;
}

void setup()
{
    Serial.begin(115200);
    dht.begin();
    delay(500);
    Serial.println("Starting");

    temp = dht.readTemperature();
    hum = dht.readHumidity();

    Serial.println(temp);
    Serial.println(hum);

    pinMode(REG_3V3_EN, OUTPUT);
    digitalWrite(REG_3V3_EN, HIGH);
    delay(300);

    // setCpuFrequencyMhz(10); // reduce clock to consume low current

    // Print the wakeup reason for ESP32
    print_wakeup_reason();

    ttn.begin(SS, UNUSED_PIN, RST_LoRa, DIO0, DIO1, DIO2);
    // Declare callback function for handling downlink messages from server
    ttn.onMessage(message);
    // Join the network

    // LMIC_setAdrMode(false);

    ttn.join(devEui, appEui, appKey);

    LMIC_setAdrMode(false);
    LMIC_setDrTxpow(DR_SF12, 14);

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
    sendData(formedAsJSON(temp, hum, state).c_str());
    // Make sure our transactions is handled before going to sleep
    waitForTransactions();

    // Configure GPIO33 as ext0 wake up source for HIGH logic level
    // esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 1);
    // OR
    // Set timer to 30 seconds
    // Sleep time in micro seconds so multiply by 1000000
    
    esp_sleep_enable_timer_wakeup(SLEEP_SECONDS * 1000000);

    esp_deep_sleep_enable_gpio_wakeup(1<<EXT_WAKEUP_PIN ,ESP_GPIO_WAKEUP_GPIO_HIGH);

    // Go to sleep now
    Serial.println("Going to sleep!");
    esp_deep_sleep_start();
    // Everything beyond this point will never be called
}

void loop()
{
    // Never called
}