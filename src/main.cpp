#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <TTN_esp32.h>
// #include "TTN_CayenneLPP.h"
#include <Adafruit_BME280.h>

#include <WiFi.h>
#include <AsyncEventSource.h>
#include <AsyncTCP.h>
#include <AsyncElegantOTA.h>
#include <WebSerial.h>
/***************************************************************************
 *  Go to your TTN console register a device then the copy fields
 *  and replace the CHANGE_ME strings below
 ****************************************************************************/
const char *devEui = "70B3D57ED004397D";                 // Change to TTN Device EUI
const char *appEui = "1300000000000013";                 // Change to TTN Application EUI
const char *appKey = "47521E11573093C237C7333983DD475C"; // Chaneg to TTN Application Key

float temp;
const char *messagee = "selam";

#define BME_SDA 21 // GPIO pin connected to BME280's SDA
#define BME_SCL 22 // GPIO pin connected to BME280's SCL
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // Create an instance of the BME280 sensor

TTN_esp32 ttn;

AsyncWebServer server(80);

// #define C3
const char *ssid = "ISUBU_WiFi";
const char *password = "DenemE123!!";

#define SLEEP_SECONDS 10

#ifdef C3

#define UNUSED_PIN 0xFF
#define SS 7
#define RST_LoRa 3
#define DIO0 1
#define DIO1 2
#define DIO2 0xFF

#else
#define UNUSED_PIN 0xFF
#define SS 5
#define RST_LoRa 15
#define DIO0 12
#define DIO1 14
#define DIO2 27
#endif

bool isSleep = false;
bool wifiAcikMi = false;
static unsigned long wifiAcilmaZamani = 0;

String sensorOku();
void wifiKontrol();
void WiFiBaglan(bool durum);
void deviceGoingToSleep();
void sendToChirpstackData();
void recvMsg(uint8_t *data, size_t len);
void webServer();

/* Message callback of WebSerial */
void recvMsg(uint8_t *data, size_t len)
{
    WebSerial.print("Webden gelen geri:");
    String d = "";
    for (int i = 0; i < len; i++)
    {
        d += char(data[i]);
    }
    WebSerial.println(d);
}

void message(const uint8_t *payload, size_t size, uint8_t port, int rssi)
{
    Serial.println("-- MESSAGE");
    Serial.printf("Received %d bytes on port %d (RSSI=%ddB) :", size, port, rssi);
    
    String gelen = "";
    for (int i = 0; i < size; i++)
    {
        Serial.printf(" %02X", payload[i]);
        //Serial.printf(" %c", payload[i]);
        gelen += char(payload[i]);

        if (payload[i] == 0x33) //uyan
            isSleep = false;

        if (payload[i] == 0x55) //uyu
            isSleep = true;
            deviceGoingToSleep();

        if (wifiAcikMi)
        {
            //WebSerial.println(gelen);
            WebSerial.printf(" %02X", payload[i]);
        }
    }
    WebSerial.println();
    
}

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

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting");

    pinMode(2, OUTPUT);
    digitalWrite(2, HIGH);
    delay(300);

    // bool status = bme.begin(0x76);

    // if (!status) {
    // Serial.println("Could not find a valid BME280 sensor, check wiring!");
    // while (1);
    // }

    // Serial.println("Bme280 init and Get Temp value");
    // temp = bme.readTemperature();

    // setCpuFrequencyMhz(10); // reduce clock to consume low current

    // Print the wakeup reason for ESP32
    print_wakeup_reason();

    ttn.begin(SS, UNUSED_PIN, RST_LoRa, DIO0, DIO1, DIO2);
    // Declare callback function for handling downlink messages from server
    ttn.onMessage(message);
    // Join the network
    ttn.join(devEui, appEui, appKey);
    Serial.print("Joining TTN ");
    while (!ttn.isJoined())
    {
        Serial.print(".");
        delay(500);
    }
    Serial.println("\njoined!");

    // Make sure any pending transactions are handled first

    sendToChirpstackData();

    // Configure GPIO33 as ext0 wake up source for HIGH logic level
    // esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 1);
    // OR
    // Set timer to 30 seconds
    // Sleep time in micro seconds so multiply by 1000000
    if (isSleep == true)
    {
        deviceGoingToSleep();
    }
    // Everything beyond this point will never be called
}

void deviceGoingToSleep()
{
    esp_sleep_enable_timer_wakeup(SLEEP_SECONDS * 1000000);
    // Go to sleep now
    Serial.println("Going to sleep!");
    esp_deep_sleep_start();
}

void sendToChirpstackData()
{
    waitForTransactions();
    // Send our data
    sendData(sensorOku().c_str());
    // Make sure our transactions is handled before going to sleep
    waitForTransactions();
}

String sensorOku()
{
    String selam = "selam";
    String tempp = (String)temp;
    String gidenVeri = selam + "-" + tempp;

    return gidenVeri;
}

void wifiKontrol()
{
    static unsigned long sonMesajGondermeZamani = 0;

    if (isSleep == false)
    {
        if (!wifiAcikMi)
        {
            WiFiBaglan(true);
        }

        else if (millis() - wifiAcilmaZamani > 300000)
        {
            WiFiBaglan(false);
        }

        else if (millis() - sonMesajGondermeZamani > 10000)
        {
            sonMesajGondermeZamani = millis();
            sendToChirpstackData();
        }

        else if (wifiAcikMi)
        {
            WebSerial.println("I'm awakeee");
        }
    }

}

void WiFiBaglan(bool durum)
{
    if (durum)
    {
        WiFi.begin(ssid, password);
        Serial.println("WiFiye Baglaniyor..");
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(200);
            Serial.print(".");
        }
        Serial.println(WiFi.localIP());
        wifiAcilmaZamani = millis();
        wifiAcikMi = true;

        Serial.println("WiFi'ye Baglandi");
        webServer();
    }

    else if (!durum)
    {
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        Serial.println("WiFi baglantisi koparildi");
        wifiAcikMi = false;
        isSleep = true;
        deviceGoingToSleep();
    }
}

void webServer()
{
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/plain", "Hi! This is a sample response."); });

    AsyncElegantOTA.begin(&server); // Start AsyncElegantOTA
    WebSerial.begin(&server);
    /* Attach Message Callback */
    WebSerial.msgCallback(recvMsg);
    server.begin();
    Serial.println("HTTP server started");
}

void loop()
{
    wifiKontrol();
    Serial.println("awakeee");
    delay(2000);
}