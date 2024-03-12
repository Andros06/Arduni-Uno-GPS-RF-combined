#include <Arduino.h>
#include <TinyGPS++.h>
#include <RH_RF95.h>
#include <AltSoftSerial.h>

// Define the pins for AltSoftSerial
#define GPS_RX_PIN 8
#define GPS_TX_PIN 7

AltSoftSerial gpsSerial; // Define AltSoftSerial object for GPS communication

TinyGPSPlus gps; // Create a TinyGPS++ object

// Define LoRa pins
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2

// Define LoRa frequency
#define RF95_FREQ 915.0

// Singleton instance of the radio driver'
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
    // Initialize serial communication for debugging
    Serial.begin(9600);

    // Initialize AltSoftSerial for GPS module
    gpsSerial.begin(9600);
    
    // Set LoRa reset pin as output
    pinMode(RFM95_RST, OUTPUT);
    
    // Set LoRa reset pin high
    digitalWrite(RFM95_RST, HIGH);
    
    // Wait for serial communication to be established
    while (!Serial);
    
    // Initialize LoRa module
    if (!rf95.init()) {
        Serial.println("LoRa radio init failed");
        while (1);
    }
    Serial.println("LoRa radio init OK!");

    // Set LoRa frequency
    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("setFrequency failed");
        while (1);
    }
    Serial.print("Set Freq to: ");
    Serial.println(RF95_FREQ);
    
    // Set LoRa transmit power
    rf95.setTxPower(23, false);
}

void loop() {
    // Read data from GPS module
    while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
            if (gps.location.isValid()) {
                // Prepare GPS data to send over LoRa
                String gpsData = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
                
                // Convert GPS data to char array
                char radiopacket[32];
                gpsData.toCharArray(radiopacket, 32);
                
                // Send GPS data over LoRa
                Serial.println("Sending GPS data over LoRa");
                rf95.send((uint8_t *)radiopacket, gpsData.length());
                
                // Wait for LoRa transmission to complete
                rf95.waitPacketSent();
            }
        }
    }
}
