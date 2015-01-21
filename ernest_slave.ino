#include <SPI.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>

// SPI pins for 2.4GHz transceiver
#define PIN_R_CE 8
#define PIN_R_CSN 7

// Output pins for status LEDs
#define PIN_L_OK 6
#define PIN_L_ERR 5
#define PIN_L_DATA 4

// Input pins for identity DIP switch
#define PIN_DIP_1 9
#define PIN_DIP_2 10
#define PIN_DIP_3 11
#define PIN_DIP_4 12

//// Hardware abstractions

// Temp/Baro sensor
SFE_BMP180 sensor;
// Altitude Medford, MA
#define ALTITUDE 4.0

// nRF24L01 radio
RF24 radio(PIN_R_CSN, PIN_R_CE);
// Pipe address to communicate on
uint64_t rf_pipe = 0xF0F0F0F0E1LL;

// delay between updates, millis
const unsigned long update_interval = 30*1000;
// Last time we sent temp data, millis
unsigned long last_update_time = 0;

//// Global data state
// Atmospheric data
float temp; // Celcius
float pressure; // Millibar
// Node data
uint8_t node_id;

struct datagram {
    float temp;
    float pressure;
    uint8_t node_id;
};

void updateTemp();
void sendTemp();
void setStatusPins(uint8_t ok, uint8_t err, uint8_t data);

void setup() {
    // Setup serial
    Serial.begin(9600);

    // Setup atmospheric sensor
    if (sensor.begin()){
    } else {
        while(1); // Busy halt
    }

    // Set output pins for the LEDs
    pinMode(PIN_L_OK, OUTPUT);
    pinMode(PIN_L_ERR, OUTPUT);
    pinMode(PIN_L_DATA, OUTPUT);

    // Setup RF
    radio.begin();
    radio.setRetries(15, 15);
    radio.setPayloadSize(8);
    radio.openWritingPipe(rf_pipe);

    radio.startListening();
    radio.printDetails();
}

/*
 * Main loop:
 */
void loop() {
    if(millis() - last_update_time > update_interval) {
        last_update_time = millis();
        updateTemp();
        sendTemp();
    }
    delay(1000);
}

void setStatusPins(uint8_t ok, uint8_t err, uint8_t data){
    digitalWrite(PIN_L_OK, ok);
    digitalWrite(PIN_L_ERR, err);
    digitalWrite(PIN_L_DATA, data);
}

void sendTemp(){
    // First, stop listening so we can talk.
    radio.stopListening();

    // Set the data & send
    struct datagram d =  { temp, pressure, node_id };
    bool ok = radio.write(&d, sizeof(struct datagram));

    if (ok) {
        printf("ok...");
    } else {
        printf("failed.\n\r");
    }

    // Now, continue listening
    radio.startListening();

    // Wait for an ACK from the master
    unsigned long started_waiting_at = millis();
    bool timeout = false;
    while (!radio.available() && !timeout)
        if (millis() - started_waiting_at > 200)
            timeout = true;

    if (timeout) {
        printf("Failed, response timed out.\n\r");
    } else {
        // Grab the response, compare, and send to debugging spew
        unsigned long got_time;
        radio.read(&got_time, sizeof(unsigned long));

        // Spew it
        printf("Got response %lu, round-trip delay: %lu\n\r", got_time, millis() - got_time);
    }
}

void updateTemp(){
    char status;
    double t_celsius,P,p0,a;
    status = sensor.startTemperature();
    if (status != 0){
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed temperature measurement:
        // Note that the measurement is stored in the variable T.
        // Function returns 1 if successful, 0 if failure.
        status = sensor.getTemperature(t_celsius);
        if (status != 0){
            temp = t_celsius;

            // Start a pressure measurement:
            // The parameter is the oversampling setting, from 0 to 3
            // If request is successful, the number of ms to wait is returned.
            // If request is unsuccessful, 0 is returned.
            status = sensor.startPressure(3);
            if (status != 0){
                // Wait for the measurement to complete:
                delay(status);

                // Get the pressure (dependent on temperature)
                status = sensor.getPressure(P,t_celsius);
                if (status != 0){
                    // The pressure sensor returns abolute pressure, which varies with altitude.
                    // To remove the effects of altitude, use the sealevel function and your current altitude.
                    p0 = sensor.sealevel(P,ALTITUDE);
                    pressure = p0;

                }
            }
        }
    }
}
