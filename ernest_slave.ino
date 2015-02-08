#include <SPI.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "printf.h"

// SPI pins for 2.4GHz transceiver
#define PIN_R_CE 10
#define PIN_R_CSN 9

// Output pins for status LEDs
#define PIN_L_OK 2
#define PIN_L_ERR 3
#define PIN_L_DATA 4

// Input pins for identity DIP switch
#define PIN_DIP_1 5
#define PIN_DIP_2 6
#define PIN_DIP_3 7
#define PIN_DIP_4 8

//// Hardware abstractions

// Temp/Baro sensor
SFE_BMP180 sensor;
// Altitude Medford, MA
#define ALTITUDE 4.0

// nRF24L01 radio
RF24 radio(PIN_R_CE, PIN_R_CSN);
// Pipe address to communicate on
uint64_t rf_pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

// delay between updates, millis
const unsigned long update_interval = 30*1000;
// Last time we sent temp data, millis
unsigned long last_update_time = 0;

//// Global data state
// Atmospheric data
float G_TEMP; // Celcius
float G_PRESSURE; // Millibar
// Node data
uint8_t G_NODE_ID;

// Bit to say whether an error has occured and the LEDs should not be cleared
// for the idle animation
bool error_code = false;

struct datagram {
    double temp;
    double pressure;
    uint8_t node_id;
};

void updateTemp();
void sendTemp();
void panic();
void idle();
void setStatusPins(uint8_t ok, uint8_t err, uint8_t data);
void updateNodeID();

/*
 * Halt and display the panic lights
 * (alternating all on, all off)
 */
void panic(){
    while(1){
        setStatusPins(LOW, LOW, LOW);
        delay(1000);
        setStatusPins(HIGH, HIGH, HIGH);
        delay(1000);
    }
}

void idle(){
    setStatusPins(HIGH, LOW, LOW);
    delay(200);
    setStatusPins(HIGH, HIGH, LOW);
    delay(200);
    setStatusPins(HIGH, HIGH, HIGH);
    delay(1000);
    setStatusPins(LOW, HIGH, HIGH);
    delay(200);
    setStatusPins(LOW, LOW, HIGH);
    delay(200);
    setStatusPins(LOW, LOW, LOW);
    delay(500);
}

void setup() {
    // Setup serial
    Serial.begin(9600);
    printf_begin();
    delay(5000);

    // Set output pins for the LEDs
    pinMode(PIN_L_OK, OUTPUT);
    pinMode(PIN_L_ERR, OUTPUT);
    pinMode(PIN_L_DATA, OUTPUT);
    setStatusPins(HIGH, LOW, LOW);

    // Setup atmospheric sensor
    if (sensor.begin()){
    } else {
        panic();
    }

    // Set input pins for the DIP switch
    pinMode(PIN_DIP_1, INPUT);
    pinMode(PIN_DIP_2, INPUT);
    pinMode(PIN_DIP_3, INPUT);
    pinMode(PIN_DIP_1, INPUT);

    // Setup RF
    radio.begin();
    radio.enableAckPayload();
    radio.setRetries(15, 15);
    radio.setPayloadSize(16);
    radio.openWritingPipe(rf_pipes[0]);
    radio.openReadingPipe(1, rf_pipes[1]);
    radio.startListening();

    // Dump details for debugging
    radio.printDetails();

}

/*
 * Main loop:
 */
void loop() {
    if(millis() - last_update_time > update_interval) {
        last_update_time = millis();
        updateNodeID();
        updateTemp();
        sendTemp();
    }

    if (error_code) {
        delay(1000);
    } else {
        idle();
    }
}

void setStatusPins(uint8_t ok, uint8_t err, uint8_t data){
    digitalWrite(PIN_L_OK, ok);
    digitalWrite(PIN_L_ERR, err);
    digitalWrite(PIN_L_DATA, data);
}

void updateNodeID(){
    G_NODE_ID = 0;
    G_NODE_ID |= digitalRead(PIN_DIP_1) << 0;
    G_NODE_ID |= digitalRead(PIN_DIP_2) << 1;
    G_NODE_ID |= digitalRead(PIN_DIP_3) << 2;
    G_NODE_ID |= digitalRead(PIN_DIP_4) << 3;
}

// Send a temp value across the aether.
// LED codes:
// G  R  B
// 0  0  1 - Sending data
// 1  0  1 - Data sent successfully
// 0  1  0 - Sending failed
// 0  1  1 - No ACK received
// 1  0  0 - All done, all well
void sendTemp(){
    // Clear error bit
    error_code = false;

    // Set LEDs for data transmit
    setStatusPins(LOW, LOW, HIGH);

    // Set the data & send
    radio.stopListening();
    struct datagram d =  { G_TEMP, G_PRESSURE, G_NODE_ID };
    bool ok = radio.write(&d, sizeof(struct datagram));
    radio.startListening();

    if (ok) {
        setStatusPins(HIGH, LOW, HIGH);
    } else {
        // Sending data failed, set error status
        setStatusPins(LOW, HIGH, LOW);
        printf("radio.write failed.");
        error_code = true;
        return;
    }

    // Check for an ACK
    uint32_t msg_ack;
    if (radio.isAckPayloadAvailable()){
        radio.read(&msg_ack, sizeof(msg_ack));
        printf("Ack: [%lu] ", msg_ack);
        setStatusPins(HIGH, LOW, LOW);
    } else {
        // Data wasn't ACKd, set data and err LEDs
        setStatusPins(LOW, HIGH, HIGH);
        error_code = true;
    }
}

void updateTemp(){
    char status;
    double t_celsius, p_mbar;
    status = sensor.startTemperature();
    if (status != 0){
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed temperature measurement:
        // Note that the measurement is stored in the variable T.
        // Function returns 1 if successful, 0 if failure.
        status = sensor.getTemperature(t_celsius);
        if (status != 0){
            G_TEMP = t_celsius;

            // Start a pressure measurement:
            // The parameter is the oversampling setting, from 0 to 3
            // If request is successful, the number of ms to wait is returned.
            // If request is unsuccessful, 0 is returned.
            status = sensor.startPressure(3);
            if (status != 0){
                // Wait for the measurement to complete:
                delay(status);

                // Get the pressure (dependent on temperature)
                status = sensor.getPressure(p_mbar, t_celsius);
                if (status != 0){
                    // The pressure sensor returns abolute pressure, which varies with altitude.
                    // To remove the effects of altitude, use the sealevel function and your current altitude.
                    G_PRESSURE = sensor.sealevel(p_mbar, ALTITUDE);
                }
            }
        }
    }
}
