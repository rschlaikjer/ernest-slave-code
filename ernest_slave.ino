#include <SPI.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>

// SPI pins for 2.4GHz transceiver
#define PIN_R_CE 8
#define PIN_R_CSN 7

// Output pins for status LEDs
#define PIN_L_OK 12
#define PIN_L_ERR 11
#define PIN_L_DATA 10

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
float G_TEMP; // Celcius
float G_PRESSURE; // Millibar
// Node data
uint8_t G_NODE_ID;

struct datagram {
    float temp;
    float pressure;
    uint8_t node_id;
};

void updateTemp();
void sendTemp();
void setStatusPins(uint8_t ok, uint8_t err, uint8_t data);
void updateNodeID();

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

    // Set input pins for the DIP switch
    pinMode(PIN_DIP_1, INPUT);
    pinMode(PIN_DIP_2, INPUT);
    pinMode(PIN_DIP_3, INPUT);
    pinMode(PIN_DIP_1, INPUT);

    // Setup RF
    radio.begin();
    radio.enableAckPayload();
    //radio.setRetries(15, 15);
    //radio.setPayloadSize(8);
    radio.openWritingPipe(rf_pipe);

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
    delay(1000);
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
    // Set LEDs for data transmit
    setStatusPins(LOW, LOW, HIGH);

    // Set the data & send
    struct datagram d =  { G_TEMP, G_PRESSURE, G_NODE_ID };
    bool ok = radio.write(&d, sizeof(struct datagram));

    if (ok) {
        setStatusPins(HIGH, LOW, HIGH);
    } else {
        // Sending data failed, set error status
        setStatusPins(LOW, HIGH, LOW);
        printf("radio.write failed.");
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
