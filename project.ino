#include "mcp2515.h"
#include "pid.h"
#include "CircularBuffer.hpp"  // Include your circular buffer header
#include <map>
#include "commands.h"
#include "pico/multicore.h"

const float Vcc = 3.3;      // Supply voltage
const int R = 10000;        // 10kΩ resistor
const int R_0 = 1100000;    // LDR resistance at 1 LUX
const float b = log10(R_0); 
const float m = 0.8;        // Nominal value
float reference = 30;
float u_old;

CircularBuffer<float, bufferSize> dutyCycleBuffer;
CircularBuffer<float, bufferSize> illuminanceBuffer;
int circular_buffer = 1;

// PID Controller Initialization
pid my_pid {0.01, 31.61, 1, 0.01 }; // K was previously 1

// CAN-BUS communication innitialization
MCP2515 can0 {spi0, 17, 19, 16, 18, 10000000}; // Initialize MCP2515
pico_unique_board_id_t pico_board_id;
struct can_frame canMsgTx, canMsgRx;
uint8_t node_address = 0x01; // Unique ID for each luminaire
int int_node_address = node_address;

unsigned long previousTime = 0;  // Last time an operation was performed
unsigned long sampInterval = 1000; // Sampling interval

std::map<int, Luminaire> luminaires;

// Shared variables for multicore processing
volatile float sharedIlluminance = 0.0;
volatile int sharedPwmValue = 0;
volatile bool newDataAvailable = false;

// Interrupt flag for CAN communication
volatile bool got_irq = false;

// Calibration Variables
bool calibrationDone = false;
uint8_t lowestNode = 255; // Initialize with a high value

int calibrationCount = 0;
bool loop1_flag = false;
int counter = 0;
const int num_iluminaires = 2;
uint8_t node_addresses[num_iluminaires] = {0x00, 0x01};
int gains[num_iluminaires] = {0, 0}; 

void performCalibration() {
    delay(1500);
    Serial.println("Calibrating...");
    float backgroundIlluminance = measureIlluminance();
    //Serial.printf("Background Illuminance (I_bg): %.2f lux\n", backgroundIlluminance);
    
    float dutyCycles[] = {0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1};
    float illuminanceValues[11];

    for (int i = 0; i < 11; i++) {
        setDutyCycle(dutyCycles[i]);
        delay(500);
        illuminanceValues[i] = measureIlluminance();
        //Serial.printf("Duty Cycle: %.2f, Measured Illuminance: %.2f lux\n", dutyCycles[i], illuminanceValues[i]);
    }
    
    float sumGain = 0.0;
    for (int i = 1; i < 11; i++) {
        float gain = (illuminanceValues[i] - backgroundIlluminance) / dutyCycles[i];
        sumGain += gain;
    }
    
    float staticGain = sumGain / 10.0;
    //Serial.printf("Static Gain (K): %.2f lux/unit\n", staticGain);
    Serial.println("Calibration done"); 

}


void setup() {
    Serial.begin(115200);
    pinMode(ANALOG_PIN, INPUT);
    pinMode(LED_PWM_PIN, OUTPUT);
    analogReadResolution(12);
    analogWriteFreq(30000); // 30KHz PWM Frequency
    analogWriteRange(4095); // Max PWM value

    can0.reset();
    can0.setBitrate(CAN_1000KBPS);
    can0.setNormalMode();

    // Set up CAN interrupt
    //const uint8_t interruptPin = 20;  // Define the interrupt pin
    //gpio_set_irq_enabled_with_callback(interruptPin, GPIO_IRQ_EDGE_FALL, true, &read_interrupt);

    // Initialize the luminaires, just one for now
    initializeLuminaires(1);
    delay(10000);
    Serial.println("Setup complete");
}

void loop() {
    unsigned long currentTime = micros();  // Current time in microseconds

    // Handle serial commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        handleCommand(command);
    }

    if (!calibrationDone && int_node_address == 0) {
      performCalibration();
      calibrationDone = true;
      canMsgTx.can_id = node_address;
      canMsgTx.can_dlc = 1;
      canMsgTx.data[0] = 0xFF;  // Signal calibration done
      can0.sendMessage(&canMsgTx);
      delay(1500);
      setDutyCycle(0); 
      calibrationCount++;
    }

    // CAN message receiving
    while (can0.readMessage(&canMsgRx) == MCP2515::ERROR_OK) {
        uint8_t sender_id = canMsgRx.can_id;
        int int_sender_id = sender_id;
        uint8_t received_data = canMsgRx.data[0];
        Serial.print("Message:");
        Serial.println(received_data);
        //Serial.println(int_node_address);

        if(received_data == 0xFF) {
            gains[int_sender_id] = measureIlluminance();
            calibrationCount++;
            Serial.print("Measured Gain:");
            Serial.println(gains[int_sender_id]);
        }
        if (!calibrationDone && received_data == 0xFF && int_sender_id == int_node_address - 1) {
                performCalibration();
                calibrationDone = true;
                canMsgTx.can_id = node_address;
                canMsgTx.can_dlc = 1;
                canMsgTx.data[0] = 0xFF;  // Signal calibration done
                can0.sendMessage(&canMsgTx);
                delay(1500);
                setDutyCycle(0); 
                calibrationCount++;
        }
        if (calibrationCount >= num_iluminaires) {
            loop1_flag = true;
            Serial.println("All luminaires calibrated");
        }
    }
    /*
    // Print buffer data periodically
    if (circular_buffer == 1 && dutyCycleBuffer.isFull()) {
        circular_buffer = 0;
        for (int i = 0; i < dutyCycleBuffer.size(); i += 20) {
            Serial.print(dutyCycleBuffer[i]);  // Print duty cycle
            Serial.print(",");
            Serial.print(illuminanceBuffer[i]);  // Print illuminance
            Serial.println();
        }
    }*/

    previousTime = currentTime;
}

// This function runs on Core 1
void loop1() {
    
    can_frame frm;
    uint32_t msg;
    uint8_t b[4];

    while (loop1_flag) {
        unsigned long currentTime = micros();  // Current time in microseconds

        // Check if the required interval has passed
        if (currentTime - previousTime > sampInterval) {
            int adcValue = analogRead(ANALOG_PIN);
            float voltage = (adcValue / 4095.0) * Vcc;
            sharedIlluminance = Luxmeter(voltage);

            float control_signal = my_pid.compute_control(reference, sharedIlluminance);
            sharedPwmValue = (int)control_signal;
            sharedPwmValue = constrain(sharedPwmValue, 0, 4095);
            analogWrite(LED_PWM_PIN, sharedPwmValue);

            // Perform housekeeping on PID controller
            u_old = my_pid.housekeep(reference, sharedIlluminance, sharedPwmValue);

            // Store duty cycle and illuminance in CircularBuffer
            dutyCycleBuffer.push(sharedPwmValue / 4095.0);  // Normalize control signal to [0, 1]
            illuminanceBuffer.push(sharedIlluminance);

            // Print metrics
            float energy = calculateEnergy();
            float visibilityError = calculateVisibilityError(reference);
            float flicker = calculateFlicker();

            luminaires[0].power_consumption = energy;

            newDataAvailable = true;  // Flag to indicate new data

            previousTime = currentTime;
        }
        
        /**        // Reading the CAN bus and writing to the FIFO
        if (got_irq) {
            got_irq = false;
            uint8_t irq = can0.getInterrupts();
            if (irq & MCP2515::CANINTF_RX0IF) {
                can0.readMessage(MCP2515::RXB0, &frm);
                rp2040.fifo.push_nb(can_frame_to_msg(&frm));
            }
            if (irq & MCP2515::CANINTF_RX1IF) {
                can0.readMessage(MCP2515::RXB1, &frm);
                rp2040.fifo.push_nb(can_frame_to_msg(&frm));
            }
            uint8_t err = can0.getErrorFlags();
            rp2040.fifo.push_nb(error_flags_to_msg(irq, err));
        }

        // Reading FIFO and writing to the CAN bus
        if (rp2040.fifo.pop_nb(&msg)) {
            msg_to_bytes(msg, b);
            if (b[3] == ICC_WRITE_DATA) {
                frm.can_id = b[2];
                frm.can_dlc = 2;
                frm.data[1] = b[1];
                frm.data[0] = b[0];
                can0.sendMessage(&frm);
            }
            uint8_t irq = can0.getInterrupts();
            uint8_t err = can0.getErrorFlags();
            rp2040.fifo.push_nb(error_flags_to_msg(irq, err));
        }
            */
    }
}
