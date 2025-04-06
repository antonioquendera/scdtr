#include "mcp2515.h"
#include "pid.h"
#include "CircularBuffer.hpp"  // Include your circular buffer header
#include <map>
#include "commands.h"
#include "pico/multicore.h"
#include <algorithm>  // For std::sort

// Constants and Parameters
#define MSG_HELLO 0x01
#define MSG_CALIBRATE 0x02

const float Vcc = 3.3;          // Supply voltage
const int R = 10000;            // 10kΩ resistor
const int R_0 = 1100000;        // LDR resistance at 1 LUX
const float b = log10(R_0);     // Calibration constant
const float m = 0.8;            // Nominal value for LUX calculation
float reference = 30;           // Reference value for PID
float u_old;                    // Old value for PID calculations
std::map<int, Luminaire> luminaires;


// CAN-BUS Communication Setup
MCP2515 can0 {spi0, 17, 19, 16, 18, 10000000}; // Initialize MCP2515 CAN controller
struct can_frame canMsgTx, canMsgRx;           // CAN message structures
uint8_t node_address;                          // Unique ID for each luminaire
int int_node_address;                          // Integer version of node address
int deskId = 0;                                // Desk ID for the system
std::vector<int> node_ids;                     // List of node IDs (CAN IDs)
int int_sender_id;                             // Sender's node ID from received message

// PID Controller Setup
pid my_pid {0.01, 31.61, 1, 0.01};            // PID controller parameters (K, Ti, Td, sampling period)

// Buffers for Circular Data Storage
CircularBuffer<float, bufferSize> dutyCycleBuffer;      // Circular buffer for duty cycle data
CircularBuffer<float, bufferSize> illuminanceBuffer;    // Circular buffer for illuminance data
int circular_buffer = 1;                            // Flag to indicate buffer state

// Time Management
unsigned long previousTime = 0;                // Last time an operation was performed
unsigned long sampInterval = 1000;              // Sampling interval in milliseconds
unsigned long lastMessageTime = 0;              // Time of last received CAN message
const unsigned long timeout = 2000;             // Timeout for waiting for new CAN message (2 seconds)

// Calibration and System Variables
bool calibrationDone = false;
bool calibrationStarted = false;                    // Flag to indicate if calibration is done
uint8_t lowestNode = 255;                       // Placeholder for the lowest node ID (high initial value)
int calibrationCount = 0;                       // Count for how many luminaires are calibrated
int counter = 0;                                // General counter variable
int num_iluminaires = 0;                        // Number of luminaires in the system
int gains[100];                                 // Array to store gain values for luminaires

// Shared Variables for Multicore Processing
volatile float sharedIlluminance = 0.0;        // Shared illuminance value
volatile int sharedPwmValue = 0;                // Shared PWM value
volatile bool newDataAvailable = false;         // Flag indicating if new data is available

// Interrupt flag for CAN communication
volatile bool got_irq = false;                  // Flag to indicate if a CAN interrupt has occurred

int addUniqueNodeId(int nodeId) {
    // Check if this nodeId is already stored
    for (int id : node_ids) {
        if (id == nodeId) {
            return 0;  // Node already in list
        }
    }
    // Add the new nodeId
    node_ids.push_back(nodeId);
    num_iluminaires++;
    return 1;  // New node added
}

void performCalibration(int deskId) {
    delay(1500);
    Serial.println("Calibrating...");
    float backgroundIlluminance = measureIlluminance(deskId);
    //Serial.printf("Background Illuminance (I_bg): %.2f lux\n", backgroundIlluminance);
    
    float dutyCycles[] = {0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1};
    float illuminanceValues[11];

    for (int i = 0; i < 11; i++) {
        setDutyCycle(deskId, dutyCycles[i]);
        delay(500);
        illuminanceValues[i] = measureIlluminance(deskId);
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

    pico_unique_board_id_t pico_board_id;
    pico_get_unique_board_id(&pico_board_id);
    int_node_address = (pico_board_id.id[5] << 8) | pico_board_id.id[6]; // Get the unique ID from the board (only 5th and 6th bytes are different)
    addUniqueNodeId(int_node_address);

    can0.reset();
    can0.setBitrate(CAN_1000KBPS);
    can0.setNormalMode();

    // Initialize the luminaires, just one for now
    initializeLuminaires(1);
 
    delay(10000);

    // Send a hello message to the CAN bus
    canMsgTx.can_id = int_node_address;
    canMsgTx.can_dlc = 1;
    canMsgTx.data[0] = MSG_HELLO; // 'HELLO' or boot message
    can0.sendMessage(&canMsgTx);
    Serial.println("Setup complete");
    Serial.print("Node Address: ");
    Serial.println(int_node_address);

    // Set up CAN interrupt
    //const uint8_t interruptPin = 20;  // Define the interrupt pin
    //gpio_set_irq_enabled_with_callback(interruptPin, GPIO_IRQ_EDGE_FALL, true, &read_interrupt);

    /*
    Serial.print("---");
    for (int i = 0; i < 8; i++) {
    if (node_address[i] < 0x10) Serial.print("0"); // Leading zero for single digit
    Serial.print(node_address[i], HEX);
    if (i < 7) Serial.print(":");
    }
    Serial.println();
    */
}

void loop() {
    unsigned long currentTime = micros();  // Current time in microseconds
    int flag;

    // Handle serial commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        handleCommand(command);
    }

    // CAN message receiving
    while (can0.readMessage(&canMsgRx) == MCP2515::ERROR_OK) {
        lastMessageTime = currentTime;
        int int_sender_id = canMsgRx.can_id;
        uint8_t received_data = canMsgRx.data[0];
        Serial.print("Message:");
        Serial.println(received_data);

        if(!calibrationDone){ //If calibration has not completed
            
            if(addUniqueNodeId(int_sender_id)){//If a message is received from a new node, add it to the list and skip loop iteration
                continue; //skip rest of loop
            }
            
            if(received_data == MSG_CALIBRATE) {
                //!!! nao poderemos utilizar "int_sender_id" como index
                gains[int_sender_id] = measureIlluminance(deskId); //measure the gain relative to the one that sent me the message
                calibrationCount++;
                ///!!! 
                Serial.print("Measured Gain:");
                Serial.println(gains[int_sender_id]);

                if(int_node_address == node_ids[calibrationCount]){// If my id is the lowest uncalibrated one, start my calibration
                    performCalibration(deskId);
                    canMsgTx.can_id = int_node_address;
                    canMsgTx.can_dlc = 1;
                    canMsgTx.data[0] = MSG_CALIBRATE;  // Signal calibration done
                    can0.sendMessage(&canMsgTx);
                    delay(1500);
                    setDutyCycle(deskId,0); 
                    calibrationCount++;
                }
                if (calibrationCount >= num_iluminaires) { //if all luminaires have been calibrated
                    calibrationDone = true;
                    Serial.println("All luminaires calibrated");
                    
                }
            }
        }
        
    }

    // If calibration has not started and 2 seconds have passed without receiving a message and i have the lowest uncalibrated id, calibrate and send MSG_CALIBRATE
    if(!calibrationStarted){
        if (currentTime - lastMessageTime >= timeout) {
            std::sort(node_ids.begin(), node_ids.end()); // Sort the ids
            num_iluminaires = node_ids.size();
            if(int_node_address == node_ids[calibrationCount]){// If my id is the lowest uncalibrated one
                performCalibration(deskId);
                canMsgTx.can_id = int_node_address;
                canMsgTx.can_dlc = 1;
                canMsgTx.data[0] = MSG_CALIBRATE;  // Signal calibration done
                can0.sendMessage(&canMsgTx);
                delay(1500);
                setDutyCycle(deskId,0); 
                calibrationCount++;
                calibrationStarted = true;
            }
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

    while (calibrationDone) {
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
