#include "mcp2515.h"
#include "pid.h"
#include "CircularBuffer.hpp"  // Include your circular buffer header
#include <map>
#include "commands.h"
#include "pico/multicore.h"
#include "hardware/watchdog.h"
#include <algorithm>  // For std::sort
#include <vector>

// Constants and Parameters


// PID parameters
float h = h_std;
float K = K_std;
float b_PID = b_std;
float Ti = Ti_std;
float Td = Td_std;
float N = N_std;
float Tt = Tt_std;

// ADMM parameters (tunable)
float admm_rho = 0.6;               // The penalty parameter for ADMM
float alpha = 0.3;                  // Energy weighting factor
float admm_x_local = 0.0;           // Local update variable (result of local optimization)
float admm_u_local = 0.0;           // Local dual variable
float admm_z_global = reference;    // Global consensus variable (initialized to the reference)

// how often the ADMM runs (ms)
const unsigned long ADMM_TIMEOUT = 200;      // CAN message wait time in ms
static unsigned long lastAdmmTime = 0;
const unsigned long ADMM_RUN_PERIOD = 3000; // 1 second, can be tuned

float best_u_local = 0.0;



const float Vcc = 3.3;          // Supply voltage
const int R = 10000;            // 10kΩ resistor
const int R_0 = 250000;        // LDR resistance at 1 LUX
const float b = 6.1;     // Calibration constant
const float m = - 0.8;            // Nominal value for LUX calculation
float reference = 200;           // Reference value for PID
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
pid my_pid {h, K, b_PID, Ti, Td, N, Tt};            // PID controller parameters (K, Ti, Td, sampling period)

// Buffers for Circular Data Storage
constexpr int bufferSize = 2000;
int circular_buffer = 1;                            // Flag to indicate buffer state

// Time Management
unsigned long previousTime = 0;                // Last time an operation was performed
unsigned long sampInterval = 10000;              // Sampling interval in microseconds (10ms)
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
int hub_node = false;
bool new_node = false;
bool stream_y = false;                          //Flag to set illuminance stream on or off
bool stream_u = false;                          //Flag to set duty cycle stream on or off

// Shared Variables for Multicore Processing
volatile float sharedIlluminance = 0.0;        // Shared illuminance value
volatile int sharedPwmValue = 0;                // Shared PWM value
volatile bool newDataAvailable = false;         // Flag indicating if new data is available

// Interrupt flag for CAN communication
volatile bool got_irq = false;                  // Flag to indicate if a CAN interrupt has occurred

// bools for streaming
bool streamDutyCycle = false;                  // Flag to indicate if duty cycle streaming is enabled
bool streamIlluminance = false;                // Flag to indicate if illuminance streaming is enabled

int timeStreamU = 0.0; // Time for duty cycle streaming
int timeStreamY = 0.0; // Time for illuminance streaming

unsigned long lastRestartTime = micros();

int num_cycles = 0; // Number of cycles for the PID controller

bool receiving_buffer = false; // Flag to indicate if the buffer is being received

float totalEnergy = 0; // Total energy consumption
float totalVisibilityError = 0; // Total visibility error
float totalFlicker = 0; // Total flicker
float avgEnergy = 0; // Average energy consumption
float avgVisibilityError = 0; // Average visibility error
float avgFlicker = 0; // Average flicker



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

float performCalibration(int deskId) {
    delay(1500);
    Serial.println("Calibrating...");
    float backgroundIlluminance = measureIlluminance();
    //Serial.printf("Background Illuminance (I_bg): %.2f lux\n", backgroundIlluminance);
    
    int dutyCycles[] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
    float illuminanceValues[11];

    for (int i = 0; i < 11; i++) {
        setDutyCycle(deskId, dutyCycles[i], node_ids.data(), node_ids[deskId], can0);
        delay(500);
        illuminanceValues[i] = measureIlluminance();
        //Serial.printf("Duty Cycle: %.2f, Measured Illuminance: %.2f lux\n", dutyCycles[i], illuminanceValues[i]);
    }
    // Perform linear regression: y = a * x + b
    float sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumX2 = 0.0;
    int n = 11;

    for (int i = 0; i < n; i++) {
        float x = dutyCycles[i]/100;
        float y = illuminanceValues[i];

        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumX2 += x * x;
    }

    float denominator = (n * sumX2 - sumX * sumX);
    float a_regression = (n * sumXY - sumX * sumY) / denominator;  // Slope (static gain)

    Serial.println("Calibration done"); 
    Serial.printf("Static Gain (a): %.2f\n", a_regression);
    return a_regression;

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
    int_node_address = pico_board_id.id[5] ;// Get the unique ID from the board (only 5th and 6th bytes are different)
    addUniqueNodeId(int_node_address);

    can0.reset();
    can0.setBitrate(CAN_1000KBPS);
    can0.setNormalMode();

    // Initialize the luminaires, just one for now
    initializeLuminaires(1);

    //get external illuminance
    int adcValue = analogRead(ANALOG_PIN);
    float voltage = (adcValue / 4095.0) * Vcc;
    luminaires[0].external_illuminance = Luxmeter(voltage);

    delay(5000);

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
    int i, j = 0; // Loop counters

    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input == "hub") {
            hub_node = true;
        }
        // Only the hub node handles serial commands
        else if(hub_node && calibrationDone){   
            // Split the input string by spaces
            int var[3] = {0};  // Store numbers here
            int varCount = 0;  // To count how many numbers we found
            
            // Split the string into parts
            String parts[10];  // Array to hold the parts (letters and numbers)
            int numParts = 0;
            int startIndex = 0;

            // Split by spaces
            for (int i = 0; i < input.length(); i++) {
                if (input.charAt(i) == ' ' || i == input.length() - 1) {
                    // Get substring for the part
                    if (i == input.length() - 1) {
                        parts[numParts++] = input.substring(startIndex, i + 1);  // Last character
                    } else {
                        parts[numParts++] = input.substring(startIndex, i);
                    }
                    startIndex = i + 1;
                }
            }

            // Extract and concatenate letters
            String letters = "";
            for (int i = 0; i < numParts; i++) {
                if (isAlpha(parts[i].charAt(0))) {
                    // Concatenate letters if the part is alphabetic
                    letters += parts[i];
                } else {
                    var[varCount] = parts[i].toInt();
                    varCount++;
                }
            }

            // Print the concatenated letters and numbers
            Serial.print("Command: ");
            Serial.println(letters);  // Prints concatenated letters
            Serial.print("Numbers: ");
            for (int i = 0; i < varCount; i++) {
                Serial.print(var[i]);
                if (i < varCount - 1) {
                    Serial.print(", ");
                }
            }
            Serial.println();

            byte cmdCode = getCommandCode(letters);

            int j = 2;  // Start writing at data[2]
            for (i = 0; i < varCount; i++) {
                uint8_t lowByte = var[i] & 0xFF;
                uint8_t highByte = (var[i] >> 8) & 0xFF;

                canMsgTx.data[j++] = lowByte;
                canMsgTx.data[j++] = highByte;
            }  
            
            canMsgTx.can_id = int_node_address;
            canMsgTx.can_dlc = j;
            canMsgTx.data[0] = MSG_COMMAND;  // Signal calibration done
            canMsgTx.data[1] = cmdCode;
            can0.sendMessage(&canMsgTx);
            handleCommand(canMsgTx, node_ids.data(), int_node_address, can0);
        }
    }

    // CAN message receiving
    while (can0.readMessage(&canMsgRx) == MCP2515::ERROR_OK && canMsgRx.data[0] != MSG_ADMM_SUM) {
        
        if(receiving_buffer == true && hub_node && !(canMsgRx.can_dlc == 1 && canMsgRx.data[0] == MSG_BUFFER_END)){
            int16_t firstValue = canMsgRx.data[0] | (canMsgRx.data[1] << 8);
            Serial.printf("%d, ", firstValue);
            continue;
        }
    
        receiving_buffer = false;
        
        lastMessageTime = currentTime;
        int int_sender_id = canMsgRx.can_id;
        uint8_t received_data = canMsgRx.data[0];
  
        if(!calibrationDone){ //If calibration has not completed
            
            if(addUniqueNodeId(int_sender_id)){//If a message is received from a new node, add it to the list and skip loop iteration
                Serial.print("New node added: ");
                Serial.println(int_sender_id);
                new_node = true;
                continue; //skip rest of loop
            }
            
            if(received_data == MSG_CALIBRATE) {
                Serial.println("Calibration message received");
                calibrationStarted = true;
                std::sort(node_ids.begin(), node_ids.end()); // Sort the ids
                num_iluminaires = node_ids.size();
                gains[calibrationCount] = measureIlluminance(); //measure the gain relative to the one that sent me the message
                Serial.print("Measured Gain:");
                Serial.println(gains[calibrationCount]);
                calibrationCount++;

                if(int_node_address == node_ids[calibrationCount]){// If my id is the lowest uncalibrated one, start my calibration
                    Serial.println("Starting calibration for my node");
                    gains[calibrationCount] = performCalibration(calibrationCount);
                    canMsgTx.can_id = int_node_address;
                    canMsgTx.can_dlc = 1;
                    canMsgTx.data[0] = MSG_CALIBRATE;  // Signal calibration done
                    can0.sendMessage(&canMsgTx);
                    delay(1500);
                    setDutyCycle(calibrationCount, 0, node_ids.data(), node_ids[calibrationCount], can0);
                    calibrationCount++;
                }
                if (calibrationCount >= num_iluminaires) { //if all luminaires have been calibrated
                    calibrationDone = true;
                    Serial.println("All luminaires calibrated");
                    
                }
            }
        }

        if(received_data == MSG_COMMAND){
            handleCommand(canMsgRx, node_ids.data(), int_node_address, can0);
        }
        if((received_data == MSG_COMMAND_GET) && (hub_node==true)){
            handleCommandGet(canMsgRx);
        }
    }

    // If calibration has not started and 2 seconds have passed without receiving a message and i have the lowest uncalibrated id, calibrate and send MSG_CALIBRATE
    if(!calibrationStarted){
        if ((currentTime - lastMessageTime >= timeout) && (new_node == true)) {
            std::sort(node_ids.begin(), node_ids.end()); // Sort the ids
            num_iluminaires = node_ids.size();
            if(int_node_address == node_ids[0]){// If my id is the lowest uncalibrated one
                gains[calibrationCount] = performCalibration(calibrationCount);
                canMsgTx.can_id = int_node_address;
                canMsgTx.can_dlc = 1;
                canMsgTx.data[0] = MSG_CALIBRATE;  // Signal calibration done
                can0.sendMessage(&canMsgTx);
                delay(1500);
                setDutyCycle(calibrationCount, 0, node_ids.data(), node_ids[calibrationCount], can0);
                calibrationCount++;
                calibrationStarted = true;
                //hub_node = true;
            }
        }
    }

}

// This function runs on Core 1
void loop1() {
    
    can_frame frm;
    uint32_t msg;
    uint8_t b[4];

    //while (calibrationDone) {
    while (calibrationDone) { //JUST FOR TESTING COMMANDS
        //Serial.println("loop1 running...");
        unsigned long currentTime = micros();  // Current time in microseconds
        num_cycles++;
        // Check if the required interval has passed
        if (currentTime - previousTime > sampInterval) {
            int adcValue = analogRead(ANALOG_PIN);
            float voltage = (adcValue / 4095.0) * Vcc;
            sharedIlluminance = Luxmeter(voltage);
            
            if (millis() - lastAdmmTime > ADMM_RUN_PERIOD) {
                // ADMM determines the best duty cycle globally
                admmIterationUpdateReference(can0);  // sets 'reference' in LUX
                lastAdmmTime = millis();
            }

           // The PID does the local real-time control
           //Serial.printf("reference: %.2f//", reference);
            float pid_output = my_pid.compute_control(reference, sharedIlluminance);
            sharedPwmValue   = constrain(pid_output, 0, 4095);
            analogWrite(LED_PWM_PIN, sharedPwmValue);

            // Perform housekeeping on PID controller
            u_old = my_pid.housekeep(reference, sharedIlluminance, sharedPwmValue);

            // Store duty cycle and illuminance in CircularBuffer
            dutyCycleBuffer.push(sharedPwmValue / 40.950);
            illuminanceBuffer.push(sharedIlluminance);


            // Print metrics
            float energy = calculateEnergy();
            float visibilityError = calculateVisibilityError(reference);
            float flicker = calculateFlicker();
            totalEnergy += energy;
            totalVisibilityError += visibilityError;
            totalFlicker += flicker;
            avgEnergy = totalEnergy / num_cycles;
            //Serial.printf("avg energy %.2f\n", avgEnergy);
            avgVisibilityError = totalVisibilityError / num_cycles;
            avgFlicker = totalFlicker / num_cycles;

            float lux_from_LED = gains[0] * (sharedPwmValue / 4095); // Calculate illuminance from LED

            luminaires[0].LDR_voltage = voltage;
            luminaires[0].power_consumption = energy;
            luminaires[0].flicker_error = flicker; 
            luminaires[0].visibility_error = visibilityError; 
            luminaires[0].elapsed_time = currentTime/1000;  // Convert milliseconds to seconds
            luminaires[0].duty_cycle = sharedPwmValue / 40.95;  // Normalize to [0, 1]S
            luminaires[0].illuminance_ref = reference;
            luminaires[0].measured_illuminance = sharedIlluminance;
            luminaires[0].buffer_u.push_back(sharedPwmValue / 40.950);  // Store duty cycle in buffer
            luminaires[0].buffer_y.push_back((float)sharedIlluminance);  // Store illuminance in buffer
            if(sharedIlluminance-lux_from_LED > 0){
                luminaires[0].external_illuminance = sharedIlluminance-lux_from_LED;  // Store external illuminance
            }
            else{
                luminaires[0].external_illuminance = 0;  // Store external illuminance
            }
            
            luminaires[0].avg_energy = avgEnergy;  // Store average energy consumption
            luminaires[0].avg_visibility_error = avgVisibilityError;  // Store average visibility error
            luminaires[0].avg_flicker = avgFlicker;  // Store average flicker

            if(streamDutyCycle && hub_node){
                Serial.printf("s u 2 %.2f %d\n", luminaires[0].duty_cycle, (int)luminaires[0].elapsed_time);
            }
            if(streamDutyCycle && !hub_node){
                int value = luminaires[0].duty_cycle; // Placeholder for duty cycle
                int elapsed_time_conv = (int)luminaires[0].elapsed_time;
                int deskId = 0;
                for(int i = 0; i < 2; i++){
                    if (node_ids[i] == int_node_address){
                        deskId = i;
                    }
                }
                uint8_t lowByte1 = deskId & 0xFF;
                uint8_t highByte1 = (deskId>> 8) & 0xFF;
                uint8_t lowByte = value & 0xFF;
                uint8_t highByte = (value >> 8) & 0xFF;
                uint8_t lowByte2 = elapsed_time_conv & 0xFF;
                uint8_t highByte2 = (elapsed_time_conv >> 8) & 0xFF;
    
                canMsgTx.can_id = int_node_address;
                canMsgTx.can_dlc = 8;
                canMsgTx.data[0] = MSG_COMMAND_GET;
                canMsgTx.data[1] = COMMAND_su;
                canMsgTx.data[2] = lowByte1;
                canMsgTx.data[3] = highByte1; 
                canMsgTx.data[4] = lowByte;
                canMsgTx.data[5] = highByte;
                canMsgTx.data[6] = lowByte2;
                canMsgTx.data[7] = highByte2;
                can0.sendMessage(&canMsgTx);
                
            }
            
            if(streamIlluminance && hub_node){
                Serial.printf("s y 2 %.2f %d\n", luminaires[0].measured_illuminance, (int)luminaires[0].elapsed_time);
            }
            if(streamIlluminance && !hub_node){
                
                int value = luminaires[0].measured_illuminance; 
                int elapsed_time_conv = (int)luminaires[0].elapsed_time;
                int deskId = 0;
                for(int i = 0; i < 2; i++){
                    if (node_ids[i] == int_node_address){
                        deskId = i;
                    }
                }
                uint8_t lowByte1 = deskId & 0xFF;
                uint8_t highByte1 = (deskId>> 8) & 0xFF;
                uint8_t lowByte = value & 0xFF;
                uint8_t highByte = (value >> 8) & 0xFF;
                uint8_t lowByte2 = elapsed_time_conv & 0xFF;
                uint8_t highByte2 = (elapsed_time_conv >> 8) & 0xFF;
    
                canMsgTx.can_id = int_node_address;
                canMsgTx.can_dlc = 8;
                canMsgTx.data[0] = MSG_COMMAND_GET;
                canMsgTx.data[1] = COMMAND_sy;
                canMsgTx.data[2] = lowByte1;
                canMsgTx.data[3] = highByte1; 
                canMsgTx.data[4] = lowByte;
                canMsgTx.data[5] = highByte;
                canMsgTx.data[6] = lowByte2;
                canMsgTx.data[7] = highByte2;
                can0.sendMessage(&canMsgTx);
                
            }
            
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


float energy_cons(float duty_cycle) {
    return alpha * duty_cycle * duty_cycle;
}


void admmIterationUpdateReference(MCP2515& can0) {
    float G_i;
    for(int i=0; i<calibrationCount; i++){
        if(node_ids[i] == int_node_address){
            G_i = gains[i];
        }
    }
    float y_bg_i= luminaires[0].external_illuminance;
    float y_min = luminaires[0].unoccupied_lower_bound; //default

    // constraints
    if (luminaires[0].occupied) {
        y_min = luminaires[0].occupied_lower_bound;
    }
    else {
        y_min = luminaires[0].unoccupied_lower_bound;
    }
    float y_max = luminaires[0].illuminance_ref;

    float best_u = 0.0f;
    float min_cost = INFINITY;

    // localTerm means we consider 'z_global - u_local' as the central guess
    float localTerm = (admm_z_global - admm_u_local);

    // we sweep duty cycle from 0..1 in 0.01 steps
    for (float u_test = 0.0f; u_test <= 1.0f; u_test += 0.01f) {
        float illum_test = G_i * u_test + y_bg_i;

        // check feasibility
        if (illum_test < y_min || illum_test > y_max) 
            continue;

        //cost function (alpha*x_i^2 + rho/2*(x_i - [z^k - u_i^k] )^2)
        float cost = energy_cons(u_test) + 0.5f* admm_rho * pow((u_test - localTerm),2);

        if(cost < min_cost) {
            min_cost = cost;
            best_u   = u_test;
        }
    }

    // store best local duty cycle
    best_u_local = best_u;

    // broadcast best_u_local + u_local
    float localSum = (best_u_local + admm_u_local);
    int16_t scaledSum = (int16_t)(localSum*100);

    struct can_frame canMsgTx;

    canMsgTx.can_id  = int_node_address;
    canMsgTx.can_dlc = 3;
    canMsgTx.data[0] = MSG_ADMM_SUM;
    canMsgTx.data[1] = scaledSum & 0xFF;
    canMsgTx.data[2] = (scaledSum >> 8) & 0xFF;
    can0.sendMessage(&canMsgTx);

    // gather peer values 
    std::vector<float> allVals;
    allVals.push_back(localSum);

    unsigned long startTime = millis();
    while((millis()-startTime) < ADMM_TIMEOUT) {
        can_frame rxMsg;
        if(can0.readMessage(&rxMsg) == MCP2515::ERROR_OK && rxMsg.data[0] == MSG_ADMM_SUM) 
        {
            Serial.print(".");
           int16_t peerScaled = rxMsg.data[1] | (rxMsg.data[2]<<8);
           float peerVal = peerScaled/100.0f;
           allVals.push_back(peerVal);
        }
    }

    // consensus
    float sum=0.0f;
    for(float v: allVals) sum+=v;
    admm_z_global = sum / allVals.size();

    // dual update
    admm_u_local += (best_u_local - admm_z_global);


    float best_duty = best_u_local; //for local choice : best_u_local for consensus : admn_z_global
    Serial.printf("gi: %.2f , bestd :%.2f, ybg: %.2f\n", G_i, best_duty, y_bg_i);
    reference = G_i * best_duty + y_bg_i;
}