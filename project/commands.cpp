#include "commands.h"
#include "pid.h"

// Assuming streamData is defined somewhere globally
StreamData streamData[MAX_DESKS]; 

extern float reference; // Reference value for PID
extern bool hub_node; // Flag to indicate if this is the hub node

extern bool streamDutyCycle; // Flag to indicate if duty cycle streaming is enabled
extern bool streamIlluminance; // Flag to indicate if illuminance streaming is enabled

extern int timeStreamU; // Time for duty cycle streaming
extern int timeStreamY; // Time for illuminance streaming

extern pid my_pid;

constexpr int bufferSize = 6000;  // definição real
CircularBuffer<float, bufferSize> dutyCycleBuffer;
extern CircularBuffer<float, bufferSize> dutyCycleBuffer;      // Circular buffer for duty cycle data
CircularBuffer<float, bufferSize> illuminanceBuffer;
extern CircularBuffer<float, bufferSize> illuminanceBuffer; // Circular buffer for illuminance data
extern std::map<int, Luminaire> luminaires; // Map to store luminaires
extern int int_node_address; // Unique ID for this node
extern int deskId; // Desk ID for the system
extern int node_ids[100]; // List of node IDs (CAN IDs)
extern bool hub_node; // Flag to indicate if this is the hub node

extern bool receiving_buffer;

// PID parameters
extern float Tt;



byte getCommandCode(String cmd) {
    if (cmd == "u") return COMMAND_u;//working
    else if (cmd == "gu") return COMMAND_gu;//working
    else if (cmd == "r") return COMMAND_r;//working
    else if (cmd == "gr") return COMMAND_gr;//working
    else if (cmd == "gy") return COMMAND_gy;//working
    else if (cmd == "gv") return COMMAND_gv;//!!!!!!!!!!!!!
    else if (cmd == "o") return COMMAND_o;//working
    else if (cmd == "go") return COMMAND_go;//working
    else if (cmd == "a") return COMMAND_a;//working
    else if (cmd == "ga") return COMMAND_ga;//working
    else if (cmd == "f") return COMMAND_f;//working
    else if (cmd == "gf") return COMMAND_gf;//working
    else if (cmd == "gd") return COMMAND_gd;
    else if (cmd == "gp") return COMMAND_gp;
    else if (cmd == "gt") return COMMAND_gt;
    else if (cmd == "su") return COMMAND_su;
    else if (cmd == "Su") return COMMAND_Su;
    else if (cmd == "sy") return COMMAND_sy;
    else if (cmd == "Sy") return COMMAND_Sy;
    else if (cmd == "gb") return COMMAND_gb;
    else if (cmd == "gE") return COMMAND_gE;
    else if (cmd == "gV") return COMMAND_gV;
    else if (cmd == "gF") return COMMAND_gF;
    else if (cmd == "gO") return COMMAND_gO;
    else if (cmd == "O") return COMMAND_O;
    else if (cmd == "gU") return COMMAND_gU;
    else if (cmd == "U") return COMMAND_U;
    else if (cmd == "gL") return COMMAND_gL;
    else if (cmd == "gC") return COMMAND_gC;
    else if (cmd == "C") return COMMAND_C;
    else if (cmd == "R") return COMMAND_R;
    else{
        return 0x00; // Unknown command
    }
}

// int16_t value = 12345; // any value you want to send

// uint8_t lowByte = value & 0xFF;           // Extract lower 8 bits
// uint8_t highByte = (value >> 8) & 0xFF;   // Extract higher 8 bits

// // Now you can send it, e.g., via CAN
// can_frame frame;
// frame.data[0] = lowByte;
// frame.data[1] = highByte;

void handleCommandGet(struct can_frame command) {
    char cmd;
    float val;
    char x;            // Declare variable for 'x'
    int deskId;        // Declare variable for 'deskId'
    unsigned long time = millis(); // Default to current time

    int16_t firstValue = command.data[2] | (command.data[3] << 8); //usually the luminaire number 0 to num_luminaieres-1
    int16_t secondValue = command.data[4] | (command.data[5] << 8);
    int16_t thirdValue = command.data[6] | (command.data[7] << 8);

    float aux = secondValue;
    Serial.print("2nd val:");
    Serial.println(aux);

    switch (command.data[1]) {

        case COMMAND_gbu_start:
            Serial.printf("b u %d\n", firstValue);
            receiving_buffer = true; // Set the flag to indicate that we are receiving data
            break;
        
        case COMMAND_gbu_end:

            break;
        
            case COMMAND_gby_start:

            break;
        
        case COMMAND_gby_end:

            break;

        case COMMAND_gu:
            Serial.print("u");
            Serial.print(" ");
            Serial.print(firstValue);
            Serial.print(" ");
            Serial.println(secondValue);
            break;
        
        case COMMAND_gr:
            Serial.print("r");
            Serial.print(" ");
            Serial.print(firstValue);
            Serial.print(" ");
            Serial.println(secondValue);
            break;
        
        case COMMAND_gy:
            Serial.print("gy");
            Serial.print(" ");
            Serial.print(firstValue);
            Serial.print(" ");
            Serial.println(secondValue);
            break;
    
        case COMMAND_gv:
            Serial.print("gv");
            Serial.print(" ");
            Serial.print(firstValue);
            Serial.print(" ");
            Serial.println(aux/100);
            break;

        case ACK:
            Serial.println("ack");
            break;
        
        case ERR:
            Serial.println("err");
            break;
    
        case COMMAND_go:
            Serial.print("go");
            Serial.print(" ");
            Serial.print(firstValue);
            Serial.print(" ");
            Serial.println(secondValue);
            break;

        case COMMAND_ga:
            Serial.print("ga");
            Serial.print(" ");
            Serial.print(firstValue);
            Serial.print(" ");
            Serial.println(secondValue);
            break;
    
        case COMMAND_gf:
            Serial.print("gf");
            Serial.print(" ");
            Serial.print(firstValue);
            Serial.print(" ");
            Serial.println(secondValue);
            break;
    
        case COMMAND_gd:
            Serial.print("gd");
            Serial.print(" ");
            Serial.print(firstValue);
            Serial.print(" ");
            Serial.println(secondValue);
            break;
    
        case COMMAND_gp:
            Serial.print("gp");
            Serial.print(" ");
            Serial.print(firstValue);
            Serial.print(" ");
            Serial.println(secondValue);
            break;
        
        case COMMAND_gt:
            Serial.print("gt");
            Serial.print(" ");
            Serial.print(firstValue);
            Serial.print(" ");
            Serial.println(secondValue);
            break;
        
        case COMMAND_su: {
            Serial.printf("s u %d %d %d\n",  firstValue, secondValue, thirdValue);
            break;
        }
        
        case COMMAND_sy: {
            Serial.printf("s y %d %d %d\n",  firstValue, secondValue, thirdValue);
            break;
        }

        case COMMAND_gb:
            Serial.print("gb");
            Serial.print(" ");
            Serial.print(firstValue);
            Serial.print(" ");
            Serial.println(secondValue);
            break;
    
        case COMMAND_gE:
            Serial.print("gE");
            Serial.print(" ");
            Serial.print(firstValue);
            Serial.print(" ");
            Serial.println(secondValue);
            break;
        
        case COMMAND_gV:
            Serial.print("gV");
            Serial.print(" ");
            Serial.print(firstValue);
            Serial.print(" ");
            Serial.println(secondValue);
            break;
        
        case COMMAND_gF:
            Serial.print("gF");
            Serial.print(" ");
            Serial.print(firstValue);
            Serial.print(" ");
            Serial.println(secondValue);
            break;

        default:
            Serial.println("err");
            break;
    }
}



void handleCommand(struct can_frame command, int* node_ids, int int_node_address, MCP2515& can0) {
    char cmd;
    float val;
    char x;            // Declare variable for 'x'
    int deskId;        // Declare variable for 'deskId'
    unsigned long time = millis(); // Default to current time

    int16_t firstValue = command.data[2] | (command.data[3] << 8); //usually the luminaire number 0 to num_luminaieres-1
    int16_t secondValue = command.data[4] | (command.data[5] << 8);

    switch (command.data[1]) {
        case COMMAND_u:
            setDutyCycle(firstValue, secondValue, node_ids, int_node_address, can0);
            break;
    
        case COMMAND_gu:
            getDutyCycle(firstValue, node_ids, int_node_address, can0); 
            break;
        
        case COMMAND_r:
            setIlluminanceRef(firstValue, secondValue, node_ids, int_node_address, can0);
            break;
    
        case COMMAND_gr:
            getIlluminanceRef(firstValue, node_ids, int_node_address, can0);
            break;
        
        case COMMAND_gy:
            measureIlluminanceCommand(firstValue, node_ids, int_node_address, can0);
            break;
    
        case COMMAND_gv:
            measureLDRVoltage(firstValue, node_ids, int_node_address, can0);
            break;
    
        case COMMAND_o:
            setOccupancyState(firstValue, secondValue, node_ids, int_node_address, can0);
            break;
        
        case COMMAND_go:
            getOccupancyState(firstValue, node_ids, int_node_address, can0);
            break;
        
        case COMMAND_a:
            setAntiWindupOnOff(firstValue, secondValue, node_ids, int_node_address, can0);
            break;
        
        case COMMAND_ga:
            getAntiWindup(firstValue, node_ids, int_node_address, can0);
            break;
        
        case COMMAND_f:
            setFeebackOnOff(firstValue, secondValue, node_ids, int_node_address, can0);
            break;
    
        case COMMAND_gf:
            getFeeback(firstValue, node_ids, int_node_address, can0);
            break;
    
        case COMMAND_gd:
            getExternalIlluminance(firstValue, node_ids, int_node_address, can0);
            break;
    
        case COMMAND_gp:
            getInstateniousPower(firstValue, node_ids, int_node_address, can0);
            break;
        
        case COMMAND_gt:
            getElapsedTime(firstValue, node_ids, int_node_address, can0);
            break;
        
        case COMMAND_su:
            startStream(firstValue, secondValue, node_ids, int_node_address, can0);
            break;
    
        case COMMAND_sy:
            startStream(firstValue, secondValue, node_ids, int_node_address, can0);
            break;   

        case COMMAND_Su:
            stopStream(firstValue, secondValue, node_ids, int_node_address, can0);
            break;
        
        case COMMAND_Sy:
            stopStream(firstValue, secondValue, node_ids, int_node_address, can0);
            break;
    
        case COMMAND_gb:
            getBuffer(firstValue, secondValue, node_ids, int_node_address, can0);
            break;
    
        case COMMAND_gE:
            calculateEnergyCommand(deskId, node_ids, int_node_address, can0);
            break;
        
        case COMMAND_gV:
            calculateVisibilityErrorCommand(deskId, node_ids, int_node_address, reference, can0); 
            break;
        
        case COMMAND_gF:
            calculateFlickerCommand(deskId, node_ids, int_node_address, can0) ; 
            break;
        
        case COMMAND_gO:
            
            break;
        
        case COMMAND_O:
 
            break;

        case COMMAND_gU:

            break;

        case COMMAND_U:

            break;

        case COMMAND_gL:

            break;
        
        case COMMAND_gC:

            break;

        case COMMAND_C:

            break;
        
        case COMMAND_R:
            watchdog_enable(1, true);  // Enable watchdog with 1 ms timeout
            while (true); 


        default:
            Serial.println("err");
            break;
    }
}
    

// Function to start stream
void startStream(char x, int deskId, int* node_ids, int int_node_address, MCP2515& can0) {

    if(!streamDutyCycle || !streamIlluminance) { //if any of the values is false
        if(hub_node == false && node_ids[deskId] == int_node_address){//im not the hub soo i need to send the message for him to Serial.print()
                
            if (x == 'u') {
                streamDutyCycle = true;
                timeStreamU = luminaires[0].elapsed_time; 
                
            } else if (x == 'y') {
                streamIlluminance = true;
                timeStreamY = luminaires[0].elapsed_time; 

            }
               
        }else if(hub_node == true) {//if im hub
            if (x == 'u') {
                streamDutyCycle = true;
                timeStreamU = luminaires[0].elapsed_time; 
                
            } else if (x == 'y') {
                streamIlluminance = true;
                timeStreamY = luminaires[0].elapsed_time; 

            }
        }
    }
}

// Function to stop stream
void stopStream(char x, int deskId, int* node_ids, int int_node_address, MCP2515& can0) {

    struct can_frame canMsgTx;

    if(hub_node == false && node_ids[deskId] == int_node_address){//im not the hub soo i need to send the message for him to Serial.print()
        if (x == 'u') {
            streamDutyCycle = false;
            timeStreamU = luminaires[0].elapsed_time; 
        } else if (x == 'y') {
            streamIlluminance = false;
            timeStreamY = luminaires[0].elapsed_time; 
        }
    }else if(hub_node == true) {//if im hub
        if (x == 'u') {
            streamDutyCycle = false;
            timeStreamU = luminaires[0].elapsed_time; 
        } else if (x == 'y') {
            streamIlluminance = false;
            timeStreamY = luminaires[0].elapsed_time; 
        }
    }
    if(hub_node == false && node_ids[deskId] == int_node_address){//im not the hub soo i need to send the message for him to Serial.print()

        canMsgTx.can_id = int_node_address;
        canMsgTx.can_dlc = 2;
        canMsgTx.data[0] = MSG_COMMAND_GET;
        canMsgTx.data[1] = ACK;
        can0.sendMessage(&canMsgTx);
    
    }else if((hub_node == true) && (node_ids[deskId] == int_node_address) ) {//if im hub and the resquested node
            Serial.println("ack");
        
    } 
}

void getOccupancyState(int deskId, int* node_ids, int int_node_address, MCP2515& can0){
    
    struct can_frame canMsgTx;

    if(hub_node == false && node_ids    [deskId] == int_node_address){//im not the hub soo i need to send the message for him to Serial.print()
            int value = luminaires[0].occupied ? 1 : 0;
            
            uint8_t lowByte1 = deskId & 0xFF;
            uint8_t highByte1 = (deskId >> 8) & 0xFF;
            uint8_t lowByte = value & 0xFF;
            uint8_t highByte = (value >> 8) & 0xFF;

            canMsgTx.can_id = int_node_address;
            canMsgTx.can_dlc = 6;
            canMsgTx.data[0] = MSG_COMMAND_GET;
            canMsgTx.data[1] = COMMAND_go;
            canMsgTx.data[2] = lowByte1;
            canMsgTx.data[3] = highByte1;
            canMsgTx.data[4] = lowByte;
            canMsgTx.data[5] = highByte;
            can0.sendMessage(&canMsgTx);
        
    }else if((hub_node == true) && (node_ids[deskId] == int_node_address) ) {//if im hub and the resquested node
            int value = luminaires[0].occupied ? 1 : 0;
            Serial.printf("o %d %d\n", deskId, value);
        
    }

}

void setOccupancyState(int deskId, int val, int* node_ids, int int_node_address, MCP2515& can0){
    struct can_frame canMsgTx;

    if(node_ids[deskId] == int_node_address) {//if it is requesting my info.
        luminaires[0].occupied = (val != 0);
        if(val == 1){
            reference = OCCUPIED;
        }else if(val == 0){
            reference = UNNOCUPIED;
        }
    }
    
    if(hub_node == false && node_ids[deskId] == int_node_address){//im not the hub soo i need to send the message for him to Serial.print()

            canMsgTx.can_id = int_node_address;
            canMsgTx.can_dlc = 2;
            canMsgTx.data[0] = MSG_COMMAND_GET;
            canMsgTx.data[1] = ACK;
            can0.sendMessage(&canMsgTx);
        
    }else if((hub_node == true) && (node_ids[deskId] == int_node_address) ) {//if im hub and the resquested node
            Serial.println("ack");
        
    }
}

void setAntiWindupOnOff(int deskId, int val, int* node_ids, int int_node_address, MCP2515& can0) {
    struct can_frame canMsgTx;

    if(node_ids[deskId] == int_node_address) {//if it is requesting my info.
        luminaires[0].anti_windup = (val != 0);
        if (luminaires[0].anti_windup == true) {
            Tt = Tt_std;
        }
        else {
            Tt = 1e9;
        }
        
    }
    if(hub_node == false && node_ids[deskId] == int_node_address){//im not the hub soo i need to send the message for him to Serial.print()

        canMsgTx.can_id = int_node_address;
        canMsgTx.can_dlc = 2;
        canMsgTx.data[0] = MSG_COMMAND_GET;
        canMsgTx.data[1] = ACK;
        can0.sendMessage(&canMsgTx);
    
    }else if((hub_node == true) && (node_ids[deskId] == int_node_address) ) {//if im hub and the resquested node
            Serial.println("ack");
        
    }
}

void setFeebackOnOff(int deskId, int val, int* node_ids, int int_node_address, MCP2515& can0) {
    struct can_frame canMsgTx;

    if(node_ids[deskId] == int_node_address) {//if it is requesting my info.
        luminaires[0].feedback_control = (val != 0);
        my_pid.setFeedbackEnabled(val != 0);

        
    }
    if(hub_node == false && node_ids[deskId] == int_node_address){//im not the hub soo i need to send the message for him to Serial.print()

        canMsgTx.can_id = int_node_address;
        canMsgTx.can_dlc = 2;
        canMsgTx.data[0] = MSG_COMMAND_GET;
        canMsgTx.data[1] = ACK;
        can0.sendMessage(&canMsgTx);
    
    }else if((hub_node == true) && (node_ids[deskId] == int_node_address) ) {//if im hub and the resquested node
            Serial.println("ack");
        
    }
}

void getFeeback(int deskId,int* node_ids, int int_node_address, MCP2515& can0) {

    struct can_frame canMsgTx;

    if(hub_node == false && node_ids[deskId] == int_node_address){//im not the hub soo i need to send the message for him to Serial.print()
            int value = luminaires[0].feedback_control ? 1 : 0;
            
            uint8_t lowByte1 = deskId & 0xFF;
            uint8_t highByte1 = (deskId >> 8) & 0xFF;
            uint8_t lowByte = value & 0xFF;
            uint8_t highByte = (value >> 8) & 0xFF;

            canMsgTx.can_id = int_node_address;
            canMsgTx.can_dlc = 6;
            canMsgTx.data[0] = MSG_COMMAND_GET;
            canMsgTx.data[1] = COMMAND_gf;
            canMsgTx.data[2] = lowByte1;
            canMsgTx.data[3] = highByte1;
            canMsgTx.data[4] = lowByte;
            canMsgTx.data[5] = highByte;
            can0.sendMessage(&canMsgTx);
        
    }else if((hub_node == true) && (node_ids[deskId] == int_node_address) ) {//if im hub and the resquested node
            int value = luminaires[0].feedback_control ? 1 : 0;
            Serial.printf("f %d %d\n", deskId, value);
        
    }
}

void getExternalIlluminance(int deskId,int* node_ids, int int_node_address, MCP2515& can0) {
    
    struct can_frame canMsgTx;

    if(hub_node == false && node_ids[deskId] == int_node_address){//im not the hub soo i need to send the message for him to Serial.print()
            int value = luminaires[0].external_illuminance;
            
            uint8_t lowByte1 = deskId & 0xFF;
            uint8_t highByte1 = (deskId >> 8) & 0xFF;
            uint8_t lowByte = value & 0xFF;
            uint8_t highByte = (value >> 8) & 0xFF;

            canMsgTx.can_id = int_node_address;
            canMsgTx.can_dlc = 6;
            canMsgTx.data[0] = MSG_COMMAND_GET;
            canMsgTx.data[1] = COMMAND_gf;
            canMsgTx.data[2] = lowByte1;
            canMsgTx.data[3] = highByte1;
            canMsgTx.data[4] = lowByte;
            canMsgTx.data[5] = highByte;
            can0.sendMessage(&canMsgTx);
        
    }else if((hub_node == true) && (node_ids[deskId] == int_node_address) ) {//if im hub and the resquested node
            int value = luminaires[0].external_illuminance;
            Serial.printf("d %d %d\n", deskId, value);
        
    }
}

void getInstateniousPower(int deskId,int* node_ids, int int_node_address, MCP2515& can0) {

    struct can_frame canMsgTx;

    if(hub_node == false && node_ids[deskId] == int_node_address){//im not the hub soo i need to send the message for him to Serial.print()

            int value = luminaires[0].power_consumption; // Placeholder for power consumption
            
            uint8_t lowByte1 = deskId & 0xFF;
            uint8_t highByte1 = (deskId >> 8) & 0xFF;
            uint8_t lowByte = value & 0xFF;
            uint8_t highByte = (value >> 8) & 0xFF;

            canMsgTx.can_id = int_node_address;
            canMsgTx.can_dlc = 6;
            canMsgTx.data[0] = MSG_COMMAND_GET;
            canMsgTx.data[1] = COMMAND_gp;
            canMsgTx.data[2] = lowByte1;
            canMsgTx.data[3] = highByte1;
            canMsgTx.data[4] = lowByte;
            canMsgTx.data[5] = highByte;
            can0.sendMessage(&canMsgTx);
        
    }else if((hub_node == true) && (node_ids[deskId] == int_node_address) ) {//if im hub and the resquested node
            int value = luminaires[0].power_consumption; // Placeholder for power consumption
            Serial.printf("p %d %d\n", deskId, value);
        
    }

}

void getAntiWindup(int deskId, int* node_ids, int int_node_address, MCP2515& can0){
    struct can_frame canMsgTx;

    if(hub_node == false && node_ids[deskId] == int_node_address){//im not the hub soo i need to send the message for him to Serial.print()

            int value = luminaires[0].anti_windup; // Placeholder for power consumption
            
            uint8_t lowByte1 = deskId & 0xFF;
            uint8_t highByte1 = (deskId >> 8) & 0xFF;
            uint8_t lowByte = value & 0xFF;
            uint8_t highByte = (value >> 8) & 0xFF;

            canMsgTx.can_id = int_node_address;
            canMsgTx.can_dlc = 6;
            canMsgTx.data[0] = MSG_COMMAND_GET;
            canMsgTx.data[1] = COMMAND_ga;
            canMsgTx.data[2] = lowByte1;
            canMsgTx.data[3] = highByte1;
            canMsgTx.data[4] = lowByte;
            canMsgTx.data[5] = highByte;
            can0.sendMessage(&canMsgTx);
        
    }else if((hub_node == true) && (node_ids[deskId] == int_node_address) ) {//if im hub and the resquested node
            int value = luminaires[0].anti_windup; // Placeholder for power consumption
            Serial.printf("a %d %d\n", deskId, value);
        
    }
}

void getElapsedTime(int deskId,int* node_ids, int int_node_address, MCP2515& can0) {
    struct can_frame canMsgTx;

    if(hub_node == false && node_ids[deskId] == int_node_address){//im not the hub soo i need to send the message for him to Serial.print()

            int value = luminaires[0].elapsed_time; // Placeholder for power consumption
            
            uint8_t lowByte1 = deskId & 0xFF;
            uint8_t highByte1 = (deskId >> 8) & 0xFF;
            uint8_t lowByte = value & 0xFF;
            uint8_t highByte = (value >> 8) & 0xFF;

            canMsgTx.can_id = int_node_address;
            canMsgTx.can_dlc = 6;
            canMsgTx.data[0] = MSG_COMMAND_GET;
            canMsgTx.data[1] = COMMAND_gt;
            canMsgTx.data[2] = lowByte1;
            canMsgTx.data[3] = highByte1;
            canMsgTx.data[4] = lowByte;
            canMsgTx.data[5] = highByte;
            can0.sendMessage(&canMsgTx);
        
    }else if((hub_node == true) && (node_ids[deskId] == int_node_address) ) {//if im hub and the resquested node
            int value = luminaires[0].elapsed_time; // Placeholder for power consumption
            Serial.printf("t %d %d\n", deskId, value);
        
    }
}


// Function to get buffer data
void getBuffer(char x, int deskId, int* node_ids, int int_node_address, MCP2515& can0) {

    struct can_frame canMsgTx;
    uint8_t deskid1 = deskId & 0xFF;
    uint8_t deskid2 = (deskId >> 8) & 0xFF;
    uint8_t lowByte1;
    uint8_t lowByte2;
    uint8_t lowByte3;
    uint8_t highByte1;
    uint8_t highByte2;
    uint8_t highByte3;
    bool valid1 = false;
    bool valid2 = false;
    bool valid3 = false;
    int size = 0;

    if(hub_node == false && node_ids[deskId] == int_node_address){//im not the hub soo i need to send the message for him to Serial.print()
        int value = 0;

        if (x == 'y') {

            size = illuminanceBuffer.size();
            const CircularBuffer<float, bufferSize>& buffery = illuminanceBuffer;

            canMsgTx.can_id = int_node_address;
            canMsgTx.can_dlc = 4;
            canMsgTx.data[0] = MSG_COMMAND_GET;
            canMsgTx.data[1] = COMMAND_gby_start;
            canMsgTx.data[2] = deskid1;//desk id
            canMsgTx.data[3] = deskid2;
            can0.sendMessage(&canMsgTx);

            for(int i = 0; i < size; i++){
                value = buffery[i];
                uint8_t lowByte = value & 0xFF;
                uint8_t highByte = (value >> 8) & 0xFF;if(canMsgTx.can_dlc >= 2){
                    int16_t firstValue = canMsgTx.data[0] | (canMsgTx.data[1] << 8);
                    Serial.printf("%d\n", firstValue);
                }

                if(i != buffery.size()-1){
                    int value1 = (int)buffery[++i];
                    uint8_t lowByte1 = value1 & 0xFF;
                    uint8_t highByte1 = (value1 >> 8) & 0xFF;
                    valid1 = true;
                }
                if(i != buffery.size()-1){
                    int value2 = (int)buffery[++i];
                    uint8_t lowByte2 = value2 & 0xFF;
                    uint8_t highByte2 = (value2 >> 8) & 0xFF;
                    valid2 = true;
                }
                if(i != buffery.size()-1){
                    int value3 = (int)buffery[++i];
                    uint8_t lowByte3 = value3 & 0xFF;
                    uint8_t highByte3 = (value3 >> 8) & 0xFF;
                    valid3 = true;
                }

                canMsgTx.can_id = int_node_address;
                canMsgTx.can_dlc = 2;
                canMsgTx.data[0] = lowByte;
                canMsgTx.data[1] = highByte;

                if(valid1){
                    canMsgTx.data[2] = lowByte1;
                    canMsgTx.data[3] = highByte1;
                    canMsgTx.can_dlc = 4;
                }
                if(valid2){
                    canMsgTx.data[4] = lowByte2;
                    canMsgTx.data[5] = highByte2;
                    canMsgTx.can_dlc = 6;
                    
                }
                if(valid3){
                    canMsgTx.data[6] = lowByte3;
                    canMsgTx.data[7] = highByte3;
                    canMsgTx.can_dlc = 8;
                }
                
                can0.sendMessage(&canMsgTx);
                valid1 = false;
                valid2 = false;
                valid3 = false;
                
            }  
            canMsgTx.can_id = int_node_address;
            canMsgTx.can_dlc = 1;
            canMsgTx.data[0] = MSG_BUFFER_END;    
            can0.sendMessage(&canMsgTx);
        
        }
        else if (x == 'u') {
            size = dutyCycleBuffer.size();
            const CircularBuffer<float, bufferSize>& bufferu = dutyCycleBuffer;


            canMsgTx.can_id = int_node_address;
            canMsgTx.can_dlc = 4;
            canMsgTx.data[0] = MSG_COMMAND_GET;
            canMsgTx.data[1] = COMMAND_gbu_start;
            canMsgTx.data[2] = deskid1;//desk id
            canMsgTx.data[3] = deskid2;
            can0.sendMessage(&canMsgTx);
        
            for(int i = 0; i < size; i++){
                value = bufferu[i];
                uint8_t lowByte = value & 0xFF;
                uint8_t highByte = (value >> 8) & 0xFF;if(canMsgTx.can_dlc >= 2){
                    int16_t firstValue = canMsgTx.data[0] | (canMsgTx.data[1] << 8);
                    Serial.printf("%d\n", firstValue);
                }

                if(i != bufferu.size()-1){
                    int value1 = (int)bufferu[++i];
                    uint8_t lowByte1 = value1 & 0xFF;
                    uint8_t highByte1 = (value1 >> 8) & 0xFF;
                    valid1 = true;
                }
                if(i != bufferu.size()-1){
                    int value2 = (int)bufferu[++i];
                    uint8_t lowByte2 = value2 & 0xFF;
                    uint8_t highByte2 = (value2 >> 8) & 0xFF;
                    valid2 = true;
                }
                if(i != bufferu.size()-1){
                    int value3 = (int)bufferu[++i];
                    uint8_t lowByte3 = value3 & 0xFF;
                    uint8_t highByte3 = (value3 >> 8) & 0xFF;
                    valid3 = true;
                }

                canMsgTx.can_id = int_node_address;
                canMsgTx.can_dlc = 2;
                canMsgTx.data[0] = lowByte;
                canMsgTx.data[1] = highByte;

                if(valid1){
                    canMsgTx.data[2] = lowByte1;
                    canMsgTx.data[3] = highByte1;
                    canMsgTx.can_dlc = 4;
                }
                if(valid2){
                    canMsgTx.data[4] = lowByte2;
                    canMsgTx.data[5] = highByte2;
                    canMsgTx.can_dlc = 6;
                    
                }
                if(valid3){
                    canMsgTx.data[6] = lowByte3;
                    canMsgTx.data[7] = highByte3;
                    canMsgTx.can_dlc = 8;
                }
                
                can0.sendMessage(&canMsgTx);
                valid1 = false;
                valid2 = false;
                valid3 = false;
                
            }  
            canMsgTx.can_id = int_node_address;
            canMsgTx.can_dlc = 1;
            canMsgTx.data[0] = MSG_BUFFER_END;    
            can0.sendMessage(&canMsgTx);
        }
    }

    else if((hub_node == true) && (node_ids[deskId] == int_node_address) ) {//if im hub and the resquested node
    
        if (x == 'y') {
            printf("b y %d", deskId);
            String data = "";
            for (int i = 0; i < illuminanceBuffer.size(); i++) {
                printf(" %.2f", illuminanceBuffer[i]);
                if (i < illuminanceBuffer.size() - 1) {
                    printf(",");
                }
            }
        }
        if (x == 'u') {
            printf("b u %d\n",  deskId);
            String data = "";
            for (int i = 0; i < dutyCycleBuffer.size(); i++) {
                printf(" %.2f", illuminanceBuffer[i]);
                if (i < dutyCycleBuffer.size() - 1) {
                    printf(",");
                }
            }
        }
        
    }
}

// Function to set Duty Cycle
void setDutyCycle(int deskId, int val, int* node_ids, int int_node_address, MCP2515& can0) {
    struct can_frame canMsgTx;

    if(node_ids[deskId] == int_node_address) {//if it is requesting my info.
        int pwm_value = (int) (val*40.95); // val is in [0, 100] and pwm_value is in [0, 4095]
        pwm_value = constrain(pwm_value, 0, 4095);
        //Serial.println(pwm_value);
        analogWrite(LED_PWM_PIN, pwm_value);
        dutyCycleBuffer.push(val);
        luminaires[0].buffer_u.push_back(val);
        luminaires[0].duty_cycle = val; // Placeholder for illuminance
        //Serial.println("ack");
    }
    if(hub_node == false && node_ids[deskId] == int_node_address){//im not the hub soo i need to send the message for him to Serial.print()

        canMsgTx.can_id = int_node_address;
        canMsgTx.can_dlc = 2;
        canMsgTx.data[0] = MSG_COMMAND_GET;
        canMsgTx.data[1] = ACK;
        can0.sendMessage(&canMsgTx);
    
    }else if((hub_node == true) && (node_ids[deskId] == int_node_address) ) {//if im hub and the resquested node
            Serial.println("ack");
        
    }
}

// Function to get Duty Cycle
void getDutyCycle(int deskId, int* node_ids, int int_node_address, MCP2515& can0) { //change to accomodate

    struct can_frame canMsgTx;

    if(hub_node == false && node_ids[deskId] == int_node_address){//im not the hub soo i need to send the message for him to Serial.print()
        if (!dutyCycleBuffer.isEmpty()) {
            int value = dutyCycleBuffer.last();
            
            uint8_t lowByte1 = deskId & 0xFF;
            uint8_t highByte1 = (deskId >> 8) & 0xFF;
            uint8_t lowByte = value & 0xFF;
            uint8_t highByte = (value >> 8) & 0xFF;

            canMsgTx.can_id = int_node_address;
            canMsgTx.can_dlc = 6;
            canMsgTx.data[0] = MSG_COMMAND_GET;
            canMsgTx.data[1] = COMMAND_gu;
            canMsgTx.data[2] = lowByte1;
            canMsgTx.data[3] = highByte1;
            canMsgTx.data[4] = lowByte;
            canMsgTx.data[5] = highByte;
            can0.sendMessage(&canMsgTx);
        }
    }else if((hub_node == true) && (node_ids[deskId] == int_node_address) ) {//if im hub and the resquested node
        if (!dutyCycleBuffer.isEmpty()) {
            int value = dutyCycleBuffer.last();
            Serial.printf("u %d %d\n", deskId, value);
        }
    }
    
}

// Function to set Illuminance Reference
void setIlluminanceRef(int deskId, int val, int* node_ids, int int_node_address, MCP2515& can0) {
    struct can_frame canMsgTx;

    if(node_ids[deskId] == int_node_address) {//if it is requesting my info.
        reference = val;
        //Serial.println("ack");
    }
    if(hub_node == false && node_ids[deskId] == int_node_address){//im not the hub soo i need to send the message for him to Serial.print()

        canMsgTx.can_id = int_node_address;
        canMsgTx.can_dlc = 2;
        canMsgTx.data[0] = MSG_COMMAND_GET;
        canMsgTx.data[1] = ACK;
        can0.sendMessage(&canMsgTx);
    
    }else if((hub_node == true) && (node_ids[deskId] == int_node_address) ) {//if im hub and the resquested node
            Serial.println("ack");
        
    }
    
}

// Function to get Illuminance Reference
void getIlluminanceRef(int deskId, int* node_ids, int int_node_address, MCP2515& can0) { 

    struct can_frame canMsgTx;
    int value = reference;

    if(hub_node == false && node_ids[deskId] == int_node_address){//im not the hub soo i need to send the message for him to Serial.print()
            
            uint8_t lowByte1 = deskId & 0xFF;
            uint8_t highByte1 = (deskId >> 8) & 0xFF;
            uint8_t lowByte = value & 0xFF;
            uint8_t highByte = (value >> 8) & 0xFF;

            canMsgTx.can_id = int_node_address;
            canMsgTx.can_dlc = 6;
            canMsgTx.data[0] = MSG_COMMAND_GET;
            canMsgTx.data[1] = COMMAND_gr;
            canMsgTx.data[2] = lowByte1;
            canMsgTx.data[3] = highByte1;
            canMsgTx.data[4] = lowByte;
            canMsgTx.data[5] = highByte;
            can0.sendMessage(&canMsgTx);

    }else if((hub_node == true) && (node_ids[deskId] == int_node_address) ) {//if im hub and the resquested node
            Serial.printf("r %d %d\n", deskId, value);
        
    }

}

// Function to measure Illuminance
void measureIlluminanceCommand(int deskId, int* node_ids, int int_node_address, MCP2515& can0) { 

    struct can_frame canMsgTx;

    if(hub_node == false && node_ids[deskId] == int_node_address){//im not the hub soo i need to send the message for him to Serial.print()
            int adcValue = analogRead(ANALOG_PIN);
            float voltage = (adcValue / 4095.0) * Vcc;
            float illuminance = Luxmeter(voltage);
            illuminanceBuffer.push(illuminance);

            int value = illuminance;

            uint8_t lowByte1 = deskId & 0xFF;
            uint8_t highByte1 = (deskId >> 8) & 0xFF;
            uint8_t lowByte = value & 0xFF;
            uint8_t highByte = (value >> 8) & 0xFF;

            canMsgTx.can_id = int_node_address;
            canMsgTx.can_dlc = 6;
            canMsgTx.data[0] = MSG_COMMAND_GET;
            canMsgTx.data[1] = COMMAND_gy;
            canMsgTx.data[2] = lowByte1;
            canMsgTx.data[3] = highByte1;
            canMsgTx.data[4] = lowByte;
            canMsgTx.data[5] = highByte;
            can0.sendMessage(&canMsgTx);

    }else if((hub_node == true) && (node_ids[deskId] == int_node_address)) {//if im hub
            int adcValue = analogRead(ANALOG_PIN);
            float voltage = (adcValue / 4095.0) * Vcc;
            float illuminance = Luxmeter(voltage);
            illuminanceBuffer.push(illuminance);

            int value = illuminance;

            Serial.printf("y %d %d\n", deskId, value);
        
    }

}

int measureIlluminance() { 
    int adcValue = analogRead(ANALOG_PIN);
    float voltage = (adcValue / 4095.0) * Vcc;
    float illuminance = Luxmeter(voltage);
    illuminanceBuffer.push(illuminance);

    return illuminance;
}

// Function to measure LDR Voltage (!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!)
void measureLDRVoltage(int deskId, int* node_ids, int int_node_address, MCP2515& can0) { 
    struct can_frame canMsgTx;

    if(hub_node == false && node_ids[deskId] == int_node_address){//im not the hub soo i need to send the message for him to Serial.print()

            //float aux = luminaires[0].LDR_voltage;
            int value = luminaires[0].LDR_voltage;
            //Serial.printf("Sending ldr voltage: %.2f\n", aux);

            uint8_t lowByte1 = deskId & 0xFF;
            uint8_t highByte1 = (deskId >> 8) & 0xFF;
            uint8_t lowByte = value & 0xFF;
            uint8_t highByte = (value >> 8) & 0xFF;

            canMsgTx.can_id = int_node_address;
            canMsgTx.can_dlc = 6;
            canMsgTx.data[0] = MSG_COMMAND_GET;
            canMsgTx.data[1] = COMMAND_gv;
            canMsgTx.data[2] = lowByte1;
            canMsgTx.data[3] = highByte1;
            canMsgTx.data[4] = lowByte;
            canMsgTx.data[5] = highByte;
            can0.sendMessage(&canMsgTx);

    }else if((hub_node == true) && (node_ids[deskId] == int_node_address) ) {//if im hub and the resquested node
            float voltage = luminaires[0].LDR_voltage;

            Serial.printf("v %d %.2f\n", deskId, voltage);
        
    }

}

// Function to get Energy
void getEnergy() {
    Serial.printf("e %.2f\n", calculateEnergy());
}

// Function to get Visibility Error
void getVisibilityError() {
    Serial.printf("v %.2f\n", calculateVisibilityError(reference));
}

// Function to get Flicker
void getFlicker() {
    Serial.printf("f %.2f\n", calculateFlicker());
}

// Function to calculate illuminance from voltage
float Luxmeter(float V) {
    if (V <= 0 || V >= Vcc) return 0;
    float R_ldr = (Vcc - V) * R / V;
    float log_LUX = (b - log10(R_ldr)) / m;
    return pow(10, log_LUX);
}

// Function to calculate Energy (E)
void calculateEnergyCommand(int deskId, int* node_ids, int int_node_address, MCP2515& can0) {

    struct can_frame canMsgTx;

    if(hub_node == false && node_ids[deskId] == int_node_address){//im not the hub soo i need to send the message for him to Serial.print()

            int value = luminaires[0].avg_energy; // Placeholder for power consumption
            
            uint8_t lowByte1 = deskId & 0xFF;
            uint8_t highByte1 = (deskId >> 8) & 0xFF;
            uint8_t lowByte = value & 0xFF;
            uint8_t highByte = (value >> 8) & 0xFF;

            canMsgTx.can_id = int_node_address;
            canMsgTx.can_dlc = 6;
            canMsgTx.data[0] = MSG_COMMAND_GET;
            canMsgTx.data[1] = COMMAND_gE;
            canMsgTx.data[2] = lowByte1;
            canMsgTx.data[3] = highByte1;
            canMsgTx.data[4] = lowByte;
            canMsgTx.data[5] = highByte;
            can0.sendMessage(&canMsgTx);
        
    }else if((hub_node == true) && (node_ids[deskId] == int_node_address) ) {//if im hub and the resquested node
            int value = luminaires[0].avg_energy; // Placeholder for power consumption
            Serial.printf("E %d %d\n", deskId, value);
        
    }
}

// Function to calculate Energy (E)
float calculateEnergy() {
    float energy = 0.0;
    energy += luminaires[0].duty_cycle * 0.01 * 3.3;
    // Assume maximum power is 3.3V
    
    return energy;
}

void calculateVisibilityErrorCommand(int deskId, int* node_ids, int int_node_address, float referenceLux, MCP2515& can0){
    struct can_frame canMsgTx;

    if(hub_node == false && node_ids[deskId] == int_node_address){//im not the hub soo i need to send the message for him to Serial.print()

            int value = luminaires[0].avg_visibility_error; // Placeholder for power consumption
            
            uint8_t lowByte1 = deskId & 0xFF;
            uint8_t highByte1 = (deskId >> 8) & 0xFF;
            uint8_t lowByte = value & 0xFF;
            uint8_t highByte = (value >> 8) & 0xFF;

            canMsgTx.can_id = int_node_address;
            canMsgTx.can_dlc = 6;
            canMsgTx.data[0] = MSG_COMMAND_GET;
            canMsgTx.data[1] = COMMAND_gV;
            canMsgTx.data[2] = lowByte1;
            canMsgTx.data[3] = highByte1;
            canMsgTx.data[4] = lowByte;
            canMsgTx.data[5] = highByte;
            can0.sendMessage(&canMsgTx);
        
    }else if((hub_node == true) && (node_ids[deskId] == int_node_address) ) {//if im hub and the resquested node
            int value = luminaires[0].avg_visibility_error; // Placeholder for power consumption
            Serial.printf("V %d %d\n", deskId, value);
        
    }
}

// Function to calculate Visibility Error (V)
float calculateVisibilityError(float referenceLux) {
    
    float visibilityError = 0.0;
    visibilityError += referenceLux - illuminanceBuffer.last(); // Placeholder for illuminance

       
    return visibilityError;
}

// Function to calculate Flicker (F)
void calculateFlickerCommand(int deskId, int* node_ids, int int_node_address,  MCP2515& can0) {
    struct can_frame canMsgTx;

    if(hub_node == false && node_ids[deskId] == int_node_address){//im not the hub soo i need to send the message for him to Serial.print()

        int value = luminaires[0].avg_flicker; // Placeholder for power consumption
        
        uint8_t lowByte1 = deskId & 0xFF;
        uint8_t highByte1 = (deskId >> 8) & 0xFF;
        uint8_t lowByte = value & 0xFF;
        uint8_t highByte = (value >> 8) & 0xFF;

        canMsgTx.can_id = int_node_address;
        canMsgTx.can_dlc = 6;
        canMsgTx.data[0] = MSG_COMMAND_GET;
        canMsgTx.data[1] = COMMAND_gF;
        canMsgTx.data[2] = lowByte1;
        canMsgTx.data[3] = highByte1;
        canMsgTx.data[4] = lowByte;
        canMsgTx.data[5] = highByte;
        can0.sendMessage(&canMsgTx);
        
    }else if((hub_node == true) && (node_ids[deskId] == int_node_address) ) {//if im hub and the resquested node
        int value = luminaires[0].avg_flicker; // Placeholder for power consumption
        Serial.printf("V %d %d\n", deskId, value);
        
    }
}

// Function to calculate Flicker (F)
float calculateFlicker() {
    float flicker = 0.0;

    for (size_t i = 2; i < dutyCycleBuffer.size(); i++) {
        float diff1 = dutyCycleBuffer[i] - dutyCycleBuffer[i-1];
        float diff2 = dutyCycleBuffer[i-1] - dutyCycleBuffer[i-2];
        if ((diff1 > 0 && diff2 < 0) || (diff1 < 0 && diff2 > 0)) {
            flicker += abs(diff1) + abs(diff2);
        }
    }
    return flicker / dutyCycleBuffer.size();
}

// Initialize luminaires                 WE ASSUME MY NODE IS LUMINAIRE 0
void initializeLuminaires(int numLuminaires) {
    luminaires.clear();  // Clear previous entries

    for (int i = 0; i < numLuminaires; i++) {
        luminaires[i] = Luminaire{
            .duty_cycle = 0.0,
            .illuminance_ref = reference,
            .measured_illuminance = 0.0,
            .LDR_voltage = 0.0,
            .occupied = false,
            .anti_windup = false,
            .feedback_control = true,
            .external_illuminance = 0.0,
            .power_consumption = 0.0,
            .flicker_error = 0.0,
            .visibility_error = 0.0,
            .elapsed_time = 0,
            .buffer_y = std::vector<float>(BUFFER_SIZE, 0.0),
            .buffer_u = std::vector<float>(BUFFER_SIZE, 0.0),
            .avg_energy = 0.0,
            .avg_visibility_error = 0.0,
            .avg_flicker = 0.0,
        };
    }
}
