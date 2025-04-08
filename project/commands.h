#ifndef COMMANDS_H
#define COMMANDS_H

#include <Arduino.h>
#include <vector>
#include "CircularBuffer.hpp"
#include <map>
#include <ctime>
#include "mcp2515.h"  // Include your CAN library
#include "can.h"

#define MSG_COMMAND_GET 0x04
class MCP2515;

#define COMMAND_u 0x01
#define COMMAND_gu 0x02
#define COMMAND_r 0x03
#define COMMAND_gr 0x04
#define COMMAND_gy 0x05
#define COMMAND_gv 0x06
#define COMMAND_o 0x07
#define COMMAND_go 0x08
#define COMMAND_a 0x09
#define COMMAND_f 0x10
#define COMMAND_gf 0x11
#define COMMAND_gd 0x12
#define COMMAND_gp 0x13
#define COMMAND_gt 0x14
#define COMMAND_s 0x15
#define COMMAND_S 0x16
#define COMMAND_gb 0x17
#define COMMAND_gE 0x18
#define COMMAND_gV 0x19
#define COMMAND_gF 0x20
#define COMMAND_gO 0x21
#define COMMAND_O 0x22
#define COMMAND_gU 0x23
#define COMMAND_U 0x24
#define COMMAND_gL 0x25
#define COMMAND_gC 0x27
#define COMMAND_C 0x28
#define COMMAND_R 0x29
#define COMMAND_ga 0x30




// Define Luminaire struct before using it
struct Luminaire {
    float duty_cycle;            // [0-1]
    float illuminance_ref;
    float measured_illuminance;
    float LDR_voltage;
    bool occupied;
    bool anti_windup;
    bool feedback_control;
    float external_illuminance;
    float power_consumption;
    unsigned long elapsed_time;
    std::vector<float> buffer_y;
    std::vector<float> buffer_u;
    float occupied_lower_bound;
    float unoccupied_lower_bound;
    float current_lower_bound;
    float energy_cost;
};

#define ANALOG_PIN 26      // Analog input pin
#define LED_PWM_PIN 15     // PWM pin for LED control
#define BUFFER_SIZE 600    // Stores last minute of data at 100Hz

#define MAX_DESKS 1  // Adjust this based on your system's configuration

struct StreamData {
    bool isStreaming;
    CircularBuffer<float, 2000> buffer;
    time_t lastTimestamp;
};

// Declare an array to store streaming data for each desk
extern StreamData streamData[MAX_DESKS];

extern float reference;
extern const float Vcc;
extern const float b;
extern const float m;

extern const int R;
extern std::map<int, Luminaire> luminaires; // Global variable for luminaires

constexpr size_t bufferSize = 2000;  // Ensure this is a compile-time constant
extern CircularBuffer<float, bufferSize> dutyCycleBuffer;
extern CircularBuffer<float, bufferSize> illuminanceBuffer;

byte getCommandCode(String cmd);

// Function declarations
void handleCommandGet(struct can_frame command);
void handleCommand(struct can_frame command, int* node_ids, int int_node_address, MCP2515& can0);
void setDutyCycle(int deskId, int val , int* node_ids, int int_node_address);
void getDutyCycle(int deskId , int* node_ids, int int_node_address, MCP2515& can0);
void setIlluminanceRef(int deskId, int val , int* node_ids, int int_node_address);
void getIlluminanceRef(int deskId, int* node_ids, int int_node_address, MCP2515& can0);
void measureIlluminanceCommand(int deskId, int* node_ids, int int_node_address, MCP2515& can0);
void setOccupancyState(int deskId, int val, int* node_ids, int int_node_address);
void getOccupancyState(int firstValue, int* node_ids,int int_node_address, MCP2515& can0);
void setAntiWindupOnOff(int deskId, int val, int* node_ids, int int_node_address);
void setFeebackOnOff(int deskId, int val, int* node_ids, int int_node_address) ;
void getExternalIlluminance(int deskId,int* node_ids, int int_node_address,  MCP2515& can0) ;
void getFeeback(int deskId,int* node_ids, int int_node_address, MCP2515& can0);
void getInstateniousPower(int deskId,int* node_ids, int int_node_address);
void getElapsedTime(int deskId,int* node_ids, int int_node_address);
int measureIlluminance();
void measureLDRVoltage(int deskId, int* node_ids, int int_node_address, MCP2515& can0);
void getEnergy();
void getVisibilityError();
void getFlicker();

// Function to calculate illuminance from voltage
extern float Luxmeter(float V);
// Function to calculate Energy (E)
extern float calculateEnergy();
void calculateEnergyCommand(int deskId, int* node_ids, int int_node_address);
// Function to calculate Visibility Error (V)
extern float calculateVisibilityError(float referenceLux);
void calculateVisibilityErrorCommand(int deskId, int* node_ids, int int_node_address, float referenceLux);
// Function to calculate Flicker (F)
extern float calculateFlicker();
void calculateFlickerCommand(int deskId, int* node_ids, int int_node_address);
// Function to initialize luminaires
extern void initializeLuminaires(int numLuminaires);

// Function to start stream
void startStream(char x, int deskId, int* node_ids, int int_node_address);
// Function to stop stream
void stopStream(char x, int deskId, int* node_ids, int int_node_address);
// Function to get buffer
void getBuffer(char x, int deskId, int* node_ids, int int_node_address);

#endif // COMMANDS_H
