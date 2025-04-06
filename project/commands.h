#ifndef COMMANDS_H
#define COMMANDS_H

#include <Arduino.h>
#include <vector>
#include "CircularBuffer.hpp"
#include <map>
#include <ctime>

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

// Function declarations
void handleCommand(String command);
void setDutyCycle(int deskId, float val);
void getDutyCycle(int deskId);
void setIlluminanceRef(int deskId, float val);
void getIlluminanceRef(int deskId);
int measureIlluminance(int deskId);
void measureLDRVoltage(int deskId);
void getEnergy();
void getVisibilityError();
void getFlicker();

// Function to calculate illuminance from voltage
extern float Luxmeter(float V);
// Function to calculate Energy (E)
extern float calculateEnergy();
// Function to calculate Visibility Error (V)
extern float calculateVisibilityError(float referenceLux);
// Function to calculate Flicker (F)
extern float calculateFlicker();
// Function to initialize luminaires
extern void initializeLuminaires(int numLuminaires);

// Function to start stream
void startStream(char x, int deskId, float val = 0, time_t time = 0);
// Function to stop stream
void stopStream(char x, int deskId);
// Function to get buffer
void getBuffer(char x, int deskId);

#endif // COMMANDS_H
