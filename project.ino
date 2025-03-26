#include "mcp2515.h"
#include "pid.h"
#include "CircularBuffer.hpp"  // Include your circular buffer header
#include <map>
#include "commands.h"

const float Vcc = 3.3;      // Supply voltage
const int R = 10000;        // 10kΩ resistor
const int R_0 = 1100000;    // LDR resistance at 1 LUX
const float b = log10(R_0); 
const float m = 0.8;        // Nominal value

const int numSteps = 10;
float dutyCycles[numSteps];
int luxAtDuty[numSteps];

CircularBuffer<float, bufferSize> dutyCycleBuffer;
CircularBuffer<float, bufferSize> illuminanceBuffer;


// PID Controller Initialization
pid my_pid {0.01, 31.61, 1, 0.01 }; //K was previously 1
float reference = 30;
float u_old;

// CAN-BUS communication setup
MCP2515 can0 {spi0, 17, 19, 16, 18, 10000000}; // Initialize MCP2515
struct can_frame canMsgTx, canMsgRx;
uint8_t node_address; // Unique ID for each luminaire
unsigned long counter = 0;
unsigned long lastTime = 0;
unsigned long messageCount = 0;
const unsigned long interval = 1000;  // 1 second interval to measure CAN message rate
int circular_buffer= 1;

unsigned long previousTime = 0;  // Last time an operation was performed
unsigned long sampInterval = 10000; // Sampling interval

std::map<int, Luminaire> luminaires;

void setup() {
    Serial.begin(115200);
    pinMode(ANALOG_PIN, INPUT);
    pinMode(LED_PWM_PIN, OUTPUT);
    analogReadResolution(12);
    analogWriteFreq(30000); // 30KHz PWM Frequency
    analogWriteRange(4095); // Max PWM value

    // CAN-BUS initialization
    pico_unique_board_id_t pico_board_id;
    pico_get_unique_board_id(&pico_board_id);
    node_address = 0x02; // Unique ID per Pico
    can0.reset();
    can0.setBitrate(CAN_1000KBPS);
    can0.setNormalMode();

    // Initialize the luminaires, just one for now
    initializeLuminaires(1);
}

void loop() {
  unsigned long currentTime = micros();  // Current time in milliseconds

  // Check if the required interval has passed
  if (currentTime - previousTime > sampInterval) {
      
      int adcValue = analogRead(ANALOG_PIN);
      float voltage = (adcValue / 4095.0) * Vcc;
      float illuminance = Luxmeter(voltage);
      float control_signal = my_pid.compute_control(reference, illuminance);
      int pwm_value = (int)control_signal;
      pwm_value = constrain(pwm_value, 0, 4095);
      analogWrite(LED_PWM_PIN, pwm_value);

      // Perform housekeeping on PID controller
      u_old = my_pid.housekeep(reference, illuminance, pwm_value);

      if (Serial.available()) {
          String command = Serial.readStringUntil('\n');
          command.trim();
          handleCommand(command);
      }

      canMsgTx.can_id = node_address;
      canMsgTx.can_dlc = 1;
      canMsgTx.data[0] = illuminance;  // Sending illuminance value

      MCP2515::ERROR err = can0.sendMessage(&canMsgTx);
      if (err == MCP2515::ERROR_OK) {
          Serial.printf("Sent: %d from Node %x\n", canMsgTx.data[0], node_address);
      } else {
          Serial.println("Send error!");
      }
      messageCount++;  // Increment message count

      while (can0.readMessage(&canMsgRx) == MCP2515::ERROR_OK) {
          Serial.printf("Received: %d from Node %x\n", canMsgRx.data[0], canMsgRx.can_id);
      }

      // Store duty cycle and illuminance in CircularBuffer
      dutyCycleBuffer.push(pwm_value / 4095.0);  // Normalize control signal to [0, 1]
      illuminanceBuffer.push(illuminance);
      if (circular_buffer == 1) {
          if (dutyCycleBuffer.isFull()) {  // Only print when the buffer is full
              circular_buffer = 0;
              for (int i = 0; i < dutyCycleBuffer.size(); i += 20) {
                  // Serial.print("Duty_Cycle:");
                  Serial.print(dutyCycleBuffer[i]);  // Print duty cycle

                  Serial.print(",");  // Tab separator for plotting multiple variables
                  // Serial.print("LUX:");
                  Serial.print(illuminanceBuffer[i]);  // Print illuminance

                  Serial.println();  // Move to the next line for proper plotting
              }
          }
      }

      // Calculate and print metrics
      float energy = calculateEnergy();
      float visibilityError = calculateVisibilityError(reference);
      float flicker = calculateFlicker();

      luminaires[0].power_consumption = energy;

      previousTime = currentTime;
  }
}

