#include "commands.h"
#include "pid.h"

// Assuming streamData is defined somewhere globally
StreamData streamData[MAX_DESKS];  // Define MAX_DESKS based on your configuration

void handleCommand(String command) {
    char cmd;
    float val;
    char x;            // Declare variable for 'x'
    int deskId;        // Declare variable for 'deskId'
    unsigned long time = millis(); // Default to current time

    // Parsing the command
    if (sscanf(command.c_str(), "%c %f", &cmd, &val) >= 1) {
        switch (cmd) {

            default: Serial.println("err"); break;
        }
    }
    else if (sscanf(command.c_str(), "%c %c %d", &cmd, &x, &deskId) >= 1) {
        switch (cmd) {
            case 's': {
                // Start streaming for a specific variable and desk
                if (sscanf(command.c_str(), "s %c %d %f %lu", &x, &deskId, &val, &time) == 4) {
                    startStream(x, deskId, val, time);
                } else {
                    startStream(x, deskId, val, time);  // Default time is '0' when not provided
                }
                break;
            }
            case 'S': {
                // Stop streaming for a specific variable and desk
                stopStream(x, deskId);
                break;
            }
            case 'g': {
                // Get buffer for the variable of a specific desk
                if (command.startsWith("g b")) {
                    getBuffer(x, deskId);
                }
                break;
            }
            default: Serial.println("err"); break;
        }
    }
    else if (sscanf(command.c_str(), "%c %d %f", &cmd, &deskId, &val) >= 1) {
        switch (cmd) {
            case 'u': {
                // Set duty cycle for luminaire i
                setDutyCycle(deskId,val);
                break;
            }
            case 'r': {
                // Set illuminance reference for luminaire i
                setIlluminanceRef(deskId,val);
                break;
            }
            case 'o': {
                // Set occupancy state for desk i
                luminaires[deskId].occupied = (val != 0);
                Serial.println("ack");
                break;
            }
            case 'a': {
                // Set anti-windup state for desk i
                luminaires[deskId].anti_windup = (val != 0);
                Serial.println("ack");
                break;
            }
            case 'f': {
                // Set feedback control state for desk i
                luminaires[deskId].feedback_control = (val != 0);
                Serial.println("ack");
                break;
            }
            default: Serial.println("err"); break;
        }
    }
    else if (sscanf(command.c_str(), "%c %d", &cmd, &deskId) >= 1) {
        switch (cmd) 
        {
            case 'g': {
                if (command.startsWith("g u")) {
                    // Get duty cycle for luminaire i
                    getDutyCycle(deskId);   
                } else if (command.startsWith("g r")) {
                    // Get illuminance reference for luminaire i
                    getIlluminanceRef(deskId);
                } else if (command.startsWith("g y")) {
                    // Measure illuminance for luminaire i
                    measureIlluminance(deskId);
                } else if (command.startsWith("g v")) {
                    // Measure LDR voltage for luminaire i
                    measureLDRVoltage(deskId);
                } else if (command.startsWith("g o")) {
                    Serial.printf("o %d %d\n", deskId, luminaires[deskId].occupied ? 1 : 0);
                } else if (command.startsWith("g d")) {
                    // Get current external illuminance of desk i
                    Serial.printf("d %d %.2f\n", deskId, luminaires[deskId].external_illuminance);
                } else if (command.startsWith("g p")) {
                    // Get instantaneous power consumption of desk i
                    Serial.printf("p %d %.2f\n", deskId, luminaires[deskId].power_consumption);
                } else if (command.startsWith("g t")) {
                    // Get the elapsed time since the last restart for desk i
                    Serial.printf("t %d %lu\n", deskId, luminaires[deskId].elapsed_time);
                } else if (command.startsWith("g a")) {
                    // Get anti-windup state for desk i
                    Serial.printf("a %d %d\n", deskId, luminaires[deskId].anti_windup ? 1 : 0);
                } else if (command.startsWith("g f")) {
                    // Get feedback control state for desk i
                    Serial.printf("f %d %d\n", deskId, luminaires[deskId].feedback_control ? 1 : 0);
                } else if (command.startsWith("g b")) {
                    // Get the last minute buffer of the variable <x> of the desk <i>
                    std::vector<float>& buffer = (x == 'y') ? luminaires[deskId].buffer_y : luminaires[deskId].buffer_u;

                    String data = "";
                    for (int i = 0; i < buffer.size(); i++) {
                        data += String(buffer[i]);
                        if (i < buffer.size() - 1) {
                            data += ",";
                        }
                    }

                    // Send buffer data
                    Serial.printf("b %c %d %s\n", x, deskId, data.c_str());
                } else if (command.startsWith("g E")) {
                    // Get average energy consumption at desk i
                    float energy = calculateEnergy();
                    Serial.printf("E %d %.2f\n", deskId, energy);
                } else if (command.startsWith("g V")) {
                    // Get average visibility error at desk i
                    float visibilityError = calculateVisibilityError(reference);
                    Serial.printf("V %d %.2f\n", deskId, visibilityError);
                } else if (command.startsWith("g F")) {
                    // Get average flicker error at desk i
                    float flicker = calculateFlicker();
                    Serial.printf("F %d %.2f\n", deskId, flicker); break;
                } else if (command.startsWith("g O")) {
                    // Get lower bound on illuminance for the occupied state
                    Serial.printf("O %d %.2f\n", deskId, luminaires[deskId].occupied_lower_bound);
                } else if (command.startsWith("g U")) {
                    // Get lower bound on illuminance for the unoccupied state
                    Serial.printf("U %d %.2f\n", deskId, luminaires[deskId].unoccupied_lower_bound);
                } else if (command.startsWith("g L")) {
                    // Get the current illuminance lower bound
                    Serial.printf("L %d %.2f\n", deskId, luminaires[deskId].current_lower_bound);
                } else if (command.startsWith("g C")) {
                    // Get the current energy cost
                    Serial.printf("C %d %.2f\n", deskId, luminaires[deskId].energy_cost);
                }
                break;
                }
            case 'O': {
                // Set lower bound on illuminance for the occupied state
                if (sscanf(command.c_str(), "O %d %f", &deskId, &val) == 2) {
                    luminaires[deskId].occupied_lower_bound = val;
                    Serial.println("ack");
                } else {
                    Serial.println("err");
                }
                break;
            }
            case 'U': {
                // Set lower bound on illuminance for the unoccupied state
                if (sscanf(command.c_str(), "U %d %f", &deskId, &val) == 2) {
                    luminaires[deskId].unoccupied_lower_bound = val;
                    Serial.println("ack");
                } else {
                    Serial.println("err");
                }
                break;
            }
            case 'C': {
                // Set the current energy cost
                if (sscanf(command.c_str(), "C %d %f", &deskId, &val) == 2) {
                    luminaires[deskId].energy_cost = val;
                    Serial.println("ack");
                } else {
                    Serial.println("err");
                }
                break;
            }
            case 'R': {
                // Restart the system
                initializeLuminaires(MAX_DESKS);
                Serial.println("ack");
                break;
            }
                default: {
                    Serial.println("err");
                    break;
                }
            }
        }
    }

// Function to start stream
void startStream(char x, int deskId, float val, time_t time) {
    if (x == 'y') {
        streamData[deskId].isStreaming = true;
        streamData[deskId].lastTimestamp = time;
        // Send ack
        Serial.println("ack");
    } else if (x == 'u') {
        streamData[deskId].isStreaming = true;
        streamData[deskId].lastTimestamp = time;
        // Send ack
        Serial.println("ack");
    }
}

// Function to stop stream
void stopStream(char x, int deskId) {
    if (x == 'y' || x == 'u') {
        streamData[deskId].isStreaming = false;
        // Send ack
        Serial.println("ack");
    }
}

// Function to get buffer data
void getBuffer(char x, int deskId) {
    if (x == 'y' || x == 'u') {
        CircularBuffer<float, 2000>& buffer = streamData[deskId].buffer;
        
        String data = "";
        for (int i = 0; i < buffer.size(); i++) {
            data += String(buffer[i]);
            if (i < buffer.size() - 1) {
                data += ",";
            }
        }
        
        // Send buffer data
        Serial.println(data);
    }
}

// Function to set Duty Cycle
void setDutyCycle(int deskId, float val) {//change to accomodate
    int pwm_value = (int)(val * 4095);
    pwm_value = constrain(pwm_value, 0, 4095);
    analogWrite(LED_PWM_PIN, pwm_value);
    dutyCycleBuffer.push(val);
    //Serial.println("ack");
}

// Function to get Duty Cycle
void getDutyCycle(int deskId) { //change to accomodate
    if (!dutyCycleBuffer.isEmpty()) {
        Serial.printf("u %.2f\n", dutyCycleBuffer.last());
    } else {
        Serial.println("err");
    }
}

// Function to set Illuminance Reference
void setIlluminanceRef(int deskId, float val) {// change to accomodate
    reference = val;
    Serial.println("ack");
}

// Function to get Illuminance Reference
void getIlluminanceRef(int deskId) { // change to accomodate
    Serial.printf("r %.2f\n", reference);
}

// Function to measure Illuminance
int measureIlluminance(int deskId) { // change to accomodate
    int adcValue = analogRead(ANALOG_PIN);
    float voltage = (adcValue / 4095.0) * Vcc;
    float illuminance = Luxmeter(voltage);
    illuminanceBuffer.push(illuminance);
    //Serial.printf("y %.2f\n", illuminance);

    return illuminance;
}

// Function to measure LDR Voltage
void measureLDRVoltage(int deskId) { // change to accomodate
    int adcValue = analogRead(ANALOG_PIN);
    float voltage = (adcValue / 4095.0) * Vcc;
    Serial.printf("v %.2f\n", voltage);
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
float calculateEnergy() {
    float energy = 0.0;
    for (size_t i = 1; i < dutyCycleBuffer.size(); i++) {
        float dutyCycle = dutyCycleBuffer[i];
        float deltaT = 0.01;  // Assuming a sampling period of 100Hz (0.01s)
        energy += dutyCycle * deltaT * 3.3;  // Assume maximum power is 3.3V
    }
    return energy;
}

// Function to calculate Visibility Error (V)
float calculateVisibilityError(float referenceLux) {
    float visibilityError = 0.0;
    for (size_t i = 0; i < illuminanceBuffer.size(); i++) {
        float illuminance = illuminanceBuffer[i];
        if (illuminance < referenceLux) {
            visibilityError += referenceLux - illuminance;
        }
    }
    return visibilityError / illuminanceBuffer.size();
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

// Initialize luminaires
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
            .elapsed_time = 0,
            .buffer_y = std::vector<float>(BUFFER_SIZE, 0.0),
            .buffer_u = std::vector<float>(BUFFER_SIZE, 0.0)
        };
    }
}


