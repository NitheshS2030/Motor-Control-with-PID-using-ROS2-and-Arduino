#include <Arduino.h>

// Motor driver pins
const int ENA = 5;   // PWM speed control for motor A
const int IN1 = 6;   // Direction control for motor A
const int IN2 = 7;   // Direction control for motor A

// Encoder pins
const int encoderA = 2; // Encoder signal A (interrupt)
const int encoderB = 3; // Encoder signal B

// PID control variables
volatile int encoderCount = 0;
unsigned long prevTime = 0;
float prevError = 0;
float integral = 0;
float targetSpeed = 0; // Target speed in RPM

// PID gains (adjust these for tuning)
float Kp = 1.2;
float Ki = 0.5;
float Kd = 0.1;

// Encoder counts per revolution (adjust based on your motor's encoder)
const int COUNTS_PER_REV = 1000;

// Function to handle encoder interrupts
void encoderISR() {
    if (digitalRead(encoderB) == HIGH) {
        encoderCount++;
    } else {
        encoderCount--;
    }
}

void setup() {
    Serial.begin(9600);

    // Motor driver setup
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    // Encoder setup
    pinMode(encoderA, INPUT);
    pinMode(encoderB, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderA), encoderISR, RISING);

    // Initialize motor state
    stopMotor();
}

void loop() {
    static char lastCommand = ' '; // Stores the last received command
    if (Serial.available() > 0) {
        char command = Serial.read();

        // Ensure we only process a new command
        if (command != lastCommand) {
            lastCommand = command;
            switch (command) {
                case 'w':
                    Serial.println("Forward");
                    moveMotor(100); // Move forward at 100 RPM
                    break;
                case 's':
                    Serial.println("Backward");
                    moveMotor(-100); // Move backward at 100 RPM
                    break;
                case 'a':
                    Serial.println("Left");
                    // Add left turning logic if needed
                    break;
                case 'd':
                    Serial.println("Right");
                    // Add right turning logic if needed
                    break;
                case 'q':
                    Serial.println("Stopping");
                    stopMotor();
                    break;
            }
        }
    }

    // PID speed control loop
    controlMotorSpeed();
}

// Function to move the motor with a target speed (RPM)
void moveMotor(float rpm) {
    targetSpeed = rpm;
    if (rpm > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    }
}

// Function to stop the motor
void stopMotor() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
    targetSpeed = 0;
}

// Function to control motor speed using PID
void controlMotorSpeed() {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - prevTime) / 1000.0; // Convert to seconds
    prevTime = currentTime;

    // Calculate actual speed in RPM
    float actualSpeed = (encoderCount / (float)COUNTS_PER_REV) * 60.0 / deltaTime;
    encoderCount = 0; // Reset encoder count

    // PID calculations
    float error = targetSpeed - actualSpeed;
    integral += error * deltaTime;
    float derivative = (error - prevError) / deltaTime;
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    prevError = error;

    // Constrain output to valid PWM range
    int pwmOutput = constrain(abs(output), 0, 255);
    analogWrite(ENA, pwmOutput);
}
