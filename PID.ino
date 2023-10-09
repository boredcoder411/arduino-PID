#include <Wire.h>
#include <L3G.h>

// Define the PID constants
double Kp = 1.0;  // Proportional constant
double Ki = 0.0;  // Integral constant
double Kd = 0.0;  // Derivative constant

// Define the setpoint (desired value)
double setpoint = 0.0;

// Variables for sensor data
L3G gyro;

// Variables for PID control
double previousError = 0.0;
double integral = 0.0;

// Variables for timing
unsigned long previousTime = 0;
double dt = 0.0;

// Function to compute the PID control output
double computePID() {
  double error = setpoint - gyro.readZ();
  double output = Kp * error + Ki * integral + Kd * (error - previousError);
  integral += error * dt;
  previousError = error;
  return output;
}

void setup() {
  // Initialize the sensor
  gyro.init();
  gyro.enableDefault();

  // Initialize Serial communication
  Serial.begin(9600);
}

void loop() {
  // Read sensor data
  gyro.read();
  
  // Calculate time difference
  unsigned long currentTime = millis();
  dt = (double)(currentTime - previousTime) / 1000.0;  // Convert to seconds
  previousTime = currentTime;
  
  // Compute PID control output
  double output = computePID();
  
  // Apply the control output (adjust as needed)
  // Example: You can use the output to control a motor or a servo
  // For simplicity, we'll just print it to Serial here
  Serial.print("PID Output: ");
  Serial.println(output);
  
  delay(100);  // Adjust the loop rate as needed
}
