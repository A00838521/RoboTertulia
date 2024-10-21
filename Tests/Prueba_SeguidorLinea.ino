#include <QTRSensors.h>

// Define the number of sensors, samples per sensor, and the emitter pin
#define NUM_SENSORS             2  
#define NUM_SAMPLES_PER_SENSOR  4  
#define EMITTER_PIN             6  

// Initialize the QTRSensorsAnalog object with the specified parameters
QTRSensorsAnalog qtra((unsigned char[]) {A0, A1}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

void setup() 
{
  // Start the serial communication at 9600 baud rate
  Serial.begin(9600);
  
  // Set pin 13 as an output and turn it on
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(1000);  // Wait for 1 second
  
  // Calibration process
  for (int i = 0; i < 400; i++) 
  {
    qtra.calibrate();  // Calibrate the sensors
    delay(20);  // Ensure time to move the surfaces over the sensors
  }
  
  // Turn off the LED on pin 13
  digitalWrite(13, LOW);
}

void loop() 
{
  // Read the sensor values after calibration
  qtra.read(sensorValues);
  
  // Print the sensor values to the serial monitor
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();  // Print a newline character

  delay(500);  // Wait for 0.5 seconds before the next loop iteration
}
