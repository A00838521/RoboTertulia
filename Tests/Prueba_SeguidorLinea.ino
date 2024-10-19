#include <QTRSensors.h>

#define NUM_SENSORS             2  
#define NUM_SAMPLES_PER_SENSOR  4  
#define EMITTER_PIN             6  

QTRSensorsAnalog qtra((unsigned char[]) {A0, A1}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

void setup() 
{
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(1000);
  
  // Calibración
  for (int i = 0; i < 400; i++) 
  {
    qtra.calibrate();
    delay(20);  // Asegura tiempo para mover las superficies sobre los sensores.
  }
  digitalWrite(13, LOW);
}

void loop() 
{
  // Leer los valores después de la calibración
  qtra.read(sensorValues);
  
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

  delay(500);
}
