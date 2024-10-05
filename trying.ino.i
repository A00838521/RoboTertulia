
// ---------------------------------------------- Implementación de Colores ----------------------------------------------

#include <TCS3200.h>
#include <NewPing.h>
#include <PID_v1.h>
#include <Wire.h>
#include <MPU6050.h>

// Definición de pines para el TCS3200 y motores
#define S0 2
#define S1 3
#define S2 4
#define S3 5
#define OUT_PIN 6
#define TRIG_PIN 9
#define ECHO_PIN 10
#define MOTOR_LEFT_PIN_A 11
#define MOTOR_LEFT_PIN_B 12
#define MOTOR_RIGHT_PIN_A 13
#define MOTOR_RIGHT_PIN_B 14

// Variables del sensor de color
int redValue = 0, greenValue = 0, blueValue = 0;
int redThreshold = 100, greenThreshold = 100, blueThreshold = 100; // Umbrales iniciales ajustables para los tres colores
int tolerance = 20;  // Tolerancia de ajuste para los colores

// Variables del sensor ultrasónico
NewPing sonar(TRIG_PIN, ECHO_PIN, 200);

// Variables PID
double setpoint = 0, input, output;
double Kp = 2.0, Ki = 5.0, Kd = 1.0;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Giroscopio
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Función para leer colores
void readColor() {
  redValue = TCS3200_ReadColor('R');
  greenValue = TCS3200_ReadColor('G');
  blueValue = TCS3200_ReadColor('B');
}

// Función para detectar el color negro
bool isBlack() {
  return redValue < redThreshold / 2 && greenValue < greenThreshold / 2 && blueValue < blueThreshold / 2;
}

// Función para calibrar colores a través del monitor serial
void calibrateColors() {
  Serial.println("Calibración manual de colores...");
  delay(2000);
  
  // Ajuste del color rojo
  Serial.println("Coloca el robot en la unidad de color rojo y presiona cualquier tecla.");
  while (Serial.available() == 0) { } // Espera a que el usuario presione una tecla
  readColor();
  redThreshold = redValue + tolerance;
  Serial.print("Valor rojo calibrado: ");
  Serial.println(redThreshold);
  while (Serial.available() > 0) { Serial.read(); } // Limpia el buffer

  // Ajuste del color verde
  Serial.println("Coloca el robot en la unidad de color verde y presiona cualquier tecla.");
  while (Serial.available() == 0) { }
  readColor();
  greenThreshold = greenValue + tolerance;
  Serial.print("Valor verde calibrado: ");
  Serial.println(greenThreshold);
  while (Serial.available() > 0) { Serial.read(); }

  // Ajuste del color azul
  Serial.println("Coloca el robot en la unidad de color azul y presiona cualquier tecla.");
  while (Serial.available() == 0) { }
  readColor();
  blueThreshold = blueValue + tolerance;
  Serial.print("Valor azul calibrado: ");
  Serial.println(blueThreshold);
  while (Serial.available() > 0) { Serial.read(); }

  Serial.println("Colores calibrados. El robot comenzará a moverse.");
}

// Función para detectar una pared usando el sensor ultrasónico
bool detectWall() {
  int distance = sonar.ping_cm();
  if (distance > 0 && distance < 30) { // Pared detectada a menos de 30 cm
    return true; 
  } else {
    return false;
  }
}

// Control de los motores usando PID
void moveForward() {
  input = analogRead(A0);  // Sensor para ajustar el movimiento
  pid.Compute();
  
  // Control de motores
  analogWrite(MOTOR_LEFT_PIN_A, output);
  analogWrite(MOTOR_RIGHT_PIN_A, output);
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_PIN_A, 0);
  analogWrite(MOTOR_RIGHT_PIN_A, 0);
}

void setup() {
  Serial.begin(9600);
  // Inicialización de sensores y motores
  TCS3200_init(S0, S1, S2, S3, OUT_PIN);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Inicialización del giroscopio MPU6050
  Wire.begin();
  mpu.initialize();
  
  // PID
  pid.SetMode(AUTOMATIC);

  // Calibración de colores
  calibrateColors();
}

void loop() {
  // Leer el color actual
  readColor();
  
  // Comprobación de color negro
  if (isBlack()) {
    stopMotors();
    Serial.println("Color negro detectado, retrocediendo...");
    // Lógica de retroceso y giro
  } else {
    // Continúa moviéndose
    moveForward();
  }

  // Detección de pared
  if (detectWall()) {
    stopMotors();
    Serial.println("Pared detectada, ajustando dirección...");
    // Lógica de ajuste de dirección
  }
}
