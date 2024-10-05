// This program is designed to control a robot using various sensors and actuators. 
// It includes PID control for motor speed, color detection using the TCS3200 sensor, 
// distance measurement using an ultrasonic sensor, and orientation correction using the MPU6050 gyroscope. 
// The robot can detect walls and adjust its direction, move forward, and respond to different colors detected on the ground.

#include <PID_v1.h>  // Librería para el control PID
#include <Wire.h>  // Librería para la comunicación I2C
#include <MPU6050.h>  // Librería para el sensor MPU6050
#include <TCS3200.h>  // Librería para el sensor de color TCS3200
#include <NewPing.h>  // Librería para el sensor ultrasónico

// Definición de pines
#define MOTOR_RIGHT_PWM 5  // Pin PWM del motor derecho
#define MOTOR_LEFT_PWM 6  // Pin PWM del motor izquierdo
#define MOTOR_RIGHT_DIR 7  // Pin de dirección del motor derecho
#define MOTOR_LEFT_DIR 8  // Pin de dirección del motor izquierdo
#define ENCODER_RIGHT_A 2  // Pin del encoder derecho
#define ENCODER_LEFT_A 3  // Pin del encoder izquierdo
#define TRIG_PIN 9  // Pin de trigger del sensor ultrasónico
#define ECHO_PIN 10  // Pin de echo del sensor ultrasónico

// Parámetros del PID
double Setpoint, Input, Output;  // Variables para el PID
double Kp = 1, Ki = 0.05, Kd = 0.25;  // Constantes del PID

// Objetos para motores con PID
PID motorRightPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);  // PID para el motor derecho
PID motorLeftPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);  // PID para el motor izquierdo

// Variables globales
volatile int encoderCountRight = 0;  // Contador del encoder derecho
volatile int encoderCountLeft = 0;  // Contador del encoder izquierdo

// Parámetros del sensor ultrasónico
NewPing sonar(TRIG_PIN, ECHO_PIN, 50);  // Objeto para el sensor ultrasónico

// Sensor de color TCS3200
TCS3200 tcs3200;  // Objeto para el sensor de color

// Giroscopio MPU6050
MPU6050 mpu;  // Objeto para el giroscopio
int16_t ax, ay, az;  // Variables para la aceleración
int16_t gx, gy, gz;  // Variables para la velocidad angular

void setup() {
    Serial.begin(9600);  // Inicializar comunicación serial

    // Configurar pines de motores
    pinMode(MOTOR_RIGHT_PWM, OUTPUT);  // Configurar pin PWM del motor derecho como salida
    pinMode(MOTOR_LEFT_PWM, OUTPUT);  // Configurar pin PWM del motor izquierdo como salida
    pinMode(MOTOR_RIGHT_DIR, OUTPUT);  // Configurar pin de dirección del motor derecho como salida
    pinMode(MOTOR_LEFT_DIR, OUTPUT);  // Configurar pin de dirección del motor izquierdo como salida

    // Configurar encoders
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), encoderRightISR, RISING);  // Interrupción para el encoder derecho
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), encoderLeftISR, RISING);  // Interrupción para el encoder izquierdo

    // Configurar PID
    Setpoint = 100;  // Velocidad deseada
    motorRightPID.SetMode(AUTOMATIC);  // Configurar PID del motor derecho en modo automático
    motorLeftPID.SetMode(AUTOMATIC);  // Configurar PID del motor izquierdo en modo automático

    // Inicializar sensor de color
    tcs3200.begin();  // Inicializar el sensor de color

    // Inicializar MPU6050
    Wire.begin();  // Inicializar comunicación I2C
    mpu.initialize();  // Inicializar el giroscopio
    if (!mpu.testConnection()) {  // Verificar conexión con el giroscopio
        Serial.println("Error en la conexión del MPU6050");  // Imprimir mensaje de error si falla la conexión
    }
}

void loop() {
    // Detectar pared
    if (detectWall()) {  // Si se detecta una pared
        adjustDirection();  // Ajustar la dirección
    } else {
        advance();  // Avanzar
        readColor();  // Leer el color
    }
}

// ISR de los encoders
void encoderRightISR() {
    encoderCountRight++;  // Incrementar contador del encoder derecho
}

void encoderLeftISR() {
    encoderCountLeft++;  // Incrementar contador del encoder izquierdo
}

// Función para avanzar el robot
void advance() {
    Setpoint = 100;  // Velocidad objetivo

    // Leer los encoders y ajustar con PID
    Input = encoderCountRight;  // Valor actual del encoder derecho
    motorRightPID.Compute();  // Calcular salida del PID para el motor derecho
    analogWrite(MOTOR_RIGHT_PWM, Output);  // Ajustar velocidad del motor derecho

    Input = encoderCountLeft;  // Valor actual del encoder izquierdo
    motorLeftPID.Compute();  // Calcular salida del PID para el motor izquierdo
    analogWrite(MOTOR_LEFT_PWM, Output);  // Ajustar velocidad del motor izquierdo

    // Reset de los encoders para la próxima lectura
    encoderCountRight = 0;  // Reiniciar contador del encoder derecho
    encoderCountLeft = 0;  // Reiniciar contador del encoder izquierdo
}

// Función para leer el color
void readColor() {
    String color = tcs3200.readColor();  // Leer color del sensor

    if (color == "black") {  // Si el color es negro
        // Retroceder y girar 90 grados
        reverseAndTurn();  // Retroceder y girar
    } else {
        // Encender LED del color detectado y guardar el color
        displayLED(color);  // Mostrar color en el LED
        logColor(color);  // Guardar el color detectado
    }
}

// Función para retroceder y girar 90°
void reverseAndTurn() {
    reverse();  // Retroceder

    // Corrección del giro usando el giroscopio
    correctTurn(90);  // Corregir giro a 90 grados
}

// Función para retroceder
void reverse() {
    digitalWrite(MOTOR_RIGHT_DIR, LOW);  // Configurar dirección del motor derecho para retroceder
    digitalWrite(MOTOR_LEFT_DIR, LOW);  // Configurar dirección del motor izquierdo para retroceder
    analogWrite(MOTOR_RIGHT_PWM, 150);  // Ajustar velocidad del motor derecho
    analogWrite(MOTOR_LEFT_PWM, 150);  // Ajustar velocidad del motor izquierdo
    delay(500);  // Retroceder por medio segundo

    // Detener motores
    analogWrite(MOTOR_RIGHT_PWM, 0);  // Detener motor derecho
    analogWrite(MOTOR_LEFT_PWM, 0);  // Detener motor izquierdo
}

// Función para corregir el giro usando el MPU6050
void correctTurn(int targetAngle) {
    int currentAngle = 0;  // Inicializar ángulo actual

    while (currentAngle < targetAngle) {  // Mientras el ángulo actual sea menor al objetivo
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  // Leer datos del giroscopio
        currentAngle += abs(gz) / 131;  // Integración aproximada para calcular ángulo

        // Girar a la derecha
        digitalWrite(MOTOR_RIGHT_DIR, LOW);  // Configurar dirección del motor derecho para retroceder
        digitalWrite(MOTOR_LEFT_DIR, HIGH);  // Configurar dirección del motor izquierdo para avanzar
        analogWrite(MOTOR_RIGHT_PWM, 150);  // Ajustar velocidad del motor derecho
        analogWrite(MOTOR_LEFT_PWM, 150);  // Ajustar velocidad del motor izquierdo

        delay(50);  // Esperar 50 milisegundos
    }

    // Detener motores
    analogWrite(MOTOR_RIGHT_PWM, 0);  // Detener motor derecho
    analogWrite(MOTOR_LEFT_PWM, 0);  // Detener motor izquierdo
}

// Función para detectar paredes usando el sensor ultrasónico
bool detectWall() {
    int distance = sonar.ping_cm();  // Medir distancia con el sensor ultrasónico
    if (distance > 0 && distance < 30) {  // Si la distancia es menor a 30 cm
        return true;  // Pared detectada
    } else {
        return false;  // No se detecta pared
    }
}

// Función para ajustar dirección
void adjustDirection() {
    correctTurn(90);  // Girar 90 grados si hay una pared
}

// Función para mostrar el color en el LED
void displayLED(String color) {
    if (color == "red") {  // Si el color es rojo
        // Encender LED rojo
    } else if (color == "green") {  // Si el color es verde
        // Encender LED verde
    } else if (color == "blue") {  // Si el color es azul
        // Encender LED azul
    }
}

// Función para guardar el color detectado
void logColor(String color) {
    // Registrar el color para futuras referencias
}
