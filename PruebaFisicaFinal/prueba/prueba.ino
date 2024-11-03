#include <PID_v1.h>

double setpoint = 1000; // Velocidad deseada en RPM
double input, output; // Variables de entrada y salida
double Kp = 100; // Ajusta según sea necesario

volatile unsigned long pulseCount = 0;

// Interrupción que se activa con cada pulso del encoder
void encoderISR() {
    pulseCount++;
}

void setup() {
    Serial.begin(9600);
    // Configura los pines
    pinMode(6, INPUT);
    pinMode(4, OUTPUT);
    pinMode(23, OUTPUT);
    pinMode(44, OUTPUT);
    pinMode(2, INPUT);
    digitalWrite(4, HIGH);
    digitalWrite(23, LOW);
    attachInterrupt(digitalPinToInterrupt(2), encoderISR, RISING);
}

void loop() {
    static unsigned long lastTime = 0;
    if (millis() - lastTime >= 1000) {
        // Calcula las RPM
        input = (pulseCount * 60) / 22; // Asegúrate de que este valor sea correcto
        Serial.print("Pulsos en 1 segundo: ");
        Serial.println(pulseCount);
        pulseCount = 0;

        double error = setpoint - input;

        // Control Proporcional Simple
        output = Kp * error;

        // Limitar la salida entre 50 y 255
        if (output < 50) output = 50;
        if (output > 255) output = 255;

        // Usa la salida para controlar el motor
        analogWrite(44, output);
        Serial.print("RPM: ");
        Serial.println(input);
        Serial.print("Salida P: ");
        Serial.println(output);
        Serial.print("Error: ");
        Serial.println(error);
        
        lastTime = millis();
    }
}
