// QTR con PID para seguir linea
// Bruno Vázquez Espinoza

#include <QTRSensors.h>

QTRSensorsAnalog qtra((unsigned char[]) {6, 7, 8, 9, 0, 1, 2, 3 }, 8);
unsigned int IR[8];

// PID
const float kp=0.2; // Constante proporcional
const float ki=0.0003; // Constante integral
const float kd=0.2; // Constante derivativa

const float kv=0.07; // Constante de velocidad (decaimiento exponencial)

float error=0;
int lastError=0;
int i=0;
int d=0;

// Variables para motores
int v = 255;
int vMin = 0;
int vMax = 255;

// Motores
Motor motorIzq(5, 4, 3, 2, 1, v, vMin, vMax); // Motor izquierdo
Motor motorDer(6, 7, 8, 9, 10, v, vMin, vMax); // Motor derecho

void setup(){
    //
}

void loop(){
    qtra.read(IR); // Leer los valores de los sensores IR
    error = -7*IR[0]-5*IR[1]-3*IR[2]-IR[3]+IR[4]+3*IR[5]+5*IR[6]+7*IR[7]; // Calcular el error ponderado de la línea
    i += error; // Acumular el error para el término integral
    d = error - lastError; // Calcular el cambio en el error para el término derivativo
    lastError = error; // Actualizar el último error
    if ((error * i) < 0) i = 0; // Si el error cambia de signo, reiniciar el término integral
    
    float PID = kp * error + ki * i + kd * d; // Calcular el valor PID
    v = vMin + (vMax - vMin) * exp(-kv * abs(kp * error)); // Calcular la velocidad con decaimiento exponencial
    avanzar(v - PID, v + PID); // Ajustar la velocidad de los motores según el valor PID
}

// Funcion para controlar motores
void avanzar(int left, int right){
    motorIzq.cambiarVelocidad(left);
    motorDer.cambiarVelocidad(right);
    motorIzq.giroHorario();
    motorDer.giroHorario();
}