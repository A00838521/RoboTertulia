#include <QTRSensors.h>  // Librería para los sensores de línea QTR
#include <math.h>        // Librería para funciones matemáticas como exp()
#include "I2Cdev.h"      // Librería para el bus I2C
#include "MPU6050.h"     // Librería para el giroscopio MPU6050
#include "Wire.h"        // Librería para el bus I2C


// ------------------------------------ Clase Motor ------------------------------------
class Motor {
  private:
    int ENC_A, ENC_B;    // Pines del encoder
    int IN1, IN2, ENA;   // Pines del motor
    int velocidad;       // Velocidad actual del motor
    int velocidadMax;    // Velocidad máxima permitida
    int velocidadMin;    // Velocidad mínima permitida
    
    // PID variables
    float kp, ki, kd;     // Coeficientes del PID
    float errorAnterior;  // Error anterior (para derivada)
    float integral;       // Acumulado del error (para integral)
    
    // Encoder variables
    static volatile long pulsos; // Contador de pulsos, ahora es estático
    int ppr;              // Pulsos por revolución del encoder
    
    unsigned long tiempoAnterior;
    
    static Motor* instanciaActual;  // Puntero estático a la instancia actual

  public:
    // Constructor
    Motor(int encA, int encB, int in1, int in2, int ena, int vMin, int vMax, float _kp, float _ki, float _kd, int _ppr) {
      ENC_A = encA;
      ENC_B = encB;
      IN1 = in1;
      IN2 = in2;
      ENA = ena;
      velocidadMin = vMin;
      velocidadMax = vMax;
      velocidad = vMax; // Iniciar con la velocidad máxima
      
      kp = _kp;
      ki = _ki;
      kd = _kd;
      ppr = _ppr;
      pulsos = 0;
      errorAnterior = 0;
      integral = 0;
      
      pinMode(ENC_A, INPUT);
      pinMode(ENC_B, INPUT);
      pinMode(IN1, OUTPUT);
      pinMode(IN2, OUTPUT);
      pinMode(ENA, OUTPUT);
      
      // Establecer la instancia actual para ser usada en la interrupción
      instanciaActual = this;
      
      // Configurar interrupción para leer el encoder
      attachInterrupt(digitalPinToInterrupt(ENC_A), incrementarPulsosWrapper, RISING);
      tiempoAnterior = millis();
    }
    
    // Función estática para acceder a la función miembro desde la interrupción
    static void incrementarPulsosWrapper() {
      instanciaActual->incrementarPulsos();
    }
    
    // Incrementar pulsos del encoder
    void incrementarPulsos() {
      pulsos++;
    }

    // Calcular las revoluciones basadas en pulsos del encoder
    float calcularRevoluciones() {
      return (float)pulsos / ppr;     // Calcula revoluciones como pulsos / pulsos por revolución
    }

    // Resetear los pulsos del encoder
    void resetearPulsos() {
      pulsos = 0;
    }

    // // Calcular PID
    // void controlarPID(float setpoint) {
    //   float error = setpoint - calcularRevoluciones();
    //   float dt = (millis() - tiempoAnterior) / 1000.0;  // Tiempo en segundos

    //   integral += error * dt;

    //   // Limitar el valor de la integral para evitar sobrecarga de acumulación
    //   float integralLimit = 50;  // Ajusta este límite según sea necesario
    //   integral = constrain(integral, -integralLimit, integralLimit);

    //   float derivada = (error - errorAnterior) / dt;
    //   float salida = (kp * error) + (ki * integral) + (kd * derivada);

    //   // Aumentar ligeramente la velocidad si el PID calcula valores cercanos a velocidadMin
    //   if (salida < velocidadMin + 10) {  // 10 es el umbral; ajústalo según necesidad
    //     salida += 10;  // Incremento mínimo
    //   }

    //   cambiarVelocidad(constrain(salida, velocidadMin, velocidadMax));

    //   errorAnterior = error;
    //   tiempoAnterior = millis();
    // }


    // Giro horario
    void giroHorario() {
      analogWrite(ENA, velocidad);  // Control de velocidad variable
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      Serial.print(F("----Giro Horario a velocidad: "));
      Serial.println(velocidad);
    }

    // Giro antihorario
    void giroAntihorario() {
      analogWrite(ENA, velocidad);  // Control de velocidad variable
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      Serial.print(F("----Giro Antihorario a velocidad: "));
      Serial.println(velocidad);
    }

    // Apagar motor
    void apagar() {
      analogWrite(ENA, 0);
      Serial.println(F("----Motor Apagado----"));
    }

    // Cambiar velocidad con límites (min y max)
    void cambiarVelocidad(int nuevaVelocidad) {
      // Asegurar que la velocidad sea como mínimo la permitida
      velocidad = max(nuevaVelocidad, velocidadMin);
      analogWrite(ENA, velocidad);
      Serial.print(F("Velocidad ajustada a: "));
      Serial.println(velocidad);
    }

    
    // Función para avanzar un número específico de revoluciones
    void avanzarRevoluciones(float revoluciones, bool sentidoHorario) {
      // Resetear contador de pulsos
      pulsos = 0;
      float setpoint = revoluciones;
      
      while (calcularRevoluciones() < setpoint) {
        
        // Aumentar temporalmente la velocidad inicial para asegurar el inicio del movimiento
        if (calcularRevoluciones() == 0) {
          cambiarVelocidad(velocidadMin + 20);  // Incremento inicial para arrancar
        }

        // controlarPID(setpoint);
        
        // Girar en sentido horario o antihorario según el motor
        if (sentidoHorario) {
          giroHorario();
        } else {
          giroAntihorario();
        }
      }
      apagar();
    }

    // Leer encoder (simulación)
    void leerEncoder() {
      Serial.print("Pulsos: ");
      Serial.println(pulsos);
    }
};



// ------------------------------------ Definiciones ------------------------------------
// Inicializar variables estáticas
volatile long Motor::pulsos = 0;
Motor* Motor::instanciaActual = nullptr;

// Definiciones para el sensor ultrasónico
#define trigPin1 28 // Derecha
#define echoPin1 10 // Derecha
#define trigPin2 27 // Frontal
#define echoPin2 9  // Frontal
#define trigPin3 26 // Izquierda
#define echoPin3 8  // Izquierda

// Definiciones para el sensor de color
const int s0 = 29;
const int s1 = 11;
const int s2 = 12;
const int s3 = 30;
const int out = 31;

// Definiciones para el LED RGB
const int pinRed = 32;
const int pinGreen = 33;
const int pinBlue = 34;

// Inicialización del giroscopio
MPU6050 sensor;
int ax, ay, az;
int gx, gy, gz;

// Definición del sensor QTR
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// PID
const float kp = 0.2;   // Constante proporcional
const float ki = 0.0003; // Constante integral
const float kd = 0.2;   // Constante derivativa
const float kv = 0.07;  // Constante de velocidad (decaimiento exponencial)

float error = 0;     // Error ponderado
int lastError = 0;   // Último error
int i = 0;           // Acumulador para el término integral
int d = 0;           // Cambio en el error para el término derivativo

// Variables para motores
int vMin = 60;        // Velocidad mínima
int vMax = 255;      // Límite superior de la velocidad
int v = 150;

// Definición de motores
// - Las constantes de PID se dejaron de usar, sin embargo las mantengo como prueba de que se intento implementar sin exito.
Motor motorIzq(2, 6, 4, 23, 44, vMin, vMax, 80, 10, 15, 360);  // Motor izquierdo
Motor motorDer(3, 7, 5, 24, 45, vMin, vMax, 0.2, 0.005, 0.05, 360); // Motor derecho




// ------------------------------------------- Setup -----------------------------------------
void setup() {
  // // ------------------------------------ Sensor QTR -------------------------------------------
  // qtr.setTypeAnalog();
  // qtr.setSensorPins((const uint8_t[]){A4, A5, A6, A7, A8, A9, A10, A11}, SensorCount);
  // qtr.setEmitterPin(35);

  // delay(250);
  // setColor(0, 255, 0);  // Encender LED RGB al iniciar

  // // analogRead() takes about 0.1 ms on an AVR.
  // // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // // Call calibrate() 400 times to make calibration take about 10 seconds.
  // for (uint16_t i = 0; i < 400; i++){
  //   qtr.calibrate();
  // }
  // digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  // for (uint8_t i = 0; i < SensorCount; i++){
  //   Serial.print(qtr.calibrationOn.minimum[i]);
  //   Serial.print(' ');
  // }
  // Serial.println();

  // // print the calibration maximum values measured when emitters were on
  // for (uint8_t i = 0; i < SensorCount; i++){
  //   Serial.print(qtr.calibrationOn.maximum[i]);
  //   Serial.print(' ');
  // }
  // Serial.println();
  // Serial.println();
  // setColor(0, 0, 0);  // Apagar al iniciar
  // delay(1000);
  // // ------------------------------------------------------------------------------------------------

  // Configuración del giroscopio
  Wire.begin();
  sensor.initialize();
  if (sensor.testConnection()) {
    Serial.println("Giroscopio iniciado correctamente.");
  } else {
    Serial.println("Error al iniciar giroscopio.");
  }

  // Configuración del sensor de color
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(out, INPUT);
  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);

  // Configuración del LED RGB
  pinMode(pinRed, OUTPUT);
  pinMode(pinGreen, OUTPUT);
  pinMode(pinBlue, OUTPUT);
  setColor(0, 0, 0);  // Apagar al iniciar
  // Apagar todos los colores al iniciar (para ánodo común, HIGH apaga)
  digitalWrite(pinRed, HIGH);
  digitalWrite(pinGreen, HIGH);
  digitalWrite(pinBlue, HIGH);
}



// ------------------------------------- Loop -------------------------------------
void loop() {
  // estados(1);
  readGyroData();
  delay(1000);
  Serial.println("-------------------");
  // antiwallMovement(motorIzq, motorDer);
  // avanzarMotoresSincronizados(motorIzq, motorDer, 2, 2, true, false);
}


// ------------------------ Funciones Principales (Retos) -------------------------
void seguidorDeLinea() {
  // Leer los valores de los sensores
  uint16_t position = qtr.readLineBlack(sensorValues);

  // Cálculo del error ponderado de la línea, basado en los sensores
  error = -7 * sensorValues[0] - 5 * sensorValues[1] - 3 * sensorValues[2] - sensorValues[3] + sensorValues[4] + 3 * sensorValues[5] + 5 * sensorValues[6] + 7 * sensorValues[7];

  // Suma del error para el término integral
  i += error;

  // Diferencia del error para el término derivativo
  d = error - lastError;

  // Actualizar el último error
  lastError = error;

  // Reiniciar el término integral si el error cambia de signo
  if ((error * i) < 0) i = 0;

  // Cálculo del valor PID
  float PID = kp * error + ki * i + kd * d;

  // Calcular la velocidad con decaimiento exponencial
  int v = vMin + (vMax - vMin) * exp(-kv * abs(kp * error));

  // Ajustar la velocidad de los motores según el valor PID
  avanzar(v - PID, v + PID);
}



// void laberintoPelota() { // Pendiente de Revisar
//   bool pelotaRecogida = false;  // Estado para verificar si la pelota fue recogida
//   float distanciaAvanzada = 0;
  
//   while (true) {
//     distanciaAvanzada = 0;  // Reinicia el contador
//     // Paso 1: Avanza 30 cm
//     avanzarMotoresSincronizados(motorIzq, motorDer, 1, 1, true, false);
//     distanciaAvanzada += 30;
    
//     // Cada 15 cm, verifica si hay línea negra
//     if (distanciaAvanzada >= 15) {
//       if (detectarLineaNegra()) {
//         // Si ya se recogió la pelota y detecta línea negra
//         if (pelotaRecogida) {
//           avanzarMotoresSincronizados(motorIzq, motorDer, 1, 1, false, true);  // Retrocede 20 cm
//           avanzarMotoresSincronizados(motorIzq, motorDer, 1.7, 1.7, true, true);  // Gira 180 grados
//         } else {
//           avanzarMotoresSincronizados(motorIzq, motorDer, 1, 1, false, true);  // Retrocede 15 cm
//           avanzarMotoresSincronizados(motorIzq, motorDer, 1.7, 1.7, true, true);  // Gira 180 grados
//           continue;  // Vuelve a intentar avanzar
//         }
//       }
//       distanciaAvanzada = 0;  // Reinicia el contador
//     }

//     // Paso 2: Detección con sensor ultrasónico frontal
//     if (leerDistanciaUltrasonico(trigPin2, echoPin2) <= 10) {
//       // Hay pared en frente
//       if (leerDistanciaUltrasonico(trigPin1, echoPin1) <= 10) {
//         // Pared a la derecha, gira a la derecha
//         avanzarMotoresSincronizados(motorIzq, motorDer, 1, 1, false, false);  // Gira a la izquierda
//       } else {
//         // No hay pared a la derecha, gira y avanza
//         avanzarMotoresSincronizados(motorIzq, motorDer, 1, 1, true, true);  // Gira a la derecha
//         avanzarMotoresSincronizados(motorIzq, motorDer, 1, 1, true, false);  // Avanza 15 cm
//       }
//     } else {
//       // No hay pared en frente, avanzar 15 cm y recoger la pelota
//       avanzarMotoresSincronizados(motorIzq, motorDer, 1, 1, true, false);  // Avanza 15 cm
//       Serial.println("Recojo pelota");
//       pelotaRecogida = true;  // Cambia el estado a recogido
//       avanzarMotoresSincronizados(motorIzq, motorDer, 1, 1, false, true);  // Retrocede 15 cm
//     }

//     // Detección del color rojo
//     if (getRojo() > 1000) {
//       Serial.println("Color rojo detectado, función terminada.");
//       motorIzq.apagar();
//       motorDer.apagar();
//       break;  // Termina el ciclo
//     }
//   }
// }



// ------------------------ Todas las funciones auxiliares ------------------------

// Función para detectar línea negra
bool detectarLineaNegra() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] >= 400) { // Si detecta línea negra
      return true; // Devuelve verdadero
    }
  }
  return false;  // Si no detecta línea negra
}

// Función para ajustar la velocidad de los motores
void avanzar(int left, int right) {
  motorIzq.cambiarVelocidad(left);
  motorDer.cambiarVelocidad(right);

  // Ambos motores deben girar en sentido horario para avanzar
  motorIzq.giroHorario();
  motorDer.giroAntihorario();
}

// Función para avanzar y evitar las paredes
void antiwallMovement(Motor& motor1, Motor& motor2) {
    const int minDist = 10; // Distancia mínima en cm para corrección

    // Variables para almacenar la distancia a los objetos detectados
    Serial.print("Frente ");
    int distFront = getDistance(trigPin2, echoPin2);
    Serial.print("Izquierda ");
    int distLeft = getDistance(trigPin3, echoPin3);
    Serial.print("Derecha ");
    int distRight = getDistance(trigPin1, echoPin1);

    // Corrección si hay obstáculos a los lados
    if (distLeft < minDist) {
        // Obstáculo a la izquierda, realizar un giro suave a la derecha
        Serial.println("Correccion Pared IZQUIERDA");
        avanzarMotoresSincronizados(motor1, motor2, 1, 0, true, false); // Revoluciones ajustadas para giro suave
    } else if (distRight < minDist) {
        // Obstáculo a la derecha, realizar un giro suave a la izquierda
        Serial.println("Correccion Pared DERECHA");
        avanzarMotoresSincronizados(motor1, motor2, 0, 1, true, false); // Revoluciones ajustadas para giro suave
    }

    // Lógica de movimiento
    if (distFront >= minDist) {
        // Avanzar en línea recta si no hay obstáculo en el frente
        Serial.println("Avanzar Recto");
        avanzarMotoresSincronizados(motor1, motor2, 1, 1, true, false);  // Mismo número de revoluciones para avanzar recto
    } else {
        // Si hay un obstáculo al frente, decidir la dirección en función del espacio a los lados
        if (distLeft > distRight) {
            // Hay más espacio a la izquierda: girar a la izquierda
            Serial.println("Avanzar a la Izquierda");
            avanzarMotoresSincronizados(motor1, motor2, 2.5, 2.5, true, true); // Giro de 180° a la izquierda
        } else {
            // Hay más espacio a la derecha: girar a la derecha
            Serial.println("Avanzar a la Derecha");
            avanzarMotoresSincronizados(motor1, motor2, 2.5, 2.5, false, false); // Giro de 180° a la derecha
        }
    }
}

// Función para avanzar una distancia específica
void avanzarMotoresSincronizados(Motor& motor1, Motor& motor2, float revolucionesIzq, float revolucionesDer, bool sentidoHorarioMotor1, bool sentidoHorarioMotor2) {
    // Resetear contador de pulsos en ambos motores
    motor1.resetearPulsos();
    motor2.resetearPulsos();
    
    // Determinar los setpoints en términos de revoluciones
    float setpoint1 = revolucionesIzq;
    float setpoint2 = revolucionesDer;
    
    // Calcula la relación de velocidad para sincronizar ambos motores
    float velocidadBase1 = (setpoint1 > setpoint2) ? v : ((setpoint1 / setpoint2) * v);
    float velocidadBase2 = (setpoint2 > setpoint1) ? v : ((setpoint2 / setpoint1) * v);

    // Bucle para avanzar ambos motores en paralelo
    while (motor1.calcularRevoluciones() < setpoint1 || motor2.calcularRevoluciones() < setpoint2) {
        // Lee la cantidad de revoluciones actual
        float revsMotor1 = motor1.calcularRevoluciones();
        float revsMotor2 = motor2.calcularRevoluciones();

        // Ajusta la velocidad del motor que va más rápido
        if (revsMotor1 > revsMotor2) {
            motor1.cambiarVelocidad(velocidadBase1 * 0.8);  // Reduce velocidad del motor 1 al 80%
            motor2.cambiarVelocidad(velocidadBase2);         // Mantiene velocidad del motor 2
        } else if (revsMotor2 > revsMotor1) {
            motor2.cambiarVelocidad(velocidadBase2 * 0.8);  // Reduce velocidad del motor 2 al 80%
            motor1.cambiarVelocidad(velocidadBase1);         // Mantiene velocidad del motor 1
        } else {
            // Ambos motores avanzan a la velocidad base si están sincronizados
            motor1.cambiarVelocidad(velocidadBase1);
            motor2.cambiarVelocidad(velocidadBase2);
        }

        // Control del sentido y avance de los motores
        if (revsMotor1 < setpoint1) {
            if (sentidoHorarioMotor1) motor1.giroHorario();
            else motor1.giroAntihorario();
        } else motor1.apagar();  // Apaga el motor cuando llega al setpoint

        if (revsMotor2 < setpoint2) {
            if (sentidoHorarioMotor2) motor2.giroHorario();
            else motor2.giroAntihorario();
        } else motor2.apagar();  // Apaga el motor cuando llega al setpoint
    }


    // Asegurarse de que ambos motores se apaguen al finalizar
    motorsOFF();
}

// Función para apagar los motores Frenado Seco
void motorsOFF() {
  motorIzq.apagar();
  motorDer.apagar();
  delay(1);
  // motorDer.giroHorario();
  // motorIzq.giroAntihorario();
  // delay(1);
  // motorDer.apagar();
  // motorIzq.apagar();
}

// Funciones para el sensor ultrasónico
float getDistance(int trigPin, int echoPin) {
  // Enviar un pulso de ultrasonido
  pinMode(trigPin, OUTPUT);  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  pinMode(echoPin, INPUT);

  // Leer el tiempo del eco
  long duration = pulseIn(echoPin, HIGH);

  // Calcular la distancia en cm
  float distancia = (duration * 0.0343) / 2;
  Serial.print("Distancia: ");
  Serial.println(distancia);
  return distancia;
}

// Funciones para el sensor de color
int getRojo(){
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  return pulseIn(out, LOW);  // Devuelve la intensidad del color rojo
}

int getAzul(){
  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH);
  return pulseIn(out, LOW);  // Devuelve la intensidad del color azul
}

int getVerde(){
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);
  return pulseIn(out, LOW);  // Devuelve la intensidad del color verde
}

// Funciones para el giroscopio (MPU6050)
void readGyroData() {
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);
  
  Serial.print("Acelerometro [x, y, z]: ");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.println();

  Serial.print("Giroscopio [x, y, z]: ");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.print(gz); Serial.println();
}

// Funciones para el LED RGB
void setColor(int red, int green, int blue) {
  analogWrite(pinRed, 255 - red);  // Anodo común
  analogWrite(pinGreen, 255 - green);
  analogWrite(pinBlue, 255 - blue);
}

// ------------------------ Funciones de Prueba ------------------------
void pruebaSensores() {
  // Prueba de los sensores QTR
  uint16_t position = qtr.readLineBlack(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();
}

void estados(int opcion){
  switch(opcion){
    case 1:
      setColor(255, 0, 0);  // Color rojo
      delay(500);
      setColor(0,0,0);  // Apagar
      delay(500);
      break;
    case 2:
      setColor(0, 255, 0);  // Color verde
      delay(500);
      setColor(0,0,0);  // Apagar
      delay(500);
      break;
    case 3:
      setColor(0, 0, 255);  // Color azul
      delay(500);
      setColor(0,0,0);  // Apagar
      delay(500);
      break;
    default:
      Serial.println("Estado por defecto");
      break;
  }
}