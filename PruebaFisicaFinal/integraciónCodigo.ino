#include <QTRSensors.h>  // Librería para los sensores de línea QTR
#include <math.h>        // Librería para funciones matemáticas como exp()
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

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
    float diametroRueda;  // Diámetro de la rueda en cm
    int ppr;              // Pulsos por revolución del encoder
    
    unsigned long tiempoAnterior;
    
    static Motor* instanciaActual;  // Puntero estático a la instancia actual

  public:
    // Constructor
    Motor(int encA, int encB, int in1, int in2, int ena, int vMin, int vMax, float _kp, float _ki, float _kd, float dRueda, int _ppr) {
      ENC_A = encA;
      ENC_B = encB;
      IN1 = in1;
      IN2 = in2;
      ENA = ena;
      velocidadMin = vMin;
      velocidadMax = vMax;
      velocidad = vMin; // Iniciar con la velocidad mínima
      
      kp = _kp;
      ki = _ki;
      kd = _kd;
      diametroRueda = dRueda;
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

    // Calcular distancia recorrida en cm basada en pulsos del encoder
    float calcularDistancia() {
      float circunferencia = PI * diametroRueda;  // Circunferencia de la rueda
      return (pulsos * circunferencia) / ppr;     // Distancia recorrida en cm
    }

    // Calcular PID
    void controlarPID(float setpoint) {
      float error = setpoint - calcularDistancia();
      float dt = (millis() - tiempoAnterior) / 1000.0;  // Tiempo en segundos

      integral += error * dt;
      float derivada = (error - errorAnterior) / dt;
      float salida = (kp * error) + (ki * integral) + (kd * derivada);

      cambiarVelocidad(constrain(salida, velocidadMin, velocidadMax));

      errorAnterior = error;
      tiempoAnterior = millis();
    }

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
      if (nuevaVelocidad >= velocidadMin && nuevaVelocidad <= velocidadMax) {
        velocidad = nuevaVelocidad;
        Serial.print(F("Se cambió la velocidad a: "));
        Serial.println(velocidad);
      } else {
        Serial.println(F("Velocidad fuera de los límites permitidos."));
      }
    }
    
    // Función para avanzar una distancia específica en cm
    void avanzarDistancia(float distancia, bool sentidoHorario) {
      // Resetear contador de pulsos
      pulsos = 0;
      float setpoint = distancia;
      
      while (calcularDistancia() < setpoint) {
        controlarPID(setpoint);
        
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

    // Inicializar (nueva función basada en el segundo código)
    void inicializar() {
      pinMode(IN1, OUTPUT);
      pinMode(IN2, OUTPUT);
      pinMode(ENA, OUTPUT);
    }
};

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
QTRSensorsAnalog qtra((unsigned char[]) {A4, A5, A6, A7, A8, A9, A10, A11 }, 8);
unsigned int IR[8];  // Matriz para almacenar los valores de los sensores IR

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
int vMin = 0;        // Velocidad mínima
int vMax = 255;      // Límite superior de la velocidad

// Definición de motores
Motor motorIzq(2, 3, 4, 23, 22, vMin, vMax, 0.1, 0.01, 0.05, 7.0, 360);  // Motor izquierdo
Motor motorDer(6, 7, 5, 24, 25, vMin, vMax, 0.1, 0.01, 0.05, 7.0, 360); // Motor derecho

void setup() {
  Serial.begin(9600);  // Inicia la comunicación serial para depuración
  motorIzq.inicializar();
  motorDer.inicializar();

  // Calibración del sensor QTR
  for (uint16_t i = 0; i < 400; i++) {
    qtra.calibrate();
  }

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
}

void loop() {
  int rojo = getRojo();
  int azul = getAzul();
  int verde = getVerde();

  // Activar seguidor de línea si se detecta rojo
  if (rojo > 1000) { 
    seguidorDeLinea();
  }
  
  // Desactivar seguidor si se detecta azul
  if (azul > 1000) {
    motorIzq.apagar();
    motorDer.apagar();
  }

  // Realizar laberinto con pelota si se detecta verde
  if (verde > 1000) {
    laberintoPelota();
  }
}

// ------------------------ Funciones Principales (Retos) -------------------------
void seguidorDeLinea() {
  // Leer los valores de los sensores IR
  qtra.read(IR);

  // Cálculo del error ponderado de la línea, basado en los sensores
  error = -7 * IR[0] - 5 * IR[1] - 3 * IR[2] - IR[3] + IR[4] + 3 * IR[5] + 5 * IR[6] + 7 * IR[7];

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

void laberintoPelota() {
  bool pelotaRecogida = false;  // Estado para verificar si la pelota fue recogida
  float distanciaAvanzada = 0;
  
  while (true) {
    // Paso 1: Avanza 30 cm
    motorIzq.avanzarDistancia(30, true);
    motorDer.avanzarDistancia(30, true);
    distanciaAvanzada += 30;
    
    // Cada 15 cm, verifica si hay línea negra
    if (distanciaAvanzada >= 15) {
      if (detectarLineaNegra()) {
        // Si ya se recogió la pelota y detecta línea negra
        if (pelotaRecogida) {
          motorIzq.avanzarDistancia(20, false);  // Retrocede 20 cm
          motorDer.avanzarDistancia(20, false);
          motorIzq.avanzarDistancia(180, true);  // Gira 180 grados
          motorDer.avanzarDistancia(180, true);
        } else {
          motorIzq.avanzarDistancia(15, false);  // Retrocede 15 cm
          motorDer.avanzarDistancia(15, false);
          motorIzq.avanzarDistancia(180, false);  // Gira 180 grados
          motorDer.avanzarDistancia(180, false);
          continue;  // Vuelve a intentar avanzar
        }
      }
      distanciaAvanzada = 0;  // Reinicia el contador
    }

    // Paso 2: Detección con sensor ultrasónico frontal
    if (leerDistanciaUltrasonico(trigPin2, echoPin2) <= 10) {
      // Hay pared en frente
      if (leerDistanciaUltrasonico(trigPin1, echoPin1) <= 10) {
        // Pared a la derecha, gira a la derecha
        motorIzq.avanzarDistancia(90, true);  // Gira a la derecha
        motorDer.avanzarDistancia(90, true);
      } else {
        // No hay pared a la derecha, gira y avanza
        motorIzq.avanzarDistancia(90, true);  // Gira a la derecha
        motorDer.avanzarDistancia(90, true);
        motorIzq.avanzarDistancia(30, true);  // Avanza 30 cm
        motorDer.avanzarDistancia(30, true);
      }
    } else {
      // No hay pared en frente, avanzar 15 cm y recoger la pelota
      motorIzq.avanzarDistancia(15, true);
      motorDer.avanzarDistancia(15, true);
      Serial.println("Recojo pelota");
      pelotaRecogida = true;  // Cambia el estado a recogido
      motorIzq.avanzarDistancia(20, false);  // Retrocede 20 cm
      motorDer.avanzarDistancia(20, false);
    }

    // Detección del color rojo
    if (getRojo() > 1000) {
      Serial.println("Color rojo detectado, función terminada.");
      motorIzq.apagar();
      motorDer.apagar();
      break;  // Termina el ciclo
    }
  }
}

// ------------------------ Todas las funciones auxiliares ------------------------

// Función para ajustar la velocidad de los motores
void avanzar(int left, int right) {
  motorIzq.cambiarVelocidad(left);
  motorDer.cambiarVelocidad(right);

  // Ambos motores deben girar en sentido horario para avanzar
  motorIzq.giroHorario();
  motorDer.giroHorario();
}

// Funciones para el sensor ultrasónico
float leerDistanciaUltrasonico(int trigPin, int echoPin) {
  // Enviar un pulso de ultrasonido
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Leer el tiempo del eco
  long duration = pulseIn(echoPin, HIGH);

  // Calcular la distancia en cm
  float distancia = (duration * 0.0343) / 2;
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