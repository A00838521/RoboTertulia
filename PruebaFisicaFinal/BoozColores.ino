#include <QTRSensors.h>  // Librería para los sensores de línea QTR
#include <math.h>        // Librería para funciones matemáticas como exp()
#include <Wire.h>        // Librería para comunicación I2C


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
      // Serial.print(F("----Giro Horario a velocidad: "));
      // Serial.println(velocidad);
    }

    // Giro antihorario
    void giroAntihorario() {
      analogWrite(ENA, velocidad);  // Control de velocidad variable
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      // Serial.print(F("----Giro Antihorario a velocidad: "));
      // Serial.println(velocidad);
    }

    // Apagar motor
    void apagar() {
      analogWrite(ENA, 0);
      // Serial.println(F("----Motor Apagado----"));
    }

    // Cambiar velocidad con límites (min y max)
    void cambiarVelocidad(int nuevaVelocidad) {
      // Asegurar que la velocidad sea como mínimo la permitida
      velocidad = max(nuevaVelocidad, velocidadMin);
      analogWrite(ENA, velocidad);
      // Serial.print(F("Velocidad ajustada a: "));
      // Serial.println(velocidad);
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
const int pinGreen = 38;
const int pinBlue = 34;

// Inicialización del giroscopio
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float elapsedTime, currentTime, previousTime;
int c = 0;
const int windowSize = 10;
float yawWindow[windowSize] = {0.0};
float pitchWindow[windowSize] = {0.0};
int windowIndex = 0;
float yawSum = 0.0;
float pitchSum = 0.0;

// Definición del sensor QTR
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// PID
// const float kp = 0.04;   // Constante proporcional
// const float ki = 0.0003; // Constante integral
// const float kd = 0.2;   // Constante derivativa
// const float kv = 0.07;  // Constante de velocidad (decaimiento exponencial)
const float kp = 0.0;   // Constante proporcional
const float ki = 0.0; // Constante integral
const float kd = 0.0;   // Constante derivativa
const float kv = 0.0;  // Constante de velocidad (decaimiento exponencial)

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

int ultimoColor = -1;      // Almacena el último color detectado para mantener el LED encendido

int count = 1;

// ------------------------------------------- Setup -----------------------------------------
void setup() {
  /*
  // ------------------------------------ Sensor QTR -------------------------------------------
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A4, A5, A6, A7, A8, A9, A10, A11}, SensorCount);
  qtr.setEmitterPin(35);

  delay(250);
  setColor(0, 255, 0);  // Encender LED RGB al iniciar

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++){
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(19200);
  for (uint8_t i = 0; i < SensorCount; i++){
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++){
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  setColor(0, 0, 0);  // Apagar al iniciar
  delay(1000);
  // ------------------------------------------------------------------------------------------------
*/
  Serial.begin(19200);
  iniciarCalibracion(); // Llama a la función de calibración al inicio
  // Configuración del giroscopio
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  Wire.beginTransmission(MPU);       // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU);        // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);

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
  estados(1);
  // while(true){
  //   seguidorDeLinea();
  //   pruebaSensores();
  // }
  // antiwallMovement(motorIzq, motorDer);
  //antiwallContinuousMovement(motorIzq, motorDer, v);
  
  while(count != 0) {
    iniciarCalibracion();
    count--;
    }
  if(count == 0) {
    int colorActual = detectarColor();
    Serial.print("Color detectado: ");
    Serial.println(colorActual);
    // Si el último color detectado no es blanco, mantener el LED encendido
    if (colorActual == -1 ) {
      Serial.println("-1");
    }
    if (colorActual == 0 && ultimoColor != 0) {
      apagarLED();
    }
  }

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
  float left = v - PID;
  float right = v + PID;
  Serial.print(left); Serial.print("/"); Serial.println(right);
  avanzar(left, right);
}



void laberintoPelota() { // Pendiente de Revisar
  // Establecer pasos
}



// ------------------------ Todas las funciones auxiliares ------------------------
//Funcion para detectar el color, prender leds correspondientes al color, y un numero representando el color

// Variables de calibración para cada color
int minRosaR, maxRosaR, minRosaG, maxRosaG, minRosaB, maxRosaB;
int minAmarilloR, maxAmarilloR, minAmarilloG, maxAmarilloG, minAmarilloB, maxAmarilloB;
int minMoradoR, maxMoradoR, minMoradoG, maxMoradoG, minMoradoB, maxMoradoB;
int minRojoR, maxRojoR, minRojoG, maxRojoG, minRojoB, maxRojoB;
int minNegroR, maxNegroR, minNegroG, maxNegroG, minNegroB, maxNegroB;
int minBlancoR, maxBlancoR, minBlancoG, maxBlancoG, minBlancoB, maxBlancoB;
int minVerdeR, maxVerdeR, minVerdeG, maxVerdeG, minVerdeB, maxVerdeB;
int minAzulR, maxAzulR, minAzulG, maxAzulG, minAzulB, maxAzulB;

// Función para calibrar un color
void calibrarColor(int &minR, int &maxR, int &minG, int &maxG, int &minB, int &maxB) {
  int frecuenciaRojo, frecuenciaVerde, frecuenciaAzul;
  // Medir frecuencias
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  frecuenciaRojo = pulseIn(out, LOW);

  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);
  frecuenciaVerde = pulseIn(out, LOW);

  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH);
  frecuenciaAzul = pulseIn(out, LOW);
  
  if(count == 1) {
      // Ajustar rango con ±10 %
    minR = frecuenciaRojo * 0.7;
    maxR = frecuenciaRojo * 1.2;
    minG = frecuenciaVerde * 0.8;
    maxG = frecuenciaVerde * 1.2;
    minB = frecuenciaAzul * 0.8;
    maxB = frecuenciaAzul * 1.2;
  }
  
}

// Función para iniciar la calibración
void iniciarCalibracion() {
    // Calibrar color Rosa
    Serial.println("Coloca el sensor sobre el color Rosa");
    delay(7000);
    calibrarColor(minRosaR, maxRosaR, minRosaG, maxRosaG, minRosaB, maxRosaB);
    delay(3000);

    // Calibrar color Amarillo
    Serial.println("Coloca el sensor sobre el color Amarillo");
    delay(7000);
    calibrarColor(minAmarilloR, maxAmarilloR, minAmarilloG, maxAmarilloG, minAmarilloB, maxAmarilloB);
    delay(3000);
  

    // Calibrar color Morado
    Serial.println("Coloca el sensor sobre el color Morado");
    delay(7000);
    calibrarColor(minMoradoR, maxMoradoR, minMoradoG, maxMoradoG, minMoradoB, maxMoradoB);
    delay(3000);


    // Calibrar color Rojo
    Serial.println("Coloca el sensor sobre el color Rojo");
    delay(7000);
    calibrarColor(minRojoR, maxRojoR, minRojoG, maxRojoG, minRojoB, maxRojoB);
    delay(3000);


    // Calibrar color Negro
    Serial.println("Coloca el sensor sobre el color Negro");
    delay(7000);
    calibrarColor(minNegroR, maxNegroR, minNegroG, maxNegroG, minNegroB, maxNegroB);
    delay(3000);

    // Calibrar color Blanco
    Serial.println("Coloca el sensor sobre el color Blanco");
    delay(7000);
    calibrarColor(minBlancoR, maxBlancoR, minBlancoG, maxBlancoG, minBlancoB, maxBlancoB);
    delay(3000);

    // Calibrar color Verde
    Serial.println("Coloca el sensor sobre el color Verde");
    delay(7000);
    calibrarColor(minVerdeR, maxVerdeR, minVerdeG, maxVerdeG, minVerdeB, maxVerdeB);
    delay(3000);

    // Calibrar color Azul
    Serial.println("Coloca el sensor sobre el color Azul");
    delay(7000);
    calibrarColor(minAzulR, maxAzulR, minAzulG, maxAzulG, minAzulB, maxAzulB);
    delay(3000);



  
}

// Función para detectar el color
int detectarColor() {
  // Imprimir valores de calibración para verificar almacenamiento

  int frecuenciaRojo, frecuenciaVerde, frecuenciaAzul;
  // Medir frecuencias
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  frecuenciaRojo = pulseIn(out, LOW);

  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);
  frecuenciaVerde = pulseIn(out, LOW);

  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH);
  frecuenciaAzul = pulseIn(out, LOW);

  // Comparar con los rangos calibrados para cada color
  if (frecuenciaRojo >= minRosaR && frecuenciaRojo <= maxRosaR &&
      frecuenciaVerde >= minRosaG && frecuenciaVerde <= maxRosaG &&
      frecuenciaAzul >= minRosaB && frecuenciaAzul <= maxRosaB) {
    setColor(255, 102, 255);  // LED Rosa
    delay(500);
    return 1;  // Rosa    
  }

  if (frecuenciaRojo >= minAmarilloR && frecuenciaRojo <= maxAmarilloR &&
      frecuenciaVerde >= minAmarilloG && frecuenciaVerde <= maxAmarilloG &&
      frecuenciaAzul >= minAmarilloB && frecuenciaAzul <= maxAmarilloB) {
    setColor(255, 150, 0);  // LED Amarillo
    delay(500);
    return 2;  // Amarillo
  }

  if (frecuenciaRojo >= minMoradoR && frecuenciaRojo <= maxMoradoR &&
      frecuenciaVerde >= minMoradoG && frecuenciaVerde <= maxMoradoG &&
      frecuenciaAzul >= minMoradoB && frecuenciaAzul <= maxMoradoB) {
    setColor(150, 0, 255);  // LED Morado
    delay(500);
    return 3;  // Morado
  }

  if (frecuenciaRojo >= minRojoR && frecuenciaRojo <= maxRojoR &&
      frecuenciaVerde >= minRojoG && frecuenciaVerde <= maxRojoG &&
      frecuenciaAzul >= minRojoB && frecuenciaAzul <= maxRojoB) {
    setColor(255, 0, 0);  // LED Rojo
    delay(500);
    return 4;  // Rojo
  }

  if (frecuenciaRojo >= minNegroR && frecuenciaRojo <= maxNegroR &&
      frecuenciaVerde >= minNegroG && frecuenciaVerde <= maxNegroG &&
      frecuenciaAzul >= minNegroB && frecuenciaAzul <= maxNegroB) {
    setColor(0, 0, 0);  // LED Negro (apagado)
    delay(500);
    return 5;  // Negro
  }

  if (frecuenciaRojo >= minBlancoR && frecuenciaRojo <= maxBlancoR &&
      frecuenciaVerde >= minBlancoG && frecuenciaVerde <= maxBlancoG &&
      frecuenciaAzul >= minBlancoB && frecuenciaAzul <= maxBlancoB) {
    setColor(255, 255, 255);  // LED Blanco
    delay(500);
    return 6;  // Blanco
  }

  // Agregar detección para el color Verde
  if (frecuenciaRojo >= minVerdeR && frecuenciaRojo <= maxVerdeR &&
      frecuenciaVerde >= minVerdeG && frecuenciaVerde <= maxVerdeG &&
      frecuenciaAzul >= minVerdeB && frecuenciaAzul <= maxVerdeB) {
    setColor(0, 255, 0);  // LED Verde
    delay(500);
    return 7;  // Verde
  }

  // Agregar detección para el color Azul
  if (frecuenciaRojo >= minAzulR && frecuenciaRojo <= maxAzulR &&
      frecuenciaVerde >= minAzulG && frecuenciaVerde <= maxAzulG &&
      frecuenciaAzul >= minAzulB && frecuenciaAzul <= maxAzulB) {
    setColor(0, 0, 255);  // LED Azul
    delay(500);
    return 8;  // Azul
  }

  return -1; // No identificado
}

void encenderLED(int red, int green, int blue) {
  analogWrite(pinRed, 255 - red);   // Invertimos el valor para ánodo común
  analogWrite(pinGreen, 255 - green);
  analogWrite(pinBlue, 255 - blue);
}

void apagarLED() {
  digitalWrite(pinRed, LOW);
  digitalWrite(pinGreen, LOW);
  digitalWrite(pinBlue, LOW);
}

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
  motorIzq.cambiarVelocidad(abs(left));
  motorDer.cambiarVelocidad(abs(right));

  // Ambos motores deben girar en sentido horario para avanzar
  left <= 0 ? motorIzq.giroHorario() : motorIzq.giroAntihorario();
  right >= 0 ? motorDer.giroAntihorario() : motorDer.giroHorario();
}

void rampa(Motor& motor1, Motor& motor2, float velocidadInicial) {
  float velocidadActual = velocidadInicial;
    bool subiendo = false;
    bool desacelerando = false;
    bool enEquilibrio = false;

    // Calibración inicial para obtener el valor estable de yaw
    float yawEstable = 0.0;
    const int numLecturas = 50; // Número de lecturas para obtener un promedio estable
    for (int i = 0; i < numLecturas; i++) {
        yawEstable += readGyroData();
        delay(10);
    }
    yawEstable /= numLecturas; // Promedio de las lecturas iniciales
    Serial.print("Yaw Estable: ");
    Serial.println(yawEstable);

    // Sensibilidad para detección de cambios
    float umbralSubida = yawEstable + 0.05;   // Ajusta según pruebas, valor relativo para detectar inicio de subida
    float umbralNeutral = yawEstable + 0.01;  // Ajuste para valores neutros
    float umbralDesaceleracion = yawEstable - 0.05; // Para detectar desaceleración

    // Iniciar en el checkpoint, motores apagados
    motorsOFF();
    
    while (!enEquilibrio) {
        // Leer datos del giroscopio
        float yaw = readGyroData();
        Serial.print("Yaw: ");
        Serial.println(yaw);
        
        // Paso 2: Detecta el inicio de la subida con valores mayores al umbral de subida
        if (!subiendo && yaw > umbralSubida) {
            subiendo = true;
            velocidadActual = velocidadInicial;
            Serial.print(yaw);
            Serial.println(" Subiendo");
            motor1.cambiarVelocidad(velocidadActual);
            motor2.cambiarVelocidad(velocidadActual);
            // motor1.giroHorario();
            // motor2.giroAntihorario();
        }
        
        // Paso 3: Sigue avanzando mientras los valores sean neutrales respecto a yawEstable
        else if (subiendo && fabs(yaw - yawEstable) <= umbralNeutral) {
            Serial.print(yaw);
            Serial.println(" Avanzando");
            motor1.cambiarVelocidad(velocidadActual);
            motor2.cambiarVelocidad(velocidadActual);
        }

        // Paso 4: Detecta desaceleración en cuanto los valores caen bajo el umbral de desaceleración
        else if (yaw <= umbralDesaceleracion && !desacelerando) {
            subiendo = false;
            desacelerando = true;
            velocidadActual *= 0.5; // Disminuye la velocidad a la mitad
            Serial.print(yaw);
            Serial.println(" Desacelerando");
            motor1.cambiarVelocidad(velocidadActual);
            motor2.cambiarVelocidad(velocidadActual);
        }

        // Paso 5: Se apaga cuando detecta un cambio positivo cercano al valor estable
        else if (desacelerando && yaw >= yawEstable) {
            motorsOFF();
            Serial.print(yaw);
            Serial.println(" Equilibrio");
            enEquilibrio = true; // Salimos del bucle
            delay(4000); // Espera para estabilizar
        }

        delay(20); // Pequeña pausa para dar estabilidad en la lectura
    }
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

// Función para avanzar y evitar las paredes de forma continua
void antiwallContinuousMovement(Motor& motor1, Motor& motor2, int v) {
    const int minDist = 8; // Distancia mínima en cm para corrección

    // Lógica de movimiento constante
    while (true) {
        // Leer las distancias a los obstáculos
        int distFront = getDistance(trigPin2, echoPin2);
        int distLeft = getDistance(trigPin3, echoPin3);
        int distRight = getDistance(trigPin1, echoPin1);

        Serial.print("Frente: "); Serial.println(distFront);
        Serial.print("Izquierda: "); Serial.println(distLeft);
        Serial.print("Derecha: "); Serial.println(distRight);

        // Corrección si hay obstáculos a los lados
        if (distLeft < minDist) {
            // Obstáculo a la izquierda, realizar un giro suave a la derecha
            Serial.println("Corrección Pared IZQUIERDA");
            motor1.cambiarVelocidad(v); // Ajusta la velocidad según tus necesidades
            motor2.cambiarVelocidad(v * 0.8); // Acelera motor2 para girar a la derecha
            motor1.giroHorario();
            motor2.giroAntihorario();
        } 
        if (distRight < minDist) {
            // Obstáculo a la derecha, realizar un giro suave a la izquierda
            Serial.println("Corrección Pared DERECHA");
            motor1.cambiarVelocidad(v * 0.8); // Acelera motor1 para girar a la izquierda
            motor2.cambiarVelocidad(v); // Ajusta la velocidad según tus necesidades
            motor1.giroHorario();
            motor2.giroAntihorario();
        } 
        if (distFront >= minDist) {
            // Avanzar en línea recta si no hay obstáculo en el frente
            Serial.println("Avanzar Recto");
            motor1.cambiarVelocidad(v); // Mantiene velocidad para avanzar recto
            motor2.cambiarVelocidad(v); // Mantiene velocidad para avanzar recto
            motor1.giroHorario();
            motor2.giroAntihorario();
        } else {
            // Si hay un obstáculo al frente, decidir la dirección en función del espacio a los lados
            if (distLeft > distRight) {
                // Hay más espacio a la izquierda: girar a la izquierda
                Serial.println("Girar a la Izquierda");
                avanzarMotoresSincronizados(motor1, motor2, 2.5, 2.5, true, true); // Giro de 180° a la izquierda
            } else {
                // Hay más espacio a la derecha: girar a la derecha
                Serial.println("Girar a la Derecha");
                avanzarMotoresSincronizados(motor1, motor2, 2.5, 2.5, false, false); // Giro de 180° a la derecha
            }
        }

        delay(20); // Pequeña pausa para dar estabilidad en la lectura
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
float readGyroData() {
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
  GyroY = GyroY - 2; // GyroErrorY ~(2)
  GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  gyroAngleX = 0.98 * gyroAngleX + 0.02 * accAngleX;
  gyroAngleY = 0.98 * gyroAngleY + 0.02 * accAngleY;

  roll = gyroAngleX;
  pitch = gyroAngleY;

  // ----------------
  yaw = calculateMovingAverage(yaw, yawWindow, windowIndex, yawSum);
  pitch = calculateMovingAverage(pitch, pitchWindow, windowIndex, pitchSum);
  // ----------------
  
  // Print the values on the serial monitor
  // Serial.print(roll); // Funciona para direcciones
  // Serial.print("/");
  // Serial.println(yaw); // Funciona para la rampa
  return yaw;
  delay(20);
}

// Función para calcular el promedio móvil
float calculateMovingAverage(float newValue, float* window, int& index, float& sum) {
    // Restamos el valor más antiguo
    sum -= window[index];
    // Agregamos el nuevo valor al índice actual
    window[index] = newValue;
    // Sumamos el nuevo valor
    sum += newValue;
    // Avanza el índice de la ventana circular
    index = (index + 1) % windowSize;
    // Calcula el promedio actual
    return sum / windowSize;
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
