class Motor {
  private:
    int ENC_A, ENC_B;    // Pines del encoder
    int IN1, IN2, ENA;   // Pines del motor
    int velocidad;       // Velocidad actual del motor
    int velocidadMax;    // Velocidad máxima permitida
    int velocidadMin;    // Velocidad mínima permitida

  public:
    // Constructor
    Motor(int encA, int encB, int in1, int in2, int ena, int vMin, int vMax) {
      ENC_A = encA;
      ENC_B = encB;
      IN1 = in1;
      IN2 = in2;
      ENA = ena;
      velocidadMin = vMin;
      velocidadMax = vMax;
      velocidad = vMin; // Iniciar con la velocidad mínima
      
      pinMode(ENC_A, INPUT);
      pinMode(ENC_B, INPUT);
      pinMode(IN1, OUTPUT);
      pinMode(IN2, OUTPUT);
      pinMode(ENA, OUTPUT);
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

    // Leer encoder (simulación para prueba)
    void leerEncoder() {
      int a = digitalRead(ENC_A);
      int b = digitalRead(ENC_B);
      Serial.print("Encoder A: ");
      Serial.print(a);
      Serial.print(" | Encoder B: ");
      Serial.println(b);
    }
};