class Motor {
  private:
    int ENC_A, ENC_B;    // Pines del encoder
    int IN1, IN2, ENA;   // Pines del motor
    int velocidad;       // Velocidad del motor

  public:
    // Constructor
    Motor(int encA, int encB, int in1, int in2, int ena, int v) {
      ENC_A = encA;
      ENC_B = encB;
      IN1 = in1;
      IN2 = in2;
      ENA = ena;
      velocidad = v;
      
      pinMode(ENC_A, INPUT);
      pinMode(ENC_B, INPUT);
      pinMode(IN1, OUTPUT);
      pinMode(IN2, OUTPUT);
      pinMode(ENA, OUTPUT);
    }

    // Giro horario
    void giroHorario() {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, velocidad);
      Serial.println(F("----Giro Horario----"));
    }

    // Giro antihorario
    void giroAntihorario() {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, velocidad);
      Serial.println(F("----Giro Antihorario----"));
    }

    // Apagar motor
    void apagar() {
      analogWrite(ENA, 0);
      Serial.println(F("----Motor Apagado----"));
    }

    // Cambiar velocidad
    void cambiarVelocidad(int v) {
      velocidad = v;
      Serial.print(F("Se cambió la velocidad a: "));
      Serial.println(velocidad);
    }

    // Leer encoder
    void leerEncoder() {
      int a = digitalRead(ENC_A);
      int b = digitalRead(ENC_B);
      Serial.print(a * 5);  // Multiplicar por 5 solo como ejemplo
      Serial.print("\t");
      Serial.println(b * 5);
    }
};