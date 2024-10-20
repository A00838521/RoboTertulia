const int ENC_A_LEFT = 6;
const int ENC_B_LEFT = 5;
const int ENC_A_RIGHT = 10; // Encoder A Phase motor derecho
const int ENC_B_RIGHT = 11; // Encoder B Phase motor derecho

const int IN1 = 2;
const int IN2 = 3;
const int ENA = 4;

const int IN3 = 9; // Motor derecho
const int IN4 = 12; // Motor derecho
const int ENB = 8;  // Motor derecho

char op = '0';
char vel[] = {' ',' ',' '};
int v = 300;

void setup() {
  Serial.begin(9600);

  // Configuración de pines para motor izquierdo
  pinMode(ENC_A_LEFT, INPUT);
  pinMode(ENC_B_LEFT, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Configuración de pines para motor derecho
  pinMode(ENC_A_RIGHT, INPUT);
  pinMode(ENC_B_RIGHT, INPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  MENU();
}

void loop() {
  encoder();
  delay(3000);
}

void serialEvent() {
  delay(20);
  op = Serial.read();
  while (Serial.available() > 0) {Serial.read();}

  switch (op) {
    case '1':
      // Giro Horario para ambos motores
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, v);

      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, v);

      Serial.println();
      Serial.print(F("ESTADO: "));
      Serial.println(F("----Giro Horario----"));
    break;

    case '2':
      // Apagar ambos motores
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);

      Serial.println();
      Serial.print(F("ESTADO: "));
      Serial.print(F("----Apagado----"));
      encoder();
    break;

    case '3':
      // Giro Antihorario para ambos motores
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, v);

      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, v);

      Serial.println();
      Serial.print(F("ESTADO: "));
      Serial.println(F("----Giro Antihorario----"));
    break;

    case '4':
      // Cambiar velocidad para ambos motores
      v = 0;
      Serial.println();
      Serial.println(F("       CAMBIO DE VELOCIDAD"));
      Serial.println(F("Ingrese la velocidad en rad/s:"));
      while (Serial.available() == 0) {;}
      Serial.readBytesUntil('\n', vel, 3);
      delay(100);
      while (Serial.available() > 0) {Serial.read();}

      v = atoi(vel);
      Serial.print(F("Se cambió la velocidad a: "));
      Serial.println(v);
    break;
  }

  MENU();
}

void encoder() {
  // Lectura del encoder del motor izquierdo
  int a_left = digitalRead(ENC_A_LEFT);
  int b_left = digitalRead(ENC_B_LEFT);
  Serial.print("Izquierdo: ");
  Serial.print(a_left * 5);
  Serial.print(" ");
  Serial.println(b_left * 5);

  // Lectura del encoder del motor derecho
  int a_right = digitalRead(ENC_A_RIGHT);
  int b_right = digitalRead(ENC_B_RIGHT);
  Serial.print("Derecho: ");
  Serial.print(a_right * 5);
  Serial.print(" ");
  Serial.println(b_right * 5);
}

void MENU() {
  Serial.println();
  Serial.println(F("         MENU"));
  Serial.println(F("Presione una opcion 1-4"));
  Serial.println(F("1. Giro izquierda"));
  Serial.println(F("2. Apagar"));
  Serial.println(F("3. Giro derecha"));
  Serial.println(F("4. Cambiar velocidad"));
}

