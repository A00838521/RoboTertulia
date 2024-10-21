// Definición de pines para el encoder y el motor
const int ENC_A = 6;
const int ENC_B = 5;
char op = '0';
char vel[] = {' ',' ',' '};
const int IN1 = 2;
const int IN2 = 3;
const int ENA = 4;
int v = 300;

void setup() {
  // Inicialización de la comunicación serial y configuración de pines
  Serial.begin(9600);
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  MENU(); // Mostrar el menú inicial
}

void loop() {
  encoder(); // Leer valores del encoder
  delay(3000); // Esperar 3 segundos
}

void serialEvent() {
  delay(20); // Esperar un poco para asegurar que los datos lleguen
  op = Serial.read(); // Leer la opción ingresada por el usuario
  while (Serial.available() > 0) { Serial.read(); } // Limpiar el buffer serial

  switch (op) {
    case '1':
      // Giro horario del motor
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, v);
      Serial.println();
      Serial.print(F("ESTADO: "));
      Serial.println(F("----Giro Horario----"));
      break;
    case '2':
      // Apagar el motor
      analogWrite(ENA, 0);
      Serial.println();
      Serial.print(F("ESTADO: "));
      Serial.print(F("----Apagado----"));
      encoder(); // Leer valores del encoder
      break;
    case '3':
      // Giro antihorario del motor
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, v);
      Serial.println();
      Serial.print(F("ESTADO: "));
      Serial.println(F("----Giro Antihorario----"));
      break;
    case '4':
      // Cambiar la velocidad del motor
      v = 0;
      Serial.println();
      Serial.println(F("       CAMBIO DE VELOCIDAD"));
      Serial.println(F("Ingrese la velocidad en rad/s:"));
      while (Serial.available() == 0) { ; } // Esperar a que el usuario ingrese la velocidad
      Serial.readBytesUntil('\n', vel, 3); // Leer la velocidad ingresada
      delay(100);
      while (Serial.available() > 0) { Serial.read(); } // Limpiar el buffer serial
      v = atoi(vel); // Convertir la velocidad a entero
      Serial.print(F("Se cambio la velocidad a: "));
      Serial.println(v);
      break;
  }
  MENU(); // Mostrar el menú nuevamente
}

void encoder() {
  // Leer y mostrar los valores del encoder
  int a = digitalRead(ENC_A);
  int b = digitalRead(ENC_B);
  Serial.print(a * 5);
  Serial.print("");
  Serial.println(b * 5);
}

void MENU() {
  // Mostrar el menú de opciones
  Serial.println();
  Serial.println(F("         MENU"));
  Serial.println(F("Presione una opcion 1-4"));
  Serial.println(F("1. Giro izquierda"));
  Serial.println(F("2. Apagar"));
  Serial.println(F("3. Giro derecha"));
  Serial.println(F("4. Cambiar velocidad"));
}
