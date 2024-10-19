const int ENC_A = 6;
const int ENC_B = 5;
char op = '0';
char vel[] = {' ',' ',' '};
const int IN1 = 2;
const int IN2 = 3;
const int ENA = 4;
int v = 300;

void setup() {
  Serial.begin(9600);
  pinMode(ENC_A,INPUT);
  pinMode(ENC_B,INPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(ENA,OUTPUT);
  MENU();
}

void loop() {
  encoder();
  delay(3000);
}

void serialEvent() {
  delay(20);
  op = Serial.read();
  while(Serial.available() > 0) {Serial.read();}
  switch (op) {
    case '1':
      digitalWrite(IN1,HIGH);
      digitalWrite(IN2,LOW);
      analogWrite(ENA,v);
      Serial.println();
      Serial.print(F("ESTADO: "));
      Serial.println(F("----Giro Horario----"));
    break;
    case '2':
      analogWrite(ENA,0);
      Serial.println();
      Serial.print(F("ESTADO: "));
      Serial.print(F("----Apagado----"));
      encoder();
    break;
    case '3':
      digitalWrite(IN1,LOW);
      digitalWrite(IN2,HIGH);
      analogWrite(ENA,v);
      Serial.println();
      Serial.print(F("ESTADO: "));
      Serial.println(F("----Giro Antihorario----"));
    break;
    case '4':
      v = 0;
      Serial.println();
      Serial.println(F("       CAMBIO DE VELOCIDAD"));
      Serial.println(F("Ingrese la velocidad en rad/s:"));
      while(Serial.available()==0) {;}
      Serial.readBytesUntil('\n',vel,3);
      delay(100);
      while(Serial.available()>0) {Serial.read();}
      v = atoi(vel);
      Serial.print(F("Se cambio la velocidad a: "));
      Serial.println(v);
    
    break;
  }
  MENU();
}

void encoder() {
  int a = digitalRead(ENC_A);
  int b = digitalRead(ENC_B);
  Serial.print(a*5);
  Serial.print("");
  Serial.println(b*5);
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
