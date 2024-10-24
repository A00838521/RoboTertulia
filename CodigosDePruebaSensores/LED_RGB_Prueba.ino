int rojo = 3;
int azul = 5;
int verde = 6;

void setup()
{
  pinMode(rojo, OUTPUT);
  pinMode(azul, OUTPUT);
  pinMode(verde, OUTPUT);
  
  for (int x=0; x <= 13; x++) {
    digitalWrite(x, LOW); // Están apagados del 0 al 13
  }
}

void loop()
{
  digitalWrite(rojo, LOW);
  delay(500); // Espera 500ms
  digitalWrite(rojo, HIGH);
  delay(500); // Espera 500ms
  digitalWrite(rojo, LOW);
  delay(500); // Espera 500ms

  digitalWrite(verde, LOW);
  delay(500); // Espera 500ms
  digitalWrite(verde, HIGH);
  delay(500); // Espera 500ms
  digitalWrite(verde, LOW);
  delay(500); // Espera 500ms

  digitalWrite(azul, LOW);
  delay(500); // Espera 500ms
  digitalWrite(azul, HIGH);
  delay(500); // Espera 500ms
  digitalWrite(azul, LOW);
  delay(500); // Espera 500ms
}
