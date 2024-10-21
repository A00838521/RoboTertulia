
// Definición de dos motores (ejemplo)
Motor motorIzquierdo(6, 5, 2, 3, 4, 150);  // Pines y velocidad inicial
Motor motorDerecho(7, 8, 9, 10, 11, 150);  // Pines y velocidad inicial

char op = '0';
char vel[] = {' ', ' ', ' '};
int nuevaVelocidad = 150;

void setup() {
  Serial.begin(9600);
  MENU(); // Mostrar el menú inicial
}

void loop() {
  if (Serial.available() > 0) {
    serialEvent();
  }
  delay(3000); // Delay para ver los resultados
}

// Manejar eventos de entrada serial
void serialEvent() {
  delay(20); // Esperar un poco para asegurar que los datos lleguen
  op = Serial.read(); // Leer la opción ingresada por el usuario
  while (Serial.available() > 0) { Serial.read(); } // Limpiar el buffer serial

  switch (op) {
    case '1':
      motorIzquierdo.giroHorario(); // Motor izquierdo giro horario
      break;
    case '2':
      motorIzquierdo.apagar(); // Apagar motor izquierdo
      motorIzquierdo.leerEncoder(); // Leer encoder motor izquierdo
      break;
    case '3':
      motorIzquierdo.giroAntihorario(); // Motor izquierdo giro antihorario
      break;
    case '4':
      // Cambiar la velocidad del motor izquierdo
      Serial.println(F("Ingrese la velocidad en rad/s:"));
      while (Serial.available() == 0) { ; } // Esperar a que el usuario ingrese la velocidad
      Serial.readBytesUntil('\n', vel, 3); // Leer la velocidad ingresada
      delay(100);
      while (Serial.available() > 0) { Serial.read(); } // Limpiar el buffer serial
      nuevaVelocidad = atoi(vel); // Convertir la velocidad a entero
      motorIzquierdo.cambiarVelocidad(nuevaVelocidad);
      break;
    case '5':
      motorDerecho.giroHorario(); // Motor derecho giro horario
      break;
    case '6':
      motorDerecho.apagar(); // Apagar motor derecho
      motorDerecho.leerEncoder(); // Leer encoder motor derecho
      break;
    case '7':
      motorDerecho.giroAntihorario(); // Motor derecho giro antihorario
      break;
    case '8':
      // Cambiar la velocidad del motor derecho
      Serial.println(F("Ingrese la velocidad en rad/s:"));
      while (Serial.available() == 0) { ; } // Esperar a que el usuario ingrese la velocidad
      Serial.readBytesUntil('\n', vel, 3); // Leer la velocidad ingresada
      delay(100);
      while (Serial.available() > 0) { Serial.read(); } // Limpiar el buffer serial
      nuevaVelocidad = atoi(vel); // Convertir la velocidad a entero
      motorDerecho.cambiarVelocidad(nuevaVelocidad);
      break;
  }
  MENU(); // Mostrar el menú nuevamente
}

// Mostrar el menú de opciones
void MENU() {
  Serial.println();
  Serial.println(F("         MENU"));
  Serial.println(F("Presione una opcion 1-8"));
  Serial.println(F("1. Motor Izquierdo - Giro horario"));
  Serial.println(F("2. Motor Izquierdo - Apagar"));
  Serial.println(F("3. Motor Izquierdo - Giro antihorario"));
  Serial.println(F("4. Motor Izquierdo - Cambiar velocidad"));
  Serial.println(F("5. Motor Derecho - Giro horario"));
  Serial.println(F("6. Motor Derecho - Apagar"));
  Serial.println(F("7. Motor Derecho - Giro antihorario"));
  Serial.println(F("8. Motor Derecho - Cambiar velocidad"));
}
