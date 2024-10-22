// Definimos el laberinto (0 = camino, 1 = pared)
#define FILAS 5
#define COLUMNAS 3
#define DISTANCIA_MINIMA 20  // Distancia mínima en cm para evitar una colisión
#define TRIG_F 9   // Pin trig del sensor ultrasónico frontal
#define ECHO_F 10  // Pin echo del sensor ultrasónico frontal
#define TRIG_L 11  // Pin trig del sensor ultrasónico izquierdo
#define ECHO_L 12  // Pin echo del sensor ultrasónico izquierdo
#define TRIG_R 13  // Pin trig del sensor ultrasónico derecho
#define ECHO_R 14  // Pin echo del sensor ultrasónico derecho

// Pines del sensor de color
const int s0 = 4;
const int s1 = 5;
const int s2 = 6;
const int s3 = 7;
const int out = 8;
const int ledPin = 9;  // LED RGB

// Crear dos objetos Motor para las llantas izquierda y derecha
Motor motorIzquierdo(2, 3, 4, 5, 6, 0, 255);  // Pines: ENC_A, ENC_B, IN1, IN2, ENA
Motor motorDerecho(7, 8, 9, 10, 11, 0, 255);  // Pines: ENC_A, ENC_B, IN1, IN2, ENA

// Definimos el laberinto
int laberinto[FILAS][COLUMNAS] = {
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
  {0, 1, 0}
};

// Para evitar bucles infinitos, llevamos un registro de las celdas visitadas
bool visitado[FILAS][COLUMNAS] = {false}; // Registro de unidades visitadas

// Almacenar los colores detectados en cada unidad del laberinto
int colores[FILAS][COLUMNAS] = {0};  // 0 = sin color, 1 = rojo, 2 = verde, 3 = azul

// Definimos las posibles direcciones: arriba, abajo, izquierda, derecha
int dirFila[] = {-1, 1, 0, 0}; // Posibles movimientos entre unidades
int dirColumna[] = {0, 0, -1, 1};

// Función para verificar si una unidad es válida para moverse
bool esValido(int fila, int columna) {
  return (fila >= 0 && fila < FILAS && columna >= 0 && columna < COLUMNAS &&
          laberinto[fila][columna] == 0 && !visitado[fila][columna]);
}

// Función para medir la distancia con el sensor ultrasónico
long medirDistancia(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duracion = pulseIn(echoPin, HIGH);
  long distancia = duracion * 0.034 / 2;  // Convertir a distancia en cm
  return distancia;
}

// Función para evitar colisiones con sensores laterales
bool evitarColision() {
  long distanciaIzq = medirDistancia(TRIG_L, ECHO_L);
  long distanciaDer = medirDistancia(TRIG_R, ECHO_R);
  return (distanciaIzq > DISTANCIA_MINIMA && distanciaDer > DISTANCIA_MINIMA);
}

// Función para avanzar el robot (motores)
void avanzar() {
  if (evitarColision()) {
    motorIzquierdo.giroHorario();  // Avanzar llanta izquierda
    motorDerecho.giroHorario();    // Avanzar llanta derecha
  } else {
    detenerMotores();
  }
}

// Función para detener los motores
void detenerMotores() {
  motorIzquierdo.apagar();
  motorDerecho.apagar();
}

// Función para leer color con el sensor
int detectarColor() {
  int R = getRojo();
  int V = getVerde();
  int A = getAzul();
  delay(200);

  // Determinar el color detectado (rojo, verde, azul)
  if (R > V && V < A && R > 30 && R < 55) {
    // Verde
    mostrarColorRGB(0, 255, 0);  // Encender LED verde
    return 2;  // Verde
  }
  else if (R < V && V > A && R >= 10 && V >= 30) {
    // Rojo
    mostrarColorRGB(255, 0, 0);  // Encender LED rojo
    return 1;  // Rojo
  }
  else if (R > V && R > A && R > 10 && R <= 70) {
    // Azul
    mostrarColorRGB(0, 0, 255);  // Encender LED azul
    return 3;  // Azul
  }
  else {
    // Sin color registrado
    mostrarColorRGB(255, 255, 255);  // Encender LED blanco
    return 0;  // Sin color
  }
}

// Función para mostrar un color en el LED RGB
void mostrarColorRGB(int rojo, int verde, int azul) {
  analogWrite(ledPin, rojo);
  analogWrite(ledPin, verde);
  analogWrite(ledPin, azul);
}

// Función para analizar cuál fue el color más repetido
int colorMasRepetido() {
  int cuentaRojo = 0, cuentaVerde = 0, cuentaAzul = 0;

  for (int i = 0; i < FILAS; i++) {
    for (int j = 0; j < COLUMNAS; j++) {
      if (colores[i][j] == 1) cuentaRojo++;
      else if (colores[i][j] == 2) cuentaVerde++;
      else if (colores[i][j] == 3) cuentaAzul++;
    }
  }

  if (cuentaRojo >= cuentaVerde && cuentaRojo >= cuentaAzul) {
    return 1;  // Rojo
  } else if (cuentaVerde >= cuentaRojo && cuentaVerde >= cuentaAzul) {
    return 2;  // Verde
  } else {
    return 3;  // Azul
  }
}

// Función DFS recursiva para explorar el laberinto
bool resolverLaberinto(int fila, int columna) {
  // Si llegamos al final (la esquina inferior derecha)
  if (fila == FILAS - 1 && columna == COLUMNAS - 1) {
    Serial.print("Se ha llegado a la salida en (");
    Serial.print(fila);
    Serial.print(", ");
    Serial.print(columna);
    Serial.println(")");

    // Determinar el color más repetido
    int color = colorMasRepetido();
    if (color == 1) {
      mostrarColorRGB(255, 0, 0);  // Rojo
    } else if (color == 2) {
      mostrarColorRGB(0, 255, 0);  // Verde
    } else {
      mostrarColorRGB(0, 0, 255);  // Azul
    }

    detenerMotores();
    return true;
  }

  // Marcamos la celda actual como visitada y detectamos el color
  visitado[fila][columna] = true;
  colores[fila][columna] = detectarColor();  // Guardar el color detectado

  // Intentamos movernos en todas las direcciones posibles (arriba, abajo, izquierda, derecha)
  for (int i = 0; i < 4; i++) {
    int nuevaFila = fila + dirFila[i];
    int nuevaColumna = columna + dirColumna[i];

    // Verificamos si podemos movernos a la nueva celda y si el frente está libre
    if (esValido(nuevaFila, nuevaColumna) && medirDistancia(TRIG_F, ECHO_F) > DISTANCIA_MINIMA) {
      avanzar();
      delay(1000);  // Simulación del avance en la nueva unidad (30 cm)

      // Llamamos recursivamente a la función para intentar resolver desde la nueva celda
      if (resolverLaberinto(nuevaFila, nuevaColumna)) {
        return true;  // Si encontramos la salida, terminamos la búsqueda
      }

      // Si no encontramos la salida, retrocedemos
      detenerMotores();
    }
  }

  // Si no podemos avanzar en ninguna dirección, retrocedemos
  visitado[fila][columna] = false;
  return false;
}

// Función de configuración
void setup() {
  Serial.begin(9600);

  // Inicializar pines del sensor ultrasónico
  pinMode(TRIG_F, OUTPUT);
  pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);

  // Inicializar pines del sensor de color
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(out, INPUT);
  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);

  // Comenzamos a resolver el laberinto desde la posición inicial (0, 0)
  resolverLaberinto(0, 0);
}

// Función principal del loop (vacío porque la resolución es recursiva)
void loop() {
  // El laberinto se resuelve dentro de setup() recursivamente
}