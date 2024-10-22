#define FILAS 5
#define COLUMNAS 3

// Definimos el laberinto (0 = camino, 1 = pared)
int laberinto[FILAS][COLUMNAS] = {
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0}
};

// Para evitar bucles infinitos, llevamos un registro de las celdas visitadas
bool visitado[FILAS][COLUMNAS] = {false}; // Registro de unidades visitadas

// Definimos las posibles direcciones: arriba, abajo, izquierda, derecha
int dirFila[] = {-1, 1, 0, 0}; // Posibles movimientos entre unidades
int dirColumna[] = {0, 0, -1, 1};

// Función para verificar si una unidad es válida para moverse
bool esValido(int fila, int columna) {
  return (fila >= 0 && fila < FILAS && columna >= 0 && columna < COLUMNAS &&
          laberinto[fila][columna] == 0 && !visitado[fila][columna]);
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
    return true;
  }

  // Marcamos la celda actual como visitada
  visitado[fila][columna] = true;

  // Intentamos movernos en todas las direcciones posibles (arriba, abajo, izquierda, derecha)
  for (int i = 0; i < 4; i++) {
    int nuevaFila = fila + dirFila[i];
    int nuevaColumna = columna + dirColumna[i];

    // Verificamos si podemos movernos a la nueva celda
    if (esValido(nuevaFila, nuevaColumna)) {
      Serial.print("Moviéndose a (");
      Serial.print(nuevaFila);
      Serial.print(", ");
      Serial.print(nuevaColumna);
      Serial.println(")");

      // Llamada recursiva para seguir explorando el laberinto
      if (resolverLaberinto(nuevaFila, nuevaColumna)) {
        return true;
      }
    }
  }

  // Si no hay más movimientos posibles, retrocedemos
  Serial.print("Retrocediendo desde (");
  Serial.print(fila);
  Serial.print(", ");
  Serial.print(columna);
  Serial.println(")");
  return false;
}

void setup() {
  // Iniciar la comunicación serial
  Serial.begin(9600);

  // Iniciar la resolución desde la esquina superior izquierda (0, 0)
  if (!resolverLaberinto(0, 0)) {
    Serial.println("No se encontró solución al laberinto.");
  }
}

void loop() {
  // No hacemos nada en loop ya que toda la lógica está en la función resolverLaberinto
}