//CODIGO PARA SENSOR DE COLORES


const int s0 = 4;
const int s1 = 5;
const int s2 = 6;
const int s3 = 7;
const int out = 8;
const int ledPin = 9; // Ajusta según el pin que uses para los LEDs


void setup()   
{  
  Serial.begin(9600);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(out, INPUT);
  pinMode(ledPin, OUTPUT);

  digitalWrite(ledPin, HIGH); // Enciende los LEDs
  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);




}  
    
void loop() 
{ 
int R = getRojo();
delay(200);
int V = getVerde();
delay(200);
int A = getAzul();
delay(200);


Serial.print("Int R "+ String(R));
Serial.print("  --  Int V "+ String(V));
Serial.println("  --  Int Z "+ String(A));


//Verde: R50 V48 A49
//Rojo: R14 V34 A30
//Azul: R62 V47 A31

  if ( R > V && V < A && R > 30 && R < 55 ) {
    Serial.print("  EL COLOR ES VERDE");
  }
  else if (R < V  && V > A && R >= 10 && V >= 30) {
    Serial.print("  EL COLOR ES ROJO");
  }
  else if (R > V && R > A && R > 10 && R <= 70  ) {
    Serial.print("  EL COLOR ES AZUL");
  }
  else
  {
    Serial.print("  EL COLOR NO HA SIDO REGISTRADO");
  }
  Serial.println(" ");


}  
    
int getRojo(){
  //leer color rojo
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  int ROJO = pulseIn(out, LOW);
  return ROJO;
}


int getAzul(){
  //leer color azul
  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH);
  int AZUL = pulseIn(out, LOW);
  return AZUL;
}


int getVerde(){
  //leer color verde
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);
  int VERDE = pulseIn(out, LOW);
  return VERDE;
}
