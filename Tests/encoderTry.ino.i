// This test checks if the encoderCorrection function works correctly
// by checking if the position is incremented or decremented based on the
// state of the encoder channel B.

// Right Motor
int RencoderChannelA = 5;
int RencoderChannelB = 6; // Channel B equals to direcction
int RmotorPWM = 10;

// Left Motor
int LencoderChannelA = 2;
int LencoderChannelB = 3;
int LmotorPWM = 9;

// Positions
int Rposition = 0;
int RlastPosition = 0;

int Lposition = 0;
int LlastPosition = 0;

int counter = LOW;

// Distance Sensor HC-SR04
int trigPin = 7;
int echoPin = 8;

float duration, distance;

void setup() {
    Serial.begin(9600);
    pinMode(RencoderChannelA, INPUT);
    pinMode(RencoderChannelB, INPUT);
    pinMode(RmotorPWM, OUTPUT);

    pinMode(LencoderChannelA, INPUT);
    pinMode(LencoderChannelB, INPUT);
    pinMode(LmotorPWM, OUTPUT);

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

void loop() {
    right();
    delay(1000); // Wait for a second
}



void advance() {
    distance();
    while(distance > 10) {
        LencoderCorrection();
        RencoderCorrection();
        digitalWrite(LmotorPWM, HIGH);
        digitalWrite(RmotorPWM, HIGH);
    }
    digitalWrite(LmotorPWM, LOW);
    digitalWrite(RmotorPWM, LOW);
}

void distance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;
    Serial.println(distance);
}

void LencoderCorrection() {
    encoderCorrection(LencoderChannelA, LencoderChannelB, Lposition, LlastPosition);
}

void RencoderCorrection() {
    encoderCorrection(RencoderChannelA, RencoderChannelB, Rposition, RlastPosition);
}

void right() {
    while(position < 50) {
        RencoderCorrection();
        digitalWrite(LmotorPWM, HIGH);
    }
    digitalWrite(LmotorPWM, LOW);
}

void left() {
    while(position < 50) {
        RencoderCorrection();
        digitalWrite(RmotorPWM, HIGH);
    }
    digitalWrite(RmotorPWM, LOW);
}

void encoderCorrection(int encoderChannelA, int encoderChannelB, int &position, int &lastPosition) {
    int counter = digitalRead(encoderChannelA); // Read the current state of encoder channel A
    if ((lastPoition == LOW) && (counter == HIGH)) { // Check if there is a rising edge on channel A
        if (digitalRead(encoderChannelB) == LOW) { // Check the state of channel B to determine direction
            position++; // Increment position if channel B is LOW
        } else {
            position--; // Decrement position if channel B is HIGH
        }
        Serial.println(position); // Print the current position
    }
    lastPosition = counter; // Update the last position
}