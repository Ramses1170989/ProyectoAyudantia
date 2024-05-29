#define PUSH 12
#define LED 5

#define MOTOR_DERECHO_POSITIVE 16
#define MOTOR_DERECHO_NEGATIVE 17
#define MOTOR_RIGTH_A 34
#define MOTOR_RIGTH_B 35

#define MOTOR_IZQUIERDO_POSITIVE 18
#define MOTOR_IZQUIERDO_NEGATIVE 4
#define MOTOR_LEFT_A 23
#define MOTOR_LEFT_B 19

#define CNY_0 36
#define CNY_1 33  // frente
#define CNY_2 25  // frente-der
#define CNY_3 26
#define CNY_4 27  // frente-izq
#define CNY_5 32  // frente
#define CNY_6 39
#define LED_ON 13

int sensores[] = {CNY_0, CNY_1, CNY_2, CNY_3, CNY_4, CNY_5, CNY_6}; // Corrected array declaration

// PID constants
const float Kp = 1.0;
const float Ki = 0.1;
const float Kd = 0.01;

// PID variables
float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;
float controlSignal = 0;
unsigned long lastMillis = 0;
const unsigned long deltaMillis = 10;
const int wallThreshold = 2000; // Ajusta este valor según tu sensor y entorno
void setup() {
  for (int i = 0; i < sizeof(sensores)/sizeof(sensores[0]); i++) {
    pinMode(sensores[i], INPUT);
  }
  pinMode(LED_ON, OUTPUT);
  pinMode(PUSH, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(MOTOR_DERECHO_POSITIVE, OUTPUT);
  pinMode(MOTOR_DERECHO_NEGATIVE, OUTPUT);
  pinMode(MOTOR_RIGTH_A, INPUT);   //como es para el encoder se poner como entrada
  pinMode(MOTOR_RIGTH_B, INPUT);   //como es para el encoder se poner como entrada
  pinMode(MOTOR_IZQUIERDO_POSITIVE, OUTPUT);
  pinMode(MOTOR_IZQUIERDO_NEGATIVE, OUTPUT);
  pinMode(MOTOR_LEFT_A, INPUT);   //como es para el encoder se poner como entrada
  pinMode(MOTOR_LEFT_B, INPUT);   //como es para el encoder se poner como entrada
}

void loop() {
  if (millis() - lastMillis >= deltaMillis) {
    lastMillis = millis();
    int sensorValue0 = analogRead(CNY_0);
    int sensorValue6 = analogRead(CNY_6);
    int sensorValue2 = analogRead(CNY_2);
    error = sensorValue0 - sensorValue6;
    integral += error * (deltaMillis / 1000.0);
    derivative = (error - previousError) / (deltaMillis / 1000.0);
    controlSignal = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;
    if (sensorValue2 < wallThreshold) {
      analogWrite(MOTOR_DERECHO_POSITIVE, 200); // Velocidad moderada
      analogWrite(MOTOR_DERECHO_NEGATIVE, 0);
      analogWrite(MOTOR_IZQUIERDO_POSITIVE, 0);
      analogWrite(MOTOR_IZQUIERDO_NEGATIVE, 200); // Motor izquierdo en reversa
    } else {
      int motorSpeed = constrain(controlSignal, -255, 255);
      if (motorSpeed > 0) {
        analogWrite(MOTOR_DERECHO_POSITIVE, motorSpeed);
        analogWrite(MOTOR_DERECHO_NEGATIVE, 0);
        analogWrite(MOTOR_IZQUIERDO_POSITIVE, 0);
        analogWrite(MOTOR_IZQUIERDO_NEGATIVE, motorSpeed);
      } else {
        analogWrite(MOTOR_DERECHO_POSITIVE, 0);
        analogWrite(MOTOR_DERECHO_NEGATIVE, -motorSpeed);
        analogWrite(MOTOR_IZQUIERDO_POSITIVE, -motorSpeed);
        analogWrite(MOTOR_IZQUIERDO_NEGATIVE, 0);
      }
    }
  }  
}
