#define PI 3.1415926535897932384626433832795
#include <PinChangeInterrupt.h>  // Importar la biblioteca para PCINT si usas pines no compatibles

// Parámetros lx y ly
float lx = 0.28;  // Distancia en metros desde el centro hasta las ruedas en el eje X
float ly = 0.28;
float lx_plus_ly = lx + ly;

// Pines de Conexion
const int enMS[4] = {5, 6 , 9 , 11};  // M1 M2 M3 M4  PWM
const int in1[4] = {A2, A4, 8, 12};  // Pines IN1 para dirección (Motores M1, M2, M3, M4)
const int in2[4] = {A3, A5, 10, 13}; // Pines IN2 para dirección (Motores M1, M2, M3, M4)
const int stby = 23;

// Pines de conexión para los encoders
const int inENC[4] = {2, 3, 18, 19};  // Pines C1 para los encoders de cada motor

// Parámetros del robot
float r = 0.05725 / 2;  // Radio R
float L = 0.145;  // Distancia R

// Velocidades
float w[4] = {0, 0, 0, 0};

// Variables de movimiento
float Vx = 0;    // VelocidadX (m/s)
float Vy = 0;    // VelocidadY (m/s)
float omega = 0; // Velocidad angular (rad/s)

// Valores Motor
int CajaR = 120;
int ResoENC = 11;

// Variables PID
float PWM[4] = {100, 100, 100, 100};
float sp[4] = {0, 0, 0, 0};
float pv[4] = {0, 0, 0, 0};

// Parámetros PID
float cv[4] = {0, 0, 0, 0};
float cv1[4] = {0, 0, 0, 0};
float error[4] = {0, 0, 0, 0};
float error1[4] = {0, 0, 0, 0};
float error2[4] = {0, 0, 0, 0};

float Kp[4] = {1, 1, 1, 1};
float Ki[4] = {5, 5, 5, 5};
float Kd[4] = {0.1, 0.1, 0.1, 0.1};
float Tm = 0.1;

// Inversión de motores (true si el motor está invertido)
const bool motorInverted[4] = {true, false, true, false};

volatile int contador1 = 0;
volatile int contador2 = 0;
volatile int contador3 = 0;
volatile int contador4 = 0;

unsigned long previousMillis = 0;
long interval = 100;  // Intervalo de 100 ms

void setup() {
  Serial.begin(9600);

  ConfigurarPinesMotores();
  ConfigurarInterrupciones();

  Serial.println("Arduino listo.");
}

void loop() {
  // Leer datos del puerto serial
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim(); // Eliminar espacios en blanco

    if (data.length() > 0) {
      // Parsear los datos
      int firstComma = data.indexOf(',');
      int secondComma = data.indexOf(',', firstComma + 1);

      String Vx_str = data.substring(0, firstComma);
      String Vy_str = data.substring(firstComma + 1, secondComma);
      String omega_str = data.substring(secondComma + 1);

      int Vx_received = Vx_str.toInt();
      int Vy_received = Vy_str.toInt();
      int omega_received = omega_str.toInt();

      // Convertir a float dividiendo entre 100
      Vx = Vx_received / 100.0;
      Vy = Vy_received / 100.0;
      omega = omega_received / 100.0;

      // Imprimir para depuración
      Serial.print("Vx recibido: ");
      Serial.println(Vx);
      Serial.print("Vy recibido: ");
      Serial.println(Vy);
      Serial.print("omega recibido: ");
      Serial.println(omega);
    }
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    pv[0] = contador1 * 0.04545; // RPM
    pv[1] = contador2 * 0.04545; // RPM
    pv[2] = contador3 * 0.04545; // RPM
    pv[3] = contador4 * 0.04545; // RPM
    contador1 = 0;
    contador2 = 0;
    contador3 = 0;
    contador4 = 0;

    // Actualizar velocidades
    ObtenerVelocidades();
    ControlPIMotores();
  }
}

void ConfigurarPinesMotores() {
  for (int i = 0; i < 4; i++) {
    pinMode(enMS[i], OUTPUT);
    pinMode(in1[i], OUTPUT);
    pinMode(in2[i], OUTPUT);
  }
  // Configurar STBY pin si es necesario
  pinMode(stby, OUTPUT);
  digitalWrite(stby, HIGH);  // Desactivar modo standby
}

void ConfigurarInterrupciones() {
  attachInterrupt(digitalPinToInterrupt(inENC[0]), interrupcion1, RISING); // Lectura de flancos de subida
  attachInterrupt(digitalPinToInterrupt(inENC[1]), interrupcion2, RISING);
  attachInterrupt(digitalPinToInterrupt(inENC[2]), interrupcion3, RISING);
  attachInterrupt(digitalPinToInterrupt(inENC[3]), interrupcion4, RISING);
}

void ObtenerVelocidades() {
  // Cinemática inversa para calcular velocidades angulares de las ruedas
  w[0] = (1 / r) * (Vx - Vy - (lx_plus_ly) * omega);  // front_left
  w[1] = (1 / r) * (Vx + Vy + (lx_plus_ly) * omega);  // front_right
  w[2] = (1 / r) * (Vx + Vy - (lx_plus_ly) * omega);  // back_left
  w[3] = (1 / r) * (Vx - Vy + (lx_plus_ly) * omega);  // back_right
}

void ControlPIMotores() {
  for (int i = 0; i < 4; i++) {
    sp[i] = abs(w[i] * 60 / (2 * PI)); // Velocidad en RPM
    PWM[i] = controlPID(Kp[i], Ki[i], Kd[i], sp[i], pv[i], cv[i], cv1[i], error[i], error1[i], error2[i]);
    controlMotor(enMS[i], in1[i], in2[i], w[i], PWM[i], motorInverted[i]);
    // Para depuración, descomenta las siguientes líneas
    /*
    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(" - SP: ");
    Serial.print(sp[i]);
    Serial.print(", PV: ");
    Serial.print(pv[i]);
    Serial.print(", PWM: ");
    Serial.println(PWM[i]);
    */
  }
}

float controlPID(float Kp, float Ki, float Kd, float sp, float pv, float &cv, float &cv1, float &error, float &error1, float &error2) {
  error = sp - pv;
  // Ecuación de Diferencias
  cv = cv1 + (Kp + Kd / Tm) * error + (-Kp + Ki * Tm - 2 * Kd / Tm) * error1 + (Kd / Tm) * error2;
  cv1 = cv;
  error2 = error1;
  error1 = error;
  // Saturación
  if (cv > 400.0) {
    cv = 400.0;
  }
  if (cv < 40.0) {
    cv = 40.0;
  }
  return cv;
}

void interrupcion1() {
  contador1++;
}

void interrupcion2() {
  contador2++;
}

void interrupcion3() {
  contador3++;
}

void interrupcion4() {
  contador4++;
}

void controlMotor(int enPin, int in1Pin, int in2Pin, float velocidad, float cv, bool inverted) {
  // Mapear cv a PWM (0-255)
  analogWrite(enPin, cv * (255.0 / 400.0)); // 0-255

  // Ajustar la dirección del motor con inversión si es necesario
  if (velocidad > 0) {
    // Mover hacia adelante
    if (inverted) {
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, HIGH);
    } else {
      digitalWrite(in1Pin, HIGH);
      digitalWrite(in2Pin, LOW);
    }
  } else if (velocidad < 0) {
    // Mover hacia atrás
    if (inverted) {
      digitalWrite(in1Pin, HIGH);
      digitalWrite(in2Pin, LOW);
    } else {
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, HIGH);
    }
  } else {
    // Detener el motor
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
}
