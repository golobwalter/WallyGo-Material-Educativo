#include <Arduino.h>

// =====================================================
// ENCODER INDUSTRIAL LPD3806-600BM-G12-24C
// =====================================================

#define ENC_A 34
#define ENC_B 35

const int COUNTS_REV = 2400;

volatile long encoderCount = 0;


// =====================================================
// KY-040
// =====================================================

#define KY_CLK 18
#define KY_DT  19
#define KY_SW  23

volatile int kyCount = 0;
volatile int lastCLK = HIGH;


// =====================================================
// BTS7960
// =====================================================

#define RPWM 25
#define LPWM 26
#define R_EN 27
#define L_EN 14

const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;


// =====================================================
// TIEMPO DE MUESTREO
// =====================================================

const float Ts = 0.01;              // 10 ms
const unsigned long Ts_us = 10000; // 10.000 us

unsigned long tiempoControl = 0;


// =====================================================
// MATRIZ Ad
// MATLAB
// =====================================================

const float Ad11 = 1.000000000000000;
const float Ad12 = 0.009446422268971;

const float Ad21 = 0.000000000000000;
const float Ad22 = 0.891366143906831;


// =====================================================
// MATRIZ Bd
// MATLAB
// =====================================================

const float Bd1 = 0.006017149250313;
const float Bd2 = 1.180802783621398;


// =====================================================
// GANANCIA DEL OBSERVADOR Ld
// MATLAB
// =====================================================

const float L1 = -0.001799117988554;
const float L2 =  0.620109011070488;


// =====================================================
// GANANCIAS DEL CONTROLADOR
// MATLAB
// =====================================================

float K1 = 0.031194756073717;
float K2 = -0.069562513456743;
float Kr = 0.031194756073717;


// =====================================================
// ESTADOS ESTIMADOS
// =====================================================

float xhat1 = 0.0;
float xhat2 = 0.0;


// =====================================================
// REFERENCIA / SET
// =====================================================

float referencia = 0.0;


// =====================================================
// PARAMETRO SELECCIONADO
//
// 0 = SET
// 1 = K1
// 2 = K2
// 3 = Kr
// =====================================================

int parametro = 0;


// =====================================================
// INTERRUPCION DEL ENCODER
// =====================================================

void IRAM_ATTR encoderISR()
{
  if (digitalRead(ENC_B))
    encoderCount++;
  else
    encoderCount--;
}


// =====================================================
// INTERRUPCION KY-040
// =====================================================

void IRAM_ATTR ky040ISR()
{
  int clk = digitalRead(KY_CLK);

  if (clk != lastCLK)
  {
    if (digitalRead(KY_DT) != clk)
      kyCount++;
    else
      kyCount--;

    lastCLK = clk;
  }
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  // ---------------------------------------------------
  // LPD3806
  // ---------------------------------------------------

  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(ENC_A),
    encoderISR,
    RISING
  );


  // ---------------------------------------------------
  // KY-040
  // ---------------------------------------------------

  pinMode(KY_CLK, INPUT_PULLUP);
  pinMode(KY_DT, INPUT_PULLUP);
  pinMode(KY_SW, INPUT_PULLUP);

  lastCLK = digitalRead(KY_CLK);

  attachInterrupt(
    digitalPinToInterrupt(KY_CLK),
    ky040ISR,
    CHANGE
  );


  // ---------------------------------------------------
  // BTS7960
  // ---------------------------------------------------

  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);

  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  ledcAttach(RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(LPWM, PWM_FREQ, PWM_RESOLUTION);

  ledcWrite(RPWM, 0);
  ledcWrite(LPWM, 0);


  // ---------------------------------------------------
  // TIEMPO INICIAL
  // ---------------------------------------------------

  tiempoControl = micros();
}


// =====================================================
// LOOP
// =====================================================

void loop()
{

  // ===================================================
  // BOTON KY-040
  //
  // SET -> K1 -> K2 -> Kr
  // ===================================================

  static int ultimoBoton = HIGH;

  int boton = digitalRead(KY_SW);

  if (boton == LOW && ultimoBoton == HIGH)
  {
    parametro++;

    if (parametro > 3)
      parametro = 0;

    delay(200);

    noInterrupts();
    kyCount = 0;
    interrupts();
  }

  ultimoBoton = boton;


  // ===================================================
  // LEER KY-040
  // ===================================================

  static int cuentaAnterior = 0;

  noInterrupts();
  int cuenta = kyCount;
  interrupts();

  int delta = cuenta - cuentaAnterior;

  if (delta != 0)
  {
    cuentaAnterior = cuenta;


    // -------------------------------------------------
    // SET
    // -------------------------------------------------

    if (parametro == 0)
    {
      referencia += delta;

      if (referencia > 180)
        referencia = 180;

      if (referencia < -180)
        referencia = -180;
    }


    // -------------------------------------------------
    // K1
    // -------------------------------------------------

    if (parametro == 1)
    {
      K1 += delta * 0.001;

      if (K1 < 0)
        K1 = 0;
    }


    // -------------------------------------------------
    // K2
    // -------------------------------------------------

    if (parametro == 2)
    {
      K2 += delta * 0.001;
    }


    // -------------------------------------------------
    // Kr
    // -------------------------------------------------

    if (parametro == 3)
    {
      Kr += delta * 0.001;

      if (Kr < 0)
        Kr = 0;
    }
  }


  // ===================================================
  // CONTROL CADA 10 ms
  // ===================================================

  unsigned long ahora = micros();

  if ((unsigned long)(ahora - tiempoControl) >= Ts_us)
  {
    tiempoControl += Ts_us;


    // -------------------------------------------------
    // POSICION REAL DEL ENCODER
    // -------------------------------------------------

    long cuentaEncoder;

    noInterrupts();
    cuentaEncoder = encoderCount;
    interrupts();

    float y =
      cuentaEncoder * 360.0 / COUNTS_REV;


    // -------------------------------------------------
    // ERROR DEL OBSERVADOR
    //
    // y - Cd*xhat
    //
    // Cd = [1 0]
    //
    // por lo tanto:
    //
    // y - xhat1
    // -------------------------------------------------

    float errorObservador =
      y - xhat1;


    // -------------------------------------------------
    // LEY DE CONTROL
    //
    // u = -Kd*xhat + Kr*referencia
    //
    // u = -K1*xhat1
    //     -K2*xhat2
    //     +Kr*referencia
    // -------------------------------------------------

    float u =
      -K1 * xhat1
      -K2 * xhat2
      +Kr * referencia;


    // -------------------------------------------------
    // SATURACION PWM
    // -------------------------------------------------

    if (u > 255)
      u = 255;

    if (u < -255)
      u = -255;


    // -------------------------------------------------
    // BTS7960
    // -------------------------------------------------

    if (u > 0)
    {
      ledcWrite(LPWM, 0);
      ledcWrite(RPWM, (int)u);
    }
    else
    {
      ledcWrite(RPWM, 0);
      ledcWrite(LPWM, (int)(-u));
    }


    // -------------------------------------------------
    // OBSERVADOR DISCRETO
    //
    // xhat(k+1) =
    // Ad*xhat(k)
    // + Bd*u(k)
    // + Ld*(y-Cd*xhat)
    // -------------------------------------------------

    float xhat1_new =
      Ad11 * xhat1
      + Ad12 * xhat2
      + Bd1 * u
      + L1 * errorObservador;


    float xhat2_new =
      Ad21 * xhat1
      + Ad22 * xhat2
      + Bd2 * u
      + L2 * errorObservador;


    // -------------------------------------------------
    // ACTUALIZAR ESTADOS
    // -------------------------------------------------

    xhat1 = xhat1_new;
    xhat2 = xhat2_new;


    // =================================================
    // MONITOR SERIAL
    // =================================================

    Serial.print("PARAM=");

    if (parametro == 0)
      Serial.print("SET");

    if (parametro == 1)
      Serial.print("K1");

    if (parametro == 2)
      Serial.print("K2");

    if (parametro == 3)
      Serial.print("Kr");


    Serial.print("  SET=");
    Serial.print(referencia, 2);

    Serial.print("  K1=");
    Serial.print(K1, 6);

    Serial.print("  K2=");
    Serial.print(K2, 6);

    Serial.print("  Kr=");
    Serial.print(Kr, 6);

    Serial.print("  Y=");
    Serial.print(y, 2);

    Serial.print("  X1=");
    Serial.print(xhat1, 2);

    Serial.print("  X2=");
    Serial.print(xhat2, 2);

    Serial.print("  U=");
    Serial.println(u, 2);
  }
}
