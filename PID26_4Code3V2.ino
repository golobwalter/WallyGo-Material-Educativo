/*
  SERVO PID ESP32 + IGNIS + BTS7960
  Version 2.0

  Hardware:
  - ESP32 WROOM LX6
  - Encoder industrial 600PPR A/B
  - BTS7960
  - Motor DC IGNIS 12V
  - Encoder KY-040 / EC11 para setpoint

  Funciones:
  - Control de posicion angular
  - Ajuste de Kp Ki Kd desde encoder
  - PID con periodo fijo
*/


#include <Arduino.h>
#include <ESP32Encoder.h>


//================================================
// ENCODER INDUSTRIAL 600 PPR
//================================================

#define ENCA 34
#define ENCB 35

ESP32Encoder motorEnc;


//================================================
// ENCODER SETPOINT KY-040
//================================================

#define CLK 18
#define DT  19
#define SW  23


int lastCLK = HIGH;

bool buttonState = HIGH;
bool lastButtonState = HIGH;


//================================================
// BTS7960
//================================================

#define RPWM 25
#define LPWM 26


#define PWM_FREQ 20000
#define PWM_RES 8



//================================================
// PID
//================================================


float Kp = 1.2;
float Ki = 0.0;
float Kd = 0.15;


float integral = 0;
float previousError = 0;


// Tiempo de muestreo PID

const unsigned long PID_TIME = 10;

unsigned long lastPID = 0;



//================================================
// ENCODER
//================================================


// 600 PPR x 4 cuadratura

const long COUNTS_REV = 2400;



//================================================
// SETPOINT
//================================================


float setpointDeg = 0;



//================================================
// MENU KY040
//================================================


enum modoAjuste
{

  AJUSTE_POSICION,

  AJUSTE_KP,

  AJUSTE_KI,

  AJUSTE_KD

};


modoAjuste modo = AJUSTE_POSICION;



//================================================
// LIMITES PID
//================================================


float integralMax = 800;



// Zona muerta

float deadBand = 2;



// PWM minimo para vencer rozamiento

int pwmMin = 35;



//================================================
// FUNCION MOTOR BTS7960
//================================================


void motorDrive(int pwm)
{


  pwm = constrain(pwm,-255,255);



  // zona muerta motor detenido

  if(abs(pwm)<pwmMin)
  {

    if(abs(pwm)<5)
    {
      pwm=0;
    }

    else
    {
      if(pwm>0)
          pwm=pwmMin;

      else
          pwm=-pwmMin;
    }

  }



  if(pwm>0)
  {

    ledcWrite(RPWM,pwm);
    ledcWrite(LPWM,0);

  }

  else
  {

    ledcWrite(RPWM,0);
    ledcWrite(LPWM,-pwm);

  }


}



//================================================
// SETUP
//================================================


void setup()
{


Serial.begin(115200);


//------------------------------
// KY040
//------------------------------

pinMode(CLK,INPUT_PULLUP);
pinMode(DT,INPUT_PULLUP);
pinMode(SW,INPUT_PULLUP);


lastCLK=digitalRead(CLK);



//------------------------------
// Encoder industrial
//------------------------------


motorEnc.attachFullQuad(
    ENCA,
    ENCB
);


motorEnc.clearCount();



//------------------------------
// PWM ESP32 CORE 3.x
//------------------------------


ledcAttach(
    RPWM,
    PWM_FREQ,
    PWM_RES
);


ledcAttach(
    LPWM,
    PWM_FREQ,
    PWM_RES
);



Serial.println();
Serial.println("=======================");
Serial.println(" SERVO PID ESP32 V2.0 ");
Serial.println("=======================");

}
//================================================
// LECTURA KY-040
//================================================

void leerEncoderSetpoint()
{


static unsigned long ultimoGiro = 0;


int clk = digitalRead(CLK);



if(clk != lastCLK)
{


  // antirrebote

  if(millis()-ultimoGiro > 3)
  {


    if(digitalRead(DT)!=clk)
    {

      // giro horario

      modificarValor(-1);

    }

    else
    {

      // giro antihorario

      modificarValor(+1);

    }


    ultimoGiro=millis();

  }


}


lastCLK=clk;


}



//================================================
// MODIFICAR VARIABLE SELECCIONADA
//================================================


void modificarValor(int incremento)
{


switch(modo)
{


case AJUSTE_POSICION:

    setpointDeg += incremento;


    if(setpointDeg>=360)
        setpointDeg-=360;


    if(setpointDeg<0)
        setpointDeg+=360;

break;



case AJUSTE_KP:


    Kp += incremento*0.05;


    if(Kp<0)
        Kp=0;


break;



case AJUSTE_KI:


    Ki += incremento*0.005;


    if(Ki<0)
        Ki=0;


break;



case AJUSTE_KD:


    Kd += incremento*0.01;


    if(Kd<0)
        Kd=0;


break;


}


}



//================================================
// PULSADOR KY-040
// Cambio de modo
//================================================


void leerBoton()
{


static unsigned long tiempoBoton=0;



bool lectura = digitalRead(SW);



if(lectura != lastButtonState)
{


 if(millis()-tiempoBoton>50)
 {


   if(lectura==LOW)
   {


      modo=(modoAjuste)(modo+1);



      if(modo>AJUSTE_KD)
          modo=AJUSTE_POSICION;


      Serial.println();

      Serial.print("Nuevo modo: ");


      mostrarModo();


   }


   tiempoBoton=millis();

 }


}


lastButtonState=lectura;


}



//================================================
// MOSTRAR MODO ACTUAL
//================================================


void mostrarModo()
{


switch(modo)
{

case AJUSTE_POSICION:

 Serial.println("POSICION");
 break;


case AJUSTE_KP:

 Serial.println("AJUSTE KP");
 break;


case AJUSTE_KI:

 Serial.println("AJUSTE KI");
 break;


case AJUSTE_KD:

 Serial.println("AJUSTE KD");
 break;


}


}



//================================================
// LECTURA ENCODER MOTOR
//================================================


float leerPosicionGrados()
{


long cuenta = motorEnc.getCount();



// Posicion circular

cuenta = cuenta % COUNTS_REV;


if(cuenta<0)
{
    cuenta += COUNTS_REV;
}



float grados =
      cuenta * 360.0 /
      COUNTS_REV;



return grados;


}



//================================================
// CUENTAS ACTUALES DEL ENCODER
//================================================


long leerCuentaEncoder()
{


long cuenta = motorEnc.getCount();



cuenta = cuenta % COUNTS_REV;


if(cuenta<0)
{
    cuenta+=COUNTS_REV;
}


return cuenta;


}
//================================================
// PID DE POSICION
//================================================


void ejecutarPID()
{


//--------------------------------
// Leer posición actual
//--------------------------------


long cuentaActual = leerCuentaEncoder();



float posicionActual =
      cuentaActual *
      360.0 /
      COUNTS_REV;



//--------------------------------
// Convertir setpoint a cuentas
//--------------------------------


float cuentaObjetivo =
      setpointDeg *
      COUNTS_REV /
      360.0;



//--------------------------------
// Error circular
//--------------------------------


float error =
      cuentaObjetivo -
      cuentaActual;



if(error > COUNTS_REV/2)
{
    error -= COUNTS_REV;
}



if(error < -COUNTS_REV/2)
{
    error += COUNTS_REV;
}



//--------------------------------
// Zona muerta
//--------------------------------


if(abs(error) < deadBand)
{

    motorDrive(0);

    integral=0;

    return;

}



//--------------------------------
// Integral con anti windup
//--------------------------------


integral += error;


if(integral > integralMax)
    integral=integralMax;


if(integral < -integralMax)
    integral=-integralMax;



//--------------------------------
// Derivada
//--------------------------------


float derivada =
       error -
       previousError;



//--------------------------------
// PID
//--------------------------------


float salida =
      Kp*error +
      Ki*integral +
      Kd*derivada;



previousError=error;



//--------------------------------
// Limitar salida
//--------------------------------


salida = constrain(
            salida,
            -255,
            255
          );



//--------------------------------
// Motor
//--------------------------------


motorDrive((int)salida);



//--------------------------------
// Monitor
//--------------------------------


static unsigned long tiempoSerie=0;



if(millis()-tiempoSerie>300)
{


tiempoSerie=millis();


Serial.println("------------------");


Serial.print("Modo : ");

mostrarModo();


Serial.print("SET  : ");

Serial.print(setpointDeg,1);

Serial.println(" grados");



Serial.print("POS  : ");

Serial.print(posicionActual,1);

Serial.println(" grados");



Serial.print("ERR  : ");

Serial.println(error);



Serial.print("PWM  : ");

Serial.println(salida);



Serial.print("Kp   : ");

Serial.println(Kp,3);



Serial.print("Ki   : ");

Serial.println(Ki,3);



Serial.print("Kd   : ");

Serial.println(Kd,3);


}


}



//================================================
// LOOP PRINCIPAL
//================================================


void loop()
{


//--------------------------------
// Interfaz usuario
//--------------------------------


leerEncoderSetpoint();


leerBoton();



//--------------------------------
// PID cada 10 ms
//--------------------------------


if(millis()-lastPID >= PID_TIME)
{


lastPID=millis();


ejecutarPID();


}



}
