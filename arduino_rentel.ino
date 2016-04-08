
// Se importan las librerías
#include <Time.h>
#include <TFT_HX8357.h>
#include <User_Setup.h>
#include <Wire.h>
#include <SFE_BMP180.h>

//Se declara una instancia de la librería
TFT_HX8357 tft = TFT_HX8357();
SFE_BMP180 pressure;

//declaramos el color cyan que no esta por defecto
#define TFT_CYAN 0x7FF  

//Se declaran las variables. Es necesario tomar en cuenta una presión inicial
//esta será la presión que se tome en cuenta en el cálculo de la diferencia de altura
double PresionBase;

//Calculo de RPM
volatile byte half_revolutions;
unsigned int rpm;
unsigned long timeold;


//COSAS DEL SENSOR DE ALTURA
//Leeremos presión y temperatura. Calcularemos la diferencia de altura
double Presion = 0;
double Altura = 0;
double Temperatura = 0;
char status;

// Variables Globales
float minVoltaje = 6;
float maxVoltaje = 12;
float difVoltaje = (maxVoltaje - minVoltaje) / 5;
float accionVoltaje = minVoltaje;
int comprobarVoltaje = 0;

float minAmperios = 10;
float maxAmperios = 20;
float difAmperios = (maxAmperios - minAmperios) / 5;
float accionAmperios = maxAmperios;
int comprobarAmperios = 0;

float tempMotorMin = 20;
float tempMotorMax = 60;
float difTempMotor = (tempMotorMax - tempMotorMin) / 5;

float tempControlMin = 20;
float tempControlMax = 80;
float difTempControl = (tempControlMax - tempControlMin) / 5;
float accionTempControl = tempControlMax;
int comprobarTempControl = 0;

float minRpm = 0;
float maxRpm = 2500;
float difRpm = (maxRpm - minRpm) / 5;
float accionRpm = maxRpm;
int comprobarRpm = 0;

float minTemp = 0;
float maxTemp = 80;
float difTemp = (maxTemp - minTemp) / 5;

float minTrabajo = minAmperios * minVoltaje;
float maxTrabajo = maxAmperios * maxVoltaje;
float difTrabajo = (maxTrabajo - minTrabajo) / 5;
float accionTrabajo = maxTrabajo;
int comprobarTrabajo = 0;


// Variables de configuracion
int menu = 0;
int cursorConfig = 0;
int multiplicador = 1;
int cursorAccion = 0;
int cursorGrafica = 0;
float tamanoBateria = 0;
int tiempoGrafica = 1000; //esta en milisegundos
int sec = 0;
int pulsosRpm = 10;

void setup() {

  Serial.begin(9600);

  tft.begin();
  // Se inicia el sensor y se hace una lectura inicial
  SensorStart();
  pinMode(A1, INPUT);
  pinMode(A0, INPUT);
  pinMode(18, INPUT);
  // pinMode(A2, INPUT);
  // LED
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(A6, OUTPUT);
  pinMode(A7, OUTPUT);
  
  // Botones
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);
  pinMode(A10, INPUT);
  pinMode(A11, INPUT);
  pinMode(A12, INPUT);
  pinMode(A14, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(18), rpm_fun, RISING);
  half_revolutions = 0;
  rpm = 0;
  timeold = 0;
//  Serial.println("Starting the I2C interface.");
  Wire.begin(); // Start the I2C interface.

  setTime(00,00,00,1,1,1970);
  tft.setRotation(1);
  tft.fillScreen(TFT_WHITE);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
}

void loop() {

  // Se hace lectura del sensor, altura y temperatura ambiente
  ReadSensor();
  int bDerecha = analogRead(A8);  
  int bIzquierda = analogRead(A9);
  int bAceptar = analogRead(A10);  
  int bCancelar = analogRead(A11);
  int bMenu = analogRead(A13);
  // Se imprimen las variables
  float voltaje = analogRead(A1);
  voltaje = voltaje / 10.2; 
  
  float consumo = analogRead(A2);

  // Amperaje
  float amp2 = consumo * 100;
  float amperaje= map(amp2, 0, 103200, -20000, 20000);

  //Temp motor
  float temp = analogRead(A0);
  float volts = (temp / 1024.0) * 5.0;
  float celsius = (volts - 0.5) * 100; 
  
  if (celsius < 0){
    celsius = celsius * -1;
  }
  if (bMenu == 1023 && menu == 0) {
      difVoltaje = (maxVoltaje - minVoltaje) / 5;
      difAmperios = (maxAmperios - minAmperios) / 5;
      difTempMotor = (tempMotorMax - tempMotorMin) / 5;
      difTempControl = (tempControlMax - tempControlMin) / 5;
      difRpm = (maxRpm - minRpm) / 5;
      difTemp = (maxTemp - minTemp) / 5;
      difTrabajo = (maxTrabajo - minTrabajo) / 5;
      menu = 1;
      tft.fillScreen(TFT_WHITE);
  } else if (bMenu == 1023) {
      menu = 1;
      tft.fillScreen(TFT_WHITE);
  }

  //Calculamos las revs
  if (half_revolutions >= 40) { 
       //Al aumentar 40 aumenta la resolucion, al disminuir la velocidad de refresco
       rpm = (30*1000/(millis() - timeold)*(half_revolutions/pulsosRpm))*2;
       timeold = millis();
       half_revolutions = 0;
     }

    //Calculamos donde estamos
    if ((bDerecha == 1023 || bAceptar == 1023 || bIzquierda == 1023) && menu != 0 && menu != 3) {
      if (menu == 2){
        menu = 1;
      } else {
        menu++;
      }
        tft.fillScreen(TFT_WHITE);
    } //else if ((bAceptar == 1023 || bIzquierda == 1023) && menu != 0 && menu != 3) {
//      if (menu == 1){
//        menu = 2;
//      } else {
//        menu--;
//      }      
//        tft.fillScreen(TFT_WHITE);
//    }
    if (bCancelar == 1023 && menu != 0 && menu != 3) {
      tft.fillScreen(TFT_WHITE);
      menu = 3;
    }
    
     tft.setTextColor(TFT_BLACK,TFT_WHITE);
     
     if (menu == 0) {
        infoColores(voltaje, amperaje, celsius, rpm);
        configuracion(bDerecha, bIzquierda, bAceptar, bCancelar);
     } else if (menu == 1) {
        pantallaGeneral(voltaje, amperaje, Temperatura, Altura, celsius, rpm);      
        hora();
     } else if (menu == 2) {
        infoColores(voltaje, amperaje, celsius, rpm);
        subMenu1(voltaje ,amperaje, rpm);
     } else if (menu == 3) {
        infoColores(voltaje, amperaje, celsius, rpm);
        accion(bDerecha, bIzquierda, bAceptar, bCancelar);
     }

    comprobar(rpm, amperaje, voltaje, celsius);
    
    time_t t = now();  
    
    if (second(t) == sec) {
        sec++;
        if (sec == 60) {
          sec = 0;
        }
        Serial.print("E");
        Serial.print(millis());
        Serial.print(";");
        Serial.print(voltaje);
        Serial.println("F");
    }
}


// COMPROBACIONES
void comprobar (float rpm, float amperaje, float voltaje, float celsius){
     if (voltaje < accionVoltaje && comprobarVoltaje != 0) {
        analogWrite(A3, 1023);
     } else {        
        analogWrite(A3, 0);
     }
     if (rpm < accionRpm && comprobarRpm != 0) {
        analogWrite(A4, 1023);
     } else {        
        analogWrite(A4, 0);
     }
     if (amperaje < accionAmperios && comprobarAmperios != 0) {
        analogWrite(A5, 1023);
     } else {        
        analogWrite(A5, 0);
     }
     if (celsius < accionTempControl && comprobarTempControl != 0) {
        analogWrite(A6, 1023);
     } else {        
        analogWrite(A6, 0);
     }
}

void accion(int bDerecha, int bIzquierda, int bAceptar, int bCancelar) {
  
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  
  tft.fillRect(240, 0, 150,20, TFT_WHITE);
  tft.drawRect(241, 1, 148,18, TFT_ORANGE);
  tft.setCursor(250, 0);
  tft.print("Suma x ");
  tft.println(multiplicador);
  
  tft.setCursor(20, 20);
  tft.print("RPM => ");
  tft.setCursor(150, 20);
  tft.print(accionRpm);
  tft.setCursor(250, 20);
  tft.print(" ====> ");
  tft.setCursor(350, 20);
  tft.print((comprobarRpm == 0)?"Off":"On ");

  tft.setCursor(20, 60);
  tft.print("W => ");
  tft.setCursor(150, 60);
  tft.print(accionTrabajo);
  tft.setCursor(250, 60);
  tft.print(" ====> ");
  tft.setCursor(350, 60);
  tft.print((comprobarTrabajo == 0)?"Off":"On ");

  tft.setCursor(20, 100);
  tft.print("Amps => ");
  tft.setCursor(150, 100);
  tft.print(accionAmperios);
  tft.setCursor(250, 100);
  tft.print(" ====> ");
  tft.setCursor(350, 100);
  tft.print((comprobarAmperios == 0)?"Off":"On ");

  tft.setCursor(20, 140);
  tft.print("V => ");
  tft.setCursor(150, 140);
  tft.print(accionVoltaje);
  tft.setCursor(250, 140);
  tft.print(" ====> ");
  tft.setCursor(350, 140);
  tft.print((comprobarVoltaje == 0)?"Off":"On ");

  tft.setCursor(20, 180);
  tft.print("Temp => ");
  tft.setCursor(150, 180);
  tft.print(accionTempControl);
  tft.setCursor(250, 180);
  tft.print(" ====> ");
  tft.setCursor(350, 180);
  tft.print((comprobarTempControl == 0)?"Off":"On ");
  
  tft.fillRect(240, 0, 150,20, TFT_WHITE);
  tft.drawRect(241, 1, 148,18, TFT_ORANGE);
  tft.setCursor(250, 0);
  tft.print("Suma x ");
  tft.println(multiplicador);
  
  
  tft.setCursor(20, 220);
  tft.print("T. Grafica (ms) => ");
  tft.setCursor(250, 220);
  tft.print(tiempoGrafica);
  tft.print("   ms");
  
  tft.setCursor(20, 260);
  tft.println("Bateria (kW) => ");
  tft.setCursor(250, 260);
  tft.print(tamanoBateria);
  tft.print("   kw");

  if (bCancelar == 1023) {
      if (multiplicador >= 1000){
        multiplicador = 1;
      }else {
        multiplicador *= 10; 
      }      
   }

   if (bDerecha == 1023) {
      if (cursorAccion == 11){
        cursorAccion = 0;
      } else {
        cursorAccion++;
      }
      tft.fillScreen(TFT_WHITE);
    }

   switch(cursorAccion) {
        case 0:
          tft.fillCircle(140, 25, 5 ,TFT_GREEN);
          break;
        case 1: 
          tft.fillCircle(340, 25, 5,TFT_GREEN);
          break;
        case 2:
          tft.fillCircle(140, 65, 5,TFT_GREEN);
          break;
        case 3: 
          tft.fillCircle(340, 65, 5,TFT_GREEN);
          break;
        case 4: 
          tft.fillCircle(140, 105, 5,TFT_GREEN);
          break;
        case 5:
          tft.fillCircle(340, 105, 5 ,TFT_GREEN);
          break;
        case 6: 
          tft.fillCircle(140, 145, 5,TFT_GREEN);
          break;
        case 7: 
          tft.fillCircle(340, 145, 5,TFT_GREEN);
          break;
        case 8: 
          tft.fillCircle(140, 185, 5,TFT_GREEN);
          break;
        case 9: 
          tft.fillCircle(340, 185, 5,TFT_GREEN);
          break;
        case 10: 
          tft.fillCircle(195, 225, 5,TFT_GREEN);
          break;
        case 11: 
          tft.fillCircle(215, 265, 5,TFT_GREEN);
          break;
        default: break;
      }

    if (bIzquierda == 1023 || bAceptar == 1023) {
      switch(cursorAccion) {
        case 0: 
          accionRpm = compBotonPulsado(bIzquierda, accionRpm, multiplicador); 
//          // (bIzquierda == 1023) ? (accionRpm -= multiplicador) : (accionRpm += multiplicador);
//          tft.fillCircle(140, 25, 5 ,TFT_GREEN);
          break;
        case 1: 
          if(comprobarRpm == 1)  {
            comprobarRpm = 0;
          } else {
            comprobarRpm = 1;
          }
          break;
        case 2:
          accionTrabajo = compBotonPulsado(bIzquierda, accionTrabajo, multiplicador); 
          break;
        case 3: 
          if(comprobarTrabajo == 1)  {
            comprobarTrabajo = 0;
          } else {
            comprobarTrabajo = 1;
          }
          break;
        case 4: 
          accionAmperios = compBotonPulsado(bIzquierda, accionAmperios, multiplicador); 
          break;
        case 5:
          if(comprobarAmperios == 1)  {
            comprobarAmperios = 0;
          } else {
            comprobarAmperios = 1;
          }
          break;
        case 6: 
          accionVoltaje = compBotonPulsado(bIzquierda, accionVoltaje, multiplicador); 
          break;
        case 7: 
          if(comprobarVoltaje == 1)  {
            comprobarVoltaje = 0;
          } else {
            comprobarVoltaje = 1;
          }
          break;
        case 8: 
          accionTempControl = compBotonPulsado(bIzquierda, accionTempControl, multiplicador); 
          break;
        case 9: 
          if(comprobarTempControl == 1)  {
            comprobarTempControl = 0;
          } else {
            comprobarTempControl = 1;
          }
          break;
        case 10: 
          tiempoGrafica = compBotonPulsado(bIzquierda, tiempoGrafica, multiplicador);
          break;
        case 11: 
          tamanoBateria = compBotonPulsado(bIzquierda, tamanoBateria, multiplicador);
          break;
        default: break;
      }
    }
}

void infoColores(float voltaje, float amperaje, float celsius, float rpm) {
  
  
  //VOLTAJE
  tft.drawRect(410,0,69,80, TFT_BLACK);
  tft.fillRect(411,1,67,78,  getColor(voltaje, difVoltaje, minVoltaje, maxVoltaje));
  tft.setTextColor(TFT_BLACK,  getColor(voltaje, difVoltaje, minVoltaje, maxVoltaje));
  tft.setCursor(430, 30);
  tft.setTextSize(2);
  tft.print("V");
  
  //AMPERAJE
  tft.drawRect(410,80,69,80, TFT_BLACK);
  tft.fillRect(411,81,67,78,getColorInvertido(amperaje, difAmperios, minAmperios, maxAmperios));  
  tft.setTextColor(TFT_BLACK, getColorInvertido(amperaje, difAmperios, minAmperios, maxAmperios));
  tft.setCursor(430, 110);
  tft.setTextSize(2);
  tft.print("A");
  
  //REVS
  tft.drawRect(410,160,69,80, TFT_BLACK);
  tft.fillRect(411,161,67,78,getColor(rpm, difRpm, minRpm, maxRpm));
  tft.setTextColor(TFT_BLACK, getColor(rpm, difRpm, minRpm, maxRpm));
  tft.setCursor(425, 190);
  tft.setTextSize(2);
  tft.print("RPM");
  
  //TEMPERATURA MOTOR
  tft.drawRect(410,240,69,80, TFT_BLACK);
  tft.fillRect(411,241,67,78,getColorInvertido(celsius, difTemp, minTemp, maxTemp));
  tft.setTextColor(TFT_BLACK, getColorInvertido(celsius, difTemp, minTemp, maxTemp));
  tft.setCursor(430, 270);
  tft.setTextSize(2);
  tft.print("C");
}


void pantallaGeneral(float voltaje, float amperaje, float Temperatura, float Altura, float celsius, int rpm) {
  
  // Rectangulos
  tft.drawRect(0,15,220,50,TFT_RED);
  
  tft.drawRect(0,75,220,50,TFT_RED);
  
  tft.drawRect(0,135,220,50,TFT_RED);
  
  tft.drawRect(235,15,220,50,TFT_RED);
  
  tft.drawRect(235,75,220,50,TFT_RED);
  
  tft.drawRect(235,135,220,50,TFT_RED);
  
  tft.drawRect(235,195,220,50,TFT_RED);
  
  //COMPROBACIONES
  //VOLTAJE
  tft.fillRect(170,16,49,48, getColor(voltaje, difVoltaje, minVoltaje, maxVoltaje));
  
  //AMPERAJE
  tft.fillRect(170,76,49,48, getColorInvertido(amperaje, difAmperios, minAmperios, maxAmperios));
    
  //REVS
  tft.fillRect(405,16,49,48,getColor(rpm, difRpm, minRpm, maxRpm));
  
  //TEMPERATURA MOTOR
  tft.fillRect(405,76,49,48,getColorInvertido(celsius, difTemp, minTemp, maxTemp));


  tft.setCursor(240, 20);
  tft.setTextSize(2);
  tft.print("RPM: ");
  
  tft.setCursor(240, 40);
  tft.setTextSize(3);
  
  if (rpm  == NULL) {    
    tft.print("0");          
  } else {    
    tft.print(rpm, DEC);      
  }
  tft.println(" RPM  ");
       



  tft.setCursor(10, 20);
  tft.setTextSize(2);
  tft.println("Voltaje: ");
  tft.setCursor(10, 40);
  tft.setTextSize(3);
  tft.print(voltaje);
  tft.print(" V  ");
  
  
  tft.setTextSize(2);
  tft.setCursor(10, 80);
  tft.println("Amperaje: ");
  tft.setCursor(10, 100);
  tft.setTextSize(3);
  tft.print(amperaje/100,1);
  tft.println(" A ");

  
  tft.setTextSize(2);
  tft.setCursor(10, 140);
  tft.println("Trabajo: ");
  tft.setCursor(10, 160);
  tft.setTextSize(3);
  tft.print((amperaje/100)*voltaje, 1);
  tft.println(" W ");

  
  tft.setTextSize(2);
  tft.setCursor(240, 140);
  tft.println("Temperatura: ");
  tft.setCursor(240, 160);
  tft.setTextSize(3);
  tft.print(Temperatura, 1);
  tft.println(" C\t  ");
  
  tft.setTextSize(2);
  tft.setCursor(240, 200);
  tft.println("Altura relativa: ");
  tft.setCursor(240, 220);
  tft.setTextSize(3);
  
  tft.print(Altura, 1);
  tft.println(" m  ");
  
    
  tft.setTextSize(2);
  tft.setCursor(240, 80);
  tft.println("T. Motor: ");
  tft.setCursor(240, 100);
  tft.setTextSize(3);
  tft.print(celsius, 1);
  tft.println(" C\t  ");

}


void configuracion(int bDerecha, int bIzquierda, int bAceptar, int bCancelar) {
/////
      switch(cursorConfig) {
        case 0:
          tft.fillCircle(50, 35, 5,TFT_GREEN);
          break;
        case 1: 
          tft.fillCircle(240, 35, 5,TFT_GREEN);
          break;
        case 2: 
          tft.fillCircle(50, 85, 5,TFT_GREEN);
          break;
        case 3: 
          tft.fillCircle(240, 85, 5,TFT_GREEN);
          break;
        case 4: 
          tft.fillCircle(50, 135, 5,TFT_GREEN);
          break;
        case 5:
          tft.fillCircle(240, 135, 5,TFT_GREEN);
          break;
        case 6: 
          tft.fillCircle(13, 185, 5,TFT_GREEN);
          break;
        case 7: 
          tft.fillCircle(113, 185, 5,TFT_GREEN);
          break;
        case 8:
          tft.fillCircle(243, 185, 5,TFT_GREEN);
          break;
        case 9: 
          tft.fillCircle(50, 235, 5,TFT_GREEN);
          break;
        case 10: 
          tft.fillCircle(240, 235, 5,TFT_GREEN);
          break;
        case 11: 
          tft.fillCircle(50, 285, 5,TFT_GREEN);
          break;
        case 12: 
          tft.fillCircle(240, 285, 5,TFT_GREEN);
          break;     
        default: break;
      }

  
    if (bCancelar == 1023) {
      if (multiplicador >= 1000){
        multiplicador = 1;
      }else {
        multiplicador *= 10; 
      }      
    }
  
    if (bDerecha == 1023) {
      if (cursorConfig == 12){
        cursorConfig = 0;
      } else {
        cursorConfig++;
      }
      tft.fillScreen(TFT_WHITE);
    }
    if (bAceptar == 1023 || bIzquierda == 1023) {
      switch(cursorConfig) {
        case 0: 
          minVoltaje = compBotonPulsado(bIzquierda, minVoltaje, multiplicador); 
          tft.fillCircle(50, 35, 5,TFT_GREEN);
          break;
        case 1: 
          maxVoltaje = compBotonPulsado(bIzquierda, maxVoltaje, multiplicador); 
          tft.fillCircle(240, 35, 5,TFT_GREEN);
          break;
        case 2: 
          minAmperios = compBotonPulsado(bIzquierda, minAmperios, multiplicador); 
          tft.fillCircle(50, 85, 5,TFT_GREEN);
          break;
        case 3: 
          maxAmperios = compBotonPulsado(bIzquierda, maxAmperios, multiplicador); 
          tft.fillCircle(240, 85, 5,TFT_GREEN);
          break;
        case 4: 
          minTemp = compBotonPulsado(bIzquierda, minTemp, multiplicador); 
          tft.fillCircle(50, 135, 5,TFT_GREEN);
          break;
        case 5:
          maxTemp = compBotonPulsado(bIzquierda, maxTemp, multiplicador); 
          tft.fillCircle(240, 135, 5,TFT_GREEN);
          break;
        case 6: 
          pulsosRpm = compBotonPulsado(bIzquierda, pulsosRpm, multiplicador); 
          tft.fillCircle(13, 185, 5,TFT_GREEN);
          break;
        case 7: 
          minRpm = compBotonPulsado(bIzquierda, minRpm, multiplicador); 
          tft.fillCircle(113, 185, 5,TFT_GREEN);
          break;
        case 8: 
          maxRpm = compBotonPulsado(bIzquierda, maxRpm, multiplicador); 
          tft.fillCircle(243, 185, 5,TFT_GREEN);
          break;
        case 9: 
          tempControlMin = compBotonPulsado(bIzquierda, tempControlMin, multiplicador); 
          tft.fillCircle(50, 235, 5,TFT_GREEN);
          break;
        case 10: 
          tempControlMax = compBotonPulsado(bIzquierda, tempControlMax, multiplicador); 
          tft.fillCircle(240, 235, 5,TFT_GREEN);
          break;
        case 11: 
          minTrabajo = compBotonPulsado(bIzquierda, minTrabajo, multiplicador); 
          tft.fillCircle(50, 285, 5,TFT_GREEN);
          break;
        case 12: 
          maxTrabajo = compBotonPulsado(bIzquierda, maxTrabajo, multiplicador); 
          tft.fillCircle(240, 285, 5,TFT_GREEN);
          break;
        default: break;
      }
    }
        
  
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  
  tft.fillRect(240, 0, 150,20, TFT_WHITE);
  tft.drawRect(241, 1, 148,18, TFT_ORANGE);
  tft.setCursor(250, 0);
  tft.print("Suma x ");
  tft.println(multiplicador);
  
  
  tft.setCursor(40, 10);
  tft.println("Voltaje: Voltios");

  tft.setCursor(60, 30);
  tft.print("Min: ");
  tft.println(minVoltaje);

  tft.setCursor(250, 30);
  tft.print("Max: ");
  tft.println(maxVoltaje);


  tft.setCursor(40, 60);
  tft.println("Amperaje: Amperios");

  tft.setCursor(60, 80);
  tft.print("Min: ");
  tft.println(minAmperios);

  tft.setCursor(250, 80);
  tft.print("Max: ");
  tft.println(maxAmperios);


  tft.setCursor(40, 110);
  tft.println("T. Motor: Celsius");

  tft.setCursor(60, 130);
  tft.print("Min: ");
  tft.println(minTemp);

  tft.setCursor(250, 130);
  tft.print("Max: ");
  tft.println(maxTemp);


  tft.setCursor(40, 160);
  tft.println("RPM: revs/min");
  
  tft.setCursor(20, 180);
  tft.print("P: ");
  tft.print(pulsosRpm);
  tft.print(" ");
  
  tft.setCursor(120, 180);
  tft.print("Min: ");
  tft.println(minRpm);

  tft.setCursor(250, 180);
  tft.print("Max: ");
  tft.println(maxRpm);


  tft.setCursor(40, 210);
  tft.println("Temp. Control: Celsius");

  tft.setCursor(60, 230);
  tft.print("Min: ");
  tft.println(tempControlMin);

  tft.setCursor(250, 230);
  tft.print("Max: ");
  tft.println(tempControlMax);


  tft.setCursor(40, 260);
  tft.println("Trabajo: W");

  tft.setCursor(60, 280);
  tft.print("Min: ");
  tft.println(minTrabajo);

  tft.setCursor(250, 280);
  tft.print("Max: ");
  tft.println(maxTrabajo);
}

void hora(){
  
  tft.drawRect(0,195,220,50,TFT_RED);
  
  time_t t = now();  
  tft.setTextSize(2);
  
  tft.setCursor(10, 200);
  
  
  tft.println("Tiempo de Vuelo: ");
  
  tft.setCursor(30, 220);
  
  tft.setTextSize(3);

  if (hour(t) < 10) {
    tft.print("0");
  }
  tft.print(hour(t));
  
  tft.print(":");
  
  if (minute(t) < 10) {
    tft.print("0");
  }
  tft.print(minute(t));
  
  tft.print(":");
  
  if (second(t) < 10) {
    tft.print("0");
  }
  tft.print(second(t));
  
  tft.setCursor(0, 0);
  if(second(t) == 50) {
     tft.fillScreen(TFT_WHITE);
  }
}


// Submenu de Voltaje y RPM
void subMenu1(float voltaje,float amperaje, float rpm) {
  tft.setTextSize(3);
//  tft.drawRect(0, 0, 410, 235, TFT_RED);
  
  float trabajo = voltaje * (amperaje / 100);
  int valorRpm = map(rpm * 1000, minRpm * 1000, maxRpm * 1000, 0, 410);
  float valorTrabajo = map(trabajo * 1000, minTrabajo * 1000, maxTrabajo * 1000, 1, 409);
  
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  
  tft.setCursor(30, 20);
  tft.println("Trabajo: ");
  tft.setTextSize(4);
  tft.setCursor(90, 60);
  tft.print(trabajo);
  tft.print(" W ");
  
  tft.drawRect(0, 100, 410, 60, TFT_ORANGE);
  
  if (valorTrabajo < 1) {
    valorTrabajo = 0;
  } else 
  if (valorTrabajo > 409) {
    valorTrabajo = 409;
  }
  tft.fillRect(1, 101, valorTrabajo, 58, getColor(trabajo, difTrabajo, minTrabajo, maxTrabajo));
  tft.fillRect(valorTrabajo + 1, 101, 408 - valorTrabajo, 58, TFT_WHITE);
  

  tft.setTextSize(3);
  tft.drawRect(0, 260, 410, 60, TFT_ORANGE);
  
  if (valorRpm < 1) {
    valorRpm = 1;
  } else if (valorRpm > 409) {
    valorRpm = 409;
  }
  tft.fillRect(1, 261, valorRpm, 58, getColor(rpm, difRpm, minRpm, maxRpm));
  tft.fillRect(valorRpm, 261, 408 - valorRpm, 58, TFT_WHITE);
  
  tft.setCursor(60, 180);
  tft.println("RPM: ");
  tft.setTextColor(4);
  tft.setCursor(90, 220);
  tft.setTextSize(4);
  tft.print(rpm);
  tft.print(" RPM ");
}

void SensorStart() {

  //Secuencia de inicio del sensor

  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {

    Serial.println("BMP180 init fail (disconnected?)\n\n");
    while (1);
  }

  //Se inicia la lectura de temperatura
  status = pressure.startTemperature();
  if (status != 0)  {
    delay(status);
    //Se lee una temperatura inicial
    status = pressure.getTemperature(Temperatura);
    if (status != 0)    {
      //Se inicia la lectura de presiones
      status = pressure.startPressure(3);
      if (status != 0)      {
        delay(status);
        //Se lee la presión inicial incidente sobre el sensor en la primera ejecución
        status = pressure.getPressure(PresionBase, Temperatura);
      }
    }
  }
}


void ReadSensor() {
  //En este método se hacen las lecturas de presión y temperatura y se calcula la altura

  //Se inicia la lectura de temperatura
  status = pressure.startTemperature();
  if (status != 0)
  {
    delay(status);
    //Se realiza la lectura de temperatura
    status = pressure.getTemperature(Temperatura);
    if (status != 0)
    {
      //Se inicia la lectura de presión
      status = pressure.startPressure(3);
      if (status != 0)
      {
        delay(status);
        //Se lleva a cabo la lectura de presión,
        //considerando la temperatura que afecta el desempeño del sensor

        status = pressure.getPressure(Presion, Temperatura);
        if (status != 0)
        {
          //Se hace el cálculo de la altura en base a la presión leída en el Setup
          Altura = pressure.altitude(Presion, PresionBase);
        }
        else Serial.println("error en la lectura de presion\n");
      }
      else Serial.println("error iniciando la lectura de presion\n");
    }
    else Serial.println("error en la lectura de temperatura\n");
  }
  else Serial.println("error iniciando la lectura de temperatura\n");

}

 void rpm_fun()
 {
   half_revolutions++;
 }

 int getColor(float valor, float diferencia, float minimo, float maximo) {
    int color = TFT_BLACK;
    if (valor <= minimo) {    
        color = TFT_RED;
    } else if (valor < (minimo + diferencia)) {
      color = TFT_ORANGE;
    } else if (valor < (minimo + diferencia * 2)){     
      color = TFT_YELLOW;
    } else if (valor < (minimo + diferencia * 3)) {
      color = TFT_GREEN;
    } else {  
      color = TFT_CYAN;
    }
    return color;
  }
 int getColorInvertido(float valor, float diferencia, float minimo, float maximo) {
    int color = TFT_BLACK;
    if (valor >= maximo) {    
        color = TFT_RED;
      } else if (valor > (maximo - diferencia)) {
        color = TFT_ORANGE;
      } else if (valor > (maximo - diferencia * 2)){     
        color = TFT_YELLOW;
      } else if (valor > (maximo - diferencia * 3)) {
        color = TFT_GREEN;
      } else {  
        color = TFT_CYAN;
      }
    return color;
  }

  
int compBotonPulsado(int bIzquierda, float cantidad, int multiplicador){  
     return (bIzquierda == 1023) ? (cantidad -= multiplicador) : (cantidad += multiplicador);
}
