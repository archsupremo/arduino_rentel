
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

float minAmperios = 10;
float maxAmperios = 20;
float difAmperios = (maxAmperios - minAmperios) / 5;

float tempMotorMin = 20;
float tempMotorMax = 60;
float difTempMotor = (tempMotorMax - tempMotorMin) / 5;

float minRpm = 0;
float maxRpm = 2500;
float difRpm = (maxRpm - minRpm) / 5;

float minTemp = 0;
float maxTemp = 80;
float difTemp = (maxTemp - minTemp) / 5;

float minTrabajo = minAmperios * minVoltaje;
float maxTrabajo = maxAmperios * maxVoltaje;
float difTrabajo = (maxTrabajo - minTrabajo) / 5;

int menu = 2;
int cursorConfig = 0;

void setup() {

  Serial.begin(9600);

  tft.begin();
  // Se inicia el sensor y se hace una lectura inicial
  SensorStart();
  pinMode(A1, INPUT);
  pinMode(A0, INPUT);
//  pinMode(A2, INPUT);
  pinMode(A3, OUTPUT);
  pinMode(18, INPUT);
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(18), rpm_fun, RISING);
  half_revolutions = 0;
  rpm = 0;
  timeold = 0;
  Serial.println("Starting the I2C interface.");
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


  //Calculamos las revs
  if (half_revolutions >= 40) { 
       //Al aumentar 40 aumenta la resolucion, al disminuir la velocidad de refresco
       rpm = (30*1000/(millis() - timeold)*(half_revolutions/10))*2;
       timeold = millis();
       half_revolutions = 0;
     }


    //Calculamos donde estamos
//    if (bDerecha == 1023) {
//      if (menu == 2){
//        menu = 0;
//      } else {
//        menu++;
//      }
//      
//        tft.fillScreen(TFT_WHITE);
//    } else if (bIzquierda == 1023) {
//      if (menu == 0){
//        menu = 2;
//      } else {
//        menu--;
//      }
//      
//        tft.fillScreen(TFT_WHITE);
//    }

    if (bDerecha == 1023) {
      if (cursorConfig == 8){
        cursorConfig = 0;
      } else {
        cursorConfig++;
      }
      tft.fillScreen(TFT_WHITE);
    }

    if (bIzquierda == 1023) {
      switch(cursorConfig) {
        case 0: minVoltaje++; break;
        case 1: maxVoltaje++; break;
        case 2: minAmperios++; break;
        case 3: maxAmperios++; break;
        case 4: minTemp++; break;
        case 5: maxTemp++; break;
        case 6: minRpm++; break;
        case 7: maxRpm++; break;
        default: break;
      }
    }
    
     tft.setTextColor(TFT_BLACK,TFT_WHITE);
     hora();
     if (menu == 0) {
        pantallaGeneral(voltaje, amperaje, Temperatura, Altura, celsius, rpm);      
     } else if (menu == 1) {
        infoColores(voltaje, amperaje, celsius, rpm);
        subMenu1(voltaje ,amperaje, rpm);
     } else if (menu == 2) {
        infoColores(voltaje, amperaje, celsius, rpm);
        configuracion();
     }

     if (voltaje > 8) {
        analogWrite(A3, HIGH);
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
  tft.println(" V  ");
  
  
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

void configuracion() {
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
  tft.println("T. Motor: Grados Centigrados");

  tft.setCursor(60, 130);
  tft.print("Min: ");
  tft.println(minTemp);

  tft.setCursor(250, 130);
  tft.print("Max: ");
  tft.println(maxTemp);


  tft.setCursor(40, 160);
  tft.println("RPM: revs/min");

  tft.setCursor(60, 180);
  tft.print("Min: ");
  tft.println(minRpm);

  tft.setCursor(250, 180);
  tft.print("Max: ");
  tft.println(maxRpm);
}

void hora(){
  

  if(menu == 1) {
    tft.drawRect(110,255,220,50,TFT_RED);
  } else {
    tft.drawRect(0,255,220,50,TFT_RED);
  }
  
  time_t t = now();  
  tft.setTextSize(2);
  
  if(menu == 1) {
    tft.setCursor(120, 260);
  } else {    
    tft.setCursor(10, 260);
  }
  
  tft.println("Tiempo de Vuelo: ");
  
  if(menu == 1) {
    tft.setCursor(140, 280);
  } else {    
    tft.setCursor(30, 280);
  }
  
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
  if(second(t)%50 == 0 && second(t) != 0) {
     tft.fillScreen(TFT_WHITE);
  }
}


// Submenu de Voltaje y RPM
void subMenu1(float voltaje,float amperaje, float rpm) {
  tft.drawRect(0, 0, 410, 235, TFT_RED);
  float trabajo = voltaje * (amperaje / 100);
  tft.drawRect(29, 29, 332, 62, TFT_ORANGE);
  float valorTrabajo = map(trabajo * 1000, minTrabajo * 1000, maxTrabajo * 1000, 30, 330);
  if (valorTrabajo < 30) {
    valorTrabajo = 30;
  } else if (valorTrabajo > 360) {
    valorTrabajo = 360;
  }
  tft.fillRect(30, 30, valorTrabajo - 30, 60, getColor(trabajo, difTrabajo, minTrabajo, maxTrabajo));
  tft.fillRect(valorTrabajo, 30, 360 - valorTrabajo, 60, TFT_WHITE);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setCursor(120, 100);
  tft.print("Trabajo: ");
  tft.print(trabajo);
  tft.print(" W ");
  
  
  tft.drawRect(29, 139, 332, 62, TFT_ORANGE);
  int valorRpm = map(rpm * 1000, minRpm * 1000, maxRpm * 1000, 30, 330);
  if (valorRpm < 30) {
    valorRpm = 30;
  } else if (valorRpm > 360) {
    valorRpm = 360;
  }
  tft.fillRect(30, 140, valorRpm - 30, 60, getColor(voltaje, difVoltaje, minVoltaje, maxVoltaje));
  tft.fillRect(valorRpm, 140, 360 - valorRpm, 60, TFT_WHITE);
  
  tft.setCursor(120, 210);
  tft.print("RPM: ");
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

