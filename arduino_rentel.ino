
// Se importan las librerías

//librerias complementarias
#include <Wire.h>
#include <SPI.h>

//giroscopio
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


//reloj interno

#include <rtc_clock.h>
RTC_clock rtc_clock(RC);
RTC_clock rtc_interno(RC);

//reloj externo
#include "RTClib1.h"
RTC_DS1307 rtc;

//tarjeta sd
#include <SD.h>

//barometro, altura, temperatura ambiente
#include <SFE_BMP180.h>

//pantalla
#include <TFT_HX8357_Due.h> // PARA DUE




//Se declara una instancia de la librería
TFT_HX8357_Due tft = TFT_HX8357_Due();

SFE_BMP180 pressure;

MPU6050 mpu;


//declaramos el color cyan que no esta por defecto y el pin q usamos para los grados
#define TFT_CYAN 0x7FF  

#define INTERRUPT_PIN 11 

//Se declaran las variables. Es necesario tomar en cuenta una presión inicial
//esta será la presión que se tome en cuenta en el cálculo de la diferencia de altura
double PresionBase;

//Calculo de RPM

volatile int rpmcount = 0;//see http://arduino.cc/en/Reference/Volatile 
unsigned long rpm = 0;
unsigned long lastmillis = 0;

int revs =0;
//COSAS DEL SENSOR DE ALTURA
//Leeremos presión y temperatura. Calcularemos la diferencia de altura
double Presion = 0;
double Altura = 0;
double Temperatura = 0;
char status;

// Variables Globales
float minVoltaje = 0;
float maxVoltaje = 100;
float difVoltaje = (maxVoltaje - minVoltaje) / 5;
float accionVoltaje = minVoltaje;
int comprobarVoltaje = 0;

float minAmperios = 0;
float maxAmperios = 100;
float difAmperios = (maxAmperios - minAmperios) / 5;
float accionAmperios = maxAmperios;
int comprobarAmperios = 0;


float minRpm = 0;
float maxRpm = 2500;
float difRpm = (maxRpm - minRpm) / 5;
float accionRpm = maxRpm;
int comprobarRpm = 0;

float minTemp = 0;//celsius en infocolores
float maxTemp = 80;
float difTemp = (maxTemp - minTemp) / 5;

float tempControladoraMin = minTemp;
float tempControladoraMax = maxTemp;
float diftempControladora = difTemp;
float accionTempControl = tempControladoraMax;
int comprobarTempControl = 0;
//
//float tempControlMin = 20;
//float tempControlMax = 80;
//float difTempControl = (tempControlMax - tempControlMin) / 5;

float minTrabajo = 0;
float maxTrabajo = 5000;
float difTrabajo = (maxTrabajo - minTrabajo) / 5;
float accionTrabajo = maxTrabajo;
int comprobarTrabajo = 0;


// Variables de configuracion
int menu = 0;
int cursorConfig = 0;
int multiplicador = 1;
int cursorAccion = 0;
int cursorGrafica = 0;
int tiempoGrafica = 1000; //esta en milisegundos
int sec = 0;
int pulsosRpm = 10;
int contador = 0 ;
int campo= 0;
String directorio="";
String dir_actual = "";
int num_actual= 0;
float last_trabajo= 0;
float last_rpm= 0;
int refresco =0;


float voltaje=0;
float voltaje_inicial = 0;
float minimoPorCiento = 0;


float porciento=0;
float consumo=0;
float amperaje=0;
float tempControladora= 0;
float watios=0;
String dir = "";
int last_sec=0;
File dataFile;

//variables para posicion

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//stado orientacion
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


float anguloX = 0;//pitch
float anguloY = 0;//roll
float anguloZ = 0;//yaw

float accelX = 0; // arriba abajo
float accelY = 0; // izquierda derecha
float accelZ = 0; // adelante atras?

int sec_pasados = 0;

float puntoMedio = 1276;
float salto = 8;
int alarma1 = 0;
int alarma2 = 0;
int alarma3 = 0;
int alarma4 = 0;
int alarma5 = 0;
  


void setup() {

  tft.begin();
  tft.fillScreen(TFT_WHITE);
  tft.setRotation(1);
  tft.setTextSize(2);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  SensorStart();
  
  Wire1.begin(); // Inicia el puerto I2C
  rtc.begin();
  //rtc.adjust(DateTime(2016,05,4,9,48,00));
  // Para ajustar la hora y/m/d h/m/s
  
  voltaje=0;
  consumo=0;
  amperaje=0;
  tempControladora= 0;
  watios=0;
  tft.print("SD:  ");
  if (SD.begin(10)) { 
  tft.print(" detectada. ");
         
    tft.println(" Leyendo archivos");
    File dir_telemetria = SD.open("dir.txt");
    String fraseCompleta = "";
    while (dir_telemetria.available()) {
      char letra = dir_telemetria.read();
      fraseCompleta += letra;
    } 
    int inicio = fraseCompleta.lastIndexOf("=");
    String valor =   fraseCompleta.substring(inicio+1);
    dir_telemetria.close();

    
    tft.print("Sacando valor ... ");
    int num = valor.toInt();
    num++;
    dir_telemetria = SD.open("dir.txt", FILE_WRITE);
    dir_telemetria.print("=");
    dir_telemetria.println(num);
    dir_telemetria.close();
    tft.println(num);
    
    tft.println("Creando directorio");
    directorio = "dir";
    directorio += num;


    



    
    SD.mkdir(directorio);
    directorio += "/";
    tft.print(directorio);  
    tft.println(" creado");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

     
    nuevoDir();
  } else {
    tft.setTextColor(TFT_RED,TFT_BLACK);
    tft.println("Problema con sd ");
    tft.println("Usando datos por defecto");
    tft.setTextColor(TFT_WHITE,TFT_BLACK);
    
  }
   
  
  
  delay(2000);

  
  
  // Se inicia el sensor y se hace una lectura inicial
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
  pinMode(2,INPUT);
  pinMode(3,INPUT);
  pinMode(4,INPUT);
  pinMode(5,INPUT);
  pinMode(6,INPUT);
  pinMode(7,INPUT);
  
  // pin de rpm
  attachInterrupt(18, rpm_fan, FALLING);

  
  DateTime pasa = rtc.now();
  sec_pasados = pasa.unixtime();


//Por problemas usamos el reloj interno para calcular la hora actual y el tiempo pasado
  
  rtc_clock.init();
  rtc_clock.set_time(pasa.hour(), pasa.minute(), pasa.second());



  
  //setup de posicion
  Wire.begin(); // Start the I2C interface.
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  Serial.begin(115200);
    // initialize device
  tft.println(F("Iniciando dispositivos I2C ..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  tft.println(F("Comprobando conexion..."));
  mpu.testConnection() ?  tft.setTextColor(TFT_GREEN, TFT_BLACK) :  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.println(mpu.testConnection() ? F("MPU6050 conectado con exito") : F("Fallo al conectar a MPU6050"));

  // wait for ready
  delay(2000);
  
  // load and configure the DMP
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println(F("Inicializando DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(800); // 1788 staba puesto, 800 da mas cercano a 0


    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.print(F("Activando giroscopio..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        tft.println(F("OK"));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.print(F("ERROR con giroscopio"));
    }
    delay(4000);
    
  tft.fillScreen(TFT_WHITE);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
}

void loop() {


  
  /*=========================================================================
                                INICIO DE CALCULO DE POSICION
   =========================================================================*/



    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        anguloZ = ypr[0] * 180/M_PI;
        if (anguloZ <0) {
          anguloZ = 360 + anguloZ; 
        }
        anguloX = ypr[1] * 180/M_PI;
        if (anguloX <0) {
          anguloX = 360 + anguloX; 
        }
        anguloY = ypr[2] * 180/M_PI;
        if (anguloY <0) {
          anguloY = 360 + anguloY; 
        }

        
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        accelX = aaReal.x;
        accelX = accelX / 8192;
        accelY = aaReal.y;
        accelY = accelY / 8192;
        accelZ = aaReal.z;
        accelZ = accelZ / 8192;

    }

    
  /*=========================================================================
                                FIN DE CALCULO DE POSICION
   =========================================================================*/

  // Se hace lectura del sensor, altura y temperatura ambiente
  ReadSensor();
  int bCampo =   digitalRead(6);//bcampo
  int bMenos = digitalRead(5); //b-
  int bMas = digitalRead(4);  //b+
  int bVuelta = digitalRead(3); //bvuelta
  int bMenu = digitalRead(2);//bmenu
  int bConf = digitalRead(7); //lastb
  // Se imprimen las variables
   voltaje = analogRead(A0);
  voltaje = voltaje / 9.3; 
  float dif_bateria = voltaje_inicial - minimoPorCiento;
  porciento = (voltaje - minimoPorCiento) / (dif_bateria / 100);
/*voltaje_inicial = 100 %
 * minimoPorCiento = 0%
 * 
 * 
 */


  
  /*
  1023 -> 3.3
  x -> 3
  x =930
  930/y=100*/
   consumo = analogRead(A1);
   /*8mv x a
   //3.3 / 1023
   // /8 
   3300 1023
   0 0
   8 2.48 =~ 2
   1 0.31
   
   */
 
  // Amperaje
  float amp2 = consumo ;
  float puntoMedioValor = (1023 * (puntoMedio/1000))/3.3;//395.56
  float maximoValor = ((salto/1000)* 200) + (puntoMedio / 1000);
  maximoValor = (1023 * maximoValor)/3.3;
  float diferenciaValor = maximoValor - puntoMedioValor;//496
  if (puntoMedioValor > diferenciaValor){
    float minimoValor = -200;
    amperaje = map(amp2, puntoMedioValor - diferenciaValor, maximoValor , minimoValor, 200);
  } else {
    float minimoValor = ((puntoMedioValor * 200) / diferenciaValor)* -1;
    amperaje = map(amp2, 0, maximoValor , minimoValor, 200);
  }


/*
  float amperaje2= map(amp2, 0, 892 , -159, 200);//697.5
   amperaje=amperaje2 ;
  
  1023 -> 3.3
  x -> 2.876
  x=891.56

puntoMedio = 1.276;
salto = 0.008;

puntoMedioValor = (1023 * puntoMedio)/3.3;//395.56
maximoValor = (salto * 200) + puntoMedio;
maximoValor = (1023 * maximoValor)/3.3;





  1.276 -1.6 = -0.324

  - 0.324 -> -200
  0 -> 
diferenciaValor = maximoValor - puntoMedioValor;496
minimoValor = ((puntoMedioValor * 200) / diferenciaValor)* -1;

 float amperaje2= map(amp2, 0, maximoValor , minimoValor, 200);


  
  496 200

  395.56 

  1023 3.3
  x    1.276
  
  1.276 + 200 * 0.008
  
  395.56 == 0 Amp
  */
   watios = voltaje * amperaje;
  //Temp motor
  float temp = analogRead(A2);
  float volts = (temp / 1024.0) * 3.3;
   tempControladora = (volts) * 100;
   tempControladora += 1 ; 



  
  if (bMenu == HIGH && menu == 0) {
      difVoltaje = (maxVoltaje - minVoltaje) / 5;
      difAmperios = (maxAmperios - minAmperios) / 5;
      diftempControladora = (tempControladoraMax - tempControladoraMin) / 5;
//      difTempControl = (tempControlMax - tempControlMin) / 5;
      difRpm = (maxRpm - minRpm) / 5;
      difTemp = (maxTemp - minTemp) / 5;
      difTrabajo = (maxTrabajo - minTrabajo) / 5;
      menu = 1;
      tft.fillScreen(TFT_WHITE);
  } else if (bMenu == HIGH) {
      menu = 1;
      tft.fillScreen(TFT_WHITE);
  }
  if (bConf == HIGH && menu != 0 && menu != 3) {        
      menu = 0;
      tft.fillScreen(TFT_WHITE);
  } else if (bConf == HIGH && menu != 3 && menu != 4) {      
      menu = 3;
      tft.fillScreen(TFT_WHITE);
  } else if (bConf == HIGH) {
      menu = 4;
      tft.fillScreen(TFT_WHITE);
  }


    //Calculamos donde estamos
    if ((bCampo == HIGH || bMas == HIGH || bMenos == HIGH) && menu != 0 && menu != 3 && menu != 4) {
      if (menu == 2){
        menu = 1;
        delay(200);
      } else {
        menu++;
        delay(200);
      }
        tft.fillScreen(TFT_WHITE);
    } 
    
     tft.setTextColor(TFT_BLACK,TFT_WHITE);
     
     /*
    if (rtc_clock.get_seconds() == sec) {
      pintarTelemetria();
    }
  */
     
     
     if (menu == 0) {
        infoColores();
        configuracion(bCampo, bMenos, bMas, bVuelta);
     } else if (menu == 1) {
        pantallaGeneral();      
        hora();
     } else if (menu == 2) {
        infoColores();
        //subMenu1(voltaje, amperaje, rpm, watios);
        subMenu2(rpm, watios);
     } else if (menu == 3) {
        infoColores();
        accion(bCampo, bMenos, bMas, bVuelta);
     } else if (menu == 4) {
        getTelemetria(bCampo, bMas, bMenos, bVuelta);
     } 
     
    comprobar();
}

void rpm_fan(){ /* this code will be executed every time the interrupt 0 (pin2) gets low.*/
  
  rpmcount++;
  
  DateTime now = rtc.now();
  
 if (now.second() != last_sec){  /*Uptade every one second, this will be equal to reading frecuency (Hz).*/
 
       detachInterrupt(18);    //Disable interrupt when calculating
       revs = rpmcount * 60 / pulsosRpm;  /* Convert frecuency to RPM, note: this works for one interruption per full rotation. For two interrups per full rotation use rpmcount * 30.*/
       
       
       rpmcount = 0; // Restart the RPM counter
       
       last_sec = now.second(); // Uptade lasmillis
       attachInterrupt(18, rpm_fan, FALLING); //enable interrupt
       //pintarTelemetria();
  }
}

void limpiar(){  
  if (refresco >= 1000) {
    tft.fillScreen(TFT_WHITE);
    refresco = 0;
  }
  refresco++;
}

void subMenu2 (float rpm, float watios) {
  limpiar();
  
  tft.setCursor(10,10);
  tft.setTextSize(3);
  tft.setTextColor(TFT_BLACK,TFT_WHITE);
  tft.println("Trabajo :");
  tft.setCursor(20,50);
  /*
  int lala = watios + 500;
  tft.print(lala);*/
  tft.print(watios);
  tft.println(" ");
  tft.setCursor(10,170);
  tft.println("Revs/min:");
  tft.setCursor(20,210);
  if (revs  < 20) {    
    tft.print("0");          
  } else {    
    tft.print(revs, DEC);      
  }
  tft.println("  ");
  
    
  if (watios >= minTrabajo && watios <= maxTrabajo){
    pintar(250,150,100,maxTrabajo - minTrabajo, watios, TFT_BLACK);
    if (watios != last_trabajo){
      pintar(250,150,100,maxTrabajo - minTrabajo, last_trabajo, TFT_WHITE);
    }
    last_trabajo= watios;
  } else {    
    pintar(250,150,100,maxTrabajo - minTrabajo, 0, TFT_BLACK);
    if (watios != last_trabajo){
      pintar(250,150,100,maxTrabajo - minTrabajo, last_trabajo, TFT_WHITE);
    }
    last_trabajo= 0;
  }
  if (revs >= minRpm && revs <= maxRpm){
    pintar(250,310,100,maxRpm - minRpm, revs, TFT_BLACK);
    if (revs != last_rpm) {
      pintar(250,310,100,maxRpm - minRpm, last_rpm, TFT_WHITE);
    }
    last_rpm= revs;
  }else {    
    pintar(250,310,100,minRpm - maxRpm, 0, TFT_BLACK);
    if (revs != last_rpm) {
      pintar(250,310,100,minRpm - maxRpm, last_rpm, TFT_WHITE);
    }
    last_rpm= 0;
  }

    tft.setTextSize(1);
    tft.setCursor(120,140);
    tft.println(minTrabajo);
    tft.setCursor(360,140);
    tft.println(maxTrabajo);
    tft.setCursor(240,20);
    tft.println((maxTrabajo - minTrabajo)/2);

    
    pintarPunto(250,150,110,60,TFT_YELLOW);
    
    float trozo = (maxTrabajo - minTrabajo)/6;
    tft.setCursor(110,90);
    tft.println(((maxTrabajo - minTrabajo)/6)* 1);
    tft.setCursor(150,40);
    tft.println(((maxTrabajo - minTrabajo)/6)* 2);
    tft.setCursor(310,40);
    tft.println(((maxTrabajo - minTrabajo)/6)* 4);
    tft.setCursor(350,90);
    tft.println(((maxTrabajo - minTrabajo)/6)* 5);



    //pintamos los bordes de referencia de los colores
    pintarPunto(250,150,110,20,TFT_CYAN);
    pintarPunto(250,150,110,40,TFT_GREEN);
    pintarPunto(250,150,110,80,TFT_ORANGE);
    pintarPunto(250,150,110,100,TFT_RED);


    pintarWarning(250,310,110,TFT_RED);
    //extra referencia
    pintarPunto(250,150,110,10,TFT_BLACK);
    pintarPunto(250,150,110,30,TFT_BLACK);
    pintarPunto(250,150,110,50,TFT_BLACK);
    pintarPunto(250,150,110,70,TFT_BLACK);
    pintarPunto(250,150,110,90,TFT_BLACK);
    pintarPunto(250,150,110,110,TFT_BLACK);

    
    
    pintarPunto(250,310,110,60,TFT_YELLOW);
    
    tft.setCursor(120,300);
    tft.println(minRpm);
    tft.setCursor(360,300);
    tft.println(maxRpm);
    tft.setCursor(240,180);
    tft.println((maxRpm - minRpm)/2);


    


    
    pintarPunto(250,310,110,20,TFT_CYAN);
    pintarPunto(250,310,110,40,TFT_GREEN);
    pintarPunto(250,310,110,80,TFT_ORANGE);
    pintarPunto(250,310,110,100,TFT_RED);
    
    pintarWarning(250,310,110,TFT_RED);
    
    pintarPunto(250,310,110,10,TFT_BLACK);
    pintarPunto(250,310,110,30,TFT_BLACK);
    pintarPunto(250,310,110,50,TFT_BLACK);
    pintarPunto(250,310,110,70,TFT_BLACK);
    pintarPunto(250,310,110,90,TFT_BLACK);
    pintarPunto(250,310,110,110,TFT_BLACK);

    
    trozo = (maxRpm - minRpm)/6;
    
    tft.setCursor(110,250);
    tft.println(trozo * 1);
    tft.setCursor(150,200);
    tft.println(trozo * 2);
    tft.setCursor(310,200);
    tft.println(trozo * 4);
    tft.setCursor(350,250);
    tft.println(trozo * 5);
}

void pintarPunto (int cx, int cy, int longitud, float campo, int color) {
    float valor = 0;
    float x =0;
    float y =0;

         
     valor = map(campo,0,120,0,100);
     x= cos(valor/100 * PI)*longitud;
     y= sin(valor/100 * PI)*longitud;
    tft.drawLine(cx - 0.85*x,cy - 0.85*y,cx -x,cy-y,color);  
}
void pintarWarning (int cx, int cy, int longitud,  int color) {
    float valor = 0;
    float x =0;
    float y =0;

         
     valor = map(120,0,120,0,100);
     x= cos(valor/100 * PI)*longitud;
     y= sin(valor/100 * PI)*longitud;
     tft.fillTriangle(cx - 0.85*x,cy - 0.85*y, cx,cy,cx+longitud, cy,color);
    //tft.drawLine(cx - 0.85*x,cy - 0.85*y,cx -x,cy-y,color);  
}





  void pintar (int cx, int cy, int longitud,int res ,float campo, int color) {
    float valor = 0;
    float x =0;
    float y =0;

     valor = map(campo,0,res,0,100);
     x= cos(valor/100 * PI)*longitud;
     y= sin(valor/100 * PI)*longitud;
    tft.drawLine(cx,cy,cx -x,cy-y,color);
    tft.drawLine(cx+1,cy+3,cx -x,cy-y,color);
    tft.drawLine(cx+2,cy+2,cx -x,cy-y,color);
    tft.drawLine(cx+3,cy+1,cx -x,cy-y,color);
    tft.drawLine(cx+4,cy,cx -x,cy-y,color);
    tft.drawLine(cx-1,cy-3,cx -x,cy-y,color);
    tft.drawLine(cx-2,cy-2,cx -x,cy-y,color);
    tft.drawLine(cx-3,cy-1,cx -x,cy-y,color);
    tft.drawLine(cx-4,cy,cx -x,cy-y,color);
    tft.fillCircle(cx -x,cy-y, 3,color);
    
    tft.drawLine(cx - longitud,cy,cx + longitud,cy,TFT_RED);
     tft.fillCircle(cx,cy, 10 , TFT_BLACK);
     tft.drawCircle(cx,cy, 10 , TFT_MAGENTA);
     tft.drawCircle(cx,cy, 11 , TFT_MAGENTA);
     tft.drawCircle(cx,cy, 12 , TFT_MAGENTA);
  }

 void pintarTelemetria(){ 
  
        String telemetria = "";
        sec++;
        if (sec == 60) {
          sec = 0;
        //Evalor;valor2F
        }
        telemetria = telemetria +  millis();
        telemetria = telemetria + "&voltaje;";
        telemetria = telemetria + voltaje;
        telemetria = telemetria + "&amperaje;";
        telemetria = telemetria + amperaje;
        telemetria = telemetria + "&watios;";
        telemetria = telemetria +(voltaje * amperaje);
        telemetria = telemetria + "&rpm;";
        telemetria = telemetria + revs;
        telemetria = telemetria + "&temp;";
        telemetria = telemetria + tempControladora;

        File dataFileVoltaje = SD.open(directorio + "tel.txt" , FILE_WRITE);
        dataFileVoltaje.println(telemetria);
        dataFileVoltaje.close();

}
  
void nuevoDir() {

     tft.setTextColor(TFT_GREEN, TFT_BLACK);
     tft.println("OK");
    
 minVoltaje = getValor("min_v");
 maxVoltaje = getValor("max_v");
 voltaje_inicial = maxVoltaje;

 minAmperios = getValor("min_a");
 maxAmperios = getValor("max_a");

salto = getValor("saltos");
puntoMedio = getValor("p_medio");
minimoPorCiento = getValor("min_bat");

 minRpm = getValor("min_r");
 maxRpm = getValor("max_r");

 minTemp = getValor("min_t");
 maxTemp = getValor("max_t");

 minTrabajo = getValor("min_w");
 maxTrabajo = getValor("max_w");
  
 pulsosRpm = getValor("pulsos");
    

     tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.println(" datos cargados");
    
    
  
}

void getTelemetria(int bCampo, int bMas, int bMenos, int bVuelta){
  /*Muestro los datos para labview*/
Serial.print("start");
Serial.print(voltaje);
Serial.print("amp");
Serial.print(amperaje);
Serial.print("wats");
Serial.print(watios);
Serial.print("tempCtrl");
Serial.print(tempControladora);
Serial.print("rpm");
Serial.print(revs);
Serial.print("tempAmb");
Serial.print(Temperatura);
Serial.print("angX");
Serial.print(anguloX);
Serial.print("angY");
Serial.print(anguloY);
Serial.print("angZ");
Serial.print(anguloZ);
Serial.print("accX");
Serial.print(accelX);
Serial.print("accY");
Serial.print(accelY);
Serial.print("accZ");
Serial.print(accelZ);
Serial.print("altura");
Serial.print(Altura);
Serial.print("porcent");
Serial.print(porciento);
Serial.print("a1");
Serial.print(alarma1);
Serial.print("a2");
Serial.print(alarma2);
Serial.print("a3");
Serial.print(alarma3);
Serial.print("a4");
Serial.print(alarma4);
Serial.print("a5");
Serial.print(alarma5);
Serial.print("fin");


    limpiar();
    tft.setTextSize(2);
    tft.setCursor(10,10);
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.println("   Abre linkduino");
    tft.println("   Pulsa -/+ para cambiar dir");
    tft.print("   ultimo dir....");
    tft.println(directorio);
    int num_final = directorio.substring(3).toInt();
    if (dir_actual == "") {
      dir_actual = directorio;
      num_actual = num_final;
    }
    tft.print("   dir actual.....");
    tft.println(dir_actual);
    
    if (bMenos == HIGH) {
      if (num_actual > 1){
        num_actual--;
        dir_actual = "dir" + String(num_actual)+"/";
      } else {
        dir_actual = directorio;
      }
    }  
    if (bMas == HIGH) {
      if (num_actual < num_final){
        num_actual++;
        dir_actual = "dir" + String(num_actual)+"/";
      } else {
        dir_actual = "dir1/";
      }
    }  
    
    
    tft.println("   Pulsa campo para ");
    tft.println("   cambiar telemetria");  
    tft.print("   campo....");

  //cancelar vuelta, bCampo campo, bMas mas, bizq menos
    if (bCampo == HIGH) {
      if (campo == 4) {
        campo = 0;
      } else {
        campo++;
      }
    } 
    String dato ="";
    switch (campo) {
      case 0:
      tft.println("Voltaje       ");
      dato="voltaje";
                      break;
      case 1:
      tft.println("Amperaje      ");
      dato="amperaje";
                      break;
      case 2:
      tft.println("Trabajo       ");
      dato="watios";
                      break;
      case 3:
      tft.println("Revoluciones");
      dato="rpm";
                      break;
      case 4:
      tft.println("Temperatura");
      dato="temp";
                      break;
       default: break;
    }
    tft.println("   Pulsa conf para mostrar ");
    ///////////////////////////////////
    File dataFileCampo = SD.open(dir_actual + "tel.txt");
      if (bVuelta == HIGH) {
        tft.println("Pintando telemetria");    
        String fraseCompleta = "";  
        while (dataFileCampo.available()) {    //saco todo el texto
          char letra =  dataFileCampo.read();
          fraseCompleta += letra;
        }
      
        int inicio = fraseCompleta.indexOf("\n");
        int final = fraseCompleta.indexOf("\n",inicio+1);
        String frase =   fraseCompleta.substring(inicio, final);
      
  
        int info_inicio = 2;
        int info_final = frase.indexOf("&");
        String info = "";
          //Evalor;valor2F
  
  
          //pintamos el primer valor y declaramos las  variables
        if (frase.length()>2) {
          info = "E";
          info += frase.substring(info_inicio, info_final);
          info += ";";
          info_inicio = frase.indexOf(dato+";");
          info_final = frase.indexOf("&",info_inicio);
          info+= frase.substring(info_inicio + dato.length()+1,info_final);
          info += "F";
          Serial.println(info);
        }
        
        while (fraseCompleta.indexOf("\n", inicio+1) > 0) {//resto de lineas
          inicio = fraseCompleta.indexOf("\n",inicio +1);
          final = fraseCompleta.indexOf("\n",inicio+1);
          frase =   fraseCompleta.substring(inicio, final);
          if (frase.length()>2) {
            info_final = frase.indexOf("&");
            info = "E"; 
            info += frase.substring(0, info_final);
            info += ";";
            info_inicio = frase.indexOf(dato+";");
            info_final = frase.indexOf("&",info_inicio);
            info+= frase.substring(info_inicio + dato.length()+1,info_final);
            info += "F";
             Serial.println(info);
          }      
        }   
          
          Serial.println("FIN DE LECTURA");
        }
  dataFileCampo.close();
         
      

}

int getValor(String campoEnviado){ 
  File prueba = SD.open("max_min.txt");
  String fraseCompleta = "";
  while (prueba.available()) {
    char letra = prueba.read();
    fraseCompleta += letra;
  } 
  String busqueda = campoEnviado + ":";
  int inicio = fraseCompleta.indexOf(busqueda);
  int final = fraseCompleta.indexOf("&",inicio);
  String valor =   fraseCompleta.substring(inicio + busqueda.length(), final);
  prueba.close();
  return valor.toInt();  
}

void setValor(String campoEnviado, int valor){ 
  File prueba = SD.open("max_min.txt");
  String fraseCompleta = "";
  while (prueba.available()) {
    char letra = prueba.read();
    fraseCompleta += letra;
  } 
  String busqueda = campoEnviado + ":";
  int inicio = fraseCompleta.indexOf(busqueda);
  int final = fraseCompleta.indexOf("&",inicio);
  String parte_inicial =   fraseCompleta.substring(0, inicio + busqueda.length());
  String parte_final =   fraseCompleta.substring(final, fraseCompleta.length());
  parte_inicial += valor;
  parte_inicial += parte_final;
  prueba.close();
  SD.remove("max_min.txt");
  prueba = SD.open("max_min.txt", FILE_WRITE);
  prueba.println(parte_inicial);  
  prueba.close();
}

// COMPROBACIONES
void comprobar (){
     if (voltaje < accionVoltaje && comprobarVoltaje != 0) {
        analogWrite(A3, 1023);
        alarma1 = 1;
     } else {        
        analogWrite(A3, 0);
        alarma1 = 0;
     }
     if (revs < accionRpm && comprobarRpm != 0) {
        analogWrite(A4, 1023);
        alarma2 = 1;
     } else {        
        analogWrite(A4, 0);
        alarma2 = 0;
     }
     if (amperaje < accionAmperios && comprobarAmperios != 0) {
        analogWrite(A5, 1023);
        alarma3 = 1;
     } else {        
        analogWrite(A5, 0);
        alarma3 = 0;
     }
     if (tempControladora < accionTempControl && comprobarTempControl != 0) {
        analogWrite(A6, 1023);
        alarma4 = 1;
     } else {        
        analogWrite(A6, 0);
        alarma4 = 0;
     }
     if (watios < accionTrabajo && comprobarTrabajo != 0) {
        analogWrite(A6, 1023);
        alarma5 = 1;
     } else {        
        analogWrite(A6, 0);
        alarma5 = 0;
     }
}

void accion(int bCampo, int bMenos, int bMas, int bVuelta) {
  limpiar();
  
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
  

  if (bVuelta == HIGH) {
      if (multiplicador >= 1000){
        multiplicador = 1;
      }else {
        multiplicador *= 10; 
      }      
   }

   if (bCampo == HIGH) {
      if (cursorAccion == 10){
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
          break;/*
        case 11: 
          tft.fillCircle(215, 265, 5,TFT_GREEN);
          break;*/
        default: break;
      }

    if (bMenos == HIGH || bMas == HIGH) {
      switch(cursorAccion) {
        case 0: 
          accionRpm = compBotonPulsado(bMenos, accionRpm, multiplicador); 
//          // (bMenos == HIGH) ? (accionRpm -= multiplicador) : (accionRpm += multiplicador);
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
          accionTrabajo = compBotonPulsado(bMenos, accionTrabajo, multiplicador); 
          break;
        case 3: 
          if(comprobarTrabajo == 1)  {
            comprobarTrabajo = 0;
          } else {
            comprobarTrabajo = 1;
          }
          break;
        case 4: 
          accionAmperios = compBotonPulsado(bMenos, accionAmperios, multiplicador); 
          break;
        case 5:
          if(comprobarAmperios == 1)  {
            comprobarAmperios = 0;
          } else {
            comprobarAmperios = 1;
          }
          break;
        case 6: 
          accionVoltaje = compBotonPulsado(bMenos, accionVoltaje, multiplicador); 
          break;
        case 7: 
          if(comprobarVoltaje == 1)  {
            comprobarVoltaje = 0;
          } else {
            comprobarVoltaje = 1;
          }
          break;
        case 8: 
          accionTempControl = compBotonPulsado(bMenos, accionTempControl, multiplicador); 
          break;
        case 9: 
          if(comprobarTempControl == 1)  {
            comprobarTempControl = 0;
          } else {
            comprobarTempControl = 1;
          }
          break;
        case 10: 
          tiempoGrafica = compBotonPulsado(bMenos, tiempoGrafica, multiplicador);
          break;/*
        case 11: 
          tamanoBateria = compBotonPulsado(bMenos, tamanoBateria, multiplicador);
          break;*/
        default: break;
      }
    }
}

void infoColores() {
  
  
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
  tft.fillRect(411,161,67,78,getColorInvertido(rpm, difRpm, minRpm, maxRpm));
  tft.setTextColor(TFT_BLACK, getColorInvertido(rpm, difRpm, minRpm, maxRpm));
  tft.setCursor(425, 190);
  tft.setTextSize(2);
  tft.print("RPM");
  
  //TEMPERATURA MOTOR
  tft.drawRect(410,240,69,80, TFT_BLACK);
  tft.fillRect(411,241,67,78,getColorInvertido(tempControladora, difTemp, minTemp, maxTemp));
  tft.setTextColor(TFT_BLACK, getColorInvertido(tempControladora, difTemp, minTemp, maxTemp));
  tft.setCursor(430, 270);
  tft.setTextSize(2);
  tft.print("C");
}


void pantallaGeneral() {
  //rtc.adjust(DateTime(2016,05,4,9,51,00));
  limpiar();
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
  tft.fillRect(405,16,49,48,getColorInvertido(rpm, difRpm, minRpm, maxRpm));
  
  //TEMPERATURA MOTOR
  tft.fillRect(405,76,49,48,getColorInvertido(tempControladora, difTemp, minTemp, maxTemp));


  tft.setCursor(240, 20);
  tft.setTextSize(2);
  tft.print("RPM: ");
  
  tft.setCursor(240, 40);
  tft.setTextSize(3);
  
  if (revs  < 20) {    
    tft.print("0");          
  } else {    
    tft.print(revs, DEC);      
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
  tft.print(amperaje, 1);
  tft.println(" A ");

  
  tft.setTextSize(2);
  tft.setCursor(10, 140);
  tft.println("Trabajo: ");
  tft.setCursor(10, 160);
  tft.setTextSize(3);
  tft.print((amperaje)*voltaje, 1);
  tft.println(" W ");
  
  tft.setTextSize(2);
  tft.drawRect(235,255,220,70,TFT_RED);
  tft.setCursor(240, 260);
  tft.println("Bateria restante");
  tft.setCursor(240, 280);
  tft.setTextSize(3);
  tft.print(porciento);
  tft.print(" % ");
  
  tft.setTextSize(2);
  tft.setCursor(240, 140);
  tft.println("Temperatura: ");
  tft.setCursor(240, 160);
  tft.setTextSize(3);
  tft.print(Temperatura, 1);//quito 5 para ajustarlo 
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
  tft.println("T. Controladora: ");
  tft.setCursor(240, 100);
  tft.setTextSize(3);
  tft.print(tempControladora, 1);
  tft.println(" C\t  ");

}


void configuracion(int bCampo, int bMenos, int bMas, int bVuelta) {
/////
  limpiar();
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
          tft.fillCircle(5, 235, 5,TFT_GREEN);
          break;
        case 10: 
          tft.fillCircle(105, 235, 5,TFT_GREEN);
          break;  
        case 11: 
          tft.fillCircle(285, 235, 5,TFT_GREEN);
          break;  
        case 12: 
          tft.fillCircle(50, 285, 5,TFT_GREEN);
          break;
        case 13: 
          tft.fillCircle(240, 285, 5,TFT_GREEN);
          break;   
        default: break;
      }

  
    if (bVuelta == HIGH) {
      if (multiplicador >= 1000){
        multiplicador = 1;
      }else {
        multiplicador *= 10; 
      }      
    }
    if (bCampo == HIGH) {
      if (cursorConfig == 13){
        cursorConfig = 0;
      } else {
        cursorConfig++;
      }
      tft.fillScreen(TFT_WHITE);
    }
    if (bMas == HIGH || bMenos == HIGH) {
      switch(cursorConfig) {
        case 0: 
          minVoltaje = compBotonPulsado(bMenos, minVoltaje, multiplicador); 
          setValor("min_v", minVoltaje);
          tft.fillCircle(50, 35, 5,TFT_GREEN);
          break;
        case 1: 
          maxVoltaje = compBotonPulsado(bMenos, maxVoltaje, multiplicador);           
          voltaje_inicial = maxVoltaje;
          setValor("max_v", maxVoltaje);
          tft.fillCircle(240, 35, 5,TFT_GREEN);
          break;
        case 2: 
          minAmperios = compBotonPulsado(bMenos, minAmperios, multiplicador); 
          setValor("min_a", minAmperios);
          tft.fillCircle(50, 85, 5,TFT_GREEN);
          break;
        case 3: 
          maxAmperios = compBotonPulsado(bMenos, maxAmperios, multiplicador); 
          setValor("max_a", maxAmperios);
          tft.fillCircle(240, 85, 5,TFT_GREEN);
          break;
        case 4: 
          minTemp = compBotonPulsado(bMenos, minTemp, multiplicador); 
          setValor("min_t", minTemp);
          tft.fillCircle(50, 135, 5,TFT_GREEN);
          break;
        case 5:
          maxTemp = compBotonPulsado(bMenos, maxTemp, multiplicador); 
          setValor("max_t", maxTemp);
          tft.fillCircle(240, 135, 5,TFT_GREEN);
          break;
        case 6: 
          pulsosRpm = compBotonPulsado(bMenos, pulsosRpm, multiplicador); 
          setValor("pulsos", pulsosRpm);
          tft.fillCircle(13, 185, 5,TFT_GREEN);
          break;
        case 7: 
          minRpm = compBotonPulsado(bMenos, minRpm, multiplicador); 
          setValor("min_r", minRpm);
          tft.fillCircle(113, 185, 5,TFT_GREEN);
          break;
        case 8: 
          maxRpm = compBotonPulsado(bMenos, maxRpm, multiplicador); 
          setValor("max_r", maxRpm);
          tft.fillCircle(243, 185, 5,TFT_GREEN);
          break;
        case 9: 
          salto = compBotonPulsado(bMenos, salto, multiplicador); //saltos:8&p_medio:1276&min_bat:50&
          setValor("saltos", salto);
          tft.fillCircle(5, 235, 5,TFT_GREEN);
          break;
        case 10: 
          puntoMedio = compBotonPulsado(bMenos, puntoMedio, multiplicador); 
          setValor("p_medio", puntoMedio);
          tft.fillCircle(105, 235, 5,TFT_GREEN);
          break;
        case 11: 
          minimoPorCiento = compBotonPulsado(bMenos, minimoPorCiento, multiplicador); 
          setValor("min_bat", minimoPorCiento);
          tft.fillCircle(285, 235, 5,TFT_GREEN);
          break;
        case 12: 
          minTrabajo = compBotonPulsado(bMenos, minTrabajo, multiplicador); 
          setValor("min_w", minTrabajo);
          tft.fillCircle(50, 285, 5,TFT_GREEN);
          break;
        case 13: 
          maxTrabajo = compBotonPulsado(bMenos, maxTrabajo, multiplicador); 
          setValor("max_w", maxTrabajo);
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

  tft.setCursor(13, 210);
  tft.println("Salto y p. medio en mv:");

  tft.setCursor(15, 230);
  tft.print("S: ");
  tft.println(salto,0);
  

  tft.setCursor(115, 230);
  tft.print("Pmid:");
  tft.println(puntoMedio,0);

  tft.setCursor(290, 210);
  tft.println("Bateria:");
  
  tft.setCursor(295, 230);
  tft.print("Min%: ");
  tft.println(minimoPorCiento,0);


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
  
  tft.drawRect(0,255,220,70,TFT_RED);

  DateTime tiempo_vuelo = 1230768000 + (millis()/1000);//calculo de tiempo pasado
  
  tft.setTextSize(2);
  
  tft.setCursor(10, 200);  
  tft.println("Tiempo de Vuelo: ");
  tft.setCursor(10, 260);  
  tft.println("Hora actual:");



  
  /*=========================================================================================
                    HORA ACTUAL
    =========================================================================================*/
  tft.setCursor(30, 280);
  
  tft.setTextSize(3);

  if (rtc_clock.get_hours() < 10) {
    tft.print("0");
  }
  tft.print(rtc_clock.get_hours());
  
  tft.print(":");
  
  if (rtc_clock.get_minutes() < 10) {
    tft.print("0");
  }
  tft.print(rtc_clock.get_minutes());
  
  tft.print(":");
  
  if (rtc_clock.get_seconds() < 10) {
    tft.print("0");
  }
  tft.print(rtc_clock.get_seconds());
  
  tft.setCursor(0, 0);

  
  
  /*=========================================================================================
                    TIEMPO PASADO
    =========================================================================================*/
  tft.setCursor(30, 220);
  if (tiempo_vuelo.hour() < 10) {
    tft.print("0");
  }
  tft.print(tiempo_vuelo.hour());
  
  tft.print(":");
  
  if (tiempo_vuelo.minute() < 10) {
    tft.print("0");
  }
  tft.print(tiempo_vuelo.minute());
  
  tft.print(":");
  
  if (tiempo_vuelo.second() < 10) {
    tft.print("0");
  }
  tft.print(tiempo_vuelo.second());

}


// Submenu de Voltaje y RPM

void SensorStart() {

  //Secuencia de inicio del sensor

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print("Barometro: ");
  if (pressure.begin()){
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("OK");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
  } else  {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println("PROBLEMA");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
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
  
  tft.setTextColor(TFT_RED, TFT_BLACK);
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
        else tft.println("error en la lectura de presion\n");
      }
      else tft.println("error iniciando la lectura de presion\n");
    }
    else tft.println("error en la lectura de temperatura\n");
  }
  else tft.println("error iniciando la lectura de temperatura\n");
  tft.setTextColor(TFT_GREEN, TFT_BLACK);

}
/*
 void rpm_fun()
 {
   half_revolutions++;
 }
*/
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

  
int compBotonPulsado(int bMenos, float cantidad, int multiplicador){  
     return (bMenos == HIGH) ? (cantidad -= multiplicador) : (cantidad += multiplicador);
}
