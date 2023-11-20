#include <MPU6050.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>
#include <SPI.h>

#define CE_PIN 9
#define CSN_PIN 10
RF24 radio(CE_PIN, CSN_PIN);
const uint64_t address = 0xF0F0F0F0E1LL;

struct Datos_xy{        //Se declara un objeto con 2 variables, x-y
  float x, y;
};

Datos_xy data;

MPU6050 sensor;

int ax, ay, az;         // Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int gx, gy, gz;

long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;



void obtener_xy(){ //tipo de dato: Datos_xy             Nombre de la función: obtener_xy
    
  sensor.getAcceleration(&ax, &ay, &az); // Leer las aceleraciones y velocidades angulares
  sensor.getRotation(&gx, &gy, &gz);
  
  dt = (millis()-tiempo_prev)/1000.0;
  tiempo_prev=millis();
  
  //Calcular los ángulos con acelerometro
  float accel_ang_x=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
  float accel_ang_y=atan(-ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
  
  //Calcular angulo de rotación con giroscopio y filtro complemento  
  ang_x = 0.98*(ang_x_prev+(gx/131)*dt) + 0.02*accel_ang_x;
  ang_y = 0.98*(ang_y_prev+(gy/131)*dt) + 0.02*accel_ang_y;
  
  
  ang_x_prev=ang_x;
  ang_y_prev=ang_y;

  //Mostrar los angulos separadas por un [tab]

  data.x=ang_x;     //Se asigna valor a x
  data.y=ang_y;     //Se asigna valor a y 
  delay(10);    //Se retorna objeto
    
}





void setup() {
  Serial.begin(9600);
  Wire.begin();
  sensor.initialize();

  while(!sensor.testConnection()){
    Serial.println("El giroscopio no se ha iniciado correctamente"); //Manda mensaje si el sensor no fue inicializado de forma correcta
    delay(5000);
  }

  Serial.begin("El giroscopio se inicio con exito");


  while(!radio.begin()){
    Serial.println("La comunicacion por Radio(NRF24L01) no se ha inciiado correctamente");
    delay(5000);
  }
  Serial.println("Comunicacion por radio iniciada correctamente");
  radio.openWritingPipe(address); //Setting the address where we will send the data
  radio.setPALevel(RF24_PA_MAX);  //You can set it as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.stopListening(); 
 
}

void loop() {
  

  bool ok = radio.write(&data, sizeof(Datos_xy));

  if(ok)
  {
     Serial.println("------------------- paquete enviado -------------"); 
  }
  else
  {
     Serial.println("********************** no se ha podido enviar *******************");
  }

  delay(150);
}
