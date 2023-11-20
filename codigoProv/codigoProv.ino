#include <Servo.h>

#include "I2Cdev.h"     // Librerias I2C para controlar el mpu6050
#include "MPU6050.h"    // la libreria MPU6050.h necesita I2Cdev.h, I2Cdev.h necesita Wire.h
#include "Wire.h"
                        // La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
                        // del estado de AD0. Si no se especifica, 0x68 estará implicito
Servo myservo;
int val;

MPU6050 sensor;

int en1=11,m11=10,m12=8,m21=7,m22=6,en2=5;//CARRO. Se declaran las entradas y pwm de los motores
int ax, ay, az;         // Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int gx, gy, gz;

long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

struct Datos_xy{        //Se declara un objeto con 2 variables, x-y
  float x, y;
};
Datos_xy obtener_xy(){ //tipo de dato: Datos_xy             Nombre de la función: obtener_xy
  Datos_xy misDatos;     //Se declara objeto de tipo Datos_xy
    // Leer las aceleraciones y velocidades angulares
  sensor.getAcceleration(&ax, &ay, &az);
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

    misDatos.x=ang_x;     //Se asigna valor a x
    misDatos.y=ang_y;     //Se asigna valor a y 
     delay(10);
    return misDatos;     //Se retorna objeto
    
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);    //Iniciando puerto serial
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor
  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");//Comprueba conexión del sensor
  else Serial.println("Error al iniciar el sensor"); //Manda mensaje si el sensor no fue inicializado de forma correcta

}
void servo(){
         myservo.attach(9);  // conecta el servo en el pin 3 al objeto servo
  Datos_xy resultado=obtener_xy(); // Se declara objeto de tipo Datos_xy y se le pasan los valores de la función a resultado
  float datoX=resultado.x;// Se le da el valor de x a datoX
  int  val = map(datoX, -40, 40, 70, 180);     // (valor recibido, de minimo, de maximo, a minimo, a maximo)
    myservo.write(val);                        //establece la posición del servo de acuerdo con el valor escalado
    Serial.println(val);
    delay(15); 
}
void loop() {

  float vx;
  // put your main code here, to run repeatedly:
Datos_xy resultado=obtener_xy(); // Se declara objeto de tipo Datos_xy y se le pasan los valores de la función a resultado
float datoX=resultado.x;// Se le da el valor de x a datoX
float datoY=resultado.y;//Se le da valor de y a datoY
  Serial.print("Rotacion en X:  ");
  Serial.print(datoX); 
  Serial.print("tRotacion en Y: ");
  Serial.println(datoY);
  ///++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++CARRO RON RON 
      if(datoY>=-6&&datoY<=6){
    servo();
    digitalWrite(m11,LOW);
    digitalWrite(m12,LOW);
    digitalWrite(m21,LOW);
    digitalWrite(m22,LOW);
    analogWrite(en1,0);
    analogWrite(en2,0);
    }
   else if(datoY>6&&datoY<=70){
        servo();
    vx=datoY*2;
    digitalWrite(m11,LOW);
    digitalWrite(m12,HIGH);
    digitalWrite(m21,LOW);
    digitalWrite(m22,HIGH);
    analogWrite(en1,vx);
    analogWrite(en2,vx);
   }
   else if(datoY>70){
     servo();
    digitalWrite(m11,LOW);
    digitalWrite(m12,HIGH);
    digitalWrite(m21,LOW);
    digitalWrite(m22,HIGH);
    analogWrite(en1,125);
    analogWrite(en2,125);
   }
else if(datoY<6 && datoY>=-70){
       servo();
    vx=datoY*-2;
    digitalWrite(m11,HIGH);
    digitalWrite(m12,LOW);
    digitalWrite(m21,HIGH);
    digitalWrite(m22,LOW);
    analogWrite(en1,vx);
    analogWrite(en2,vx);
   }
   else if(datoY>-70){
        Serial.println("cara dep erro");
        servo();
    digitalWrite(m11,HIGH);
    digitalWrite(m12,LOW);
    digitalWrite(m21,HIGH);
    digitalWrite(m22,LOW);
    analogWrite(en1,125);
    analogWrite(en2,125);
   }
 //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++CARRO RON RON

delay(10);
}