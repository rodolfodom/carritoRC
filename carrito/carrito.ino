#include <ESP32_Servo.h>
#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>
#include <SPI.h>


RF24 radio(2, 5); 
const uint64_t address = 0xF0F0F0F0E1LL; 
float x, y;


struct Datos_xy{        //Se declara un objeto con 2 variables, x-y
  float x, y;
};

Datos_xy data;

Servo myservo;

int en1 = 16, m11 = 17, m12 = 21, m21 = 22, m22 = 23, en2 = 27, servoPin = 26;


void setup() {
  Serial.begin(115200);  
  radio.begin();
  radio.openReadingPipe(0, address);   //Setting the address at which we will receive the data
  radio.setPALevel(RF24_PA_MAX);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.startListening();

}



void servo(){
  myservo.attach(servoPin);  // conecta el servo en el pin 3 al objeto servo
  float datoX=data.x;// Se le da el valor de x a datoX
  int  val = map(datoX, -40, 40, 70, 180);     // (valor recibido, de minimo, de maximo, a minimo, a maximo)
  myservo.write(val);                        //establece la posición del servo de acuerdo con el valor escalado
  Serial.print("Rotacion en X:  ");
  Serial.print(datoX); 
  delay(15); 
}


void velocidad(){
  float datoY = data.y;
  Serial.print("tRotacion en Y: ");
  Serial.println(datoY);

  if(datoY>=-6&&datoY<=6){

    digitalWrite(m11,LOW);
    digitalWrite(m12,LOW);
    digitalWrite(m21,LOW);
    digitalWrite(m22,LOW);
    analogWrite(en1,0);
    analogWrite(en2,0);

  } else if(datoY>6&&datoY<=70){

    float vx= datoY * 2;
    digitalWrite(m11,LOW);
    digitalWrite(m12,HIGH);
    digitalWrite(m21,LOW);
    digitalWrite(m22,HIGH);
    analogWrite(en1,vx);
    analogWrite(en2,vx);

  } else if(datoY>70){
    
    digitalWrite(m11,LOW);
    digitalWrite(m12,HIGH);
    digitalWrite(m21,LOW);
    digitalWrite(m22,HIGH);
    analogWrite(en1,125);
    analogWrite(en2,125);

  } else if(datoY<6 && datoY>=-70){

    float vx=datoY*-2;
    digitalWrite(m11,HIGH);
    digitalWrite(m12,LOW);
    digitalWrite(m21,HIGH);
    digitalWrite(m22,LOW);
    analogWrite(en1,vx);
    analogWrite(en2,vx);
   
  } else if(datoY>-70){

    digitalWrite(m11,HIGH);
    digitalWrite(m12,LOW);
    digitalWrite(m21,HIGH);
    digitalWrite(m22,LOW);
    analogWrite(en1,125);
    analogWrite(en2,125);

  }
}

int recvData(){
  if (radio.available()) {
    
    radio.read(&data, sizeof(Datos_xy));
    return 1;
    
  }
    
  return 0;
}






void loop() {
  if(recvData()){
    servo();
    velocidad();
    
  }

}
