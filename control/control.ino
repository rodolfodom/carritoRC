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
MPU6050 sensor;
const uint64_t address = 0xF0F0F0F0E1LL;
int counter = 0;
struct MyData 
{
  int counter;
  float temperature;
  float humidity;
  float altitude;
  float pressure;
};

MyData data;

void setup() {
  Serial.begin(115200);

  radio.begin();                  //Starting the Wireless communication
  radio.openWritingPipe(address); //Setting the address where we will send the data
  radio.setPALevel(RF24_PA_MAX);  //You can set it as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.stopListening(); 

}

void loop() {
  data.counter = counter;
  data.temperature = 1;
  data.pressure = 2;
  data.altitude = 3;
  data.humidity = 4;

  Serial.print("Packet No. = ");
  Serial.println(data.counter);

  bool ok = radio.write(&data, sizeof(MyData));

  if(ok)
  {
     Serial.println("------------------- paquete enviado -------------"); 
  }
  else
  {
     Serial.println("********************** no se ha podido enviar *******************");
  }

  counter++;
  delay(5000);
}
