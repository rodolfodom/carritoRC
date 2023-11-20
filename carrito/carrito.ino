#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>

#include <SPI.h>

RF24 radio(2, 5); 
const uint64_t address = 0xF0F0F0F0E1LL;  

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
  
  radio.begin();
  radio.openReadingPipe(0, address);   //Setting the address at which we will receive the data
  radio.setPALevel(RF24_PA_MAX);       //You can set this as minimum or maximum depending on the distance between the transmitter and receiver.
  radio.startListening();


}


int recvData()
{
  if ( radio.available() ) 
  {
    radio.read(&data, sizeof(MyData));
    return 1;
    }
    return 0;
}



void loop() {
  if(recvData()){
 
    Serial.println("Data Received:");
    Serial.print("Packet No. = ");
    Serial.println(data.counter);
    
    Serial.print("Temperature = ");
    Serial.print(data.temperature);
    Serial.println("*C");
  
    Serial.print("Pressure = ");
    Serial.print(data.pressure);
    Serial.println("hPa");
  
    Serial.print("Approx. Altitude = ");
    Serial.print(data.altitude);
    Serial.println("m");
  
    Serial.print("Humidity = ");
    Serial.print(data.humidity);
    Serial.println("%");
  
    Serial.println();
  }

}
