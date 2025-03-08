#include <SPI.h>
#include <LoRa.h>

/*************LORA PINS INIT***************************************************************/
#define SCK 12
#define MISO 13
#define MOSI 11
#define SS 10
#define RST 8
#define DIO0 21 
/******************************************************************************************/


#define DATARECEIVERLED 5  //led connected to this pin to simulate ackowledgement

void setup() {
  
  Serial.begin(115200);
  
  pinMode(DATARECEIVERLED,OUTPUT);
  digitalWrite(DATARECEIVERLED,LOW);

  /****setting Lora parameters**************************/
  LoRa.setPreambleLength(12);
  LoRa.setCodingRate(4);
  LoRa.setSignalBandwidth(250000);
  LoRa.setSpreadingFactor(8);
  
  while (!Serial);
    LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(433.5E6)) {
    while (1); 
  }
}


void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String receivedMessage = "";
    while (LoRa.available()) {
      receivedMessage += (char)LoRa.read();
      
   }
   Serial.println(receivedMessage);

    Serial.print("RSSI : ");
    Serial.println(LoRa.packetRssi()); // Indicates  signal power
    Serial.print("SNR : ");
    Serial.println(LoRa.packetSnr());  // Indicates SNR

    digitalWrite(DATARECEIVERLED,HIGH);
    delay(100); // little delay to avoid conflits
    
  }
  else{digitalWrite(DATARECEIVERLED,LOW);}
}
 


