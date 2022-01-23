#include <SPI.h>
#include <LoRa.h>

#define LORA_PIN_SS     16 // pin 27, IO16
#define LORA_PIN_RESET  -1
#define LORA_PIN_DIO0   26 // pin 11, IO26

#define RXD1 25 // Serial port connected to LPC1768
#define TXD1 27

#define MILLIS_BETWEEN_PACKETS 750

int counter = 0;
int millisLastSerialByte;
int packetReceived;
char packet[256];   // packet length for LoRa < (52 | 223) depending on mode
int idxPacket;
  
void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);   
  
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa Sender");

  LoRa.setPins(LORA_PIN_SS, LORA_PIN_RESET, LORA_PIN_DIO0);
  if (!LoRa.begin(868.1E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  //LoRa.setTxPower(2); // 2 dB , valid values 2 - 20

  millisLastSerialByte = -1000000;
  packetReceived = 0;
  idxPacket = 0;


  // check packetChecksumIsOk(char *packet)
  packet[0] = 5;
  packet[1] = 1;
  packet[2] = 2;
  packet[3] = 3;
  packet[4] = 6;
  Serial.print("check packetChecksumIsOk (should return 1): ");
  Serial.println(packetChecksumIsOk(packet));

  packet[0] = 5;
  packet[1] = 1;
  packet[2] = 2;
  packet[3] = 3;
  packet[4] = 7;
  Serial.print("check packetChecksumIsOk (should return 0): ");
  Serial.println(packetChecksumIsOk(packet));

  delay(1000);
  while(Serial1.available() > 0){
      Serial1.read();
  }
  
}

int packetChecksumIsOk(char *packet){
  // first byte -> packet length
  // last byte -> checksum
  
  int packetLength = (int)packet[0];
  uint8_t sum = 0;
  for(int i=1; i<packetLength-1; i++){
    sum += (uint8_t)packet[i];
  }

  return sum == (uint8_t)packet[packetLength-1];
}

uint8_t packetChecksum(char *packet){
  // first byte -> packet length
  // last byte -> checksum
  
  int packetLength = (int)packet[0];
  uint8_t sum = 0;
  for(int i=1; i<packetLength-1; i++){
    sum += (uint8_t)packet[i];
  }

  return sum;
}

void loop() {
  int packetLength;

  if(Serial1.available() > 0){
    //Serial.println("Serial1 bytes available");
    if(millis() - millisLastSerialByte > MILLIS_BETWEEN_PACKETS){
      idxPacket = 0;
    }
    while(Serial1.available() > 0){
      packet[idxPacket] = (char)Serial1.read();
      idxPacket++;
    }
    millisLastSerialByte = millis();
    packet[idxPacket] = 0;  // to use string functions with packet
    //Serial.print("Received: ");
    //Serial.println(packet);
  }
  
  if(packet[0] <= idxPacket+1){ // first byte defines the packet length
    Serial.println("");
    Serial.print("Complete packet received: ");
    Serial.println(packet);
    Serial.print("packet length: ");
    Serial.println((int)packet[0]);
    Serial.print("packet indicated checksum: ");
    Serial.println((int)packet[packet[0]-1]);
    Serial.print("packet calculated checksum: ");
    Serial.println(packetChecksum(packet));
    if(packetChecksumIsOk(packet)){
      Serial.println("Checksum OK");
      packetReceived = 1;  
      packetLength = idxPacket;
    }else{
      Serial.println("Checksum NOT ok");
    }
    idxPacket = 0;
  }
  
  if(packetReceived){
    packetReceived = 0;
    digitalWrite(LED_BUILTIN, HIGH); 
  
    Serial.print("Sending packet: '");
    //Serial.print(packet);
    //Serial.println("'");
    for(int ii=0;ii<32;ii++){
      Serial.println((int)packet[ii]);
    }

    // encript packet
  
    // send packet
    LoRa.beginPacket();
    //LoRa.print(packet);
    LoRa.write((uint8_t *)packet, packetLength);
    LoRa.endPacket();
    
    digitalWrite(LED_BUILTIN, LOW); 
  }
  delay(100);
  
}
