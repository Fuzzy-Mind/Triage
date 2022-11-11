#include "SoftwareSerial.h"

int incomingByte=0;

byte incomingPackage[64];

SoftwareSerial mySerial(3, 4); // RX, TX

void sendHandShake(){
  byte handShake[] = {0xfa, 0x0a, 0x02, 0x01, 0x01, 0x2f, 0x00, 0x00, 0x00, 0x3d};

  for(int i=0; i<sizeof(handShake); i++){
    mySerial.write(handShake[i]);
  }
}

void sendStartMeasurement(){
  byte startMeasurement[] = {0xfa, 0x0a, 0x02, 0x01, 0x21, 0x2f, 0x00, 0x00, 0x00, 0x5d};

  for(int i=0; i<sizeof(startMeasurement); i++){
    mySerial.write(startMeasurement[i]);
  }
}

void inquireTestResultsStatus(){
  byte testResultsStatus[] = {0xfa, 0x0a, 0x02, 0x02, 0x03, 0x2f, 0x00, 0x00, 0x00, 0x40};

  for(int i=0; i<sizeof(testResultsStatus); i++){
    mySerial.write(testResultsStatus[i]);
  }
}



void readPackage(){
  byte lengthPackage = 0;
  memset(incomingPackage, 0, 64);
  
  while(1){
    if(mySerial.available()>0){
      incomingByte = mySerial.read();
      if(incomingByte == 0xfa){
        //Serial.println("Start of Message");
        delay(1);
        if(mySerial.available()>0){
          incomingByte = mySerial.read();
          lengthPackage = incomingByte-2;
          //Serial.print("Length : ");
          //Serial.println(lengthPackage);
          mySerial.readBytes(incomingPackage, lengthPackage);
          /*for(int i=0; i<lengthPackage; i++){
            Serial.print(incomingPackage[i], HEX);
            Serial.print("/");
          }
          Serial.println("\n");*/
          if(incomingPackage[2] == 0x81){
            sendHandShake();
          }
          else if(incomingPackage[1] == 0x03 && incomingPackage[2] == 0x80 && incomingPackage[7] == 0x07){
            //Serial.println("Basarili");
            //delay(2000);
            sendStartMeasurement();
          }

          else if(incomingPackage[1] == 0x04 && incomingPackage[2] == 0x86 && incomingPackage[8] == 0x00){
            //Serial.println("********** Bitti *************************");
            inquireTestResultsStatus();
          }
          else if(incomingPackage[1] == 0x03 && incomingPackage[2] == 0x83 && incomingPackage[3] == 0x2f && incomingPackage[16] == 0x00){
            byte outgoingPackage[] = {0xf1, incomingPackage[7], incomingPackage[8], incomingPackage[9], incomingPackage[10],
                                      incomingPackage[11], incomingPackage[12], incomingPackage[13], incomingPackage[14], 0x1f};
            for(int i=0; i<10; i++){
              Serial.write(outgoingPackage[i]);
            }
          }
          else if(incomingPackage[1] == 0x03 && incomingPackage[2] == 0x83 && incomingPackage[3] == 0x2f && incomingPackage[16] != 0x00){
            byte outgoingPackage[] = {0xf1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e};
            for(int i=0; i<10; i++){
              Serial.write(outgoingPackage[i]);
            }
          }
        }
        break;
      }
    }
  }
}


void setup() {
  mySerial.begin(4800);
  Serial.begin(9600);
  
  //sendHandShake();
}

void loop() {
  readPackage();

}
