// nrf24_reliable_datagram_server.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging server
// with the RHReliableDatagram class, using the RH_NRF24 driver to control a NRF24 radio.
// It is designed to work with the other example nrf24_reliable_datagram_client
// Tested on Uno with Sparkfun WRL-00691 NRF24L01 module
// Tested on Teensy with Sparkfun WRL-00691 NRF24L01 module
// Tested on Anarduino Mini (http://www.anarduino.com/mini/) with RFM73 module
// Tested on Arduino Mega with Sparkfun WRL-00691 NRF25L01 module

#include <RHReliableDatagram.h>
#include <RH_NRF24.h>
#include <SPI.h>
//#include <RHSoftwareSPI.h>

#define SERVER_ADDRESS 0

RH_NRF24 driver;//(7,53);
// RH_NRF24 driver(8, 7);   // For RFM73 on Anarduino Mini

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);

void setup() 
{
  Serial.begin(9600);
  delay(1000);
  
  boolean initR = manager.init();
  if (initR){
    Serial.println("HI;1");//success
  } else {
    while (!initR) {
      Serial.println("HI;0");//failed
      delay(5000);
      initR = manager.init();
    }
  }
}

void serialEvent() {
  processIncomingSerial();
  digitalWrite(13,HIGH);
  delay(500);
  digitalWrite(13,LOW);
}

//parsed must be large enough to hold the parsed string
void parseNextSegment(char *str, byte strLen, byte* index, char seperator, char* parsed){
  byte pIndex = 0;
  byte i = *index;
  
  //copy chars into string until 'separator' or terminator/newline is found
  while (str[i] != seperator && str[i] != '\0' && str[i] != '\r' && str[i] != '\n'
         && str[i] != 16 && str[i] != 14 && i < strLen){//NOTE added strlen test
//    Serial.println((char)str[i]);
    parsed[pIndex++] = str[i];
    i++;
  }
  parsed[pIndex] = '\0';
  
  //Serial.print(F("Parsed '"));  /* DEBUG */
  //Serial.print(parsed);  /* DEBUG */
  //Serial.print(F("' of '"));  /* DEBUG */
  //Serial.print(str);  /* DEBUG */
  //Serial.println(F("'"));  /* DEBUG */

  //advance index past 'seperator', in preparation for the next call to parseNextSegment()
  if (str[i] != '\0' && i < strLen){
    i++;
  }

  *index = i;
}

//returns 0 if succesfull, else an integer identifying the error
short processIncomingSerial(){
  //Serial.println(F("Processing incoming serial"));
  int MAX_CMD_LEN = 200;
  int receiver;
  char cmd[256] = "";
  char c;
  int i = 0;
  
  receiver = Serial.parseInt();
  Serial.read();//read past first semi-colon
  delay(10);
  while (Serial.available() > 0){
    if (i > MAX_CMD_LEN)
      return 2;
    delay(10); //prevent it from reading ahead of stream
    c = (char)Serial.read();
    //Serial.print(c);
    if (c == '\n' || c == '\r' || c == '\0')
      Serial.flush();
    else {
      cmd[i] = c;
      cmd[i+1] = '\0';
    }
    i++;
  }
  //Serial.println("compiled string:");
  //Serial.println(cmd);
  uint8_t uia[strlen(cmd)];
  for (int i = 0 ; i < strlen(cmd); i++){
    uia[i] = cmd[i];
  }
  if (manager.sendtoWait(uia, sizeof(uia), receiver)){
    //Serial.println("Success"); //Server only needs to know when a msg fails to send
  } else {
    char* TID = "\0";
    byte TIDi = 0;
    parseNextSegment(cmd,strlen(cmd),&TIDi,';',TID);
    Serial.print("TXFail:TID:");
    Serial.print(TID);
    Serial.print(":MID:");
    Serial.println(receiver);
  }
}

// Dont put this on the stack:
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];

void loop()
{
  if (manager.available())
  {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from))
    {
      String b = (char*)buf;
      if (b.startsWith("D:")){
        b.replace("D:","");
        String s = "D;" + String(from) + ";" + b;
        Serial.println(s);
      } else if (b.startsWith("N:")){
        b.replace("N:","");
        String s = "N;" + String(from) + ";" + b;
        Serial.println(s);
      } else
        Serial.println(String(from) + ";" + (char*)buf);
      for (byte b = 0; b < RH_NRF24_MAX_MESSAGE_LEN; b++)
        buf[b] = (uint8_t)'\0';
    }
  }
}
