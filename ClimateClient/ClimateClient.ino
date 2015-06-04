// -*- mode: C++ -*-

//Includes
#include <MemoryFree.h>
#include <DHT.h>
#include <RHReliableDatagram.h>
#include <RH_NRF24.h>
#include <SPI.h>
#include <EEPROM.h>

//LED Pins
#define PIN_DHT_READING 4
#define PIN_HEARTBEAT 5
#define PIN_NRF_TRANSMITTING 6
#define PIN_GEN_ERR 7
//pin connected to the setup button
#define PIN_SETUP 9

#define EE_ID 0 //ID0 is server; ID1-253 valid client IDs; ID254 reserved for unitialized clients; ID255 invalid/EEPROM default after startup
#define EE_TK 1 //valid range for tokens 0-254; Token 255 is invalid/EEPROM default

//Initialize DHT sensor with data pin 2
DHT dht(2, DHT22);

//NRF setup
#define SERVER_ADDRESS 0
#define NRF_MAX_RETRIES 4
RH_NRF24 driver(8,10);
RHReliableDatagram *manager = new RHReliableDatagram(driver,EEPROM.read(EE_ID));

//Transaction history/memory setup
#define TRAN_HIST_LEN 15
#define TRAN_HIST_WORD_SIZE 5 //includes NULL terminator
char tranHistory[TRAN_HIST_LEN][TRAN_HIST_WORD_SIZE] = {};
byte tranHistIndex = 0;

#define PENDING_RESPONSE_COUNT 10
#define PENDING_RESPONSE_TIMEOUT 10000
#define PENDING_RESPONSE_MAX_RETRIES 3
char pendingTransactions[PENDING_RESPONSE_COUNT][RH_NRF24_MAX_MESSAGE_LEN+1] = {};
long pendingTransactionMillis[PENDING_RESPONSE_COUNT] = {};
byte pendingTransactionTranCount[PENDING_RESPONSE_COUNT] = {};
byte nextTID = 0;
byte limitTID = 1000; //exclusive limit

//Error codes
const byte ERR_INVALID_VAL_RECIEVED[2] = {1,1};
const byte ERR_NRF_FAILSEND[2] = {2,0};
const byte ERR_NRF_MSG_TOOLONG[2] = {2,1};
const byte ERR_NRF_FAILEDINIT[2] = {2,2};

//main loop vars
//set to true when hub says ok to start working
boolean okToStart = false;
#define LOOP_INIT 12
#define LOOP_HALT 11
byte loopState;
boolean firstRead = true;
unsigned long previousDHTReadMillis = 0;
unsigned long previousBeginRequest = -100000;
double tempTotal, humTotal, currentReadCount; 

//vars
long readDelay = 3 * 1000;
byte readCount = 2;

/*
*  TODO
 *    Minimum read delay, or fix to allow reset delay after it is set low
 *    Try atoi
 */


/*
 * Commands this program accepts:
 *   Set read delay:    S;R_DLY,<Millis>   -> SR;R_DLY,<intVal>
 *   Set read count:    S;R_CNT,<Int>      -> SR;R_CNT,<intVal>
 *   Set ID             S;ID,<oldToken>,<newID>,<newToken>(cannot assign ID 0, 254, or 255)
 *                                         -> SR;ID,<oldId>,<oldToken>,<newId>,<newToken>
 *   ---
 *   Get read delay:    G;R_DLY -> GR;R_DLY,<intVal>
 *   Get read count:    G;R_CNT -> GR;R_CNT,<intVal>
 *   Get token          G;T     -> GR;T,<bVal>
 *   Get ID             G;ID   -> GR;ID,<bVal>,<bToken>
 *   Get Status(alive?) G;S     -> GR;S,1,<ID>,<Token>
 *   ---
 *   Reset token*       C;RT    -> CR;RT,<bToken>
 *   Setup(set id 254)  C;SETUP -> CR;SETUP,1
 *   Begin Exec         C;BEGIN -> CR;BEGIN,1
 *   Halt Exec          C;HALT  -> CR;HALT,1
 */

void setup(){
  Serial.begin(9600);
  
  for (byte b = 0; b < PENDING_RESPONSE_COUNT; b++){
    pendingTransactionMillis[b] = 0;
  }
  
  //if setup button is being pressed, enable setup mode
  if (digitalRead(PIN_SETUP) == HIGH || constrain(EEPROM.read(EE_ID),1,253) != EEPROM.read(EE_ID)){
    Serial.println(F("Setup mode activated"));
    loopState = LOOP_INIT;
    EEPROM.write(EE_ID,254);
  } else {
    loopState = 0;
  }
  
  //Init serial and random funcs
  Serial.println(F("Initializing EEPROM and Random func"));
  randomSeed(analogRead(0) * analogRead(1) * analogRead(2));
  if (EEPROM.read(EE_ID) == 255)//first run
    EEPROM.write(EE_ID,254);//254 is the 'I need an ID' address
  if (EEPROM.read(EE_TK) == 255)//if token not initialized
    EEPROM.write(EE_TK,random(255));
  randomSeed(analogRead(0) * analogRead(1) * analogRead(2) * analogRead(3));   


  initNRF();
  
}

//codes: {shortCount, longCount}
void flashErrorCode(byte pin, const byte* code){
  Serial.print(F("Flashing error code "));
  Serial.print(code[0]);
  Serial.println(code[1]);
  for (byte b = 0 ; b < code[0]; b++){
    delay(250);
    digitalWrite(pin,HIGH);
    delay(250);
    digitalWrite(pin,LOW);
  }
  delay(1000);
  for (byte b = 0 ; b < code[1]; b++){
    delay(750);
    digitalWrite(pin,HIGH);
    delay(750);
    digitalWrite(pin,LOW);
  }
}

void perpetualErrorCode(byte pin, const byte* code){
  while(true){
    flashErrorCode(pin,code);
    delay(2000);
  }
}

boolean transmit(char* msg){
  return transmit(msg, false);
}

boolean transmit(char* msg, boolean waitforResponse){
  Serial.print(F("Transmitting: "));  /* DEBUG */
  Serial.print(msg);  /* DEBUG */
  Serial.print(F(" "));  /* DEBUG */
  for (byte i = 0; i < NRF_MAX_RETRIES; i++){
    if (i == 0) {
      if (_transmit(msg, waitforResponse))
        return true;
    } else {
      Serial.print(F("  Retransmitting... "));  /* DEBUG */
      if (_transmit(msg, waitforResponse))
        return true;
    }
  }
  return false;
}

boolean _transmit(char* msg, boolean waitforResponse){
  digitalWrite(PIN_NRF_TRANSMITTING,HIGH);
  //send data, return success or no
  if (strlen(msg) >= RH_NRF24_MAX_MESSAGE_LEN){
    Serial.print(F("TX too long ("));  /* DEBUG */
    Serial.print(strlen(msg));  /* DEBUG */
    Serial.println(F(")"));  /* DEBUG */
    flashErrorCode(PIN_GEN_ERR,ERR_NRF_MSG_TOOLONG);
    return false;
  }
  boolean result = (*manager).sendtoWait((uint8_t*)msg, strlen(msg), SERVER_ADDRESS);
  if (result){
    Serial.println(F("Successful"));  /* DEBUG */
    //find an unused transaction response slot
    if (waitforResponse){
      byte b;
      for (b = 0; b < PENDING_RESPONSE_COUNT; b++){
        if (pendingTransactionMillis[b] == 0)
          break;
      }
      if (b != PENDING_RESPONSE_COUNT){
        Serial.print(F("Recording transmission at ID "));  /* DEBUG */
        Serial.println(b);  /* DEBUG */
        strcpy(pendingTransactions[b],msg);
        unsigned long tranTime = millis();
        if (tranTime == 0)
          tranTime = 1;
        pendingTransactionMillis[b] = tranTime;
        pendingTransactionTranCount[b] = 0;
      } else {
        Serial.print(F("No available pending response IDS!"));  /* DEBUG */
      }
    }
  } else {
    Serial.println(F("Failed"));  /* DEBUG */
    flashErrorCode(PIN_GEN_ERR,ERR_NRF_FAILSEND);
  }
  digitalWrite(PIN_NRF_TRANSMITTING,LOW);
  return result;
}

void checkTransactionTimeOuts(){
  byte b;
  for (b = 0; b < PENDING_RESPONSE_COUNT; b++){
    if (pendingTransactionMillis[b] != 0
        && millis() - pendingTransactionMillis[b] > PENDING_RESPONSE_TIMEOUT){
      if (pendingTransactionTranCount[b] >= PENDING_RESPONSE_MAX_RETRIES){
        Serial.print(F("Server failed to respond to the following transmition at slot "));  /* DEBUG */
        Serial.print(b);  /* DEBUG */
        Serial.println(F(" and its retransmissions: "));  /* DEBUG */
        Serial.print(F(" "));  /* DEBUG */
        Serial.println(pendingTransactions[b]);  /* DEBUG */
        pendingTransactionMillis[b] = 0;
        strcpy(pendingTransactions[b],"");
      } else {
        Serial.print(F("Transaction at slot "));  /* DEBUG */
        Serial.print(b);  /* DEBUG */
        Serial.println(F(" timed out, retransmitting it"));  /* DEBUG */
        transmit(pendingTransactions[b]);
        pendingTransactionTranCount[b] = pendingTransactionTranCount[b] + 1;
        pendingTransactionMillis[b] = millis();
      }
    }
  }
}

void clearPendingTransactions(){
  for (byte b = 0; b < PENDING_RESPONSE_COUNT; b++)
    pendingTransactionMillis[b] = 0;
}

byte getNextTID(){
  nextTID = ++nextTID % limitTID;
  return nextTID;
}

void addToTranHistory(char TID[]){
  strcpy(tranHistory[tranHistIndex++],TID);
  tranHistIndex %= TRAN_HIST_LEN;
}

boolean tranHistoryContains(char* TID){
  for (byte i = 0; i < TRAN_HIST_LEN; i++)  {
    if (strcmp(TID,tranHistory[i]) == 0)    {
      return true;
    }
  }
  return false;
}

void genToken(){
  EEPROM.write(EE_TK,random(255) + 1);
  Serial.print(F("Wrote new token: "));  /* DEBUG */
  Serial.println(EEPROM.read(EE_TK));  /* DEBUG */
}

boolean initNRF(){
  delete manager;
  manager = new RHReliableDatagram(driver,EEPROM.read(EE_ID));//255 is the ID before init
  if ((*manager).init()){
    Serial.print(F("Init success. ID '"));  /* DEBUG */
    Serial.print(EEPROM.read(EE_ID));  /* DEBUG */
    Serial.print(F("', TK '"));  /* DEBUG */
    Serial.print(EEPROM.read(EE_TK));  /* DEBUG */
    Serial.println(F("'"));  /* DEBUG */
    return true;
  } else {
    Serial.println(F("Init failed"));  /* DEBUG */
    flashErrorCode(PIN_GEN_ERR,ERR_NRF_FAILEDINIT);
    return false;
  }
}  

boolean reinitNRF(int newID){
  Serial.print(F("Re-Initializing NRF w/ID "));  /* DEBUG */
  Serial.println(newID);  /* DEBUG */
//  Serial.print(F("Manager size before deletion: "));  /* DEBUG */
//  Serial.println(sizeof(manager));  /* DEBUG */
  delete manager;  
  manager = new RHReliableDatagram(driver,newID);//255 is the ID before init
//  Serial.print(F("Manager size after creation: "));  /* DEBUG */
//  Serial.println(sizeof(manager));  /* DEBUG */
  return (*manager).init();
}


uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];

//returns true if a command was recieved(regardless of whether it was valid)
boolean checkNRF() {
  uint8_t len = sizeof(buf);
  uint8_t from;
  if ((*manager).recvfromAck(buf, &len, &from)) {
    buf[len] = '\0';//Testing
    Serial.print(F("NRF rx: "));  /* DEBUG */
    Serial.println((char*)buf);  /* DEBUG */
    processRequest((char*)buf,len);
    return true;
  } else {
    return false;
  }
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

int strToInt(char* num){
  int result = 0;
  byte i = 0;
  while (num[i] != '\0'
           && num[i] != 16
           && num[i] != 14
           && num[i] != '\r'
           && num[i] != '\n'){
    result = result * 10;
    result = result + num[i] - 48;
    //Serial.println((byte)num[i]);
    
    i++;
  }
  
  return result;
}

byte intLen(double i){
  byte reps = 0;
  while (floor(i) > 1) {
    i = i / 10.0;
    reps++;
  }
}

void processRequest(char* str, int strLen){
  Serial.print(F("Processing '"));  /* DEBUG */
  Serial.print(str);  /* DEBUG */
  Serial.print(F("'("));  /* DEBUG */
  Serial.print(strLen);  /* DEBUG */
  Serial.println(F(")"));  /* DEBUG */

  byte pIndex = 0;//int for parsing function to use
  char TID[6] = "";
  char cType[4] = "";
  char cmd[30] = "";
  
  parseNextSegment(str,strLen,&pIndex,';',TID);
  
  /* SERIAL COMMANDS */
  if (strcmp(TID,".phist") == 0){
    Serial.println(F("Transaction History:"));
    for (byte i = 0; i < TRAN_HIST_LEN; i++){
     if (strlen(tranHistory[i]) > TRAN_HIST_WORD_SIZE + 1)
       continue;
     else
       Serial.println(tranHistory[i]);
    }
    return;
  } else if (strcmp(TID,".status") == 0){
    Serial.println(F("Module Status:"));
    Serial.print(F(" ID: "));
    Serial.println(EEPROM.read(EE_ID));
    Serial.print(F(" TK: "));
    Serial.println(EEPROM.read(EE_TK));
    Serial.print(F(" Loopstate: "));
    Serial.println(loopState);
    Serial.print(F(" R_DLY: "));
    Serial.println(readDelay);
    Serial.print(F(" R_CNT: "));
    Serial.println(readCount);
    return;
  } else if (strcmp(TID,".tx") == 0){
    parseNextSegment(str,strLen,&pIndex,'\0',cmd);
    transmit(cmd);
    return;
  } else if (strcmp(TID,".setloop") == 0){
    parseNextSegment(str,strLen,&pIndex,';',cmd);
    loopState = atoi(cmd);
    Serial.print(F("Loop state set to "));
    Serial.println(loopState);
    return;
  }/* */
  
  parseNextSegment(str,strLen,&pIndex,';',cType);
  parseNextSegment(str,strLen,&pIndex,',',cmd);

  //response to server 
  Serial.print(F("TID: "));  /* DEBUG */
  Serial.println(TID);  /* DEBUG */
  Serial.print(F("cType: "));  /* DEBUG */
  Serial.println(cType);  /* DEBUG */
  Serial.print(F("cmd: "));  /* DEBUG */
  Serial.println(cmd);  /* DEBUG */
  
  //make sure we dont act on a request/command multiple times
  if (tranHistoryContains(TID)){
    Serial.print(F("Received duplicate transmission with TID '"));  /* DEBUG */
    Serial.print(TID);  /* DEBUG */
    Serial.println(F("'"));  /* DEBUG */
    return;
  } else {
    Serial.print(F("Adding '"));  /* DEBUG */
    Serial.print(TID);  /* DEBUG */
    Serial.println(F("' to history"));  /* DEBUG */
    addToTranHistory(TID);
  }

  //Check for responses from server
  if (strcmp(cType,"DR") == 0 || strcmp(cType,"NR") == 0){
    byte bi;
    char rTID[4] = "";
    for (byte b = 0 ; b < PENDING_RESPONSE_COUNT; b++){
      bi = 6;
      parseNextSegment(pendingTransactions[b],strlen(pendingTransactions[b]),&bi,';',rTID);
      /*Serial.print(F("::Identified tid "));
      Serial.print(rTID);
      Serial.print(F(" from '"));
      Serial.print(pendingTransactions[b]);
      Serial.println(F("'"));/* */
      
      if (strcmp(cmd,rTID) == 0){
        Serial.print(F("Matched server response with transaction at slot "));  /* DEBUG */
        Serial.print(b);  /* DEBUG */
        Serial.print(F("("));  /* DEBUG */
        Serial.print(pendingTransactions[b]);  /* DEBUG */
        Serial.println(F(")"));  /* DEBUG */
        pendingTransactionMillis[b] = 0;
        strcpy(pendingTransactions[b],"");
        return;
      }
    }
  }
  
  if (strcmp(cType,"S") == 0){
    if (strcmp(cmd,"R_DLY") == 0){
      Serial.println(F("Setting read delay"));  /* DEBUG */
      char val[8] = "";
      parseNextSegment(str,strLen,&pIndex,';',val);
      int newDelay = strToInt(val);
      if (newDelay == 0){ //invalid integer TODO send message saying invalid delay recieved
        flashErrorCode(PIN_GEN_ERR,ERR_INVALID_VAL_RECIEVED);
        Serial.print(F("invalid int read from S;R_DLY: "));  /* DEBUG */
        Serial.println(val);  /* DEBUG */
      } else {
        readDelay = newDelay;
        
        char str[25] = "";
        char dly[7] = "";
        itoa(readDelay,dly,10);
        
        strcat(str,"DHT;");
        strcat(str,TID);
        strcat(str,";SR;R_DLY,");
        strcat(str,dly);
        transmit(str);
          
        //transmit("DHT;" + String(TID) + ";SR;R_DLY," + String(readDelay));
      }
    } else if (strcmp(cmd,"R_CNT") == 0){
      Serial.println(F("Setting read count"));  /* DEBUG */
      char val[8] = "";
      parseNextSegment(str,strLen,&pIndex,';',val);

      int newcount = strToInt(val);
      if (newcount == 0){ //invalid integer TODO send message saying invalid delay recieved
        flashErrorCode(PIN_GEN_ERR,ERR_INVALID_VAL_RECIEVED);
        Serial.print(F("invalid int read from S;R_CNT: "));  /* DEBUG */
        Serial.println(val);  /* DEBUG */
      } else {
        readCount = newcount;
        char str[25] = "";
        char rc[4] = "";
        itoa(readCount,rc,10);
        strcat(str,"DHT;");
        strcat(str,TID);
        strcat(str,";SR;R_CNT,");
        strcat(str,rc);
        transmit(str);
      }
    } else if (strcmp(cmd,"ID") == 0){
      Serial.println(F("Setting ID"));  /* DEBUG */
      char oldToken[4] = "";
      char val[4] = "";
      char token[4] = "";
      parseNextSegment(str,strLen,&pIndex,',',oldToken);
      parseNextSegment(str,strLen,&pIndex,',',val);
      parseNextSegment(str,strLen,&pIndex,',',token);
      Serial.print(F("oldToken: "));  /* DEBUG */
      Serial.println(oldToken);  /* DEBUG */
      Serial.print(F("val: "));  /* DEBUG */
      Serial.println(val);  /* DEBUG */
      Serial.print(F("token: "));  /* DEBUG */
      Serial.println(token);  /* DEBUG */
      
      //test whether an int was supplied as token, and whether it matches the stored token
      boolean validVal = true;//True until otherwise
      if (strToInt(oldToken) == EEPROM.read(EE_TK)){
        for (int i = 0; i < strlen(val); i++)
          if (!isDigit(val[i])){
            validVal = false;
            Serial.println(F("invalid ID supplied to set ID"));  /* DEBUG */
            flashErrorCode(PIN_GEN_ERR,ERR_INVALID_VAL_RECIEVED);
            break;
          }

        //if new ID is valid, assign to this unit(must be < 256)
        if (validVal){
          byte nID = strToInt(val);
          byte nToken = strToInt(token);
          if (constrain(nID,1,253) != nID || nToken == 255){
            flashErrorCode(PIN_GEN_ERR,ERR_INVALID_VAL_RECIEVED);
            return; //invalid token or id to set
          }
          bool initSuccess = reinitNRF(nID);
          if (!initSuccess){
            Serial.println(F("Failed to reinitialize NRF"));     /* DEBUG */
            perpetualErrorCode(PIN_GEN_ERR,ERR_NRF_FAILEDINIT);
          }
          
            
          char str[25] = "";
          char oid[4] = "";
          char otk[4] = "";
          char id[4] = "";
          char tk[4] = "";
          
          itoa(EEPROM.read(EE_ID),oid,10);
          itoa(EEPROM.read(EE_TK),otk,10);
          Serial.println(F("Succesfully reinitialized NRF"));  /* DEBUG */
          EEPROM.write(EE_ID,nID); 
          EEPROM.write(EE_TK,nToken);
          
          itoa(EEPROM.read(EE_ID),id,10);
          itoa(EEPROM.read(EE_TK),tk,10);
          strcat(str,"DHT;");
          strcat(str,TID);
          strcat(str,";SR;ID,");
          strcat(str,oid);
          strcat(str,",");
          strcat(str,otk);
          strcat(str,",");
          strcat(str,id);
          strcat(str,",");
          strcat(str,tk);
          transmit(str);
          
          Serial.print(F("Assigned ID: "));  /* DEBUG */
          Serial.println(EEPROM.read(EE_ID));  /* DEBUG */
          Serial.print(F("Assigned Token: "));  /* DEBUG */
          Serial.println(EEPROM.read(EE_TK));  /* DEBUG */
          
          Serial.println(F("Setting loop state to LOOP_INIT"));
          loopState = LOOP_INIT;
        }
      } else {
        Serial.println(F("invalid token supplied to set id <val> <token>"));  /* DEBUG */
      }
    }
  } 
  else if (strcmp(cType,"G") == 0) {
    if (strcmp(cmd,"R_DLY") == 0){
      Serial.println(F("Getting readDelay val to hub"));  /* DEBUG */
      char str[30] = "";
      char dly[8] = "";
      itoa(readDelay,dly,10);
      strcat(str,"DHT;");
      strcat(str,TID);
      strcat(str,";GR;R_DLY,");
      strcat(str,dly);
      transmit(str);
    } else if (strcmp(cmd,"R_CNT") == 0){
      Serial.println(F("Getting readCount val to hub"));  /* DEBUG */
      char str[25] = "";
      char cnt[6] = "";
      itoa(readCount,cnt,10);
      strcat(str,"DHT;");
      strcat(str,TID);
      strcat(str,";GR;R_CNT,");
      strcat(str,cnt);
      transmit(str);
    } else if (strcmp(cmd,"ID") == 0){
      Serial.println(F("Getting ID to hub"));  /* DEBUG */
      char str[28] = "";  
      char id[4] = "";
      char tk[4] = "";
      itoa(EEPROM.read(EE_ID),id,10);
      itoa(EEPROM.read(EE_TK),tk,10);
      strcat(str,"DHT;");
      strcat(str,TID);
      strcat(str,";GR;ID,");
      strcat(str,id);
      strcat(str,",");
      strcat(str,tk);
      transmit(str);
    } else if (strcmp(cmd,"T") == 0){
      Serial.println(F("Getting token to hub"));  /* DEBUG */
      char str[25] = "";
      char tk[4] = "";
      itoa(EEPROM.read(EE_TK),tk,10);
      strcat(str,"DHT;");
      strcat(str,TID);
      strcat(str,";GR;T,");
      strcat(str,tk);
      transmit(str);
    } 
    else if (strcmp(cmd,"S") == 0){
      Serial.println(F("Getting status to hub"));  /* DEBUG */
      char str[25] = "";
      char id[4] = "";
      char tk[4] = "";
      itoa(EEPROM.read(EE_ID),id,10);
      itoa(EEPROM.read(EE_TK),tk,10);
      strcat(str,"DHT;");
      strcat(str,TID);
      strcat(str,";GR;S,1,");
      strcat(str,id);
      strcat(str,",");
      strcat(str,tk);
      transmit(str);
    }
  } else if (strcmp(cType,"C") == 0) {
    if (strcmp(cmd,"RT") == 0){
      Serial.println(F("commanded to reset token"));  /* DEBUG */
      genToken();
      char str[25] = "";
      char tk[4] = "";
      itoa(EEPROM.read(EE_TK),tk,10);
      strcat(str,"DHT;");
      strcat(str,TID);
      strcat(str,";CR;T,");
      strcat(str,tk);
      transmit(str);
    } else if (strcmp(cmd,"SETUP") == 0){//TODO maybe include token as begin param to avoid confusion?
      Serial.println(F("commanded to enter setup mode"));  /* DEBUG */
      char str[25] = "";
      char oid[4] = "";
      char otk[4] = "";
      itoa(EEPROM.read(EE_ID),oid,10);
      itoa(EEPROM.read(EE_TK),otk,10);
      EEPROM.write(EE_ID,254);
      reinitNRF(254);
      loopState = LOOP_INIT;
      strcat(str,"DHT;");
      strcat(str,TID);
      strcat(str,";CR;SETUP,");
      strcat(str,oid);
      strcat(str,",");
      strcat(str,otk);
      transmit(str);
      Serial.println(F("Setting loop state to LOOP_INIT"));
      loopState = LOOP_INIT;
    } else if (strcmp(cmd,"HALT") == 0){//TODO maybe include token as begin param to avoid confusion?
      Serial.println(F("commanded to wait for server"));  /* DEBUG */
      char str[25] = "";
      loopState = LOOP_HALT;
      clearPendingTransactions();
      strcat(str,"DHT;");
      strcat(str,TID);
      strcat(str,";CR;HALT,1");
      transmit(str);
    } else if (strcmp(cmd,"BEGIN") == 0){//TODO maybe include token as begin param to avoid confusion?
      Serial.println(F("commanded to begin"));  /* DEBUG */
      char str[25] = "";
      loopState = 0;
      strcat(str,"DHT;");
      strcat(str,TID);
      strcat(str,";CR;BEGIN,1");
      transmit(str);
    }
  }
}

void serialEvent(){
  char buf[20] = "";
  int i;
  for (i = 0; i < 19 && Serial.available(); i++){
    buf[i] = Serial.read();
    if (buf[i] == '\0')
      break;
    delay(10);
  }
  if (buf[i] != '\0')
    buf[i] = '\0';
  Serial.print(F("Rx Serial: "));  /* DEBUG */
  Serial.println(buf);  /* DEBUG */
  processRequest(buf,strlen(buf)+1);  
}

void loop() {
  checkTransactionTimeOuts();
  while (checkNRF()) {}
  
  if (loopState != LOOP_INIT && loopState != LOOP_HALT && (EEPROM.read(EE_ID) >= 254)) {
    Serial.println(F("Loop state not LOOP_INIT or LOOP_HALT, but ID is >= 254, resetting loopstate to LOOP_INIT"));  /* DEBUG */
    loopState = LOOP_INIT;
  }
  
  /*
  * Loop States
   *  0- Loop Start - Heartbeat
   *  1- Read DHT
   *  2- Send data
   */
  switch(loopState){
  case LOOP_INIT: //Server has not acknowledged this module yet
    if ((millis() - previousBeginRequest) > 30000){
      char toSend[15] = "";
      char tk[4] = "";
      itoa(EEPROM.read(EE_TK),tk,10);
      strcat(toSend,"N:DHT;I;");
      strcat(toSend,tk);
      transmit(toSend);
      previousBeginRequest = millis() - random(5000);
    }
    break;
  case LOOP_HALT: //The server has not allowed this module to begin executing yet
    delay(10);
    break;
  case 0: //Loop Start - Heart beat
    Serial.println(F("Loop starting"));
    digitalWrite(PIN_HEARTBEAT, HIGH);
    delay(100);
    digitalWrite(PIN_HEARTBEAT, LOW);
    delay(200);   

    digitalWrite(PIN_HEARTBEAT, HIGH);
    delay(100);
    digitalWrite(PIN_HEARTBEAT, LOW);
    delay(200);   

    loopState = 1; //progress forward;
    break;
  case 1: // Read DHT
    //if this is the first iteration, do prep
    if (firstRead){
      firstRead = false;
      previousDHTReadMillis = millis();
      tempTotal = 0;//reset globals
      humTotal = 0;
      currentReadCount = 0;
    }
    //only continue if readDelay has elapsed
    if ((unsigned long)(millis() - previousDHTReadMillis) > readDelay){
      digitalWrite(PIN_DHT_READING,HIGH);
      tempTotal += dht.readTemperature(true);    
      humTotal += dht.readHumidity();
      digitalWrite(PIN_DHT_READING,LOW);
      currentReadCount++;
      previousDHTReadMillis = millis();
    }
    if (currentReadCount < readCount)
      break; //exit without incrementing loopState so this step repeats
    tempTotal = tempTotal / currentReadCount; //divide by # of iterations to produce the average
    humTotal = humTotal / currentReadCount;
    Serial.println(F("Finished reading DHT"));  /* DEBUG */
    Serial.print("Temp: ");
    Serial.println(tempTotal);
    Serial.print("Humidity: ");
    Serial.println(humTotal);
    firstRead = true; //required so that it resets everything on the first run through
    loopState = 2; //next step
    break;
  case 2: //Send DHT readings
    Serial.println(F("Sending DHT readings..."));  /* DEBUG */
    char humidity[6] = "";
    char temp[6] = "";
    char TID[4] = "";
    itoa(humTotal*100.0,humidity,10);
    itoa(tempTotal*100.0,temp,10);
    itoa(getNextTID(),TID,10);
    char toSend[25] = "D:DHT;";
    strcat(toSend,TID);
    strcat(toSend,";T,");
    strcat(toSend,temp);
    strcat(toSend,";H,");
    strcat(toSend,humidity);
    transmit(toSend, true);
    loopState = 0;
    break;
  }
  delay(100);
}
