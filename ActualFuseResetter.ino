/* Fuse Resetter for ATTINY85
 * info: https://arduinodiy.wordpress.com/2015/05/16/high-voltage-programmingunbricking-for-attiny/
 * AVR High-voltage Serial Programmer
 * Originally created by Paul Willoughby 03/20/2010
 * http://www.rickety.us/2010/03/arduino-avr-high-voltage-serial-programmer/
 * Inspired by Jeff Keyzer http://mightyohm.com
 * revamped code by oh_bother, added buttons and indicator LEDs cause they are cool
 * Serial Programming routines from ATtiny25/45/85 datasheet
 */

//serial
#define BAUD 115200

//fuses
//ATtiny25/45/85, default is h:DF l:62
#define  HFUSE_UNLOCK  0xDD
#define  HFUSE_LOCK  0x5D
#define  LFUSE 0xE1

// For Attiny13 use
// #define HFUSE 0xFF
// #define LFUSE 0x6A

//pins (pro micro)
#define CHECK_B 3 //check button to ground 
#define FLASH_B 4 //fuse reset button to ground
#define RED_LED A2 // red led to ground
#define GRN_LED A1 // grn led to ground
#define RXLED 17 //built in LED (tx is auto-initialized)

#define RST 9 //level shifter RST, Pin 1
#define CLKOUT 15 //SCI, Pin 2
#define DATAIN 10 //SDO, Pin 7
#define INSTOUT 16 //SII, Pin 6
#define DATAOUT 14 //SDIm, Pin 5 
#define VCC A0 //VCC, Pin 8

//ledStuff
#define PERIOD 2000

//variables
uint8_t BPress = 0;
uint8_t hData = 0;
uint8_t lData = 0; 
uint8_t eData = 0;

void setup(){
  //pins
  pinMode(CHECK_B, INPUT_PULLUP);
  pinMode(FLASH_B, INPUT_PULLUP);
  pinMode(RED_LED, OUTPUT);
  pinMode(GRN_LED, OUTPUT);
  pinMode(RXLED, OUTPUT);
  
  pinMode(VCC, OUTPUT);
  pinMode(RST, OUTPUT);
  pinMode(DATAOUT, OUTPUT);
  pinMode(INSTOUT, OUTPUT);
  pinMode(CLKOUT, OUTPUT);
  pinMode(DATAIN, OUTPUT);
  pinMode(RXLED, LOW);
  TXLED0; //STFU bright LED
  
  //Init
  digitalWrite(RST, HIGH); 
  digitalWrite(GRN_LED, LOW);
  digitalWrite(RED_LED, LOW);
  

  //serial
  Serial.begin(BAUD);
  while(!Serial){delay(2000); break;}
  Serial.println("use buttons to start");
}


void loop(){
  breathLED();
  checkButtons(); 
  delay(100);
}

void breathLED(){
  uint8_t fadeyBOY = 128 + 127*cos(2*PI/PERIOD*millis());
  analogWrite(RXLED, fadeyBOY);
}

void checkButtons(){
  if(!digitalRead(CHECK_B)){
    Serial.println("Reading");
    doStuff(false);
  }else if(!digitalRead(FLASH_B)){
    Serial.println("Programming");
    doStuff(true);
  }
}

void doStuff(bool flipper){
    PStart();
    readFuses();
    setLEDs();
    printFuses();
    
    if(flipper){
      if(hData == 0xFF || hData == 0){
        Serial.println("Fs in the chat bois");
        Serial.println();
        digitalWrite(RED_LED, HIGH);
        digitalWrite(GRN_LED, HIGH);
        return;
      }
      
      uint8_t Rbit = hData >> 7;
      
      if(Rbit){
        Serial.print("locking + ");
        writeh(HFUSE_LOCK);
      }else{
        Serial.print("unlocking + ");
        writeh(HFUSE_UNLOCK);
      }
      
      Serial.println("lfuseing");
      writel(LFUSE);
      
      readFuses();
      setLEDs();
      printFuses();
    }
    
    PStop();
    
    delay(1000);
    Serial.println("ready");
    Serial.println();
}

void PStart(){
  // Initialize pins
  pinMode(DATAIN, OUTPUT);
  digitalWrite(DATAOUT, LOW);
  digitalWrite(INSTOUT, LOW);
  digitalWrite(DATAIN, LOW);
  digitalWrite(RST, HIGH);
  
  //Apply VCC
  digitalWrite(VCC, HIGH);
  delayMicroseconds(20);

  //Turn on 12v
  digitalWrite(RST, LOW);   
  delayMicroseconds(10);
  
  //Release DATAIN
  pinMode(DATAIN, INPUT);
  delayMicroseconds(300);
}

void PStop(){
  //reset pins to initial state
  digitalWrite(CLKOUT, LOW);
  digitalWrite(VCC, LOW);
  digitalWrite(RST, HIGH);
}

void setLEDs(){
  uint8_t  Rbit = hData >> 7;
  //11011101
  //01011101
  if(hData == 0xFF || hData == 0){
    digitalWrite(RED_LED, LOW);
    digitalWrite(GRN_LED, LOW);
    return;
  }
 
  if(Rbit){
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GRN_LED, LOW);
  }
  
  if(!Rbit){
    digitalWrite(GRN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
  }
}

void printFuses(){
  Serial.print("l:");
  Serial.print(lData, HEX);
  Serial.print(" h:");
  Serial.print(hData, HEX);
  Serial.print(" e:");
  Serial.println(eData, HEX);
}

void writel(byte val){
  digitalWrite(RXLED, HIGH);
  shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x40, 0x4C);
  shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, val, 0x2C);
  shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x64);
  shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x6C);
  digitalWrite(RXLED, LOW);
}

void writeh(byte val){
  digitalWrite(RXLED, HIGH);
  shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x40, 0x4C);
  shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, val, 0x2C);
  shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x74);
  shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x7C);
  digitalWrite(RXLED, LOW);
}

int shiftOut2(uint8_t dataPin, uint8_t dataPin1, uint8_t clockPin, uint8_t bitOrder, byte val, byte val1){
  int i;
  int inBits = 0;
  //Wait until DATAIN goes high
  while (!digitalRead(DATAIN));
  
  //Start bit
  digitalWrite(DATAOUT, LOW);
  digitalWrite(INSTOUT, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);
  
  for (i = 0; i < 8; i++){
    if (bitOrder == LSBFIRST) {
      digitalWrite(dataPin, !!(val & (1 << i)));
      digitalWrite(dataPin1, !!(val1 & (1 << i)));
    }else{
      digitalWrite(dataPin, !!(val & (1 << (7 - i))));
      digitalWrite(dataPin1, !!(val1 & (1 << (7 - i))));
    }
    inBits <<=1;
    inBits |= digitalRead(DATAIN);
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);
  }
  
  //End bits
  digitalWrite(DATAOUT, LOW);
  digitalWrite(INSTOUT, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);
  digitalWrite(clockPin, HIGH);
  digitalWrite(clockPin, LOW);
  
  return inBits;
}

void readFuses(){
  //Read lfuse
  digitalWrite(RXLED, HIGH);
  shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x04, 0x4C);
  shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x68);
  lData = shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x6C);
  digitalWrite(RXLED, LOW);
  
  //Read hfuse
  digitalWrite(RXLED, HIGH);
  shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x04, 0x4C);
  shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x7A);
  hData = shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x7E);
  digitalWrite(RXLED, LOW);
  
  //Read efuse
  digitalWrite(RXLED, HIGH);
  shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x04, 0x4C);
  shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x6A);
  eData = shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x6E);
  digitalWrite(RXLED, LOW);
}
