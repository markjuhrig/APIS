#include <TimeLib.h>
#include <SD.h>            // for MP3 replay
#include <SPI.h>           // for MP3 replay
#include <arduino.h>       // for MP3 replay
#include <MP3-Shield.h>   // for MP3 replay
#include <avr/wdt.h>      // for Watch-Dog Support

// Adafruit RGB Character LCD Shield and Library
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>

// These #defines make it easy to set the backlight color
#define BL_OFF 0x0
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7
//**********************

// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

/* *****************************
inputs:
D0 - RX Data from Weather Sensor
D1 - TX Debug Data to USB Serial Port
D2 - Squelch output from Radio 
D3 - Mute Weather announcement
D4 - RC/Modellflug Key
D5 - KFZ Key

outputs:
D6 - Relays 2 - PCL - Pilot Controlled Lighting
D7 - Relays 1 - PTT - Push To Talk input of Radio

inputs used by MP3-Shield:

Pins used for Play Control: 
D3 - Receiving signal from button for Volume Up. 
D4 - Receiving signal from switch for Next Song function. 
D5 - Receiving signal from switch for Play&Stop and Record function. 
D6 - Receiving signal from switch for Previous Song function. 
D7 - Receiving signal from button for Volume Down. 
D8 - Green Led instructions. 

Pins Used for SPI Interface: 
D10 - SPI Chip Select 
D11 - SPI MOSI 
D12 - SPI MISO 
D13 - SPI SCK 

Pins Used for VS1053 Interface: 
A0 - Reset of VS1053 
A1 - Data Require of VS1053 
A2 - Data Select of VS1053 
A3 - Chip Select of VS1053 

outputs assigned to Relays shield:
D4 - Relays 4
D5 - Relays 3 
D6 - Relays 2
D7 - Relays 1

Info (millis-rollover): https://www.faludi.com/2007/12/18/arduino-millis-rollover-handling/

********************************/

// defines:

#define default_volume 10  // default MP3 volume, 0 = max, 254 = min

#define SQL_PIN  2   // the number of the Squelch pin
#define MUTE_PIN 3   // pin to mute weather announcement
#define RC_PIN   4   // RC / Modelllfug Ops
#define KFZ_PIN  5   // KFZ Test Ops
#define PTT_PIN  7   // PTT Relay
#define PCL_PIN  6   // PCL Relay
#define GRN_LED  8   // GREEN LED (used by MP3-Player)

// Constants:

const int PTT_DELAY = 500;  // 500 ms MP3 delay after PTT release

const int PCL_MIN_PRESS_TIME   =  100;  //  200 milliseconds (for PCL)
const int PCL_SHORT_PRESS_TIME = 1000;  // 1000 milliseconds (for PCL)
const int PCL_LONG_PRESS_TIME  = 7000;  // 7000 milliseconds (for weather announcement)

const int OPS_MIN_PRESS_TIME   =  100;  //  100 milliseconds (for ops announcement key)
const int OPS_SHORT_PRESS_TIME = 2000;  // 2000 milliseconds (for ops announcement key)
const int OPS_LONG_PRESS_TIME  = 3000;  // 3000 milliseconds (for ops announcement key)

const int MUTE_MIN_PRESS_TIME   =  100; //  100 milliseconds (for muting button)
const int MUTE_SHORT_PRESS_TIME = 1000; // 1000 milliseconds (for muting button)
const int MUTE_LONG_PRESS_TIME  = 2000; // 2000 milliseconds (for muting button)
const int MUTE_RESET_PRESS_TIME = 5000; // 5000 milliseconds (for muting button)

const unsigned long PTTTimeout = 1500;    // timeout for PTT counting 1.5 sec
const unsigned int  PCLTimeout = 1800;    // PCL on time = 30 minutes
const unsigned int  PCLWarnTimeout = 60;  // PCL warining 1 minute befor turning lights off

const unsigned char Mute_Auto_Off_Hrs=18; // time (18:00 hrs UTC) to automatically unmute 

const unsigned long RC_KFZ_Time = 14400;  // 4 hours

const unsigned char SensTimeout = 30;     // 60 seconds

const unsigned char PCL_ON_Klicks   = 3;       // 3 clicks to turn lights on
const unsigned char PCL_OFF_Klicks  = 5;       // 5 clicks to turn lights off
const unsigned char MUTE_OFF_Klicks = 7;       // 7 clicks to un-mute weather announcement

const unsigned int WeatherTimeout = 180;  // 3 minutes (weather announcements blocked for this time)

const float WCA = 20;                 // wind sensor correction angle = +20°

const int QNH_Offset = 42;                // to compensate elevation of airfield (43 hPa = ~1170 ft)

const char STX = 2;                       //ASCII-Code 02, text representation of the STX code
const char ETX = 3;                       //ASCII-Code 03, text representation of the ETX code

// Variables will change:
unsigned char lastState_port_D    = bit(SQL_PIN) + bit(MUTE_PIN);  // the previous state Port-D
unsigned char currentState_port_D = bit(SQL_PIN) + bit(MUTE_PIN);  // the current reading of Port-D

unsigned long pressedTime  = 0;
unsigned long releasedTime = 0;

#define RC_pressedTime  pressedTime     // use pressed/releasedTime variables too
#define RC_releasedTime releasedTime
#define KFZ_pressedTime  pressedTime
#define KFZ_releasedTime releasedTime

unsigned long WeatherTime = 0;            // keep time of last weather announcement

int PTTCounter = 0;                       // counts number of PPT clicks (e.g. 3 = runway lighting on, 5 = ...)
unsigned long PTTStart = 0;               // Timeout counter for PCL

unsigned long PCLTime = 0;                // keep time of switching on the lights
unsigned char PCL = 0;                    // 1 = lights on, 0 = light 0ff
unsigned char PCL_Warning = 1;            // will be set to 0 after transmitting the "one minute lights off" warning
unsigned char Mute = 0;                   // mute anoutcement

unsigned char RC_KFZ = 0;                 // 0 = no special operations, 1 = RC, 2 = KFZ
unsigned long RC_KFZ_Off_Time = 0;        // special ops timeout  

float windspeed = 0;                      // actual readings (1 Hz update rate)
unsigned int winddir = 0;                 // actual readings (1 Hz update rate)
unsigned int pressure = 0;                // actual readings (1 Hz update rate)

//float temperature = 0, humidity = 0;      // currently not processed
// float Temperature=0;

float WindSpeed=0;                        // filtered (low pass) values
float UX = 0.0, UY = 0.0;                 // filtered (low pass) values

unsigned char SensorOKAY = 0;             // 1 = weather sensor data OKAY
unsigned char new_weather = 0;            // 1 = new weather data received
unsigned char SensorTimeout = 0;          // counter for sensor timeout

long  Time_Last_Processing = 0;           // stores last UNIX time of processing weather data

playingstatetype playerState;             // used to check state of MP3 player

// code:

void setup()
{
  Serial.begin(9600);
  Serial.setTimeout(100);   // 100 ms serial timeout (weather frame ~50ms)

  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  lcd.setBacklight(WHITE);  
  lcd.setCursor(0, 0);
  lcd.print(F("  Michelstadt  "));
  lcd.setCursor(0, 1);
  lcd.print(F("WetteransageRevE"));

  pinMode(SQL_PIN, INPUT_PULLUP);
  pinMode(MUTE_PIN, INPUT_PULLUP);
  pinMode(RC_PIN, INPUT);
  pinMode(KFZ_PIN, INPUT);
  pinMode(PTT_PIN, OUTPUT);
  digitalWrite(PTT_PIN, LOW); 
  pinMode(PCL_PIN, OUTPUT);
  digitalWrite(PCL_PIN, LOW); 

  Serial.println();
  Serial.println(F("Reset, Time not set"));
  setTime(1357041600);  // 12:00
  //setTime(1357041500); // vor 11:58

  player.begin();                      //will initialize the hardware and set default mode to be normal.
  player.setVolume(default_volume);
  digitalWrite(PTT_PIN, HIGH); 
  delay(PTT_DELAY);
  player.setPlayMode(PM_NORMAL_PLAY); //
  player.addToPlaylist((char*)("nix.mp3"));
  player.addToPlaylist((char*)("wakt.mp3"));
  WeatherTime = now() - WeatherTimeout; 
  PCLTime = now()+PCLTimeout; 
  delay(2000);
  wdt_enable(WDTO_8S);              // Watch-Dog activated, 8 second trip time
}

void LCD_RC_KFZ(unsigned char RC_KFZ_State){
  lcd.setCursor(15, 0);
  if (RC_KFZ_State == 0) lcd.print(F(" ")); 
  if (RC_KFZ_State == 1) lcd.print(F("M")); 
  if (RC_KFZ_State == 2) lcd.print(F("P")); 
}

void LCD_PCL(unsigned char PCL_State){
  lcd.setCursor(15, 1);
  if (PCL_State)
    lcd.print(F("B"));
  else
    lcd.print(F(" "));  
}

void LCD_MUTE(unsigned char Mute_State){
  lcd.setCursor(14, 1);
  if (Mute_State)
      lcd.print(F("X")); 
  else
    lcd.print(F(" "));  
}

void loop(){    
  //wdt_reset();  // reset watch-dog at the begin of every main loop execution
  if (Serial.available()) {
    processSyncMessage();   // e.g. T1357041600
  } 

  if ((now() > (Time_Last_Processing + 1 )) ||  new_weather  ){
    Time_Last_Processing = now();               // save time of processing the weather 
    new_weather = 0;
    if (SensorTimeout) SensorTimeout--;

    if (timeStatus()!= timeNotSet) {
      Serial.println();
      Serial.print(F("SYST:"));  
      Serial.print(now());   
      Serial.print(F(" ")); 
      ProcessWeather(1);      // process wind date, 1 = vector based, 0 = direction based
      Serial.print(F(" B:")); 
      Serial.print(PCL);
      Serial.print(F(" BT:")); 
      if ((now() - PCLTime) < PCLTimeout ) Serial.print(PCLTimeout - now() + PCLTime);
      else Serial.print(0);
      Serial.print(F(" M:")); 
      Serial.print(Mute);	  
      Serial.print(F(" WT:")); 
      if ((now() - WeatherTime) < WeatherTimeout ) Serial.print(WeatherTimeout - now() + WeatherTime);
      else Serial.print(0);
      if (RC_KFZ){
        Serial.print(F(" OPS:"));  
        Serial.print(RC_KFZ);  
        Serial.print(F(" "));                        
        Serial.print(RC_KFZ_Off_Time - now());
      }
      Serial.print(F(" MP3:")); 
      Serial.print(playerState);
      Serial.print(F(" ST:")); 
      Serial.print(SensorTimeout);
      if (Mute)
        if (hour() >= Mute_Auto_Off_Hrs) {
          Mute = 0; 
          LCD_MUTE(Mute);  
          Serial.print(F(" Mute Off")); 
        }
    }
  }
  //delay(10);
  player.play(playerState);  // process MP3-Player

  // process inputs
  // read Port-D
  currentState_port_D = PIND;

  // process SQL Input (active low):
  if((lastState_port_D & bit(SQL_PIN)) && !(currentState_port_D & bit(SQL_PIN)))        // button is pressed
    pressedTime = millis();
  else if(!(lastState_port_D & bit(SQL_PIN)) && (currentState_port_D & bit(SQL_PIN))) { // button is released
    releasedTime = millis();

    int pressDuration = (int)(releasedTime - pressedTime);
    Serial.print(F(" press_duration:"));
    Serial.print(pressDuration);
    
     if( (pressDuration < PCL_SHORT_PRESS_TIME) &&(pressDuration > PCL_MIN_PRESS_TIME)  ){
      Serial.print(F(" SKP"));
      lcd.setCursor(14, 0);
      lcd.print(F("S"));   
      PTTCounter++;
      if (PTTCounter > 0) {
        PTTStart = releasedTime;    // start timeout counter and increase PTT Counter
      }
    }
    if( pressDuration > PCL_LONG_PRESS_TIME ){
      Serial.print(F(" LKP"));
      lcd.setCursor(14, 0);
      lcd.print(F("L"));  
      if (Mute == 0)
        if ((now() - WeatherTime ) > WeatherTimeout ){
          WeatherTime = now(); 
          lcd.setCursor(14, 0);
          lcd.print(F("W"));   
          Say_Weather();
        }
    }
  }
  
  // process Mute Input (active low):
  if((lastState_port_D & bit(MUTE_PIN)) && !(currentState_port_D & bit(MUTE_PIN)))        // button is pressed
    pressedTime = millis();
  else if(!(lastState_port_D & bit(MUTE_PIN)) && (currentState_port_D & bit(MUTE_PIN))) { // button is released
    releasedTime = millis();

    int pressDuration = (int)(releasedTime - pressedTime);
    if( (pressDuration < MUTE_SHORT_PRESS_TIME) && (pressDuration > MUTE_MIN_PRESS_TIME) ){
      Serial.print(F(" Mute On"));
      lcd.setCursor(14, 0);
      lcd.print(F("S"));       
      Mute = 1;  
      LCD_MUTE(Mute);
    } 
    if(( pressDuration > MUTE_LONG_PRESS_TIME ) && ( pressDuration < MUTE_RESET_PRESS_TIME )){
      Serial.print(F(" Mute Off"));
      lcd.setCursor(14, 0);
      lcd.print(F("L"));  
      Mute = 0;
      LCD_MUTE(Mute);      
    }
    if( pressDuration > MUTE_RESET_PRESS_TIME ){
      Serial.print(F(" APIS Reset, Mute Off, Lights Off, Special Ops Off"));
      lcd.setCursor(14, 0);
      lcd.print(F("R"));  
      Mute = 0;
      LCD_MUTE(Mute);  
      PCL = 0;
      PCLTime = 0;
      digitalWrite(PCL_PIN, LOW); 
      PCL_Warning = 1;
      LCD_PCL(PCL);      
      Say_PCL_Off(); 
      RC_KFZ = 0;                // 0 = no special operations, 1 = RC, 2 = KFZ
      LCD_RC_KFZ(RC_KFZ);       
      RC_KFZ_Off_Time = 0; 
      WeatherTime = now() - WeatherTimeout;  // reset weather time
      //Say_OPS();        
    }
  }

  // process RC - Modellflug key (active high) *********************
  if(!(lastState_port_D & bit(RC_PIN)) && (currentState_port_D & bit(RC_PIN)))        // button is pressed
    RC_pressedTime = millis();
  else if((lastState_port_D & bit(RC_PIN)) && !(currentState_port_D & bit(RC_PIN))) { // button is released
    RC_releasedTime = millis();
    int pressDuration = (int)(RC_releasedTime - RC_pressedTime);
    if( pressDuration > OPS_LONG_PRESS_TIME ){
      Serial.print(F(" RC-Key-Long"));
      RC_KFZ = 0;                // 0 = no special operations, 1 = RC, 2 = KFZ
      LCD_RC_KFZ(RC_KFZ);       
      RC_KFZ_Off_Time = 0; 
      Say_OPS();           
    }
    else
      if( (pressDuration < OPS_SHORT_PRESS_TIME) && (pressDuration > OPS_MIN_PRESS_TIME) ){
        Serial.print(F(" RC-Key-Short"));
        RC_KFZ = 1;                // 0 = no special operations, 1 = RC, 2 = KFZ 
        LCD_RC_KFZ(RC_KFZ);           
        RC_KFZ_Off_Time = now() + RC_KFZ_Time; 
        Say_OPS();   
      }
  }

  // process KFZ Key  (active high) ***************************
  if(!(lastState_port_D & bit(KFZ_PIN)) && (currentState_port_D & bit(KFZ_PIN)))        // button is pressed
    KFZ_pressedTime = millis();
  else if((lastState_port_D & bit(KFZ_PIN)) && !(currentState_port_D & bit(KFZ_PIN))) { // button is released
    KFZ_releasedTime = millis();
    int pressDuration = (int)(KFZ_releasedTime - KFZ_pressedTime);
    if( pressDuration > OPS_LONG_PRESS_TIME ){
      Serial.print(F(" KFZ-Key-Long"));
      RC_KFZ = 0;                // 0 = no special operations, 1 = RC, 2 = KFZ
      LCD_RC_KFZ(RC_KFZ); 
      RC_KFZ_Off_Time = 0;  
      Say_OPS();                         
    }
    else
      if( (pressDuration < OPS_SHORT_PRESS_TIME) && (pressDuration > OPS_MIN_PRESS_TIME) ){
        Serial.print(F(" KFZ-Key-Short"));
        RC_KFZ = 2;                // 0 = no special operations, 1 = RC, 2 = KFZ      
        LCD_RC_KFZ(RC_KFZ);
        RC_KFZ_Off_Time = now() + RC_KFZ_Time;  
        Say_OPS();          
      }
  }

  // save the last port-D state
  lastState_port_D = currentState_port_D;

  if ((now()> RC_KFZ_Off_Time) && RC_KFZ ){
    Serial.print(F(" KFZ-RC Off"));
    RC_KFZ = 0;                // 0 = no special operations, 1 = RC, 2 = KFZ
    LCD_RC_KFZ(RC_KFZ); 
    RC_KFZ_Off_Time = 0;     
  }
 
  if ( ((now() - PCLTime) > (PCLTimeout - PCLWarnTimeout) ) && PCL && PCL_Warning){
        Serial.print(F(" Lights Off in 1 Minute"));  
        PCL_Warning = 0;
        Say_PCL_Off_in_1();  
  }

  if ( ((now() - PCLTime) > PCLTimeout ) && PCL ){
        PCL = 0;
        digitalWrite(PCL_PIN, LOW); 
        PCL_Warning = 1;
        Serial.print(F(" Lights Off"));  
        LCD_PCL(PCL);      
        Say_PCL_Off();  
  }
  
  if (((millis() - PTTStart) > PTTTimeout) && (PTTCounter>0)){
    if (PTTCounter == PCL_ON_Klicks) { 
      PCLTime = now ();
      PCL = 1; 
      PCL_Warning = 1;
      digitalWrite(PCL_PIN, HIGH);       
      Serial.print(F(" Lights On"));
      LCD_PCL(PCL);
      Say_PCL_On_30();
    }
    if (PTTCounter == PCL_OFF_Klicks) { 
        WeatherTime = now() - WeatherTimeout;  // reset weather time
        PCLTime = 0;
        PCL = 0;
        digitalWrite(PCL_PIN, LOW); 
        PCL_Warning = 1;
        Serial.print(F(" Lights Off"));  
        LCD_PCL(PCL);      
        Say_PCL_Off(); 
    }
    if (PTTCounter == MUTE_OFF_Klicks) { 
        WeatherTime = now() - WeatherTimeout;  // reset weather time
        Serial.print(F(" Mute Off"));
        lcd.setCursor(14, 0);
        lcd.print(F("L"));  
        Mute = 0;
        LCD_MUTE(Mute);  
        Say_Weather(); 
    }
    PTTCounter = 0;
    PTTStart = 0;
    Serial.print(F(" CLR PPT_CNT"));  
  }
  wdt_reset();  // reset watch-dog at the end of every main loop execution
}

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}  

void processSyncMessage() {
  char str[50];
  if(Serial.find(STX)) {    
    int tage, monate, jahre, stunden, minuten, sekunden;
    long checksumme = 1; 
    int XOR_CS = 0;
    char* ptr = str;              // set ptr to start of string
    String serial_str = Serial.readString();
    serial_str.toCharArray(str, 50);
    //Serial.println(str);

    if (ptr != NULL)
    {
      char *ptr2;               // parse checksum (8-bit hex number in string)
      checksumme = strtol(ptr+46, &ptr2, 16);
      for (int i = 0; i < 45; i++) {    // calculate XOR checksum
        XOR_CS ^= str[i];
      }  
    }
    //strcat(str, "001.4 272 -03.3  94  985.0 23.03.21 06:59:46 *27"); // append wind sensor string
    // test string: !001.4 272 +03.3  94  985.0 23.03.21 06:59:46 *27   
    // test string: !001.5 285 +03.3  94  985.0 23.03.21 06:59:28 *26   
    // test string: !001.4 272 -03.3  94  985.0 23.03.21 06:59:46 *21 
    //              !011.5 085 +03.3  94  985.0 23.03.21 06:59:28 *25
    //              !001.5 085 +03.3  94  985.0 23.03.21 06:59:28 *24

    if (XOR_CS == checksumme){
      if (ptr != NULL)
        windspeed = atof(ptr) * 2;       // m/s * 2 = kts
      if (ptr != NULL)
        winddir = atoi(ptr + 6);
      //if (ptr != NULL)
      //  temperature = atof(ptr + 10);
      //if (ptr != NULL)
      //  humidity = atof(ptr + 16);
      if (ptr != NULL)
        pressure = QNH_Offset + atoi(ptr + 20);
      if (ptr != NULL)
        tage = atoi(ptr + 27);
      if (ptr != NULL)
        monate = atoi(ptr + 30);
      if (ptr != NULL)
        jahre = atoi(ptr + 33);
      if (ptr != NULL)
        stunden = atoi(ptr + 36);
      if (ptr != NULL)
        minuten = atoi(ptr + 39);
      if (ptr != NULL)
        sekunden = atoi(ptr + 42);

      setTime(stunden, minuten, sekunden, tage, monate, jahre);  //int hr,int min,int sec,int day, int month, int yr
      //Serial.print(F("CS OKAY ==> Time Set"));

      if ((stunden == 0) && (minuten == 0) && (sekunden >= 52)) while(1){} // Reset Arduino via Watch-Dog at UTC==00:00:52 (once a day)
      
      new_weather = 1;
      SensorTimeout = SensTimeout;
      
      if (SensorOKAY == 0){
        Say_Sensor_OKAY ();
        SensorOKAY = 1;
      }
    }
    else {
      Serial.println();
      Serial.print(F("CS Error ==> Time + Weather not set"));
    }
  }
  else Serial.print(F("STX Error"));
  serialFlush();
}

float lowPass(float  input, float output ) {
    output = output + 0.05 * (input - output);  
      return output;
  } 

void LCD_4_Places(unsigned int number){
  if (number < 9999) {
    if (number < 1000) lcd.print(F(" "));
    if (number < 100) lcd.print(F("0"));
    if (number < 10) lcd.print(F("0"));
    lcd.print(number, DEC);
  }
}

void LCD_3_Places(unsigned int number){
  if (number < 999) {
    if (number < 100) lcd.print(F("0"));
    if (number < 10)  lcd.print(F("0"));
    lcd.print(number, DEC);
  }
}

void LCD_2_Places(unsigned int number){
  if (number < 99) {
    if (number < 10) lcd.print(F("0"));
    lcd.print(number, DEC);
  }
}

void LCDClockDisplay(){
  // digital clock display of the time hh:mm
  LCD_2_Places((unsigned int)hour());
  lcd.print(F(":")); 
  LCD_2_Places((unsigned int)minute());
  //lcd.print(F(" ")); 
}

void ProcessWeather(bool vector_based) {

  if (vector_based) {
    UX = lowPass( windspeed * cos(((float)winddir+WCA) * DEG_TO_RAD), UX);     // wind vector
    UY = lowPass( windspeed * sin(((float)winddir+WCA) * DEG_TO_RAD), UY);     // wind vector
  }
  else {
    UX = lowPass( cos(((float)winddir+WCA) * DEG_TO_RAD), UX);                   // wind direction only 
    UY = lowPass( sin(((float)winddir+WCA) * DEG_TO_RAD), UY);                   // wind direction only  
  }
  if (SensorTimeout){
    //WindDir = (int)((RAD_TO_DEG * atan2(UY, UX))+360) % 360;       // filtering wind direction / vector    
    WindSpeed = lowPass(windspeed, WindSpeed);                      // filtering wind speed
  }
  else {
    UX = 1.0f;
    UY = 0.0f;
    WindSpeed = 0;   
    pressure = 0;  
  }

  Serial.print(F("V:")); Serial.print(WindSpeed, 1);
  Serial.print(F(" W:")); Serial.print((int)((RAD_TO_DEG * atan2(UY, UX))+360) % 360);  
  Serial.print(F(" QNH:")); Serial.print(pressure, DEC);  

  lcd.setCursor(0, 0);
  lcd.print(F("Wind:"));
  lcd.setCursor(5, 0);  
  LCD_3_Places((unsigned int)((RAD_TO_DEG * atan2(UY, UX))+360) % 360);
  lcd.print((char)0xDF);                // 0xDF = °
  //lcd.print(F(" "));
  lcd.setCursor(9, 0);  
  LCD_2_Places((unsigned int)WindSpeed);
  lcd.print(F("kts ")); 

  lcd.setCursor(0, 1);
  lcd.print(F("QNH:"));
  LCD_4_Places(pressure);
  lcd.print(F(" ")); 
  LCDClockDisplay();
  LCD_RC_KFZ(RC_KFZ);  
  LCD_PCL(PCL);
  LCD_MUTE(Mute);
}

void Say_Digit(int digit){
  switch (digit) {
  case 0:
    player.addToPlaylist((char*)("0.mp3"));
    break;
  case 1:
    player.addToPlaylist((char*)("1.mp3"));
    break;
  case 2:
    player.addToPlaylist((char*)("2.mp3"));
    break;
  case 3:
    player.addToPlaylist((char*)("3.mp3"));
    break;
  case 4:
    player.addToPlaylist((char*)("4.mp3"));
    break;
  case 5:
    player.addToPlaylist((char*)("5.mp3"));
    break;
  case 6:
    player.addToPlaylist((char*)("6.mp3"));
    break;
  case 7:
    player.addToPlaylist((char*)("7.mp3"));
    break;
  case 8:
    player.addToPlaylist((char*)("8.mp3"));
    break;
  case 9:
    player.addToPlaylist((char*)("9.mp3"));
    break;
  default:
    // Statement(s)
    break; // Wird nicht benötigt, wenn Statement(s) vorhanden sind
  }
}

void Say_Simple_Wind(float Vin, int Win){
  float WW;
  int W;
  WW = Win + 0.5f;
  WW = (WW + 22.5f) / 45.0f;
  W = (int)WW;

  if ((Vin >= 2.0f) && (Vin < 5.0f)) player.addToPlaylist((char*)("schw.mp3"));
  if ((Vin >= 5.0f) && (Vin < 10.0f)) player.addToPlaylist((char*)("msgw.mp3"));
  if (Vin >= 10.0f) player.addToPlaylist((char*)("stkw.mp3"));
  if (Vin < 2.0f) player.addToPlaylist((char*)("still.mp3"));
  else
  {
     switch (W) {
    case 8:
      player.addToPlaylist((char*)("n.mp3"));
      break;
    case 0:
      player.addToPlaylist((char*)("n.mp3"));
      break;
    case 1:
      player.addToPlaylist((char*)("no.mp3"));
      break;
    case 2:
      player.addToPlaylist((char*)("o.mp3"));
      break;
    case 3:
      player.addToPlaylist((char*)("so.mp3"));
      break;
    case 4:
      player.addToPlaylist((char*)("s.mp3"));
      break;
    case 5:
      player.addToPlaylist((char*)("sw.mp3"));
      break;
    case 6:
      player.addToPlaylist((char*)("w.mp3"));
      break;
    case 7:
      player.addToPlaylist((char*)("nw.mp3"));
      break;
      default:
      // Statement(s)
      break; // Wird nicht benötigt, wenn Statement(s) vorhanden sind
    }
  }
}

void Say_Mixed_Wind(float Vin, int Win){
  float WW;
  int W, V;
  WW = Win + 0.5f;
  WW = (WW + 22.5f) / 45.0f;
  W = (int)WW;

  V = Vin;
  player.addToPlaylist((char*)("wndm.mp3"));
  
  switch (V) {
    case 0:
      player.addToPlaylist((char*)("0.mp3"));
      break;
    case 1 ... 2:
      player.addToPlaylist((char*)("2.mp3"));
      break;
    case 3 ... 4:
      player.addToPlaylist((char*)("4.mp3"));
      break;
    case 5 ... 6:
      player.addToPlaylist((char*)("6.mp3"));
      break;
    case 7 ... 8:
      player.addToPlaylist((char*)("8.mp3"));
      break;
    case 9 ... 10:
      player.addToPlaylist((char*)("10.mp3"));
      break;
    case 11 ... 12:
      player.addToPlaylist((char*)("12.mp3"));
      break;
    case 13 ... 14:
      player.addToPlaylist((char*)("14.mp3"));
      break;
    case 15 ... 16:
      player.addToPlaylist((char*)("16.mp3"));
      break;
    case 17 ... 18:
      player.addToPlaylist((char*)("18.mp3"));
      break;
    case 19 ... 22:
      player.addToPlaylist((char*)("20.mp3"));
      break;
    case 23 ... 27:
      player.addToPlaylist((char*)("25.mp3"));
      break;
    case 28 ... 32:
      player.addToPlaylist((char*)("30.mp3"));
      break;
    case 33 ... 37:
      player.addToPlaylist((char*)("35.mp3"));
      break;
    case 38 ... 42:
      player.addToPlaylist((char*)("40.mp3"));
      break;
    case 43 ... 47:
      player.addToPlaylist((char*)("45.mp3"));
      break;
    case 48 ... 1000:
      player.addToPlaylist((char*)("50.mp3"));
      break;
    default:
      // Statement(s)
      break; // Wird nicht benötigt, wenn Statement(s) vorhanden sind
  }      

  player.addToPlaylist((char*)("kaus.mp3"));
 
  switch (W) {
    case 8:
      player.addToPlaylist((char*)("n.mp3"));
      break;
    case 0:
      player.addToPlaylist((char*)("n.mp3"));
      break;
    case 1:
      player.addToPlaylist((char*)("no.mp3"));
      break;
    case 2:
      player.addToPlaylist((char*)("o.mp3"));
      break;
    case 3:
      player.addToPlaylist((char*)("so.mp3"));
      break;
    case 4:
      player.addToPlaylist((char*)("s.mp3"));
      break;
    case 5:
      player.addToPlaylist((char*)("sw.mp3"));
      break;
    case 6:
      player.addToPlaylist((char*)("w.mp3"));
      break;
    case 7:
      player.addToPlaylist((char*)("nw.mp3"));
      break;
    default:
      // Statement(s)
      break; // Wird nicht benötigt, wenn Statement(s) vorhanden sind
  }
}

void Say_WindDir(int number){
  int rounded_number = 0;
  rounded_number = (number + 5) / 10 * 10;
  if (rounded_number >= 360) rounded_number = 0;
  for (int i=1000; i >= 1; i=i/10){
    Say_Digit(rounded_number % i / (i/10));    
  }
}

void Say_WindSpeed(int number){
  int i;
  if (number < 100)
  {
    i = number % 100 /10;
    if (i > 0) {
      Say_Digit(i);
      i = number - i * 10;
      Say_Digit(i);
    } 
    else {
      i = number - i * 10;
      if (i == 1) player.addToPlaylist((char*)("nem.mp3"));
      else Say_Digit(i);       
    }
  } 
}

void Say_QNH(unsigned int number){
  int rounded_number = 0;
  rounded_number = number;
  for (int i=10000; i >= 1; i=i/10){
    if (!((i == 10000)&&((rounded_number % i / (i/10)==0))))
    Say_Digit(rounded_number % i / (i/10));    
  }
}

void Prepare_MP3 (){
  do {
    delay(10);
    player.play(playerState);  // process MP3-Player
  } while (playerState != PS_IDLE);
  player.initializePlaylist();
  player.setVolume(default_volume);
  player.setPlayMode(PM_NORMAL_PLAY);
  digitalWrite(PTT_PIN, HIGH); 
  delay(PTT_DELAY);
}

void Say_Weather (){
  Serial.print(F(" Say Weather"));
  Prepare_MP3();
  player.addToPlaylist((char*)("radi.mp3"));
  if (SensorTimeout){
    //Say_Simple_Wind(WindSpeed, WindDir);
    Say_Mixed_Wind(WindSpeed, (int)((RAD_TO_DEG * atan2(UY, UX))+360) % 360 );  
    player.addToPlaylist((char*)("qnh.mp3"));
    Say_QNH(pressure);        
  }    
  else {
    player.addToPlaylist((char*)("serr.mp3"));
  }
  player.addToPlaylist((char*)("ende.mp3"));
  //player_void(); // player.addToPlaylist((char*)("paus.mp3"));
  if (RC_KFZ==1) player.addToPlaylist((char*)("rc.mp3"));
  if (RC_KFZ==2) player.addToPlaylist((char*)("kfz.mp3"));
}


void Say_OPS (){
  Prepare_MP3();
  player.addToPlaylist((char*)("radi.mp3"));  
  if (RC_KFZ==1) player.addToPlaylist((char*)("rc.mp3"));
  if (RC_KFZ==2) player.addToPlaylist((char*)("kfz.mp3"));
}

void Say_Sensor_OKAY (){
  lcd.setCursor(0, 0);
  lcd.print(F("  Wetterdaten   "));
  lcd.setCursor(0, 1);
  lcd.print(F("   empfangen    "));
  Prepare_MP3();
  player.addToPlaylist((char*)("wdok.mp3"));
}

void Say_PCL_On_30 (){
  Prepare_MP3();
  player.addToPlaylist((char*)("pb30.mp3"));
}

void Say_PCL_Off (){
  Prepare_MP3();
  player.addToPlaylist((char*)("pbas.mp3"));
}

void Say_PCL_Off_in_1 (){
  Prepare_MP3();
  player.addToPlaylist((char*)("pba1.mp3"));
}
