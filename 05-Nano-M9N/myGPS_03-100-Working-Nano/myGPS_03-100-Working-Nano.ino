#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
// Connect the GPS RX/TX to arduino pins 3 and 5
//SoftwareSerial serial = SoftwareSerial(5,7);

const char UBLOX_INIT[] PROGMEM = {
  // Disable NMEA
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32, // GxGSA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39, // GxGSV off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47, // GxVTG off

  // Disable UBX
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0xDC, //NAV-PVT off
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9, //NAV-POSLLH off
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0, //NAV-STATUS off

  // Enable UBX
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //NAV-PVT on
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH on
  //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS on

  // Rate
  //0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
  0xB5,0x62,0x06,0x08,0x06,0x00,0x78,0x00,0x01,0x00,0x01,0x00,0x8E,0x8A, //(8.33Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0x82,0x00,0x01,0x00,0x01,0x00,0x98,0xC6, //(7.69Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0x96,0x00,0x01,0x00,0x01,0x00,0xAC,0x3E, //(6.67Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)
};

#define GPS Serial    

#define button 3
int state = 0;
int old = 0;
int buttonPoll = 0;

#include <Adafruit_SSD1306.h> 
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 

//On an arduino UNO: A4(SDA), A5(SCL)
#define OLED_RESET -1 //Reset pin # (or -1 if sharing Arduino reset pin) 
#define SCREEN_ADDRESS 0x3C //See datasheet for Address   
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); 

const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

struct NAV_PVT {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;          // GPS time of week of the navigation epoch (ms)
  
  unsigned short year;         // Year (UTC) 
  unsigned char month;         // Month, range 1..12 (UTC)
  unsigned char day;           // Day of month, range 1..31 (UTC)
  unsigned char hour;          // Hour of day, range 0..23 (UTC)
  unsigned char min;           // Minute of hour, range 0..59 (UTC)
  unsigned char sec;           // Seconds of minute, range 0..60 (UTC)
  unsigned char valid;         // Validity Flags (see graphic below)
  unsigned long tAcc;          // Time accuracy estimate (UTC) (ns)
  long nano;                   // Fraction of second, range -1e9 .. 1e9 (UTC) (ns)
  unsigned char fixType;       // GNSSfix Type, range 0..5
  unsigned char flags;         // Fix Status Flags
  unsigned char flags2;
  unsigned char numSV;         // Number of satellites used in Nav Solution
  long lon;                    // Longitude (deg)
  long lat;                    // Latitude (deg)
  long height;                 // Height above Ellipsoid (mm)
  long hMSL;                   // Height above mean sea level (mm)
  unsigned long hAcc;          // Horizontal Accuracy Estimate (mm)
  unsigned long vAcc;          // Vertical Accuracy Estimate (mm)
  long velN;                   // NED north velocity (mm/s)
  long velE;                   // NED east velocity (mm/s)
  long velD;                   // NED down velocity (mm/s)
  long gSpeed;                 // Ground Speed (2-D) (mm/s)
  long headMot;                // Heading of motion 2-D (deg)
  unsigned long sAcc;          // Speed Accuracy Estimate
  unsigned long headAcc;       // Heading Accuracy Estimate
  unsigned short pDOP;         // Position dilution of precision
  unsigned short flags3;
  long headVeh;

  // unsigned char reserved1;
  
  short magDec;
  unsigned short magAcc;

/////////////////////////////////////////////////////////////////
  unsigned long buffer1;     // To Pass Checksum check
  // unsigned long buffer2;     // To Pass Checksum check
/////////////////////////////////////////////////////////////////
};

NAV_PVT pvt;

void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_PVT); i++) {
    CK[0] += ((unsigned char*)(&pvt))[i];
    CK[1] += CK[0];
  }
}

bool processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_PVT);

  
  while ( GPS.available() ) {
    byte c = GPS.read();
    if ( fpos < 2 ) {
      if ( c == UBX_HEADER[fpos] )
        fpos++;
      else
        fpos = 0;
    }
    else {      
      if ( (fpos-2) < payloadSize )
        ((unsigned char*)(&pvt))[fpos-2] = c;

      fpos++;

      if ( fpos == (payloadSize+2) ) {
        calcChecksum(checksum);
      }
      else if ( fpos == (payloadSize+3) ) {
        if ( c != checksum[0] )
          fpos = 0;
      }
      else if ( fpos == (payloadSize+4) ) {
        fpos = 0;
        if ( c == checksum[1] ) {
          return true;
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        fpos = 0;
      }
    }
  }
  return false;
}

void setup() 
{
  // Serial.begin(9600);
  GPS.begin(9600);
  // GPS.begin(19200);
  // GPS.begin(38400);

  pinMode(button, INPUT);
  digitalWrite(LED_BUILTIN, LOW);

  for(int i = 0; i < sizeof(UBLOX_INIT); i++) {                        
    GPS.write( pgm_read_byte(UBLOX_INIT+i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.display();
  delay(2000);

}

int numSV = 0;
unsigned long gSpeed = 0;
unsigned long lastScreenUpdate = 0;
int speedCalc = 0;
String speedoText = "N/A";

char* spinner = "/-\\|";
byte screenRefreshSpinnerPos = 0;
byte gpsUpdateSpinnerPos = 0;

void loop() {
  //############################################################################

  buttonPoll = digitalRead(button);
  if(buttonPoll == 1){
    // delay(50);
    delay(100);
    buttonPoll = digitalRead(button);
    if(buttonPoll == 0){
      state = old + 1;
    }
  }
  // else{
  //   delay(100);
  // }

  
  //################################################################################# 

  if ( processGPS() ) {
    // Serial.println(speedoText);
    // Serial.println(pvt.gSpeed);
    // Serial.println(pvt.numSV);
    
    gSpeed = pvt.gSpeed; 
    numSV = pvt.numSV;
    gpsUpdateSpinnerPos = (gpsUpdateSpinnerPos + 1) % 4;
  }
  
  unsigned long now = millis();
  // if ( now - lastScreenUpdate > 100 ) { // 10Hz
  if ( now - lastScreenUpdate > 50 ) { // 20Hz
    updateScreen();
    lastScreenUpdate = now;
    screenRefreshSpinnerPos = (screenRefreshSpinnerPos + 1) % 4;
  }
}

void updateScreen(){
  // Button state change
  switch (state){
      case 1:
        digitalWrite(LED_BUILTIN, HIGH);
        old = state;
        speedoText = "mph";
        speedCalc = (gSpeed / 447); // mph

        // ########## Delete after test the above ############
        // speedCalc = gSpeed * 0.0223694; // mph
        // speedCalc = (speedCalc / 10) + 0.5;
        // ########## Delete after test the above ############

        // speedoText = "Km/h";
        // speedCalc = gSpeed * 0.036; // kmh
        // speedCalc = (speedCalc / 10) + 0.5
        break;

      default:
        digitalWrite(LED_BUILTIN, LOW);
        old = 0;
        // speedoText = "mph";
        // speedCalc = gSpeed * 0.0223694; // mph
        // speedCalc = (speedCalc / 10) + 0.5
        speedoText = "Km/h";
        //speedCalc = (gSpeed / 277.8); // kmh

        // speedCalc = (gSpeed / 264.8); // kmh >>> Too high
        // speedCalc = (gSpeed / 271.3); // kmh >>> High
        // speedCalc = (gSpeed / 274.55); // kmh >>> High
        // speedCalc = (gSpeed / 276.175); // kmh >>> High
        speedCalc = (gSpeed / 276.581); // Kmh
        // speedCalc = (gSpeed / 276.9875); // Kmh >>> Low
        // speedCalc = (gSpeed / 277.8); // kmh
        
        // speedCalc = (gSpeed / 284.3); // kmh
        // speedCalc = (gSpeed / 290.8); // kmh

        // ########## Delete after test the above ############
        // speedCalc = (gSpeed * 0.036); // kmh
        // speedCalc = (speedCalc / 10) + 0.7;
        // ########## Delete after test the above ############
      break;
    }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(2);

  display.setCursor(0, 0);
  display.print(spinner[screenRefreshSpinnerPos]);
  
  display.setCursor(15, 0);
  display.print(spinner[gpsUpdateSpinnerPos]);

  display.setCursor(30, 0);
  display.print(numSV);

  display.setTextSize(5);
  display.setCursor(0, 25);
  display.print(speedCalc); // Km/h or mph, calculation
  
  display.setTextSize(1);
  display.setCursor(100, 55);
  display.print(speedoText); // Km/h or mph, text only

  display.display(); 
}

