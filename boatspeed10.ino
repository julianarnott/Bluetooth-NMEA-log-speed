// This code is a NMEA 0183 boatspeed through water instrument
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	pin D3	-	Paddlewheel circuit output pin connects to this
//	pin D10	-	Software serial RX pin	-	bluetooth TX pin connects to this
//	pin D11	-	Software serial TX pin	-	bluetooth RX pin connects to this
//	pin A0	-	calibration wiper pin (2) connects to this
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int pulsePin = 3;   // Pulse connected to this digital pin. The pulse from the paddlewheel circuit attaches here.
int calPot = 0;     // Calibration pot wiper
int cal_reading;
long baud;

#include <SoftwareSerial.h>
SoftwareSerial BTSerial(10, 11);             // RX, TX

#define DEFAULT_BAUD 4800                    // the baud rate standaed for nmea
#define DEFAULT_KMI_PER_PULSE (1./20000.)    // nautical miles per pulse (Airmar says 1/20e3)
#define DEFAULT_A_FILT 1                     // filter pole radians/sec.  time constant = 1/a seconds
#define DEFAULT_DT_PRINT 1                   // s between output strings    

// speed calcs
float kmi_per_pulse;                         // nautical miles per pulse (airmar spec is 1/20e3)
float speed_scale;                           // scale factor for calibration.  Multiplies kmi_per_pulse
float speed_raw = 0;                         // unfiltered speed
float speed_filt = 0;                        // low-pass filtered speed
float a_filt;                                // filter pole radians/sec.  time constant = 1/a seconds
float adt = 0;
long dt_update = 0; 
unsigned long last_update_time;

// pulse timing calcs
volatile long dtpulse;
volatile unsigned long edgetime;
volatile unsigned char gotdata=0;

// output stuff
float dt_print;                              // s between output strings
float dt_char= 3;                            // ms between output characters ~10/baud*1000
float print_time = 0, char_time=0;
char nmeastring[50] = "$VWVHW,,T,,M,0.00,N,0.00,K*";
int kt,kttenths,kph,kphtenths;
char checksum=0xAA;
int ii,jj;

// testing timer stuff
extern unsigned long timer0_overflow_count;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()   
{                
  // initialize the digital pin as an output:
  pinMode(pulsePin, INPUT);
  digitalWrite(pulsePin, HIGH);              // turns on pullup

  baud = DEFAULT_BAUD;
  kmi_per_pulse = DEFAULT_KMI_PER_PULSE;     // nautical miles per pulse (Airmar says 1/20e3)
  a_filt = DEFAULT_A_FILT;                   // filter pole radians/sec.  time constant = 1/a seconds
  dt_print = DEFAULT_DT_PRINT;               // s between output strings    
  
  dt_char= 20000/(float)baud;                // ms between output characters; minimum ~10/baud*1000
  Serial.begin(baud);
  BTSerial.begin(baud);
  attachInterrupt(1,falling_edge,FALLING);   // catch falling edge and handle with falling_edge();
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()                     
{
  CalibrationTrimPot();

  if (gotdata)
  {
    speed_raw = kmi_per_pulse/(float)dtpulse*3.60e9*speed_scale;   // kt
    dt_update = (micros() - last_update_time);                     // seconds
    last_update_time = micros();                                   // ms
    adt = a_filt*(float)dt_update*1.0e-6;
    if (adt < 1.0)
    {
      speed_filt = speed_filt + adt*(speed_raw - speed_filt);     // for stability a*dt < 1
    } else {
      speed_filt = speed_raw;                                     // un-filterable (shouldn't happen if set up properly and working)
    }  
    gotdata = 0;
    
    // make NMEA string
    kt = (int) speed_filt;
    kttenths = speed_filt*100 - kt*100;
    
    // this is a hack to check filtering
    kph = (int)speed_raw;
    kphtenths = speed_raw*100-kph*100;
    
    // this is the correct kph values
    kph = (int)(speed_filt*1.852);
    kphtenths = (speed_filt*185.2-kph*100);
    sprintf(nmeastring,"$VWVHW,,T,,M,%2d.%02d,N,%2d.%02d,K*",kt,kttenths,kph,kphtenths);
    checksum = 0;
    jj=1;
    while (nmeastring[jj+1]!=0)
    {
      checksum ^= nmeastring[jj];
      jj++;
    }
  }
   if (millis() > print_time)
  {
    if (millis() > char_time)
    {
      if (nmeastring[ii]==0)
      { // end of string
        if (checksum < 0x10) Serial.print('0');
        BTSerial.print(checksum,HEX);
        BTSerial.print("\r\n");                                   // <CR><LF> for android
        ii=0;
        print_time = (float)millis() + dt_print*1000;
      } else {
        BTSerial.print(nmeastring[ii]);
        ii++;
      }
      char_time = millis() + dt_char;
    }
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// what to do on a falling edge
void falling_edge(void)
{
    dtpulse = micros() - edgetime;
    edgetime = micros();
    gotdata = 1;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// calibration trim pot so can easily set on boat
void CalibrationTrimPot()
{
  cal_reading = analogRead(calPot);
  speed_scale = map(cal_reading, 0, 1023, 0, 2048);
  speed_scale = speed_scale/1000;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
