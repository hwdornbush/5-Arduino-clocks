/**************************************************************************
hwd 2023-04-18 modified for ESP32
hwd 2023-04-27 modify for NTP time clock source also
       Title:   GPS Clock with TFT display
      Author:   Bruce E. Hall, w8bh.net
        Date:   12 May 2022
    Hardware:   Blue Pill Microcontroller, 2.8" ILI9341 TFT display,
                Adafruit "Ultimate GPS" module v3
    Software:   Arduino IDE version  1.8.13 
                STM32 core version   2.2.0  
                TFT_eSPI library     2.4.32     
                Time library         1.6.0
                TinyGPSPlus library  1.0.3
                Timezone library     1.2.4
       Legal:   Copyright (c) 2022  Bruce E. Hall.
                Open Source under the terms of the MIT License. 
    
 Description:   GPS Clock, accurate to within a few microseconds!

                Set your local time zone and daylight saving rules at top
                of the sketch under TimeChangeRules: one rule for standard 
                time and one rule for daylight saving time.

                Choose Local/UTC time and 12/24hr format in defines near
                top of the sketch.  These can be toggled during runtime via
                touch presses, if your display module supports touch. 
                
                Connect GPS "Tx" line to Blue Pill PA10 (Serial RX1)
                Connect GPS "PPS" line to Blue Pill PA11

                A few notes about the touch control:
                This sketch contains 3 'screens' of information.
                Screen 0 is time/date/location/grid display;
                Screen 1 is a dual time/date display (local & utc)
                Screen 2 is latitude/longitude/altitude/speed/bearing.
                * To go from screen#0 to #1, touch the location at bottom right
                * To go from screen#0 to #2, touch the time
                * Touch anywhere on screens #1 and #2 to return to screen 0.
                * On Screen #0, touch the timezone to switch between UTC/local.
                * On screen #0, touch the AM/PM marker to toggle 12/24 hour modes.

               Check out the DEFINES near the top of sketch for some time formatting
               options.

               Note: I used Imperial units (altitude in feet, speed in miles per hour).
               If you, like most of the world, prefers metric units, try using the following:
                    alt = gps.altitude.meters
                    speed = gps.speed.kmph (kilometers per hour)
                    speed = gps.speed.mps (meters per second)
               Also, change the year/month/date order in showDate() to suit your preference.

 **************************************************************************/

#include <TFT_eSPI.h>                              // https://github.com/Bodmer/TFT_eSPI
#include <TimeLib.h>                               // https://github.com/PaulStoffregen/Time
#include <TinyGPS++.h>                             // https://github.com/mikalhart/TinyGPSPlus
#include <Timezone.h>                              // https://github.com/JChristensen/Timezone

#if defined(ESP32)
#include <WiFi.h>                                  // use this WiFi lib for ESP32, or
#elif defined (ESP8266)
#include <ESP8266WiFi.h>                           // use this WiFi lib for ESP8266
#endif
#include <WiFiUdp.h>  // from TimeNTP...
static const char ntpServerName[] = "us.pool.ntp.org";
WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets

time_t getNtpTime();
void sendNTPpacket(IPAddress &address);

TimeChangeRule EDT                                 // Local Timezone setup. My zone is EST/EDT.
  = {"EDT", Second, Sun, Mar, 2, -240};            // Set Daylight time here.  UTC-4hrs
TimeChangeRule EST                                 // For ex: "First Sunday in Nov at 02:00"
  = {"EST", First, Sun, Nov, 2, -300};             // Set Standard time here.  UTC-5hrs
TimeChangeRule *tz;                                // pointer to current time change rule
Timezone myTZ(EDT, EST);                           // create timezone object with rules above

#define GPS_TX                16                 // GPS data on hardware serial RX1
#define GPS_PPS               13                 // GPS 1PPS signal to hardware interrupt pin

#define USING_PPS             true                 // true if GPS_PPS line connected; false otherwise.
//#define DEBUGLEVEL            INFO                 // NONE, ERROR, INFO, DEBUG
#define BAUD_RATE             9600                 // data rate of GPS module
#define BAUDRATE            115200                 // data rate of serial monitor
#define GRID_SQUARE_SIZE         8                 // 0 (none), 4 (EM79), 6 (EM79vr), up to 10 char
#define FIRST_SCREEN             1                 // 0 = time/locn; 1= dual time; 2= location/elevation/speed
#define SYNC_MARGINAL         3600                 // orange status if no sync for 3600s = 1 hour
#define SYNC_LOST            86400                 // red status if no sync for 1 day
#define serial             Serial2                 // hardware UART#1 for GPS data 
#define TITLE            "GPS TIME"                // shown at top of display
#define TITLENTP         "NTP TIME"                // shown if NTP time source
#define SHOW_LOCAL_TIME       true                 // show local or UTC time at startup?
#define LOCAL_FORMAT_12HR     true                 // local time format 12hr "11:34" vs 24hr "23:34"
#define UTC_FORMAT_12HR      false                 // UTC time format 12 hr "11:34" vs 24hr "23:34"
#define HOUR_LEADING_ZERO    false                 // "01:00" vs " 1:00"
#define DATE_LEADING_ZERO     true                 // "Feb 07" vs. "Feb 7"
#define DATE_ABOVE_MONTH     false                 // "12 Feb" vs. "Feb 12" 

#define TIMECOLOR         TFT_CYAN
#define DATECOLOR       TFT_YELLOW
#define ZONECOLOR        TFT_GREEN
#define LABEL_FGCOLOR   TFT_YELLOW
#define LABEL_BGCOLOR     TFT_BLUE

#define SCREEN_ORIENTATION       1                 // landscape mode, should be '1' or '3'
// flip set for '1', reverse both if '3'
#define TOUCH_FLIP_X         false                 // touch: left-right orientation      
#define TOUCH_FLIP_Y         true                  // touch: up-down orientation
#define TOUCH_ADJUST_X           0                 // touch: fine tune x-coordinate
#define TOUCH_ADJUST_Y           0                 // touch: fine tune y-coordinate 
#define LED_PIN                 2                  // built in LED is on GPIO 2
#define WIFI_SSID          "AR300M"               
#define WIFI_PWD           "fuzzysoxfuzzy"      
#define NTP_SERVER         "pool.ntp.org"          // time.nist.gov, pool.ntp.org, etc    
    
#define TZ_RULE            "EST5EDT,M3.2.0/2:00:00,M11.1.0/2:00:00"
#define PRINTED_TIME             1                 // 0=NONE, 1=UTC, or 2=LOCAL
//#define TIME_FORMAT         COOKIE                 // COOKIE, ISO8601, RFC822, RFC850, RFC3339, RSS
#define DISPLAY_AMPM          true                 // if true, show 'A' for AM, 'P' for PM  

// ============ GLOBAL VARIABLES =====================================================

TFT_eSPI tft = TFT_eSPI();                         // display object 
TinyGPSPlus gps;                                   // gps object
time_t t,oldT      = 0;                            // UTC time, latest & displayed
time_t lt,oldLt    = 0;                            // Local time, latest & displayed
time_t lastSync    = 0;                            // UTC time of last GPS sync
volatile byte pps  = 0;                            // GPS one-pulse-per-second flag
bool use12hrFormat = LOCAL_FORMAT_12HR;            // 12-hour vs 24-hour format?
bool useLocalTime  = SHOW_LOCAL_TIME;              // display local time or UTC?
int screenID = 1;                                  // 0=time, 1=dual time, 2=location
char gridSquare[12]= {0};                          // holds current grid square string
int myClock = 0;                                   // hwd 0=NTP, 1=GPS
int lastNtpTime = 0;

typedef struct {
  int x;                                           // x position (left side of rectangle)
  int y;                                           // y position (top of rectangle)
  int w;                                           // width, such that right = x+w
  int h;                                           // height, such that bottom = y+h
} region;

region rPM     = {240,100,70,40};                // AM/PM screen region
region rTZ     = {240,60,70,40};                 // Time Zone screen region
region rTime   = {20,50,200,140};                // Time screen region
region rSeg    = {160,180,120,40};               // Segment screen region
region rStatus = {20,180,120,40};                // Clock status screen region
region rTitle  = {0,0,310,35};                   // Title bar screen region
region rLocn   = {180,165,140,80};               // Location screen region
region rSpkr   = {225,0,40,40};                  // Location of speaker icon
region rNTP    = {0,62,240,60};                  // hwd location of NTP choice
region rGPS    = {0,122,240,60};                  // hwd location of GPS cloice

// //================================  Speaker Support ================================


// void drawSpeakerOn() {                           // draw speaker icon, filled color 
//   const int x=225, y=5, c=TFT_YELLOW;            // position and color
//   tft.fillTriangle(x,y+10,x+13,y,x+13,y+20,c);   // draw diaphragm = filled triangle
//   tft.fillRect(x,y+6,5,8,c);                     // draw coil = filled rectangle
// }

// void drawSpeakerOff() {                          // draw speaker icon, outline only
//   const int x=225, y=5, c=TFT_YELLOW;            // position and outline color
//   tft.fillRect(x,y,14,21,TFT_BLUE);              // erase old icon
//   tft.drawTriangle(x,y+10,x+13,y,x+13,y+20,c);   // draw diaphragm = open triangle
//   tft.drawRect(x,y+6,5,8,c);                     // draw coil = open rectangle
// }

// void drawSpeaker() {                             // draw icon according to status:
//   if (secondTick) drawSpeakerOn();               // doing ticks = filled icon 
//   else drawSpeakerOff();                         // no ticks = outline icon
// }

// //================================  Battery Monitor ================================

// void soundAlarm()
// // generate an alarm tone, one second in duration
// {
//   const int hiTone=2000, loTone=1000,            // set alarm pitch here
//     duration=80, cycles=5;                       // time = 2*cycles*duration = 0.80s
//   for (int i=0; i<cycles; i++)                   
//   {
//     tone(AUDIO,hiTone); delay(duration);         // alternate between hi-pitch tone
//     tone(AUDIO,loTone); delay(duration);         // and low-pitch tone
//   }
//   noTone(AUDIO);                                 // silence alarm when done
// }

// void lowBatteryWarning()
// {
//   soundAlarm();                                   // audibly alert user
// }

// float batteryVoltage()
// // analog input is qualtized in 1024 steps, with value of 1023 = 3.3V
// // therefore, voltage input = (reading/1023)*3.3 = reading/310
// // battery voltage = 2 x measured voltage = reading/155
// {
//   int raw = analogRead(BATTERY);                  // read battery pin
//   return raw/155.0 + VOLTAGE_ADJUST;              // and convert to batt voltage
// }

// int batteryLevel(float v)
// // Returns a level of 0..4, where 0 is depleted and 4 is full (100%)
// // Below 3.4V, the regulator fails to maintain a 3.3V supply and therefore
// // the returned analog value will 'stall' around 3.3V even as the voltage plummets.
// {              
//   if      (v>=3.90) return 4;                     // 3.90-4.20V or 100%
//   else if (v>=3.75) return 3;                     // 3.75-3.89V or 75%
//   else if (v>=3.65) return 2;                     // 3.65-3.74V or 50%
//   else if (v>=3.49) return 1;                     // 3.49-3.64v or 25%
//   else return 0;                                  // <3.49v = DEPLETED
// }

// void showBatteryVoltage(float v) {
//   const int x=250,y=12;
//   tft.setTextColor(TFT_YELLOW,TFT_BLUE); 
//   tft.drawFloat(v,2,x,y,1);
// }

// void drawBatteryIcon(int level)
// // draws a batter icon according to the level 0=4
// // first draws a one-pixel-thick horizontal rectangle for battery icon
// // then draws boxes inside to indicate the level.
// {
//   const int boxH=5, boxW=6, 
//      x=250, y=4, nBoxes=4;                         // screen position, box size, etc
//   const int w = (boxW +1) * nBoxes +1;             // total width of battery icon
//   const int h = boxH + 2;                          // total height of battery icon
//   int outlineColor = (level>0) ?                  
//     TFT_WHITE : TFT_RED;                           // set outline color
//   tft.drawRect(x,y,w,h,outlineColor);              // draw outline
//   tft.fillRect(x+1,y+1,w-2,h-2,TFT_BLACK);         // erase previous data
                                                                 
//   int boxColor = TFT_GREEN;                        // default data color
//   if (level<2) boxColor = TFT_YELLOW;              // low battery warning color
//   for (int i=0; i<level; i++)                      // now draw a number of boxes
//   {
//     tft.fillRect(x+ i*(boxW+1)+1, y+1, boxW, boxH, boxColor);
//   }
// }

// void checkBatteryTimer()
// {
//   if ((millis()-battTimer)> BATTERYINTERVAL)       // has enough time elapsed?
//   {
//     battTimer = millis();                          // reset timer
//     float v = batteryVoltage();                    // get battery voltage         
//     int level = batteryLevel(v);                   // and level 0..4
//     drawBatteryIcon(level);                        // display battery icon
//     showBatteryVoltage(v);                         // and voltage
//     if (level) setBrightness(level*25);            // dim screen to conserve power
//     else {                                         // power is nearly exhausted, so
//       soundAlarm();                                // sound an alarm
//       setBrightness(10);                           // and make display very dim
//     }
//   }
// }



// ============ GRID SQUARE ROUTINES ================================================

void getGridSquare(char *gs, float lat, float lon, const byte len=10) {
  int lon1,lon2,lon3,lon4,lon5;                    // GridSquare longitude components
  int lat1,lat2,lat3,lat4,lat5;                    // GridSquare latitude components 
  float remainder;                                 // temp holder for residuals

  gs[0] = 0;                                       // if input invalid, return null
  lon += 180;                                      // convert (-180,180) to (0,360)
  lat += 90;                                       // convert (-90,90) to (0,180);
  if ((lon<0)||(lon>360)) return;                  // confirm good lon value
  if ((lat<0)||(lat>180)) return;                  // confirm good lat value
  if (len>10) return;                              // allow output length 0-10 chars
  
  remainder = lon;                                 // Parsing Longitude coordinates:
  lon1 = remainder/20;                             // first: divisions of 20 degrees
  remainder -= lon1*20;                            // remove 1st coord contribution 
  lon2 = remainder/2;                              // second: divisions of 2 degrees
  remainder -= lon2*2;                             // remove 2nd coord contribution
  lon3 = remainder*12;                             // third: divisions of 5 minutes   
  remainder -= lon3/12.0;                          // remove 3nd coord contribution
  lon4 = remainder*120;                            // forth: divisions of 30 seconds
  remainder -= lon4/120.0;                         // remove 4th coord contribution
  lon5 = remainder*2880;                           // fifth: division of 1.25 seconds

  remainder = lat;                                 // Parsing Latitude coordinates:           
  lat1 = remainder/10;                             // first: divisions of 10 degrees
  remainder -= lat1*10;                            // remove 1st coord contribution
  lat2 = remainder;                                // second: divisions of 1 degrees
  remainder -= lat2;                               // remove 2nd coord contribution
  lat3 = remainder*24;                             // third: divisions of 2.5 minutes
  remainder -= lat3/24.0;                          // remove 3rd coord contribution
  lat4 = remainder*240;                            // fourth: divisions of 15 seconds
  remainder -= lat4/240.0;                         // remove 4th coord contribution
  lat5 = remainder*5760;                           // fifth: divisions of 0.625 seconds  

  gs[0] = lon1 + 'A';                              // first coord pair are upper case alpha
  gs[1] = lat1 + 'A';
  gs[2] = lon2 + '0';                              // followed by numbers
  gs[3] = lat2 + '0';
  gs[4] = lon3 + 'a';                              // followed by lower case alpha
  gs[5] = lat3 + 'a';
  gs[6] = lon4 + '0';                              // followed by numbers
  gs[7] = lat4 + '0';
  gs[8] = lon5 + 'A';                              // followed by upper case alpha
  gs[9] = lat5 + 'A';
  gs[len] = 0;                                     // set desired string length (0-10 chars)
}

void showGridSquare(int x, int y) {
  const int f=4;                                   // text font
  char gs[12];                                     // buffer for new grid square
  if (!gps.location.isValid()) return;             // leave if no fix
  float lat = gps.location.lat();                  // get latitude
  float lon = gps.location.lng();                  // and longitude
  getGridSquare(gs,lat,lon, GRID_SQUARE_SIZE);     // compute current grid square
  tft.setTextPadding(tft.textWidth("WWWWWWWWWW",4)); // width of 10-char display
  tft.drawString(gs,x,y,f);                        // show current grid square
  tft.setTextPadding(0);
}

// ============ SCREEN 2 (LAT/LON) DISPLAY ROUTINES ==============================================

void locationScreen() {
  tft.fillScreen(TFT_BLACK);                       // start with empty screen
  tft.fillRoundRect(0,0,319,32,10,LABEL_BGCOLOR);  // put title bar at top
  tft.drawRoundRect(0,0,319,239,10,TFT_WHITE);     // draw edge around screen
  tft.setTextColor(LABEL_FGCOLOR,LABEL_BGCOLOR);   // set label colors
  tft.drawString("GPS LOCATION",20,4,4);           // show title at top
  tft.drawString("  Altitude    ",30,180,2);       // label for clock status
  tft.drawString("  Speed/Course   ",160,180,2);   // label for segment status
  tft.setTextColor(DATECOLOR);
  tft.drawString("mph",200,204,2);                 // label for speed units
  tft.drawString("deg",275,204,2);                 // label for bearing units
  tft.drawString("ft.",94,204,2);                  // label for altitude units
}

void showAltitude() {
  int x=90, y=200, f=4;                            // screen position & font
  int alt = gps.altitude.feet();                   // get altitude in feet
  tft.setTextPadding(tft.textWidth("18888",f));    // width of altitude display
  tft.drawNumber(alt,x,y,f);                       // display altitude
}

void showSpeed() {
  int x=196, y=200, f=4;                           // screen position & font
  int dir = gps.course.deg();                      // get bearing in degrees
  int mph = gps.speed.mph();                       // get speed in miles/hour
  tft.setTextPadding(tft.textWidth("888",f));      // width of speed & bearing 
  tft.drawNumber(mph,x,y,f);                       // display speed  
  x += 76;                                         // x-offset for bearing            
  if (mph) tft.drawNumber(dir,x,y,f);              // display bearing
  else tft.drawString("",x,y,f);                   // but no bearing if speed=0 
}

void showLatLon(int x, int y, int f, int spacer, int decimals) {
  char c;                                          // cardinal points N,S,E,W
  if (!gps.location.isValid()) return;             // only show valid data
  double lat = gps.location.lat();                 // get current latitude
  double lon = gps.location.lng();                 // get current longitude
  tft.setTextColor(TIMECOLOR, TFT_BLACK);          // set time color
  tft.drawFloat(abs(lat),decimals,x,y,f);          // display latitude
  tft.drawFloat(abs(lon),decimals,x,y+spacer,f);   // display longitude
  c = (lat>0)?'N':'S';                             // neg latiude is S                
  tft.drawChar(c,x+6,y,4);                         // show N/S for latitude
  c = (lon>0)?'E':'W';                             // neg longitude is W
  tft.drawChar(c,x+6,y+spacer,4);                  // show E/W for longitude
}

void updateLocationScreen() {
  int x=280, y=52, f=7;                            // screen position & font
  if (!gps.location.isValid()) return;             // only show valid data
  tft.setTextDatum(TR_DATUM);                      // right-justify the following numbers
  showLatLon(x,y,f,60,5);                          // display lat/lon @ 5 decimals
  showAltitude();                                  // show altitude in feet
  showSpeed();                                     // show speed in mph and direction in degrees
  tft.setTextDatum(TL_DATUM);                      // turn off right justification
  tft.setTextPadding(0);                           // turn off text padding
}

// ============ SCREEN 1 (DUAL TIME) DISPLAY ROUTINES ==========================================

void dualScreen() {
  tft.fillScreen(TFT_BLACK);                       // start with empty screen
  tft.fillRoundRect(0,0,319,32,10,LABEL_BGCOLOR);  // title bar for local time
  tft.fillRoundRect(0,126,319,32,10,LABEL_BGCOLOR);// title bar for UTC
  tft.setTextColor(LABEL_FGCOLOR,LABEL_BGCOLOR);   // set label colors
  if (myClock==0) {                                // if this is NTP time source
    tft.drawString(TITLENTP,130,4,4);              // show NTP title
  } else {
    tft.drawString(TITLE,130,4,4);                 // show GPS title
  }
  tft.drawRoundRect(0,0,319,110,10,TFT_WHITE);     // draw edge around local time
  tft.drawRoundRect(0,126,319,110,10,TFT_WHITE);   // draw edge around UTC
}

void showTimeBasic (int h, int m, int s, 
                    bool hr12, int x, int y) {
  const int f=7;                                   // time font
  if (hr12) {                                      // adjust hours for 12 vs 24hr format:
    if (h==0) h=12;                                // 00:00 becomes 12:00
    if (h>12) h-=12;                               // 13:00 becomes 01:00
  }
  if (h<10) {                                      // is hour a single digit?
    if ((!hr12)||(HOUR_LEADING_ZERO))              // 24hr format: always use leading 0
      x+= tft.drawChar('0',x,y,f);                 // show leading zero for hours
    else {
      tft.setTextColor(TFT_BLACK,TFT_BLACK);       // black on black text     
      x+=tft.drawChar('8',x,y,f);                  // will erase the old digit
      tft.setTextColor(TIMECOLOR,TFT_BLACK);      
    }
  }
  x+= tft.drawNumber(h,x,y,f);                     // hours
  x+= tft.drawChar(':',x,y,f);                     // show ":"
  if (m<10) x+= tft.drawChar('0',x,y,f);           // leading zero for minutes
  x+= tft.drawNumber(m,x,y,f);                     // show minutes          
  x+= tft.drawChar(':',x,y,f);                     // show ":"
  if (s<10) x+= tft.drawChar('0',x,y,f);           // add leading zero for seconds
  x+= tft.drawNumber(s,x,y,f);                     // show seconds  
}

void showTimeD (time_t t, bool hr12, int x, int y) {
  tft.setTextColor(TIMECOLOR, TFT_BLACK);          // set time color
  int h=hour(t); int m=minute(t); int s=second(t); // get hours, minutes, and seconds
  //showAMPM(h);                                     // display AM/PM, if needed
  showTimeBasic(h,m,s,hr12,x,y);
}

void showDateD (time_t t, int x, int y) {
  const int f=4,yspacing=30;                       // screen font, spacing
  const char* months[] = {"JAN","FEB","MAR",
     "APR","MAY","JUN","JUL","AUG","SEP","OCT",
     "NOV","DEC"};
  if (!lastSync) return;                           // if no time yet, forget it
  tft.setTextColor(DATECOLOR, TFT_BLACK);
  int i=0, m=month(t), d=day(t);                   // get date components  
  tft.fillRect(x,y,50,60,TFT_BLACK);               // erase previous date       
  if (DATE_ABOVE_MONTH) {                          // show date on top -----
    if ((DATE_LEADING_ZERO) && (d<10))             // do we need a leading zero?
      i = tft.drawNumber(0,x,y,f);                 // draw leading zero
    tft.drawNumber(d,x+i,y,f);                     // draw date 
    y += yspacing;                                 // and below it, the month
    tft.drawString(months[m-1],x,y,f);             // draw month
  } else {                                         // put month on top ----
    tft.drawString(months[m-1],x,y,f);             // draw month
    y += yspacing;                                 // and below it, the date
    if ((DATE_LEADING_ZERO) && (d<10))             // do we need a leading zero?
      x+=tft.drawNumber(0,x,y,f);                  // draw leading zero
    tft.drawNumber(d,x,y,f);                       // draw date
  }
}

void showTimeZoneD (int x, int y) {
  const int f=4;                                   // text font
  tft.setTextColor(LABEL_FGCOLOR,LABEL_BGCOLOR);   // set text colors
  tft.setTextPadding(tft.textWidth("WWWWW",4));    // width of 5-char TZ
  if (!useLocalTime) 
    tft.drawString("UTC",x,y,f);                   // UTC time
  else if (tz!=NULL)
    tft.drawString(tz->abbrev,x,y,f);              // show local time zone
  tft.setTextPadding(0);
}

void showTimeDateD (time_t t, time_t oldT, bool hr12, int x, int y) {   
  showTimeD(t,hr12,x,y);                           // display time HH:MM:SS 
  if ((!oldT)||(hour(t)!=hour(oldT)))              // did hour change?
    showTimeZoneD(x,y-44);                         // update time zone
  if (day(t)!=day(oldT))                           // did date change? 
    showDateD(t,x+250,y);                          // update date
}

void updateDualScreen() {
  useLocalTime = true;                             // use local timezone
  showTimeDateD(lt,oldLt,use12hrFormat,10,46);     // show new local time
  useLocalTime = false;                            // use UTC timezone
  showTimeDateD(t,oldT,UTC_FORMAT_12HR,10,172);    // show new UTC time
}

// misc NTP routines

void showConnectionProgress(){  
  int elapsed = 0;           
  tft.drawString("WiFi starting",5,50,4);                   
  while (WiFi.status()!=WL_CONNECTED) {            // while waiting for connection                             
    tft.drawNumber(elapsed++,230,50,4);            // show we are trying!
    delay(1000);
  }
  tft.drawString("IP = " +                         // connected to LAN now
    WiFi.localIP().toString(),5,80,4);             // so show IP address 
  elapsed = 0;
  tft.drawString("Waiting for NTP",5,140,4);       // Now get NTP info
  // while (timeStatus()!=timeSet) {                  // wait until time retrieved
  //   events();                                      // allow ezTime to work
  //   tft.drawNumber(elapsed++,230,140,4);           // show we are trying
  //   delay(1000);
  setSyncProvider(getNtpTime);  // from TimeNTP_ESP8266WiFi.ino
  setSyncInterval(300);
}

void printDigits(int digits)
{
  // utility for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void printTime() {                                 // print time to serial port
  if (!PRINTED_TIME) return;                       // option 0: dont print
  Serial.print("TIME: ");
//  if (PRINTED_TIME==1)
//    Serial.println(dateTime(TIME_FORMAT));         // option 1: print UTC time
//  else 
//    Serial.println(local.dateTime(TIME_FORMAT));   // option 2: print local time
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(".");
  Serial.print(month());
  Serial.print(".");
  Serial.print(year());
  Serial.println();
}

void blink(int count=1) {                          // diagnostic LED blink
  pinMode(LED_PIN,OUTPUT);                         // make sure pin is an output
  for (int i=0; i<count; i++) {                    // blink counter
    digitalWrite(LED_PIN,0);                       // turn LED on 
    delay(200);                                    // for 0.2s
    digitalWrite(LED_PIN,1);                       // and then turn LED off
    delay(200);                                    // for 0.2s
  } 
  pinMode(LED_PIN,INPUT);                          // works for both Vcc & Gnd LEDs.
}

// ============ SCREEN 0 (SIGNLE TIME) DISPLAY ROUTINES ========================================

void timeScreen() {
  tft.fillScreen(TFT_BLACK);                       // start with empty screen
  tft.fillRoundRect(0,0,319,32,10,LABEL_BGCOLOR);  // put title bar at top
  tft.drawRoundRect(0,0,319,239,10,TFT_WHITE);     // draw edge around screen
  tft.setTextColor(LABEL_FGCOLOR,LABEL_BGCOLOR);   // set label colors
  tft.drawString(TITLE,20,4,4);                    // show title at top
  tft.drawString(" Grid Square  ",20,165,2);       // label for grid square
  tft.drawString(" Location ",186,165,2);          // label for lat/long data
  tft.setTextColor(DATECOLOR);
  useLocalTime  = SHOW_LOCAL_TIME;                 // start with TZ preference 
  showDate(t);                                    
  showTimeZone();                             
  showLocation();                                  // grid square & lat/lon
  //drawSpeaker();
}

void showTime(time_t t) {
  int x=20, y=85;                                  // screen position & font
  tft.setTextColor(TIMECOLOR, TFT_BLACK);          // set time color
  int h=hour(t); int m=minute(t); int s=second(t); // get hours, minutes, and seconds
  showAMPM(h);                                     // display AM/PM, if needed
  showTimeBasic(h,m,s,use12hrFormat,x,y);
}

void showDate(time_t t) {
  int x=20,y=35,f=4;                              // screen position & font
  const char* days[] = {"Sun","Mon","Tue",
    "Wed","Thu","Fri","Sat"};
  if (t<=0) return;                                // wait for valid date
  tft.setTextColor(DATECOLOR, TFT_BLACK);
  tft.fillRect(x,y,265,26,TFT_BLACK);              // erase previous date  
  x+=tft.drawString(days[weekday(t)-1],x,y,f);     // show day of week
  x+=tft.drawString(", ",x,y,f);                   // and     
  x+=tft.drawNumber(month(t),x,y,f);               // show date as month/day/year
  x+=tft.drawChar('/',x,y,f);
  x+=tft.drawNumber(day(t),x,y,f);
  x+=tft.drawChar('/',x,y,f);
  x+=tft.drawNumber(year(t),x,y,f);
}

void showAMPM (int hr) {
  int x=250,y=110,ft=4;                             // screen position & font
  tft.setTextColor(TIMECOLOR,TFT_BLACK);           // use same color as time
  if (!use12hrFormat) 
    tft.fillRect(x,y,50,20,TFT_BLACK);             // 24hr display, so no AM/PM 
  else if (hr<12) 
    tft.drawString("AM",x,y,ft);                   // before noon, so draw AM
  else 
    tft.drawString("PM",x,y,ft);                   // after noon, so draw PM
}

void showTimeZone () {
  int x=250,y=80,ft=4;                             // screen position & font
  tft.setTextColor(ZONECOLOR,TFT_BLACK);           // zone has its own color
  if (!useLocalTime) 
    tft.drawString("UTC",x,y,ft);                  // UTC time
  else if (tz!=NULL)
    tft.drawString(tz->abbrev,x,y,ft);             // show local time zone
}

void showTimeDate(time_t t, time_t oldT) {         
  showTime(t);                                     // display time (includes AM/PM)
  if ((!oldT)||(hour(t)!=hour(oldT)))              // did hour change?
    showTimeZone();                                // update time zone
  if (day(t)!=day(oldT))                           // did date change? 
    showDate(t);                                   // update date
}

void showSatellites() {
  int x=200,y=200,w=50,h=28,ft=4;                  // screen position and size
  tft.setTextColor(TFT_YELLOW);                    
  tft.fillRect(x,y,w,h,TFT_BLACK);                 // erase previous count
  tft.drawNumber(satCount(),x,y,ft);               // show latest satellite count
}

time_t prevDisplay = 0;   //when the digital clock was displayed

void showClockStatus() {
  const int x=290,y=1,w=28,h=29,f=2;               // screen position & size
  int color;
  int syncAge;
  if (second()%10) return;                         // update every 10 seconds
  if (myClock==0) {
    syncAge = now()-lastNtpTime;         // how long has it been since last sync?
    //syncAge = now()-prevDisplay;         // how long has it been since last sync?
    if (syncAge < SYNC_MARGINAL)                     // GREEN: time is good & in sync
      color = TFT_GREEN;
    else if (syncAge < SYNC_LOST)                    // ORANGE: sync is 1-24 hours old
      color = TFT_ORANGE;
    else color = TFT_RED;                            // RED: time is stale, over 24 hrs old
    if (WiFi.status()!=WL_CONNECTED) {               //          
      color = TFT_DARKGREY;                          // GRAY: WiFi connection was lost
      WiFi.disconnect();                             // so drop current connection
      WiFi.begin(WIFI_SSID,WIFI_PWD);                // and attempt to reconnect
    }
  }
  else syncAge = now()-lastSync;                    // how long has it been since last sync?
  if (syncAge < SYNC_MARGINAL)                     // time is good & in sync
    color = TFT_GREEN;
  else if (syncAge < SYNC_LOST)                    // sync is 1-24 hours old
    color = TFT_ORANGE;
  else color = TFT_RED;                            // time is stale & should not be trusted
  tft.fillRoundRect(x,y,w,h,10,color);             // show clock status as a color
  tft.setTextColor(TFT_BLACK,color);
  tft.drawNumber(satCount(),x+8,y+6,f);            // and number of satellites
}

void showLocation() {
  char c; int x=280, y=185, f=4;                   // screen position & font                       
  if (!gps.location.isValid()) return;             // only show valid data
  tft.setTextColor(TIMECOLOR, TFT_BLACK);          // set time color
  showGridSquare(20,y);                            // display grid sqare
  tft.setTextDatum(TR_DATUM);                      // right-justify numbers
  showLatLon(x,y,f,20,4);                          // display lat/lon to 4 decimals   
  tft.setTextDatum(TL_DATUM);                      // back to left justification         
}

void updateTimeScreen() {
  if (useLocalTime) showTimeDate(lt,oldLt);        // either show local time    
  else showTimeDate(t,oldT);                       // or UTC time
  if (!(second()%20)) showLocation();              // update locn every 20s
}


// ============ CLOCK & GPS ROUTINES ================================================

time_t localTime() {                
  if (!lastSync) return 0;                         // make sure time has been set
  else return myTZ.toLocal(now(),&tz);             // before converting to local
}

void syncWithGPS() {                               // set Arduino time from GPS
  if (!gps.time.isValid()) return;                 // continue only if valid data present
  if (gps.time.age()>1000) return;                 // dont use stale data
  int h = gps.time.hour();                         // get hour value
  int m = gps.time.minute();                       // get minute value
  int s = gps.time.second();                       // get second value
  int d = gps.date.day();                          // get day
  int mo= gps.date.month();                        // get month
  int y = gps.date.year();                         // get year
  setTime(h,m,s,d,mo,y);                           // set the system time
  adjustTime(1);                                   // and adjust forward 1 second
  lastSync = now();                                // remember time of this sync
}

int satCount() {                                   // return # of satellites in view
  int sats = 0;                                    // number of satellites
  if (gps.satellites.isValid())
    sats = gps.satellites.value();                 // get # of satellites in view
  return sats;                                   
}

void syncCheck() {
  if (pps||(!USING_PPS)) syncWithGPS();            // is it time to sync with GPS?
  pps=0;                                           // reset flag, regardless
}


void feedGPS() {                                   // feed GPS into tinyGPS parser
  while (serial.available()) {                     // look for data from GPS module
     char c = serial.read();                       // read in all available chars
     gps.encode(c);                                // and feed chars to GPS parser
  }
}

void ppsHandler() {                                // 1pps interrupt handler:
  pps = 1;                                         // flag that signal was received
}


// ============ TOUCH ROUTINES ===================================================

void newScreen () {
  oldLt=oldT=0;                                    // force time/date redraw
  switch (screenID) {
    case 1: dualScreen(); break;
    case 2: locationScreen(); break;
    default: timeScreen(); break;
  }
}



// void tick() {
//   if (HOUR_TICKS && (!minute(t)) && (!second(t)))  // if its the top of the hour...
//     tone(AUDIO,TICK_PITCH,600);                    // emit the hour tone
//   else if ((minute(t)==59) &&
//   (second(t) > 60-HOUR_TICKS))                     // if last few seconds in hour
//     tone(AUDIO,TICK_PITCH/2,300);                  // emit the hour tone, 1 octave lower
//   else if (secondTick)                            // otherwise, just emit a short
//     tone(AUDIO,TICK_PITCH,3);                      // tick every second
// }

void updateDisplay() {
  t = now();                                       // check latest time
  if (t!=oldT) {                                   // are we in a new second yet?
    lastNtpTime = t;
    lt = localTime();                              // keep local time current
    Serial.println(localTime());
    switch (screenID) {
      case 1:  updateDualScreen(); break;
      case 2:  updateLocationScreen(); break;
      default: updateTimeScreen(); break;
    }
    showClockStatus();                             // show GPS status & sat count 
    printTime();                                   // send time to serial port
//    checkBatteryTimer();
    oldT=t; oldLt=lt;                              // remember currently displayed time
    //tick();
  }
}

bool touched() {                                   // true if user touched screen     
  const int threshold = 500;                       // ignore light touches
  return tft.getTouchRawZ() > threshold;
}

boolean inRegion (region b, int x, int y) {        // true if regsion contains point (x,y)
  if ((x < b.x ) || (x > (b.x + b.w)))             // x coordinate out of bounds? 
    return false;                                  // if so, leave
  if ((y < b.y ) || (y > (b.y + b.h)))             // y coordinate out of bounds?
    return false;                                  // if so, leave 
  return true;                                     // x & y both in bounds 
}

void touchedTime(int x, int y) {
  screenID = 1;                                    // user wants dual time
  newScreen();                                     // switch to a new screen
}

void touchedLocation(int x, int y) {
  screenID = 2;                                    // user wants to see location data
  newScreen();                                     // switch to a new screen
}

void touchedPM(int x, int y) {
  use12hrFormat = !use12hrFormat;                  // toggle 12/24 hr display
  if (useLocalTime) showTime(lt);                  // show local time 
  else showTime(t);                                // or show UTC time 
}

void touchedTZ(int x, int y) {                   
  useLocalTime = !useLocalTime;                    // toggle the local/UTC flag
  use12hrFormat = useLocalTime;                    // local time is 12hr; UTC is 24hr
  if (useLocalTime) showTimeDate(lt,0);            // show local time zone
  else showTimeDate(t,0);                          // or show UTC time zone
}

void touchedTitle(int x, int y) {
}

bool getTouchPoint(uint16_t &x, uint16_t &y) {
  x=0; y=0;
  tft.getTouch(&x,&y);                             // get x,y touch coordinates
  if ((x>0)&&(y>0)&&(x<320)&&(y<240)) {            // are they valid coordinates?
    if (TOUCH_FLIP_X) x = 320-x;                   // flip orientations, if necessary
    if (TOUCH_FLIP_Y) y = 240-y;
    x += TOUCH_ADJUST_X;                           // fine tune coordinates
    y += TOUCH_ADJUST_Y; 
  } 
  return (x>0)&&(y>0);                             // if true, point is valid
}

void checkForTouch() {
  uint16_t x,y;
  if (touched()) {                                 // did user touch the display?
    //setBrightness(100);                            // temporarily light up screen
    getTouchPoint(x,y);                            // get touch coordinates
    if (screenID) {                                // if user touched anywhere in dual/locn screens
      screenID=0;                                  // switch to single display
      newScreen();                                 // and show it
    } 
    else if (inRegion(rTime,x,y))                  // was time touched?
      touchedTime(x,y);
    else if (inRegion(rLocn,x,y))                  // was location touched?
      touchedLocation(x,y);
    else if (inRegion(rPM,x,y))                    // was AM/PM touched?
      touchedPM(x,y);                              
    else if (inRegion(rTZ,x,y))                    // was timezone touched?
      touchedTZ(x,y);  
    // else if (inRegion(rSpkr,x,y)) {
    //   secondTick = !secondTick;
    //   drawSpeaker();
    //}
    delay(300);                                    // touch debouncer
  }  
}


// ============ MAIN PROGRAM ===================================================

// //void setBrightness(int level)                     // level 0 (off) to 100 (full on)       
// {
//   if (PNP_DRIVER) level = 100-level;              // invert levels if PNP driver present
//   analogWrite(BACKLIGHT, level*255/100);          // conver 0-100 to 0-255
//   delay(5);  
// }

void initScreen() {
  tft.init();                                      // initialize display library
  tft.setRotation(SCREEN_ORIENTATION);             // portrait screen orientation
  // pinMode(BACKLIGHT,OUTPUT);                       // enable backlight control
  // pinMode(BATTERY,INPUT);
  // setBrightness(100);                              // turn backlight on 100%
}

void startupScreen() {  
  tft.fillScreen(TFT_BLACK);                       // start with empty screen
  tft.fillRoundRect(0,0,319,30,10,LABEL_BGCOLOR);  // title bar
  tft.drawRoundRect(0,0,319,239,10,TFT_WHITE);     // draw edge screen
  tft.setTextColor(LABEL_FGCOLOR,LABEL_BGCOLOR);   // set label colors
  tft.drawCentreString("NTP",160,2,4);             // show sketch title on screen
  tft.setTextColor(LABEL_FGCOLOR, TFT_BLACK);      // set text color
}

void setupNTP() {                                 // if user chooses NTP clock
  startupScreen();                                   // show title
  blink(3);                                        // show sketch is starting
  Serial.begin(BAUDRATE);                          // open serial port
  //setDebug(DEBUGLEVEL);                            // enable NTP debug info
  //setServer(NTP_SERVER);                           // set NTP server
  WiFi.begin(WIFI_SSID, WIFI_PWD);                 // start WiFi
  showConnectionProgress();                        // WiFi and NTP may take time
  //local.setPosix(TZ_RULE);                         // estab. local TZ by rule 
  dualScreen();                                 // show title & labels 
  lastSync = now();
}

void setupGPS() {                                 // hwd if user chose GPS clock
  screenID = FIRST_SCREEN;                         // user preference for startup screen
  newScreen();                                     // show title & labels
  attachInterrupt(GPS_PPS, ppsHandler, RISING);   // enable 1pps GPS time sync
}

void chooseClock() {
  tft.fillScreen(TFT_BLACK);                      // fill screen with black
  tft.fillRoundRect(0,0,319,30,10,LABEL_BGCOLOR); // title bar
  tft.drawRoundRect(0,0,319,239,10,TFT_WHITE);    //draw edge screen
  tft.setTextColor(LABEL_FGCOLOR,LABEL_BGCOLOR);  // set title color
  tft.drawCentreString("CHOOSE TIME SOURCE",160,2,4); // show title
  tft.setTextColor(LABEL_FGCOLOR,TFT_BLACK);      // set body color
  tft.drawCentreString("NTP time source",160,62,4);
  tft.drawCentreString("GPS time source",160,122,4);
  myClock = -1;
  while(myClock == -1) {                             // wait for selection
    uint16_t x,y;
    if (touched()) {                              // was screen touched
      getTouchPoint(x,y);
      tft.drawCentreString("touched",160,202,4);
      if (inRegion(rNTP,x,y)) {                    // was NTP touched
        tft.drawCentreString("NTP",160,162,4);
        myClock = 0;                               // NTP clock selected
        setupNTP();                                // set up NTP
      }          
      if (inRegion(rGPS,x,y)) {                   // was GPS touched?
        tft.drawCentreString("GPS",160,162,4);
        myClock = 1;                              // GPS clock selected 
        setupGPS();                               // set up GPS
      }
      delay(300);                                 // debounce touch
    }
  }      
}

void setup() {
  serial.begin(BAUD_RATE);                         // open UART connection to GPS
  initScreen();
  chooseClock();                                   // choose which time source                     
}

void loop() {
  if (myClock == 1) {
    feedGPS();                                       // decode incoming GPS data
    syncCheck();                                     // synchronize time with GPS
    updateDisplay();                                 // keep display current
    checkForTouch();                                 // act on any user touch     
  }
  else {                                             // my clock must be NTP
    //events();                                        // get periodic NTP updates

    updateDisplay();                                 // update clock every second
  }                           
}

/*-------- NTP code ----------*/
// from TimeNTP...

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      //return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
      //lastSync = now();
      return secsSince1900 - 2208988800UL;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}