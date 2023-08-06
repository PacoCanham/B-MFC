#include <Pangodream_18650_CL.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <TinyGPS++.h>
#include <TinyGPSPlus.h>
//Screen setup
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

//definitions for geometric conversions
#define deg2rad 0.017453292519943295 //(PI / 180)
#define rad2deg 57.29577951308232087 //(180/ PI)
#define a 6377563.396       // OSGB semi-major axis
#define b 6356256.91        // OSGB semi-minor axis
#define e0 400000           // OSGB easting of false origin
#define n0 -100000          // OSGB northing of false origin
#define f0 0.9996012717     // OSGB scale factor on central meridian
#define e2 0.0066705397616  // OSGB eccentricity squared
#define lam0 -0.034906585039886591  // OSGB false east
#define phi0 0.85521133347722145    // OSGB false north
#define af0  6375020.48098897069 //(a * f0)
#define bf0 6353722.49048791244 //(b * f0)
#define n 0.0016732202503250876 //(af0 - bf0) / (af0 + bf0)
#define WGS84_AXIS 6378137 // a
#define WGS84_ECCENTRIC 0.00669438037928458 //e
#define OSGB_AXIS 6377563.396 //a2
#define OSGB_ECCENTRIC 0.0066705397616  //e2
#define _xp -446.448  //OSGB/Airy datums/parameters
#define _yp 125.157
#define _zp -542.06
#define xrot -0.000000728190149026 //_xr -0.1502; (_xr / 3600) * deg2rad;
#define yrot -0.000001197489792340 //_yr -0.247; (_yr / 3600) * deg2rad;
#define zrot -0.000004082616008623 //_zr -0.8421; (_zr / 3600) * deg2rad;
#define _sf 0.0000204894  // s=20.4894 ppm

// pinouts from https://github.com/Xinyuan-LilyGO/TTGO-T-tft
#define TFT_MOSI 19
#define TFT_SCLK 18
#define TFT_CS 5
#define TFT_DC 16
#define TFT_RST 23
#define TFT_BL 4
#define LEFTBUTTON 0
#define RIGHTBUTTON 35

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// Color definitions
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0 
#define WHITE    0xFFFF

//gps setup
#define RXD2 22
#define TXD2 21
HardwareSerial neogps(1);
TinyGPSPlus gps;

//MODIFIED Battery level setup
Pangodream_18650_CL BL;

//setup for BNG
uint32_t resultN;
uint32_t resultE;
signed int resultEl;
char result1;
char result2;
void LLtoNE(
  float latConv,  // in degrees
  float lonConv,   // in degrees 
  float heightConv,  // in meters  
  uint32_t * const p_os_northings, //pointer to output variable for OS northings
  uint32_t * const p_os_eastings, //pointer to output variable for OS eastings
  signed int * const p_airy_elevation //pointer to output variable for elevation (based on Airy elipsoid used for heights OS maps)
  );
void prefix(uint32_t northings, uint32_t eastings, char * prefixN, char * prefixE);

void setup(void){
  pinMode(RIGHTBUTTON, INPUT);
  pinMode(LEFTBUTTON, INPUT);
  pinMode(TFT_BL, OUTPUT);      // TTGO T-tft enable Backlight pin 4
  digitalWrite(TFT_BL, HIGH);   // T-tft turn on Backlight
  tft.init(135, 240);           // Initialize ST7789 240x135
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  tft.setRotation(3);
  tft.fillScreen(WHITE);
  tft.fillScreen(BLUE);
  tft.setTextSize(3);
  tft.print("THIS IS NOT A GPS");
  delay(2000);
  tft.fillScreen(RED);
  tft.fillScreen(GREEN);
  tft.fillScreen(CYAN);
  tft.fillScreen(MAGENTA);
  tft.fillScreen(YELLOW);
  tft.fillScreen(BLACK);
  delay(2000);

  //set default screen border and text color/size
  tft.setTextSize(2);  
  tft.setTextColor(WHITE, BLACK);
  tft.drawRoundRect(1, 1,239,129,15, RED );
}

void loop() {
  signed int screenNumber = 0;
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (neogps.available())
    {
      if (gps.encode(neogps.read()))
      {
        newData = true;
      }
    }
  }

  //If newData is true
  if(newData == true)
  {
  LLtoNE(gps.location.lat(),gps.location.lng(),gps.altitude.meters(),&resultN,&resultE, &resultEl);
  prefix(resultN,resultE,&result1,&result2);
    newData = false;
    if (screenNumber == 0){
    print_screen0();
    }
    else if (screenNumber == 1){
      print_screen1();
    }
    else if (screenNumber == 2){
      print_screen2();
    }
  }
  else
  {
    print_nodata();
  }   
}

void print_screen0()
{       
  if (gps.location.isValid() == 1)
  {
    tft.setTextSize(2);
    tft.setTextWrap(false);
    tft.setCursor(25, 10);
    tft.print(result1);
    tft.print(result2);
    tft.print(" ");
    tft.printf("%05d", resultE %100000);
    tft.print(", ");
    tft.printf("%05d", resultN %100000);    

    tft.setCursor(25, 50);
    tft.setTextSize(2);
    tft.print("Charge: ");
    tft.setCursor(105, 50);
    tft.print(BL.getBatteryChargeLevel()); //BL.getBatteryVolts()); 
    tft.print(" %     ");

    tft.setTextSize(2);
    tft.setCursor(25, 70);
    tft.print("Time:   ");
    tft.setCursor(100, 70);
    tft.setTextSize(2);
    tft.printf("%02d", gmthour());
    tft.printf("%02d",gps.time.minute());
    tft.print("    ");
    
//bottom left
    tft.setTextSize(2);
    tft.setCursor(6, 110);
    tft.print("SAT:");
    tft.setCursor(50, 110);
    tft.print(gps.satellites.value());
    tft.print("  ");
    
//bottom right
    tft.setTextSize(1);
    tft.setTextColor(RED,WHITE);
    tft.setTextWrap(false);
    tft.setCursor(150, 118);
    tft.print(" Made by Paco.");
    tft.setTextColor(WHITE,BLACK);
    
  }
  else
  {
    print_nodata();    
  }
}
void print_screen1()
{
  //Clock HH:MM
  tft.setTextSize(6);
  tft.setCursor(30, 45);
  tft.printf("%02d:", gmthour());
  tft.printf("%02d",gps.time.minute());
}
void print_screen2()
{
  
}
void print_nodata()
{
  tft.setTextSize(3);
  tft.setCursor(60, 5);
  tft.println("No Data");
//  tft.setTextSize(4);
//  tft.setCursor(60, 45);
//  tft.printf("%02d:", gmthour());
//  tft.printf("%02d",gps.time.minute());
  tft.setTextSize(2);
  tft.setCursor(6, 110);
  tft.printf("Charge:%i%%", BL.getBatteryChargeLevel());  
}


signed int gmthour(){
  if (gps.date.month() > 3 && gps.date.month() < 11){
    if (gps.time.hour()+1 > 23){
        return gps.time.hour()-23;
    }
        else{
        return gps.time.hour()+1;
        }
  }
}

float Marc(float phi) // used in LLtoNE function below
{
  float Marc = bf0 * (((1 + n + ((5 / 4) * (n * n)) + ((5 / 4) * (n * n * n))) * (phi - phi0))
    - (((3 * n) + (3 * (n * n)) + ((21 / 8) * (n * n * n))) * (sin(phi - phi0)) * (cos(phi + phi0)))
    + ((((15 / 8) * (n * n)) + ((15 / 8) * (n * n * n))) * (sin(2 * (phi - phi0))) * (cos(2 * (phi + phi0))))
    - (((35 / 24) * (n * n * n)) * (sin(3 * (phi - phi0))) * (cos(3 * (phi + phi0)))));
  return (Marc);
}

void LLtoNE(
  float latConv,  // in degrees
  float lonConv,   // in degrees 
  float heightConv,  // in meters  
  uint32_t * const p_os_northings, //pointer to output variable for OS northings
  uint32_t * const p_os_eastings, //pointer to output variable for OS eastings
  signed int * const p_airy_elevation //pointer to output variable for elevation (based on Airy elipsoid used for heights OS maps)
  )
{
  latConv*= deg2rad;      // convert latitude to radians
  lonConv*= deg2rad;      // convert longitude to radians
  
  // Convert WGS84/GRS80 into OSGB36/Airy
  // convert to cartesian
  float v = WGS84_AXIS / (sqrt(1 - (WGS84_ECCENTRIC *(sin(latConv) * sin(latConv)))));
  float x = (v + heightConv) * cos(latConv) * cos(lonConv);
  float y = (v + heightConv) * cos(latConv) * sin(lonConv);
  float z = ((1 - WGS84_ECCENTRIC) * v + heightConv) * sin(latConv);
  // transform cartesian
  float hx = x + (x * _sf) - (y * zrot) + (z * yrot) + _xp;
  float hy = (x * zrot) + y + (y * _sf) - (z * xrot) + _yp;
  float hz = (-1 * x * yrot) + (y * xrot) + z + (z * _sf) + _zp;
  // Convert back to lat, lon
  lonConv = atan(hy / hx);
  float p = sqrt((hx * hx) + (hy * hy));
  latConv = atan(hz / (p * (1 - OSGB_ECCENTRIC)));
  v = OSGB_AXIS / (sqrt(1 - OSGB_ECCENTRIC * (sin(latConv) * sin(latConv))));
  float errvalue = 1.0;
  float lat1 = 0;
  while (errvalue > (1/1024))
  {
    lat1 = atan((hz + OSGB_ECCENTRIC * v * sin(latConv)) / p);
    errvalue = abs(lat1 - latConv);
    latConv = lat1;
  }
  *p_airy_elevation = p / cos(latConv) - v;

  // Convert OSGB36/Airy into OS grid eastings and northings
  // easting
  float slat2 = sin(latConv) * sin(latConv);
  float nu = af0 / (sqrt(1 - (e2 * (slat2))));
  float rho = (nu * (1 - e2)) / (1 - (e2 * slat2));
  float eta2 = (nu / rho) - 1;
  float pp = lonConv - lam0;
  float IV = nu * cos(latConv);
  float clat3 = pow(cos(latConv), 3);
  float tlat2 = tan(latConv) * tan(latConv);
  float V = (nu / 6) * clat3 * ((nu / rho) - tlat2);
  float clat5 = pow(cos(latConv), 5);
  float tlat4 = pow(tan(latConv), 4);
  float VI = (nu / 120) * clat5 * ((5 - (18 * tlat2)) + tlat4 + (14 * eta2) - (58 * tlat2 * eta2));
  *p_os_eastings = e0 + (pp * IV) + (pow(pp, 3) * V) + (pow(pp, 5) * VI);
  // northing
  float M = Marc(latConv);
  float I = M + (n0);
  float II = (nu / 2) * sin(latConv) * cos(latConv);
  float III = ((nu / 24) * sin(latConv) * pow(cos(latConv), 3)) * (5 - pow(tan(latConv), 2) + (9 * eta2));
  float IIIA = ((nu / 720) * sin(latConv) * clat5) * (61 - (58 * tlat2) + tlat4);
  *p_os_northings = I + ((pp * pp) * II) + (pow(pp, 4) * III) + (pow(pp, 6) * IIIA);
}

void prefix(uint32_t northings, uint32_t eastings, char * prefixN, char * prefixE){
// First Letter
      
      //if first digit is < 6 & second digit is > 6 first letter is T
      //if first digit is < 6 & second digit is < 6 first letter is S
      //if first digit is > 6 & second digit is < 6 first letter is N
      //if first digit is > 6 & second digit is > 6 first letter is O
signed int Ndigit = northings / 100000;
signed int Edigit = eastings / 100000;
  if (Ndigit < 5 && Edigit < 5){
    *prefixN = 'S';
  }
  else if (Ndigit < 5 && Edigit > 5){
    *prefixN = 'T';
  }
  else if (Ndigit > 5 && Edigit < 5){
    *prefixN = 'N';
  }
  else if (Ndigit > 5 && Edigit > 5){
    *prefixN = 'O';
  }
  else {
    *prefixN = ' ';
  }

//Second Letter
  
  while (Edigit > 4){
    Edigit -= 5;
  }
  
  if (Ndigit ==0){
    if (Edigit ==0){
      *prefixE = 'V';
    }
    else if (Edigit ==1){
      *prefixE = 'W';
    }
    else if (Edigit ==2){
      *prefixE = 'X';
    }
    else if (Edigit ==3){
      *prefixE = 'Y';
    }
    else if (Edigit ==4){
      *prefixE = 'Z';
    }
  }
  else if (Ndigit ==1){
    if (Edigit ==0){
      *prefixE = 'Q';
    }
    else if (Edigit ==1){
      *prefixE = 'R';
    }
    else if (Edigit ==2){
      *prefixE = 'S';
    }
    else if (Edigit ==3){
      *prefixE = 'T';
    }
    else if (Edigit ==4){
      *prefixE = 'U';
    }
  }
  else if (Ndigit ==2){
    if (Edigit ==0){
      *prefixE = 'L';
    }
    else if (Edigit ==1){
      *prefixE = 'M';
    }
    else if (Edigit ==2){
      *prefixE = 'N';
    }
    else if (Edigit ==3){
      *prefixE = 'O';
    }
    else if (Edigit ==4){
      *prefixE = 'P';
    }
  }
  else if (Ndigit ==3){
    if (Edigit ==0){
      *prefixE = 'F';
    }
    else if (Edigit ==1){
      *prefixE = 'G';
    }
    else if (Edigit ==2){
      *prefixE = 'H';
    }
    else if (Edigit ==3){
      *prefixE = 'J';
    }
    else if (Edigit ==4){
      *prefixE = 'K';
    }
  }
  else if (Ndigit ==4){
    if (Edigit ==0){
      *prefixE = 'A';
    }
    else if (Edigit ==1){
      *prefixE = 'B';
    }
    else if (Edigit ==2){
      *prefixE = 'C';
    }
    else if (Edigit ==3){
      *prefixE = 'D';
    }
    else if (Edigit ==4){
      *prefixE = 'E';
    }
  }
  else{
    *prefixE = ' ';
  }
}
