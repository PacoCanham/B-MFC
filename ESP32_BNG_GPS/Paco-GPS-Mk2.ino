#include <TinyGPS++.h>
#include <TinyGPSPlus.h>

#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include "Button2.h"
#include <esp_adc_cal.h>
#include "bmp.h"
#include "driver/rtc_io.h"
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
// TFT Pins has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
#define TFT_MOSI            19
#define TFT_SCLK            18
#define TFT_CS              5
#define TFT_DC              16
#define TFT_RST             23
#define TFT_BL              4   // Display backlight control pin
#define ADC_EN              14  //ADC_EN is the ADC detection enable port
#define ADC_PIN             34
#define BUTTON_1            35
#define BUTTON_2            0
#include <Adafruit_ST7789.h>

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



TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library
Button2 btn1(BUTTON_1);
Button2 btn2(BUTTON_2);

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
#define RXD2 37
#define TXD2 36
HardwareSerial neogps(1);
TinyGPSPlus gps;
bool ischarging = false;
char buff[512];
int vref = 1100;
int btn1SClick = false;
int btn1DClick = false;
int btn1TClick = false;
uint32_t resultN;
uint32_t resultE;
signed int resultEl;
char result1;
char result2;
unsigned int screenNumber = 0;
int mils;
int pwrlastState = LOW;
int pwrCurrentState;
unsigned long pwrpressedTime = 0;
unsigned long pwrreleasedTime = 0;
bool buttonPressed = false;
bool hasData = false;
int battery_percentage;
bool firstpress;

//! Long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
//void delay(int ms)
//{
//    esp_sleep_enable_timer_wakeup(ms * 1000);
//    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
//    esp_light_sleep_start();
//}

signed int gmthour(){
  if (gps.date.month() > 5 && gps.date.month() < 11)
  {
        if (gps.time.hour()+1 > 23){
            return gps.time.hour()-23;
        }
            else{
            return gps.time.hour()+1;
            }
  }
  else {
    return gps.time.hour();
  }
}

void print_nodata(){
  tft.setRotation(1);
   setCpuFrequencyMhz(240);
    tft.setTextSize(4);
    tft.setCursor(5, 10);
    tft.println("NO SIGNAL");
    tft.setTextSize(1);
    tft.setTextColor(RED,WHITE);
    tft.setTextWrap(false);
    tft.setCursor(150, 118);
    tft.print(" Made by Paco."); 
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(2);
}

void print_screen(int screenNumber){
  mils = (gps.course.deg() * 17.77);
  showVoltage();
  if (gps.location.isValid() == 1){ 
    tft.setRotation(1);
        if (screenNumber == 0){    
            tft.setTextWrap(false);
            tft.setTextSize(2);
            tft.setCursor(25, 20);
            tft.print(result1);
            tft.print(result2);
            tft.print(" ");
            tft.printf("%05d", resultE %100000);
            tft.print(" ");
            tft.printf("%05d", resultN %100000);
        
            tft.setCursor(25, 55);
            tft.print("Time: ");
            tft.printf("%02d", gmthour());
            tft.printf("%02i", gps.time.minute());
        
            tft.setCursor(7, 110);
            tft.print("SAT:");
            tft.print(gps.satellites.value());
            tft.print(" ");
        
            tft.setTextSize(2);
            tft.setCursor(100, 110);
            if (ischarging){
              tft.print("Charging  ");
            } else if (!ischarging){
            tft.print("Charge:");
            tft.printf("%i", battery_percentage);
//            tft.print(BL.getBatteryChargeLevel()); //BL.getBatteryVolts()); 
            tft.print("% ");
            } else {
              tft.print("Error");
            }
          setCpuFrequencyMhz(240);
          }
          else if (screenNumber == 3){
                tft.setCursor(95, 7);
                tft.print(gps.date.value());
                tft.setTextWrap(false);
                tft.setTextSize(5);
                tft.setCursor(10, 40);
                tft.setTextColor(RED, BLACK);
                tft.printf("%02i", gmthour());
                tft.printf("%02i ", gps.time.minute());
                tft.printf("%02i ", gps.time.second());
                tft.print(" ");
                tft.setTextSize(2);
                tft.setTextColor(WHITE,BLACK);
                setCpuFrequencyMhz(60);
          }
          else if (screenNumber == 4){
                tft.setTextSize(3);
                tft.setCursor(80, 10);
                tft.println("Grid");
                tft.setCursor(6,50);
                tft.print(result1);
                tft.print(result2);
                tft.setTextSize(1);
                tft.print(" ");
                tft.setTextSize(3);
                tft.printf("%05d", resultE %100000);
                tft.setTextSize(1);
                tft.print(" ");
                tft.setTextSize(3);
                tft.printf("%05d", resultN %100000);  
                tft.setCursor(138, 67);
                tft.setTextSize(1);
                tft.print(",");
                tft.setTextSize(1);
                tft.setCursor(70, 90);
                tft.print("Power Saver Screen");
                tft.setTextSize(2);
                tft.setTextColor(WHITE,BLACK);
                delay(1000);
                setCpuFrequencyMhz(40);
          }
          else if (screenNumber == 5){
                setCpuFrequencyMhz(20);
                tft.setTextSize(6);
                tft.setCursor(50, 40);
                tft.print("PACO");
          }
          else if (screenNumber == 1){
                setCpuFrequencyMhz(240);
                mils = (gps.course.deg() * 17.77);
                tft.setTextSize(3);
                tft.setCursor(40, 10);
                tft.println(" Vehicle");
                tft.setTextSize(2);
                
                tft.setCursor(25, 50);
                tft.print(result1);
                tft.print(result2);
                tft.print(" ");
                tft.printf("%05d", resultE %100000);
                tft.print(" ");
                tft.printf("%05d", resultN %100000);

                if (gps.speed.kmph() < 1.5){
                    tft.setCursor(6,70);
                    tft.print("Speed   : ");
                    tft.print("Static  ");
                }
                else {
                    tft.setCursor(6,70);
                    tft.print("Speed   : ");
                    tft.print(gps.speed.kmph());
                    tft.print("km/h");
                }

                tft.setCursor(6,90);
                tft.print("Bearing : ");
                tft.printf("%04i",mils);
                tft.print(" Mils");

                tft.setTextSize(2);
                tft.setTextColor(WHITE,BLACK);
          }
          else if (screenNumber == 2){
                setCpuFrequencyMhz(240);
                mils = (gps.course.deg() * 17.77);
                  tft.setTextSize(2);
                tft.setCursor(25, 50);
                tft.print(result1);
                tft.print(result2);
                tft.print(" ");
                tft.printf("%05d", resultE %100000);
                tft.print(" ");
                tft.printf("%05d", resultN %100000);

                tft.setCursor(6,90);
                tft.print("Bearing : ");
                int tempmils = ((gps.course.deg()) * 17.77);
                Serial.print(gps.course.deg());
                Serial.print(tempmils);
                tft.printf("%04i", tempmils);
                tft.print(" Mils");

                tft.setTextSize(2);
                tft.setTextColor(WHITE,BLACK);
          }
          else if (screenNumber == 10){
                 setCpuFrequencyMhz(240);
                 mils = (gps.course.deg() * 17.77);
                 Serial.println("hasData = ");
                 Serial.println(hasData);
//                 pwrCurrentState = digitalRead(PWRBUTTON);
                 if (pwrCurrentState == HIGH){
                if (!hasData){
                  tft.fillScreen(BLACK);
                  tft.setCursor(5,5);
                  tft.print("Press Top Button ->");
                  tft.setCursor(5,25);
                  tft.print("to Start");
                  tft.setCursor(5, 45);
                  tft.print("AZ determination");
                  }
                  else if (hasData) {
                  tft.fillScreen(BLACK);
                  tft.setCursor(5,5);
                  tft.print("Press Top Button ->");
                  tft.setCursor(5,25);
                  tft.print("to Restart");
                  tft.setCursor(5,50);
                  tft.print("Sight to Post :");
                  tft.setCursor(5,70);
                  tft.printf("%04i Mils", mils);
                  tft.setCursor(5,90);
                  tft.print("Post to Sight :");
                  tft.setCursor(5,110);
                  tft.printf("%04i Mils", backbearing(mils));         
                  }
                 }
                  if (pwrCurrentState == LOW){
                  tft.setTextColor(WHITE, BLACK);
                  tft.fillScreen(BLACK);
                  tft.setTextSize(3);
                  tft.setCursor(5,45);
                  tft.print("HOLD STILL");
                  tft.setCursor(30,70);
                  tft.print("ON C2 SIGHT");
                  delay(1000);
                  for (int i = 1 ; i < 5 ; i++){
                    tft.setTextSize(6);
                    tft.setCursor(90,60);
                    tft.printf("%i",i);
                    delay(700);
                    }
                  tft.fillScreen(BLACK);
                  tft.setCursor(5,20);
                  tft.setTextSize(2);
                  tft.print("WALK TO AIMING POST");
                  delay(2000);
                  tft.setCursor(90,60);
                  for (int i = 25; i > 0 ; i--){
                      tft.fillScreen(BLACK);
                      tft.setTextSize(6);
                      tft.setCursor(100,50);
                      tft.printf("%i",i);
                      delay(1000);
                      }
                  tft.fillScreen(BLACK);
                  tft.setCursor(5,20);
                  tft.setTextSize(2);
                  tft.print("HOLD STILL ON POST");
                  delay(1000);
                  tft.setCursor(90,60);
                  for (int i = 5; i > 0 ; i--){
                      tft.fillScreen(BLACK);
                      tft.setTextSize(6);
                      tft.setCursor(100,50);
                      tft.printf("%i",i);
                      delay(1000);
                      }
                  tft.setTextSize(2);
                  tft.fillScreen(BLACK);
                  tft.setCursor(5,30);
                  tft.println("Sight to Post :");
                  tft.printf("%04i Mils", mils);
                  tft.println();
                  tft.println("Post to Sight :");
                  tft.printf("%04i Mils", backbearing(mils));
                  hasData = true;
                  Serial.println("hasData = ");
                  Serial.println(hasData);
//                  delay(5000);
                  }
                }
    else
      {
        print_nodata();   
      } 
}
}

signed int backbearing(int tmpmils){
  if (tmpmils > 3200){
    tmpmils = tmpmils-3200;
  }
  else {
    tmpmils = tmpmils+3200;
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

void showVoltage()
{
    static uint64_t timeStamp = 0;
    if (millis() - timeStamp > 1000) {
        timeStamp = millis();
        uint16_t v = analogRead(ADC_PIN);
        float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
        if (battery_voltage > 4.8){
        ischarging = true;
        }
        else {
         ischarging = false;
         battery_percentage = ((battery_voltage - 3.2)/(4.345-3.2) * 100);
        }
    }
}

void button_init()
{
    btn1.setLongClickHandler([](Button2 & c) { //RH long press
        btn1SClick = false;
        neogps.end();
        int r = digitalRead(TFT_BL);
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("Press again to wake up",  tft.width() / 2, tft.height() / 2 );
        delay(4000);
        digitalWrite(TFT_BL, !r);

        tft.writecommand(TFT_DISPOFF);
        tft.writecommand(TFT_SLPIN);
        //After using light sleep, you need to disable timer wake, because here use external IO port to wake up
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        // esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);
        rtc_gpio_init(GPIO_NUM_14);
        rtc_gpio_set_direction(GPIO_NUM_14, RTC_GPIO_MODE_OUTPUT_ONLY);
        rtc_gpio_set_level(GPIO_NUM_14, 1);

        delay(500); 
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0);
        delay(200);
        esp_deep_sleep_start();
    });
    
    btn1.setClickHandler([](Button2 & c) { //single press right button
      btn1SClick = true;
      firstpress = true;
//      if (firstpress){
      tft.fillScreen(BLACK);
      tft.setRotation(3);
      tft.drawRoundRect(1, 1,239,129,15, RED );
      tft.setTextColor(WHITE, BLACK);
//      }
    });
    
    btn1.setDoubleClickHandler([](Button2 & c){
 
    });    
    
    btn1.setTripleClickHandler([](Button2 & c){

    });

    btn2.setPressedHandler([](Button2 & c) { //single press ;eft button
        tft.fillScreen(BLACK);
        if (screenNumber < 5){
          screenNumber ++;
        }
        else {
          screenNumber = 0;
        }
    });
}

void button_loop()
{
    btn1.loop();
    btn2.loop();
}

void IRAM_ATTR menu(){
    btn1SClick = false;
    firstpress = false;
    tft.fillScreen(BLACK);
    tft.setRotation(0);
    tft.setTextSize(1);
    tft.setTextColor(GREEN, BLACK);
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(5, 15);
    tft.printf("Screen Number : %i", screenNumber);
    tft.setCursor(2, 32);
    tft.println("0 - Default");
    tft.println();
    tft.println("1 - Vehicle");
    tft.println();
    tft.println("2 - Grid Only");
    tft.println();
    tft.println("3 - Clock");
    tft.println();
    tft.println("4 - Grid (Power Saver)");
    tft.println();
//    tft.println("5 - AZ Determination");
//    tft.println("6 - AZ Results");
  tft.setRotation(1);
  tft.setCursor(120, 5);
  tft.print("Hold - turn off ->");
  tft.setCursor(120, 16);
  tft.print("Press - load screen");
  tft.setCursor(132, 110);
  tft.print("Return to menu /");
  tft.setCursor(132, 120);
  tft.print("Select screen ->");
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Start");
    neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
    Serial.println("neogps started");
    pinMode(0, INPUT);
    attachInterrupt(0, menu, RISING);

    /*
    ADC_EN is the ADC detection enable port
    If the USB port is used for power supply, it is turned on by default.
    If it is powered by battery, it needs to be set to high level
    */
    pinMode(ADC_EN, OUTPUT);
    digitalWrite(ADC_EN, HIGH);
    
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(WHITE, BLACK);
    tft.setCursor(0, 0);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(1);

    /*
    if (TFT_BL > 0) {                           // TFT_BL has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
        pinMode(TFT_BL, OUTPUT);                // Set backlight pin to output mode
        digitalWrite(TFT_BL, TFT_BACKLIGHT_ON); // Turn backlight on. TFT_BACKLIGHT_ON has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
    }
    */

//    tft.setSwapBytes(true);
//    tft.pushImage(0, 0,  240, 135, ttgo);
//    delay(5000);

    button_init();

    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);    //Check type of calibration value used to characterize ADC
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
        vref = adc_chars.vref;
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
    } else {
        Serial.println("Default Vref: 1100mV");
    }
    menu();
}

void gps_loop(int screenNumber){
  boolean newData = false;
  LLtoNE(gps.location.lat(),gps.location.lng(),gps.altitude.meters(),&resultN,&resultE, &resultEl);
  prefix(resultN,resultE,&result1,&result2);
  
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
  if(newData == true)
  {
    newData = false;
    print_screen(screenNumber);
  }
  else
  {
    print_nodata();
  }   
}

void loop()
{
  button_loop();
  if (btn1SClick){
    gps_loop(screenNumber);    
  }
}
