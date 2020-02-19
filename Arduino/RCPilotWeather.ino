/*
    Speed Altitude   1.0
    (C) Alastair Cormack (alastair.cormack@gmail.com)

    A project to support RC Pilots by giving them a simple weather station that provides:
    * Air Temperature
    * Humidity
    * Air Pressure (P)
    * Wind Speed (WS)
    * Maximum Wind Speed (MWS)
    * Altitude (A)
    * Density Atlitude

    The density altitude was the main feature I wanted as it gives you the altitude relative to standard atmospheric conditions. 
    So for us speed heli pilots we have another excuse as to why we were slow that day!

    I am using a GPS to get as close as possible to the real altitude (I know its not perfect for elevation but IMHO its good enough for hobby grade). 
    There is then a lot of maths that uses the info from the weather station sensors with the GPS altitude to calculate the density altitude.
    
    This is for personnal use only with the usual disclaimer of: Use this at your own risk.   
    This would only have been possible to do this through the excellent efforts of:
    AeroCalc routines in Python, Copyright (c) 2008, Kevin Horton
    http://pydoc.net/AeroCalc/0.11/aerocalc.std_atm
    Code snippets and hardware ideas for the weather components are based on the work here http://cactus.io/projects/weather/arduino-weather-station-bme280-sensor
    other useful website is https://wahiduddin.net/calc/calc_da_rh.htm This will give some backgournd on the calculations


    Hardware Needed:
    - Arduino Nano
    - Solder Finished Prototype PCB for DIY 5x7cm Circuit Board Breadboard 
    - Hall Effect KY-003 Magnetic Sensor Module DC 5V For Arduino PIC - The sensor is sensitive to the magnet pole. So if it does not work flip the magent around.
    - 4mm dia x 2mm thick N35 Neodymium Magnet - CAUTION: Be really realy careful with these if you have kids. They require surgery if swallowed.
    - 608ZZ Deep Groove Ball Bearing Double Shield 608-2Z 80018 8mm x 22mm x 7mm - Make sure this is as free as possible as drag will distort the speed
    - DS18B20 Waterproof Temperature Sensors Transducer Thermal Probe Compatible with DIY Arduino Raspberry Pi Etc.
    - I2C OLED Display Module 0.91 Inch I2C SSD1306 OLED Display Module White I2C OLED Screen Driver DC 3.3V to 5V for Arduino
    - GY-BME280 High Precision Digital Sensor Breakout Barometric Pressure Temperature Humidity Module Board for Arduino Raspberry Pi DIY I2C SPI 5V 
    - APM2.5 UBlox NEO-M8N GPS Module (or similar.. neeeds to be at 9600 baud setting)
    - 4 * M3*6
    - 4 * M3 brass knurled nuts (5mm deep) - the holes in the 3D model are a little tight as I had to use a dremel to expand them about 0.5mm. 
    - XT60 Female connector
    
    I designed a 3D printed base to work with a Anemometer model that is available at:  https://www.thingiverse.com/thing:2559929
    Download the Base and Lid from my Thingverse at:https://www.thingiverse.com/thing:4171499

    A lot of libraries are used here:
    
    SoftwareSerial
    SPI
    Wire
    Adafruit Circuit Playground
    Adafruit Unified Sensor
    Adafruit SSD1306
    AdafruitGFX
    cactus_io_BME280_I2C
    DallasTemperature
    OneWire
    TimerOne
    TinyGPS

*/
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "TimerOne.h" 
#include <math.h> 

#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include "cactus_io_BME280_I2C.h" 
#include <OneWire.h>
#include <DallasTemperature.h>

TinyGPS gps;
SoftwareSerial ss(4, 3);

#define OLED_RESET 0

Adafruit_SSD1306 display(OLED_RESET);

#define WindSensor_Pin (2) // digital pin for wind speed sensor 
volatile bool isSampleRequired; // this is set every 2.5sec to generate wind speed 
volatile unsigned int timerCount; // used to count ticks for 2.5sec timer count 
volatile unsigned long rotations; // cup rotation counter for wind speed calcs 
volatile unsigned long contactBounceTime = 0; // timer to avoid contact bounce in wind speed sensor 
volatile float windSpeed; 

float maxWindspeed=0;

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 12

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

BME280_I2C bme; // I2C using address 0x76 

float alt_m = 0;

#define g  9.80665  //# Acceleration of gravity at 45.542 deg latitude, m/s**s
#define Rd  287.05307  //# Gas constant for dry air, J/kg K
#define Rv  461.495  //gas constant for water vapour

#define T0  288.15  //# Temperature at sea level, degrees K
#define L0  -6.5  //# Temperature lapse rate, at sea level deg K/km
#define P0  29.9213  //# Pressure at sea level, in HG
#define Rho0  1.2250  //# Density at sea level, kg/m**3


float circ_cm = (2 * PI) * (8); // radius of the cups is 8cm to the middle of the cup. So this gets the circumference.
float dist_km = circ_cm / 100000; // convert cm to km
   
//# conditions starting at 11 km, in an isothermal region

float T11 = T0 + 11 * L0;  //# Temperature at 11,000 m, degrees K
float PR11 = pow((T11 / T0),((-1000 * g) / (Rd * L0))) ; //# pressure ratio at 11,000 m
float P11 = PR11 * P0;
float Rho11 = (Rho0 * PR11) * (T0 / T11);

static const unsigned char PROGMEM gImage_ACLogo[] = { /* 0X22,0X01,0X50,0X00,0X1E,0X00, */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x01, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x07, 0xC0, 0x00,
0x00, 0x3F, 0xF0, 0x00, 0x38, 0x7F, 0xF8, 0x30, 0x03, 0xFF, 0xFF, 0x30, 0x03, 0xF8, 0x7F, 0x80,
0x01, 0xF0, 0x3F, 0x00, 0x01, 0xE0, 0x1F, 0x00, 0x01, 0xE0, 0x1F, 0x00, 0x01, 0xE0, 0x1F, 0x00,
0x01, 0xE0, 0x1F, 0x00, 0x01, 0xF0, 0x1E, 0x00, 0x00, 0xF8, 0x7E, 0x00, 0x00, 0xFF, 0xFC, 0x00,
0x00, 0x7F, 0xF8, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x07, 0xC0, 0x00, 0x00, 0x03, 0x00, 0x00,
0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

float _sat_press(float T) {
   /* """ 
    Return the saturation pressure in mb of the water vapour, given 
    temperature in deg C.  Equation from:
    http://wahiduddin.net/calc/density_altitude.htm
    """
*/
    float eso = 6.1078;
    float c0 = 0.99999683;
    float c1 = -0.90826951e-2;
    float c2 = 0.78736169e-4;
    float c3 = -0.61117958e-6;
    float c4 = 0.43884187e-8;
    float c5 = -0.29883885e-10;
    float c6 = 0.21874425e-12;
    float c7 = -0.17892321e-14;
    float c8 = 0.11112018e-16;
    float c9 = -0.30994571e-19;

    float p = c0 + T * (c1 + T * (c2 + T * (c3 + T * (c4 + T * (c5 + T * (c6 + T * (c7 + T * (c8 + T * c9))))))));
    float sat_press = eso / pow(p,8);
    return sat_press;
}
    
float sat_press(float T, float RH) {
  
    float Pv;
    Pv = _sat_press(T) * 100;
    Pv *= RH;

    //Pv = U.press_conv(Pv, from_units='pa', to_units=press_units)

    return Pv;
}

float pressure_alt(float H, float alt_setting){
   // Return the pressure altitude, given the barometric altitude and the
   // altimeter setting. 

    float Hft = H * 3.28084;
    
    float alt_settingHG = alt_setting * 0.029529983071445;
    
    float HP = Hft + 145442.2 * (1 - pow((alt_settingHG / P0),0.190261));
    HP = HP / 3.28084;
    return HP;
}

float _alt2press_ratio_gradient(
    float H,
    float Hb,
    float Pb,
    float Tb,
    float L
    ) {

    //# eqn from USAF TPS PEC binder, page PS1-31

    return (Pb / P0) * pow((1 + (L / Tb) * (H - Hb)),((-1000 * g) / (Rd  * L)));
    }

float _alt2press_ratio_isothermal(
    float H,
    float Hb,
    float Pb,
    float Tb
    ){

    //# eqn from USAF TPS PEC binder, page PS1-26

    return (Pb / P0) * exp((-1 * (H - Hb)) * ((1000 * g) / (Rd * Tb)));

    }
float alt2press_ratio(float H){

   // Return the pressure ratio (atmospheric pressure / standard pressure

    float Hkm = H / 1000.0;
    if (Hkm <= 11)
        return _alt2press_ratio_gradient(Hkm, 0, P0, T0, L0);
    else
        return 0;
}



float alt2press(float H){
   // Return the atmospheric pressure for a given altitude, with the 

    float press = P0 * alt2press_ratio(H);

    return (press * 3386.39);
}

float dry_press(float H, float Pv, float alt_setting) {

    float HP = pressure_alt(H, alt_setting);
    float P = alt2press(HP);
    float Pd = P - Pv;

    return Pd;
}

float  _density2alt_gradient(
    float Rho,
    float Rhob,
    float Hb,
    float Tb,
    float L
    ){

    return Hb + (Tb / L) * (pow((Rho / Rhob), (-1 / ((1000 * g) / (Rd * L) + 1))) - 1);
    }

float _density2alt_isothermal(
    float Rho,
    float Rhob,
    float Hb,
    float Tb
    ){

    return Hb - ((Rd * Tb) * log(Rho / Rhob)) / (1000 * g);
    }

float density2alt(float Rho){
  
    float H;

    if (Rho > Rho11)
        H = _density2alt_gradient(Rho, Rho0, 0, T0, L0);
    else
        H=-1;

    return (H * 1000);
}


float  density_ratio2alt(float DR){
   // Return the altitude for the specified density ratio. 
    float D = DR * Rho0;
    return density2alt(D);
}

void setup()  {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  display.clearDisplay();
  display.drawBitmap(0, 0,  gImage_ACLogo, 32, 32, 1);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(32,4);
  display.print("RC Pilot Weather"); 
  display.setCursor(32,18);
  display.print("by AlastairC"); 
  display.display();
  delay(5000);
  
  bme.begin();
  ss.begin(9600);

  sensors.begin();
  Serial.begin(115200);

  // setup anemometer values 
  rotations = 0; 
  isSampleRequired = false; 

  // setup timer values 
  timerCount = 0; 

  pinMode(WindSensor_Pin, INPUT); 
  attachInterrupt(digitalPinToInterrupt(WindSensor_Pin), isr_rotation, FALLING); 
  // setup the timer for 0.5 second 
  Timer1.initialize(500000); 
  Timer1.attachInterrupt(isr_timer); 

  sei();// Enable Interrupts 
}

// Interrupt handler routine for timer interrupt 
void isr_timer() { 

  timerCount++; 
  if(timerCount == 5) { 
    //Serial.println(rotations);
    float km_per_second = dist_km * (rotations / 2.5); //2.5 seconds is the measuring time // calculate KM per second
    windSpeed = km_per_second * 3600; // calculate KPH
    if (windSpeed > maxWindspeed) maxWindspeed = windSpeed;
    rotations = 0; 
    isSampleRequired = true; 
    timerCount = 0; 
  } 
} 

// Interrupt handler routine to increment the rotation count for wind speed 
void isr_rotation() { 
  if((millis() - contactBounceTime) > 10 ) { // debounce the switch contact 
      rotations++; 
      contactBounceTime = millis(); 
  } 
} 

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void loop()
{
  bme.readSensor(); 
 
  sensors.requestTemperatures();
  alt_m=gps.f_altitude();
  if (alt_m > 10000) alt_m = 0;
  
  float RH = bme.getHumidity();
  float alt_setting = bme.getPressure_MB();
  float T = sensors.getTempCByIndex(0);
 
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.clearDisplay();
  display.print("T:"); display.print(T,1); display.print("c"); 
  display.setCursor(66,0);
  display.print("H:");display.print(RH,1); display.println("%"); 
  display.print("P:");display.print(alt_setting,1); display.print("mb");
  display.setCursor(66,8);
  display.print("A:");display.print(alt_m,1); display.println("m");
  display.print("WS:"); display.print(windSpeed,1); display.println("kph");
  display.setCursor(66,16);
  display.print("MWS:");display.print(maxWindspeed,1); display.println("");
  
  
  RH = RH /100; //convert from percent
  float Pv = sat_press(T, RH);
  float Pd = dry_press(alt_m, Pv, alt_setting);
  float Tk = T + 273.15;
  float D = Pd / (Rd * Tk) + Pv / (Rv * Tk);
  float DR = D / Rho0;
  float DenAlt = density_ratio2alt(DR); 
  display.print("Dens Alt:  ");display.print( DenAlt,1 ); display.println("m");
  display.display(); 

  smartdelay(1000);
}

