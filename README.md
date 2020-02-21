# RCPilotWeather
A project to support RC Pilots by giving them a simple weather station that provides:
* Air Temperature (T)
* Humidity(H)
* Air Pressure (P)
* Wind Speed (WS)
* Maximum Wind Speed (MWS)
* Altitude (A)
* Density Atlitude (Dens Alt)

<img src="https://github.com/AlCormack/RCPilotWeather/blob/master/images/RCWeatherDevice.jpeg" width="800"><br />

The density altitude was the main feature I wanted as it gives you the altitude relative to standard atmospheric conditions. 
So for us speed heli pilots we have another excuse as to why we were slow that day!
<img src="https://github.com/AlCormack/RCPilotWeather/blob/master/images/Display.jpeg" width="500"><br />

* Ver 1.0: Initial version

I am using a GPS to get as close as possible to the real altitude (I know its not perfect for elevation but IMHO its good enough for hobby grade). 
There is then a lot of maths that uses the info from the weather station sensors with the GPS altitude to calculate the density altitude.

This is for personnal use only... with the usual disclaimer of: Use this at your own risk. 

I had most of the parts lying around to build. So had to buy very little. The arduino, display, sensors and bearing (unless you are a skater) you may have to buy. Parts cost me about $30 for this build. 

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
- XT60 Female connector (make sure you solder the wires on before glueing this into the mouting hole with CA adhesive)
- 4K7 Pullup Resistor
- 8 Pin IC Socket DIP 2.54mm Wide (This holds the OLED display and the BME-280
- 2S Battery for power (I have a big 4000mah pack that I can velco to the bottom)

The wiring is as follows:
<img src="https://github.com/AlCormack/RCPilotWeather/blob/master/images/RCWeather.png" width="800"><br />

The layout is as follows (note your GPS unit choice might require different positioning):
<img src="https://github.com/AlCormack/RCPilotWeather/blob/master/images/Inside.jpeg" width="800"><br />

I designed a 3D printed base to work with a Anemometer model that is available at:  https://www.thingiverse.com/thing:2559929
Download the base, lid and cube to support the end of the OLED display (this can be attached by double sided foam tape) from my Thingverse at:https://www.thingiverse.com/thing:4171499

