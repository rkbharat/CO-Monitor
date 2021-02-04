  

#include "SparkFun_SGP30_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_SGP30
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_SGP30.h"

Adafruit_SGP30 sgp;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Adafruit_BMP280 bmp; // I2C
SGP30 mySensor; //create an object of the SGP30 class

#define PRE_PIN          8
#define VNOX_PIN         A0
#define VRED_PIN         A1

#define PRE_HEAT_SECONDS 15

int vnox_value = 0;
int vred_value = 0;
int co = 0;
int no2 = 0;
int tvoc = 0;
int eco2 = 0;
int rawh2 = 0;
int rawehtanol = 0;
float temperature = 0.0;
float pressure = 0.0;
float altitude = 0.0;
int counter = 0;
int screen = 1; 

const int buzzerPin = 9; //buzzer to arduino pin 9

void setup() {
  Serial.begin(9600);
  Wire.begin();

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
  display.setTextSize(1); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.println(F("Pre Heating"));
  display.display();
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
                  
  //Initialize sensor
  if (mySensor.begin() == false) {
    Serial.println("No SGP30 Detected. Check connections.");
    while (1);
  }
  //Initializes sensor for air quality readings
  //measureAirQuality should be called in one second increments after a call to initAirQuality
  mySensor.initAirQuality();

  // Setup preheater pin
  pinMode(PRE_PIN, OUTPUT);
  
  // Initialize serial port
  Serial.begin(9600);


  // Wait for preheating
  digitalWrite(PRE_PIN, 1);
  delay(PRE_HEAT_SECONDS * 1000);
  digitalWrite(PRE_PIN, 0);
  Serial.println("Done");

  if (! sgp.begin()){
    Serial.println("Sensor not found :(");
    while (1);
  }

  pinMode(buzzerPin, OUTPUT);
  
  
}

void loop() {
  //First fifteen readings will be
  //CO2: 400 ppm  TVOC: 0 ppb
   //Wait 1 second

    temperature = bmp.readTemperature();
    pressure = bmp.readPressure();
    altitude = bmp.readAltitude(1013.25);

      // Read analog values, print them out, and wait
    no2 = analogRead(VNOX_PIN);
    co = analogRead(VRED_PIN);

    float  Vout = co/409.2; // take reading and convert ADC value to voltage
    float Rs = 100000/((3.3/Vout) - 1);   // find sensor resistance from Vout, using 5V input & 100kOhm load resistor
    co = 911.19*pow(2.71828,(-8.577*Rs/100000));    //convert Rs to ppm concentration CO

    Vout = analogRead(no2)/409.2; // take reading and convert ADC value to voltage
    Rs = 22000/((3.3/Vout) - 1);   // find sensor resistance from Vout, using 5V input & 22kOhm load resistor
    no2 = (.000008*Rs - .0194)*1000;    //convert Rs to ppb concentration NO2 (equation derived from data found on http://airpi.es/sensors.php
   

    if (! sgp.IAQmeasure()) {
    Serial.println("Measurement failed");
    return;
  }


  if (! sgp.IAQmeasureRaw()) {
    Serial.println("Raw Measurement failed");
    return;
  }


    counter++;
    if (counter == 30) {
      counter = 0;
  
      uint16_t TVOC_base, eCO2_base;
      if (! sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) {
        Serial.println("Failed to get baseline readings");
        return;
      }
  
    }
  
    tvoc = sgp.TVOC;
    eco2 = sgp.eCO2;
    rawh2 = sgp.rawH2;
    rawehtanol = sgp.rawEthanol;

    
    display.clearDisplay();
    display.setTextSize(2); // Draw 2X-scale text
    display.setTextColor(SSD1306_WHITE);
    if (screen == 1){
        display.setCursor(5, 0);
        display.println(round(temperature));
        display.setCursor(33, 0);
        display.setTextSize(1);
        display.println((char)247); // degree symbol 
        display.setCursor(38, 0);
        display.println("C");
        display.setTextSize(2);
        display.setCursor(70, 0);
        display.println(round(pressure/100));
        display.setCursor(108, 0);
        display.setTextSize(1);
        display.println("hPa"); // degree symbol 
        display.setTextSize(2);
        display.setCursor(35, 18);
        display.println(round(altitude));
        display.setCursor(75, 18);
        display.setTextSize(1);
        display.println("m"); // degree symbol 
        display.setTextSize(2);
        screen = 2;
      }
      else if (screen == 2){
        display.setTextSize(1);
        display.setCursor(3, 3);
        display.println("TVOC : ");
        display.setCursor(95, 3);
        display.println("ppb");
        display.setCursor(3, 21);
        display.println("eCO2 : ");
        display.setCursor(95, 21);
        display.println("ppm");
        display.setTextSize(2);
        display.setCursor(45, 0);
        display.println(tvoc);
        display.setCursor(45, 18);
        display.println(eco2);
  
        
        screen = 3;
      }
      else if (screen == 4){
        display.setTextSize(1);
        display.setCursor(3, 3);
        display.println("H2  :");
  
        display.setCursor(3, 21);
        display.println("Eth :");
  
        display.setTextSize(2);
        display.setCursor(43, 0);
        display.println(rawh2);
        display.setCursor(43, 18);
        display.println(rawehtanol);
        
        screen = 1;
      }
      else if (screen == 3){
        display.setTextSize(1);
        display.setCursor(3, 3);
        display.println("CO  :");
        display.setCursor(95, 3);
        display.println("ppm");
        display.setCursor(3, 21);
        display.println("NO2 :");
        display.setCursor(95, 21);
        display.println("ppm");
        display.setTextSize(2);
        display.setCursor(43, 0);
        display.println(co);
        display.setCursor(43, 18);
        display.println(no2);
  
        screen = 4;
    }
    
    if (co <= 200){   
      digitalWrite (buzzerPin, HIGH);
      delay(3000);
    }
    else if(co > 200 && co <= 400){

        digitalWrite (buzzerPin, LOW);
        delay (50);
        digitalWrite (buzzerPin, HIGH);
    
        delay(3000);
      
    }
    else if(co > 400 && co <= 600){
      digitalWrite (buzzerPin, LOW);
      delay (200);
      digitalWrite (buzzerPin, HIGH);
      delay(1000);
    }
    else if(co > 600){
      digitalWrite (buzzerPin, LOW);
      delay (1000);
      digitalWrite (buzzerPin, HIGH);
      delay(1000);
      screen = 3;
    }
    
    display.display();

    Serial.println();

}
