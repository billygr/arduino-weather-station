#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <LowPower.h>

#define ONE_WIRE_BUS 8
#define DHTPIN 2

int sensorValue = 0;

// Resistor values in Ohm
int R1=6660;
int R2=2154;

const int analogInPin = A0;

// Software SPI
// pin 13 - Serial clock out (SCLK)
// pin 11 - Serial data out (DIN)
// pin 12 - Data/Command select (D/C)
// pin 9 - LCD chip select (CS)
// pin 10 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(13, 11, 12, 9, 10);

// Hardware SPI
// SCK is LCD serial clock (SCLK) - this is pin 13 on Arduino Uno
// MOSI is LCD DIN - this is pin 11 on an Arduino Uno
// pin 12 - Data/Command select (D/C)
// pin 9 - LCD chip select (CS)
// pin 10 - LCD reset (RST)
//Adafruit_PCD8544 display = Adafruit_PCD8544(12, 9, 10);


// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

Adafruit_BMP085_Unified BMP180 = Adafruit_BMP085_Unified(10085);

static const byte degrees_glyph[] = {
  0x00,0x07,0x05,0x07,0x00
  };

typedef  struct  {
  float voltage;
  float temperature;
  float humidity;
  float pressure;
  float altitude;
} Sensor;

void setup() {
  Serial.begin(9600);
  
  analogReference(INTERNAL);

  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);

    /* Initialise the sensor */
  if(!BMP180.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  // Start up the library
  sensors.begin();
  dht.begin();
  
  display.begin();
  display.setContrast(55);
  //display.createChar(1,degrees_glyph);
  display.clearDisplay();
  //display.println("Hello, world!");
  //display.display();
  
  //Serial.println("test!");
}

void loop() {
  Sensor sensor;

  float voltage=0.0;
  float ADCvalue=0.0;

  //Fisrt result is usually wrong
  ADCvalue = analogRead(analogInPin);
  ADCvalue = analogRead(analogInPin);

  // With correction factor
  voltage = ((1.1/1023*ADCvalue)*(R1+R2)/R2)/1.0124;
  sensor.voltage = voltage;

  //voltage = ADCvalue * (1.1/1024);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  sensors.requestTemperatures(); // Send the command to get temperatures
  
 /* Get a new sensor event for BMP180 */ 
  sensors_event_t event;
  BMP180.getEvent(&event);
  
  sensor.humidity = h;

  Serial.print("ADC Value:");Serial.print(ADCvalue);Serial.print("\r\n");
  Serial.print("Voltage:");Serial.print(voltage);Serial.print("\r\n");
  Serial.print("DHT Humidity: ");Serial.print(h);Serial.print(" %\r\n");
  Serial.print("DHT Temperature: ");Serial.print(t);Serial.print(" *C\r\n");
  Serial.print("DS18B20 Temperature: ");Serial.print(sensors.getTempCByIndex(0));Serial.print(" *C \r\n");
//  Serial.print("Heat index: ");//  Serial.print(hic);//  Serial.print(" *C\r\n");
 
  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure)
  {
    /* Display atmospheric pressue in hPa */
    Serial.print("BMP180 Pressure:    ");
    Serial.print(event.pressure);
    Serial.println(" hPa");
    
    /* Calculating altitude with reasonable accuracy requires pressure    *
     * sea level pressure for your position at the moment the data is     *
     * converted, as well as the ambient temperature in degress           *
     * celcius.  If you don't have these values, a 'generic' value of     *
     * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
     * in sensors.h), but this isn't ideal and will give variable         *
     * results from one day to the next.                                  *
     *                                                                    *
     * You can usually find the current SLP value by looking at weather   *
     * websites or from environmental information centers near any major  *
     * airport.                                                           *
     *                                                                    *
     * For example, for Paris, France you can check the current mean      *
     * pressure and sea level at: http://bit.ly/16Au8ol                   */
     
    /* First we get the current temperature from the BMP085 */
    float temperature;
    BMP180.getTemperature(&temperature);
    Serial.print("BMP180 Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");
    sensor.temperature=temperature;

    /* Then convert the atmospheric pressure, and SLP to altitude         */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    Serial.print("BMP180 Altitude:    "); 
    Serial.print(BMP180.pressureToAltitude(seaLevelPressure,
                                        event.pressure)); 
    Serial.println(" m");
    Serial.println("");
  }
  else
  {
    Serial.println("BMP180 Sensor error");
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("DHT T:");display.print(t);display.print(" C");
  display.setTextSize(2);
  display.print("");display.print(sensors.getTempCByIndex(0));display.setTextSize(1);display.print("o");display.setTextSize(2);display.print("C");
  display.setTextSize(1);
  display.setCursor(0,31);
  display.print("Voltage=");display.print(voltage);
  display.setTextSize(1);
  display.setCursor(0,40);
  display.print("Humidity=");display.print(h);display.print("%");
  display.display();digitalWrite(13,LOW);
  // Show for one second clear and move to the second screen
  //delay(2000);
  LowPower.powerDown(SLEEP_2S,ADC_OFF,BOD_OFF);
  
  display.clearDisplay();
  display.setTextSize(1);
  display.print("BMP180 \r\n");
  display.print("T:");display.print(sensor.temperature);display.print(" C\r\n");
  display.print("P:");display.print(event.pressure);display.print(" hPa");
  display.display();digitalWrite(13,LOW);
  //delay(1000);
  LowPower.powerDown(SLEEP_1S,ADC_OFF,BOD_OFF);
  display.clearDisplay();digitalWrite(13,LOW);
  
  
}
