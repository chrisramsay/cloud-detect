#include <i2cmaster.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "mlx90614.h"
 
// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 2
#define TEMP_OBJ 0
#define TEMP_AMB 1
 
// Setup a oneWire instance to communicate with any OneWire devices 
OneWire oneWire(ONE_WIRE_BUS);
 
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

int led = 13;
bool heater = false;

void setup(){
  Serial.begin(9600);
  Serial.println("Setup...");
  
  pinMode(led, OUTPUT);
  i2c_init(); //Initialise the i2c bus
  PORTC = (1 << PORTC4) | (1 << PORTC5);//enable pullups
  // DS18B20
  sensors.begin();
}

void loop(){
  int dev = 0x5A<<1;
  // int dev = 0x55<<1;
  sensors.requestTemperatures(); // Send the command to get temperatures

  float t_dew = sensors.getTempCByIndex(0);
  float t_max = t_dew + 2.0;
  // read sensor object temperature
  float t_ref = read_dev(dev, TEMP_OBJ);

  // Ambient less than dew
  // So turn heater ON
  if (t_ref < t_dew) {
  	heater = true;
    Serial.print("Ref ");
    Serial.print(t_ref);
    Serial.print(" < ");
    Serial.print("Dew ");
    Serial.print(t_dew);
    Serial.print(" : heat: ");
    Serial.println(heater);
  	digitalWrite(led, LOW);
  }
  // Ambient greater than dew + hysteresis gap
  // So turn heater OFF
  else if (t_ref > t_max) {
  	heater = false;
    Serial.print("Ref ");
    Serial.print(t_ref);
    Serial.print(" > ");
    Serial.print("Max ");
    Serial.print(t_max);
    Serial.print(" : heat: ");
    Serial.println(heater);
  	digitalWrite(led, HIGH);
  // In hysteresis gap
  // No further action required
  } else {
    Serial.print("Ref ");
    Serial.print(t_ref);
    Serial.print(", ");
    Serial.print("Dew ");
    Serial.print(t_dew);
    Serial.print(", ");
    Serial.print("Max ");
    Serial.print(t_max);
    Serial.print(" : hysteresis - heat:  ");
    Serial.println(heater);
  }

  delay(2000); // wait a second before printing again
}

void print_status(int dev, float temp) {
  Serial.print("Dev ");
  Serial.print(dev);
  Serial.print(", a is: ");
  Serial.println(temp);
}
