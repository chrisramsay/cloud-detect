#include <i2cmaster.h>
#include <OneWire.h>
#include <DallasTemperature.h>
 
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
  float t_max = t_dew + 5.0;
  // read sensor object temperature
  float t_ref = read_dev(dev, TEMP_OBJ);

  // Ambient less than dew
  // So turn heater ON
  if (t_ref < t_dew) {
  	heater = true;
    Serial.print(t_ref);
    Serial.print(" > ");
    Serial.print(t_dew);
    Serial.print(" : heat: ");
    Serial.println(heater);
  	digitalWrite(led, HIGH);
  }
  // Ambient greater than dew + hysteresis gap
  // So turn heater OFF
  else if (t_ref > t_max) {
  	heater = false;
    Serial.print(t_ref);
    Serial.print(" > ");
    Serial.print(t_max);
    Serial.print(" : heat: ");
    Serial.println(heater);
  	digitalWrite(led, LOW);
  // In hysteresis gap
  // No further action required
  } else {
    Serial.print("OK - heat:  ");
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

float read_dev(int device, int TaTo) {
  int dev = device;
  int data_low = 0;
  int data_high = 0;
  int pec = 0;

  i2c_start_wait(dev+I2C_WRITE);

  // read object (0) or ambient temperature (1)
  if (TaTo) i2c_write(0x06); else i2c_write(0x07);

  // i2c_write(0x07);

  // read
  i2c_rep_start(dev+I2C_READ);
  // Read 1 byte and then send ack
  data_low = i2c_readAck();
  // Read 1 byte and then send ack
  data_high = i2c_readAck();
  pec = i2c_readNak();
  i2c_stop();

  // This converts high and low bytes together and processes temperature, 
  // MSB is a error bit and is ignored for temps
  // 0.02 degrees per LSB (measurement resolution of the MLX90614)
  double tempFactor = 0.02;
  // zero out the data
  double tempData = 0x0000;
  // data past the decimal point
  int frac; 

  // This masks off the error bit of the high byte,
  // then moves it left 8 bits and adds the low byte.
  tempData = (double)(((data_high & 0x007F) << 8) + data_low);
  tempData = (tempData * tempFactor)-0.01;

  float celcius = tempData - 273.15;
  return celcius;
}
