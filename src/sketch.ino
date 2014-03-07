#include <i2cmaster.h>
#include <OneWire.h>
#include <DallasTemperature.h>
 
// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 2
 
// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
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

  float dew_point = sensors.getTempCByIndex(0);
  float max_point = dew_point + 2.0;
  float amb_point = read_dev(dev);

  

  Serial.print("   l is: ");
  Serial.print(dew_point);
  Serial.println("");

  if (amb_point < dew_point) {
  	heater = true;
    Serial.print("| a < l  Heat:  ");
    Serial.print(heater);
    Serial.print(" ");
    print_status(dev, amb_point);
  	digitalWrite(led, HIGH);
  }
  else if (amb_point > max_point) {
  	heater = false;
    Serial.print("| a > l  Heat:  ");
    Serial.print(heater);
    Serial.print(" ");
    print_status(dev, amb_point);
  	digitalWrite(led, LOW);
  } else {
    Serial.print("| ok     Heat:  ");
    Serial.print(heater);
    Serial.print(" ");
    print_status(dev, amb_point);
  }


  //read_dev(0x55<<1);
  delay(2000); // wait a second before printing again
}

void print_status(int dev, float temp) {
  Serial.print("Dev ");
  Serial.print(dev);
  Serial.print(", a is: ");
  Serial.println(temp);
}

float read_dev(int device) {
  int dev = device;
  int data_low = 0;
  int data_high = 0;
  int pec = 0;

  i2c_start_wait(dev+I2C_WRITE);
  i2c_write(0x07);

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
