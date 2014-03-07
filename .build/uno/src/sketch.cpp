#include <Arduino.h>
#include <i2cmaster.h>
void setup();
void loop();
void print_status(int dev, float temp);
float read_dev(int device);
#line 1 "src/sketch.ino"
//#include <i2cmaster.h>

int led = 13;
bool heater = false;

void setup(){
  Serial.begin(9600);
  Serial.println("Setup...");
  
  pinMode(led, OUTPUT);
  i2c_init(); //Initialise the i2c bus
  PORTC = (1 << PORTC4) | (1 << PORTC5);//enable pullups
}

void loop(){
  int dev = 0x5A<<1;
  // int dev = 0x55<<1;
  float low_point = 16.0;
  float high_point = 20.0;
  float amb_temp = read_dev(dev);

  if (amb_temp < low_point) {
  	heater = true;
    Serial.print("| Heat:  ");
    Serial.print(heater);
    Serial.print(" ");
    print_status(dev, amb_temp);
  	digitalWrite(led, HIGH);
  }
  else if (amb_temp > high_point) {
  	heater = false;
    Serial.print("| Heat:  ");
    Serial.print(heater);
    Serial.print(" ");
    print_status(dev, amb_temp);
  	digitalWrite(led, LOW);
  } else {
    Serial.print("| Heat:  ");
    Serial.print(heater);
    Serial.print(" ");
    print_status(dev, amb_temp);
  }


  //read_dev(0x55<<1);
  delay(2000); // wait a second before printing again
}

void print_status(int dev, float temp) {
  Serial.print("Dev ");
  Serial.print(dev);
  Serial.print(": ");
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
