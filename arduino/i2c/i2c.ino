/*
  slave I2C
  Sets the Arduino up as a slave I2C device
  
  Reads in the I2C signal from a pin and
  reports the value on the serial port
*/

#include <Wire.h>

byte req_count=0;

void setup() {
  Wire.begin(8);               // join i2c bus with address #8
  Wire.onReceive(receiveData); // register receiveData for data sent by host
  Wire.onRequest(sendData);    // register sendData for data request by host
  Serial.begin(9600);          // start serial for output
}

void loop() {
}

void receiveData(int howMany) {
  // Process received data from the master
  int x = Wire.read();    // receive byte as an integer
  Serial.println(x);      // print the integer
}

void sendData() {
  // Fulfill a request for data from the master
  Wire.write(req_count);
  req_count++;
}

