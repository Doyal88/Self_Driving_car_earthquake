#include <Wire.h>
#include <TinyGPS++.h> // Include the TinyGPS++ library
TinyGPSPlus tinyGPS; // Create a TinyGPSPlus object

#define GPS_BAUD 9600 // GPS module baud rate. GP3906 defaults to 9600.

// If you're using an Arduino Uno, RedBoard, or any board that uses the
// 0/1 UART for programming/Serial monitor-ing, use SoftwareSerial:
#include <SoftwareSerial.h>
#define ARDUINO_GPS_RX 9 // GPS RX, Arduino TX pin
#define ARDUINO_GPS_TX 8 // GPS TX, Arduino RX pin Use this to get values
SoftwareSerial ssGPS(ARDUINO_GPS_TX, ARDUINO_GPS_RX); // Create a SoftwareSerial

// Set gpsPort to either ssGPS if using SoftwareSerial or Serial1 if using an
// Arduino with a dedicated hardware serial port
#define gpsPort ssGPS  // Alternatively, use Serial1 on the Leonardo

// Define the serial monitor port. On the Uno, and Leonardo this is 'Serial'
//  on other boards this may be 'SerialUSB'
#define SerialMonitor Serial
#define CMPS11_ADDRESS 0x60  // Address of CMPS11 shifted right one bit for arduino wire library
#define ANGLE_8  1           // Register to read 8bit angle from

unsigned char high_byte, low_byte, angle8;
char pitch, roll;
unsigned int angle16;

void setup()
{
  Serial.begin(9600);  // Start serial port
  Wire.begin();
  gpsPort.begin(GPS_BAUD);
}


void loop()
{
  printGPSInfo();
  
  Wire.beginTransmission(CMPS11_ADDRESS);  //starts communication with CMPS11
  Wire.write(ANGLE_8);                     //Sends the register we wish to start reading from
  Wire.endTransmission();
 
  // Request 5 bytes from the CMPS11
  // this will give us the 8 bit bearing, 
  // both bytes of the 16 bit bearing, pitch and roll
  Wire.requestFrom(CMPS11_ADDRESS, 5);       
  
  while(Wire.available() < 5);        // Wait for all bytes to come back
  
  angle8 = Wire.read();               // Read back the 5 bytes
  high_byte = Wire.read();
  low_byte = Wire.read();
  pitch = Wire.read();
  roll = Wire.read();
  
  angle16 = high_byte;                 // Calculate 16 bit angle
  angle16 <<= 8;
  angle16 += low_byte;

  //Serial.print("angle:");     // Display 16 bit angle with decimal place
  Serial.print(angle16 / 10, DEC);
  Serial.print(".");
  Serial.print(angle16 % 10, DEC); Serial.print(":");

  // "Smart delay" looks for GPS data while the Arduino's not doing anything else
  smartDelay(100); 
}

void printGPSInfo()
{
  // Print latitude, longitude, altitude in feet, course, speed, date, time,
  // and the number of visible satellites.
  //SerialMonitor.print("Lat:"); SerialMonitor.println(tinyGPS.location.lat(), 6);
  //SerialMonitor.print("Long:"); SerialMonitor.println(tinyGPS.location.lng(), 6);
  if (tinyGPS.location.isValid()) {
    //SerialMonitor.print("Lat:"); SerialMonitor.println(tinyGPS.location.lat(), 6);
    //SerialMonitor.print("Long:"); SerialMonitor.println(tinyGPS.location.lng(), 6);
    SerialMonitor.print(tinyGPS.location.lat(), 6); Serial.print(":");
    SerialMonitor.println(tinyGPS.location.lng(), 6);
  }
  else {
    SerialMonitor.println("No fix");
  }
  //SerialMonitor.print("Alt: "); SerialMonitor.println(tinyGPS.altitude.feet());
  //SerialMonitor.print("Course: "); SerialMonitor.println(tinyGPS.course.deg());
  //SerialMonitor.print("Speed: "); SerialMonitor.println(tinyGPS.speed.mph());
  //SerialMonitor.print("Date: "); printDate();
  //SerialMonitor.print("Time: "); printTime();
  
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    // If data has come in from the GPS module
    while (gpsPort.available())
      tinyGPS.encode(gpsPort.read()); // Send it to the encode function
    // tinyGPS.encode(char) continues to "load" the tinGPS object with new
    // data coming in from the GPS module. As full NMEA strings begin to come in
    // the tinyGPS library will be able to start parsing them for pertinent info
  } while (millis() - start < ms);
}