//#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!


// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// Initializing communication ports
//SoftwareSerial mySerial(10, 9); // TX/RX pins


#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5
// You can also use software SPI
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);


void setupSensor()
{
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);



  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);

}

void setup() 
{
  Serial.begin(115200);
  //mySerial.begin(115200); //115200 for max speed


  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
  
  Serial.println("LSM9DS1 data read demo");
  
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  setupSensor();
}

void loop() 
{
  float longstart, longrem, longend;
  float oldx, newx, oldy, newy, oldz, newz;
  float  xacc, yacc, zacc, mag;
  float  xgyro, ygyro, zgyro;
   
  longstart = millis();
  lsm.read();  /* ask it to read in the data */ 

  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp); 
  xacc = a.acceleration.x;
  yacc = a.acceleration.y;
  zacc = a.acceleration.z;
  xgyro = g.gyro.x;
  ygyro = g.gyro.y;
  zgyro = g.gyro.z;
  
  //mag = sqrt(pow(xacc,2) + pow(yacc,2) + pow(zacc,2));

  Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
  Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
  Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");
  Serial.println(mag);
  Serial.println("\n");



  
  Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" dps");
  Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" dps");
  Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" dps");
  
  Serial.println();

  //Serial.println(sqrt(pow(g.gyro.x,2)+pow(g.gyro.y,2)+pow(g.gyro.z,2)));

  /*
  mySerial.print(*((long*) &xacc), HEX);
  mySerial.print(",");
  mySerial.print(*((long*) &yacc), HEX);
  mySerial.print(",");
  mySerial.print(*((long*) &zacc), HEX);
  mySerial.print(",");
  mySerial.print(*((long*) &xgyro), HEX);
  mySerial.print(",");
  mySerial.print(*((long*) &ygyro), HEX);
  mySerial.print(",");
  mySerial.print(*((long*) &zgyro), HEX);
  mySerial.println();
  */

  longend = millis();
  longrem = 20 - (longend - longstart);
  if (longrem > 0){
    delay(longrem);
    Serial.print("Rem: ");
    Serial.println(longrem);
  }
  else {
    Serial.println("Too Slow");
  }
}

void printHex(int num, int precision) {
     char tmp[16];
     char format[128];

     sprintf(format, "0x%%.%dX", precision);

     sprintf(tmp, format, num);
     Serial.print(tmp);
}

