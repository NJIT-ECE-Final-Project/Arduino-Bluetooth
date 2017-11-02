//#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
//#include <Adafruit_LSM9DS0.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);



// Initializing communication ports
//SoftwareSerial mySerial1(11, 9); // TX/RX pins (10 -> TX on HC06, 9 -> RX on HC06)
//SoftwareSerial *mySerial = &mySerial1;
HardwareSerial* mySerial = &Serial1;




#ifndef MAX_LEN
#define MAX_LEN 256
#endif

#ifndef ACCEL
#define ACCEL
#endif

#ifndef __HR
#define __HR
#endif

#ifndef __STEPS
#define __STEPS
#endif

#ifndef __SCREEN
#define __SCREEN
#endif

#ifdef __SCREEN
#define XPOS 0
#define YPOS 1
#define DELTAY 2


#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 

#define PIXELS_HEIGHT 32
#define PIXELS_WIDTH  128

//#if (SSD1306_LCDHEIGHT != 32)
//#error("Height incorrect, please fix Adafruit_SSD1306.h!");
//#endif
#endif

#ifdef __HR
void checkForBeat();
volatile int rate[10];                    // array to hold last ten IBI values
volatile unsigned long sampleCounter = 0;          // used to determine pulse timing
volatile unsigned long lastBeatTime = 0;           // used to find IBI
volatile int P =512;                      // used to find peak in pulse wave, seeded
volatile int T = 512;                     // used to find trough in pulse wave, seeded
volatile int thresh = 512;                // used to find instant moment of heart beat, seeded
volatile int amp = 100;                   // used to hold amplitude of pulse waveform, seeded
volatile boolean firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
volatile boolean secondBeat = false;      // used to seed rate array so we startup with reasonable BPM

// Sensor and pins variables
int pulsePin = 5;
int blinkPin = 13;

// Pulse rate variable
volatile int BPM;    

// Raw signal
volatile int Signal;

// Interval between beats
volatile int IBI = 600;

// Becomes true when the pulse is high
volatile boolean Pulse = false;

// Becomes true when Arduino finds a pulse
volatile boolean QS = false;
#endif


#ifdef ACCEL
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

void setupSensor()
{
  // 1.) Set the accelerometer range
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}
#endif

void processMessage(String message);

//global variables that get placed on the display
#ifdef __SCREEN
String curr_time = "";
#endif

void setup()  
{
  while (!Serial);
  Serial.println("Setup");
  
  Serial.begin(9600);
  mySerial->begin(115200); //115200 for max speed

  //pinMode(10, OUTPUT);
  //pinMode(9, INPUT);

  #ifdef ACCEL
  Serial.println("Initializing LSM9DS1"); 
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS0. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS0 9DOF");
  #endif

  #ifdef __SCREEN
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  // init done

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(2000);

  // Clear the buffer.
  display.clearDisplay();
  #endif

  Serial.println("Setup complete");
}

String getMessage(){
  String msg = "";
  char a;
  
  while(mySerial->available()) {
      //Serial.println("avail");
      a = mySerial->read();
      msg+=String(a);
  }
  return msg;
}

String data = "ABCD1234,ABCD1234,ABCD1234,ABCD1234,ABCD1234,ABCD1234";

char buffer[MAX_LEN];
unsigned int count = 0;

int dispCount = 0;

#ifdef __STEPS
const float accel_thresh = 1.0;
const float accel_max = 5.0;
const int min_delta_t = 3;
int t = 0;
float accmag, longstart, longrem, longend, threshold = 10.2;
int steps = 0, flag = 0;
float xavg = 0, yavg = 0, zavg = 0;

int t_start, t_end, accel_peak;

int step_count = 0;
#endif

void loop()
{

    long start = millis();
    
    //if (mySerial->available()) {  // if the data came
    //  buffer[count++] = mySerial->read(); // read byte
    //}
    //buffer[count] = '\0'; 
    //Serial.print(buffer);

    String message = getMessage();
    if(message.length() != 0) {
      Serial.print("Message: ");
      Serial.println(message);

      processMessage(message);
    }
    count = 0;

    // Send the text you entered in the input field of the Serial Monitor to the HC-06
    //if(Serial.available()){
    //  Serial.print()
    //  mySerial.write(Serial.read());
    //}

    // or send a string
    //Serial.println(millis());

    //mySerial.println(data);

    #ifdef __HR
    checkForBeat();
    #endif


    #ifdef ACCEL
     /* Get a new sensor event */ 
    sensors_event_t accel, mag, gyro, temp;
  
    lsm.getEvent(&accel, &mag, &gyro, &temp); 
  
    // print out accelleration data

    /*    
    Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" ");
    Serial.print("  \tY: "); Serial.print(accel.acceleration.y);       Serial.print(" ");
    Serial.print("  \tZ: "); Serial.print(accel.acceleration.z);     Serial.println("  \tm/s^2");
  
    // print out magnetometer data
    Serial.print("Magn. X: "); Serial.print(mag.magnetic.x); Serial.print(" ");
    Serial.print("  \tY: "); Serial.print(mag.magnetic.y);       Serial.print(" ");
    Serial.print("  \tZ: "); Serial.print(mag.magnetic.z);     Serial.println("  \tgauss");
    
    // print out gyroscopic data
    Serial.print("Gyro  X: "); Serial.print(gyro.gyro.x); Serial.print(" ");
    Serial.print("  \tY: "); Serial.print(gyro.gyro.y);       Serial.print(" ");
    Serial.print("  \tZ: "); Serial.print(gyro.gyro.z);     Serial.println("  \tdps");
    */

    #ifdef __HR
    checkForBeat();
    #endif

    
    // send raw sensor data
    mySerial->print("S:");
    mySerial->print(*((long*) &accel.acceleration.x), HEX);
    mySerial->print(",");
    mySerial->print(*((long*) &accel.acceleration.y), HEX);
    mySerial->print(",");
    mySerial->print(*((long*) &accel.acceleration.z), HEX);
    mySerial->print(",");
    delay(1);
    mySerial->print(*((long*) &gyro.gyro.x), HEX);
    mySerial->print(",");
    mySerial->print(*((long*) &gyro.gyro.y), HEX);
    mySerial->print(",");
    mySerial->print(*((long*) &gyro.gyro.z), HEX);
    mySerial->println();
    
    #ifdef __STEPS
    float xaccl = accel.acceleration.x;
    float yaccl = accel.acceleration.y;
    float zaccl = accel.acceleration.z;

    accmag = sqrt(xaccl * xaccl + yaccl * yaccl + zaccl * zaccl) - 9.81;

    if (accmag > accel_thresh  && flag == 0)
    {
      //Serial.println("Started peak");
      steps = steps + 1;
      flag = 1;
      t_start = t;
      accel_peak = accmag;
    }
    else if (accmag > accel_thresh  && flag == 1)
    {
      if (accmag > accel_peak) {
        //Serial.println("New peak max");
        accel_peak = accmag;
      }
    }
    if (accmag < accel_thresh  && flag == 1)
    { 
      flag = 0;
      //Serial.println("Ended peak");
      t_end = t;
      if ((t_end - t_start + 1) >= min_delta_t && accel_peak < accel_max) {
        Serial.println("Step");
        step_count++;
      } else {
        //Serial.print("DeltaT: ");
        //Serial.println(t_end - t_start);
        //Serial.print("AccelPeak: ");
        //Serial.println(accel_peak);
      }
    }
    t++;
    #endif
    
    #endif

    #ifdef __HR
    checkForBeat();
    #endif
    
    long end = millis();

    long rem = 20 - (end - start);  

    //Serial.println(rem);

    #ifdef __HR
    if (QS == true) {
          
      // Print heart rate      
      Serial.print("Current Heart Rate Sample: ");            
      Serial.println(BPM);

      // send heart rate
      mySerial->print("H:");
      mySerial->println(BPM);
      
      // Reset the Quantified Self flag for next time      
      QS = false;                       
     } else {
      //Serial.println("Q==false");
     }
    #endif

    #ifdef __SCREEN
    if(dispCount++ == 20) {
      dispCount = 0;

      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      display.clearDisplay();
      display.println(curr_time);
      display.print("Heart Rate: ");
      display.print(BPM);
      display.println(" BPM");
      #ifdef __STEPS      
      display.print("Steps: ");
      display.println(step_count);
      #endif
      display.display();
    }
    #endif

    if(rem > 0) {
      //Serial.print("Rem: ");
      //Serial.println(rem);
      delay(rem);
    } else {
      Serial.println("Cannot keep up!");
    }
}

#ifdef __HR
void checkForBeat() {
  Signal = analogRead(pulsePin);              // read the Pulse Sensor 
  
 
  //sampleCounter += 2;                         // keep track of the time in mS with this variable
  sampleCounter = millis();
  int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise

  //Serial.println(Signal);

    //  find the peak and trough of the pulse wave
  if(Signal < thresh && N > (IBI/5)*3){       // avoid dichrotic noise by waiting 3/5 of last IBI
    if (Signal < T){                          // T is the trough
      T = Signal;                             // keep track of lowest point in pulse wave 
    }
  }

  if(Signal > thresh && Signal > P){          // thresh condition helps avoid noise
    P = Signal;                               // P is the peak
  }                                           // keep track of highest point in pulse wave

  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
  // signal surges up in value every time there is a pulse
  if (N > 250){    
    // avoid high frequency noise
    if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) ){

      //Serial.println("Pulse");
              
      Pulse = true;                               // set the Pulse flag when we think there is a pulse
      digitalWrite(blinkPin,HIGH);                // turn on pin 13 LED
      IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
      lastBeatTime = sampleCounter;               // keep track of time for next pulse

      if(secondBeat){                        // if this is the second beat, if secondBeat == TRUE
        secondBeat = false;                  // clear secondBeat flag
        for(int i=0; i<=9; i++){             // seed the running total to get a realisitic BPM at startup
          rate[i] = IBI;                      
        }
      }

      if(firstBeat){                         // if it's the first time we found a beat, if firstBeat == TRUE
        firstBeat = false;                   // clear firstBeat flag
        secondBeat = true;                   // set the second beat flag
        return;                              // IBI value is unreliable so discard it
      }   


      // keep a running total of the last 10 IBI values
      word runningTotal = 0;                  // clear the runningTotal variable    

      for(int i=0; i<=8; i++){                // shift data in the rate array
        rate[i] = rate[i+1];                  // and drop the oldest IBI value 
        runningTotal += rate[i];              // add up the 9 oldest IBI values
      }

      rate[9] = IBI;                          // add the latest IBI to the rate array
      runningTotal += rate[9];                // add the latest IBI to runningTotal
      runningTotal /= 10;                     // average the last 10 IBI values 
      BPM = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
      QS = true;                              // set Quantified Self flag 
      // QS FLAG IS NOT CLEARED INSIDE THIS ISR
    }                       
  }

  if (Signal < thresh && Pulse == true){   // when the values are going down, the beat is over
    digitalWrite(blinkPin,LOW);            // turn off pin 13 LED
    Pulse = false;                         // reset the Pulse flag so we can do it again
    amp = P - T;                           // get amplitude of the pulse wave
    thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
    P = thresh;                            // reset these for next time
    T = thresh;
  }

  if (N > 2500){                           // if 2.5 seconds go by without a beat
    thresh = 512;                          // set thresh default
    P = 512;                               // set P default
    T = 512;                               // set T default
    lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date        
    firstBeat = true;                      // set these to avoid noise
    secondBeat = false;                    // when we get the heartbeat back
  }
}
#endif



void processMessage(String message) {

  #ifdef __SCREEN
  if(message.substring(0,2).equals("T:")) {
    Serial.print("Got time: ");
    curr_time = message.substring(2, message.length() - 1);
    Serial.print(curr_time);
  }
  #endif
}

