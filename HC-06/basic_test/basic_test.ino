#include <SoftwareSerial.h>

// Initializing communication ports
SoftwareSerial mySerial(10, 9); // TX/RX pins

void setup()  
{
  Serial.begin(9600);
  mySerial.begin(115200); //115200 for max speed
}

String getMessage(){
  String msg = "";
  char a;
  
  while(mySerial.available()) {
      a = mySerial.read();
      msg+=String(a);
  }
  return msg;
}

String data = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA";

void loop()
{

    long start = millis();
  
    // Check if a message has been received
    //String msg = getMessage();
    //if(msg!=""){
    //  Serial.println(msg);
    //}

    // Send the text you entered in the input field of the Serial Monitor to the HC-06
    //if(Serial.available()){
    //  Serial.print()
    //  mySerial.write(Serial.read());
    //}

    // or send a string
    Serial.println(millis());


    mySerial.println(data);
    
    long end = millis();

    long rem = 20 - (end - start);

    Serial.println(rem);

    if(rem > 0) {
      delay(rem);
    } else {
      Serial.println("Cannot keep up!");
    }
}
