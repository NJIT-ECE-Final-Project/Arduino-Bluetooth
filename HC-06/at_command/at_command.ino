#include <SoftwareSerial.h>
SoftwareSerial BTSerial(10, 9); // RX | TX

int baud=9600;


void setup()
{

  while (!Serial);  // required for Flora & Micro
  delay(500);
  
  Serial.begin(baud);
  Serial.println("Enter AT commands:");

  pinMode(10, INPUT);
  pinMode(9, OUTPUT);
  
  BTSerial.begin(baud);  // HC-06 current bound rate (default 9600)
  BTSerial.print("AT+VERSION");
}
void loop()
{
  // Keep reading from HC-06 and send to Arduino Serial Monitor
  if (BTSerial.available())
    Serial.write(BTSerial.read());
  // Keep reading from Arduino Serial Monitor and send to HC-06
  if (Serial.available())
    BTSerial.write(Serial.read());
}

//AT  OK  Used to verify communication
//AT+VERSION  OKlinvorV1.8  The firmware version (version might depend on firmware)
//AT+NAMExyz  OKsetname Sets the module name to “xyz”
//AT+PIN1234  OKsetPIN  Sets the module PIN to 1234
//AT+BAUD1  OK1200  Sets the baud rate to 1200
//AT+BAUD2  OK2400  Sets the baud rate to 2400
//AT+BAUD3  OK4800  Sets the baud rate to 4800
//AT+BAUD4  OK9600  Sets the baud rate to 9600
//AT+BAUD5  OK19200 Sets the baud rate to 19200
//AT+BAUD6  OK38400 Sets the baud rate to 38400
//AT+BAUD7  OK57600 Sets the baud rate to 57600
//AT+BAUD8  OK115200  Sets the baud rate to 115200
//AT+BAUD9  OK230400  Sets the baud rate to 230400
//AT+BAUDA  OK460800  Sets the baud rate to 460800
//AT+BAUDB  OK921600  Sets the baud rate to 921600
//AT+BAUDC  OK1382400 Sets the baud rate to 1382400
