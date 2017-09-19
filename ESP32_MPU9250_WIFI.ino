/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.
 Modified by Brent Wilkins July 19, 2016

 Demonstrate basic MPU-9250 functionality including parameterizing the register
 addresses, initializing the sensor, getting properly scaled accelerometer,
 gyroscope, and magnetometer data out. Added display functions to allow display
 to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
 Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini
 and the Teensy 3.1.

 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.

 Hardware setup:
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- 21
 SCL ----------------------- 22
 GND ---------------------- GND
 */

#include "MPU9250.h"
#include "SD_IO.h"

#define AHRS true         // Set to false for basic data read
#define SerialDebug false  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling

MPU9250 myIMU;
SD_IO mySD;

#include <WiFi.h>       // use for ESP32
#include <WiFiUdp.h>

#define sendInterval 100

//const char* ssid = "TP-LINK_B979AC";
//const char* password = "wjl13146792866";
const char* ssid = "NETGEAR26";
const char* password = "zanywater094";
//const char* ssid = "ShawnX";
//const char* password = "86753099";
WiFiUDP Udp;
static IPAddress remoteIp = IPAddress();
static uint16_t remotePort = 4210;
unsigned int localUdpPort = 4210;  // local port to listen on
char incomingPacket[256];  // buffer for incoming packets

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
bool time_arrive = false;
int f = 0;
void IRAM_ATTR onTimer(){
  time_arrive = true;
}
void init_timer(){
  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, 10000, true);//10000-100Hz-10ms
  //timerAlarmWrite(timer, 1000000, true);//1000000-1Hz-1000ms

  // Start an alarm
  timerAlarmEnable(timer);
}

void init_wifi(){
  //Setup wifi and udp connection
  Serial.println();
  Serial.printf("Connecting to %s ", ssid);
    
  // delete old config
  WiFi.disconnect(true);
    
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  
  Serial.println(" connected");
  Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  WiFi.localIP().toString().c_str(), localUdpPort; 
}
void setup()
{
  Wire.begin(21, 22, 400000);
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(115200);

  mySD.Init();
  init_wifi();
  
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
  myIMU.Init();
  init_timer();
}

bool isRecording = false;
String filename;
char filepath[40];

void filename2path(String fn){
  String path;
  path = "/" + fn + ".csvv";
  path.toCharArray(filepath, path.length());
  //filepath[path.length()] = 0;
}
Packet_u pack;
int wifi_delay = 0;
int time_syn = 0;
void loop()
{
  myIMU.update(SerialDebug);
  if(isRecording && time_arrive){
    myIMU.FillMessage(&pack, wifi_delay);
    mySD.write(pack.buf, sizeof(sensorData_t));
    time_arrive = false;
    time_syn++;
    if (time_syn==100){
      time_syn = 0;
      int delt_t;
      delt_t = millis();
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(pack.buf, sizeof(sensorData_t));
      Udp.endPacket();    
      wifi_delay = millis() - delt_t; 
    } else
      wifi_delay = 0;
    //Serial.printf("time %d\n", pack.sensor.time_);
  }
  int packetSize = Udp.parsePacket();
  if (packetSize){
      // receive incoming UDP packets
    remoteIp = Udp.remoteIP();
    remotePort = Udp.remotePort();
    //Serial.printf("Received %d bytes from %s, port %d\n", packetSize, remoteIp.toString().c_str(), remotePort);
    int len = Udp.read(incomingPacket, 255);
    incomingPacket[len] = 0;
    //Serial.printf("UDP packet contents: %s\n", incomingPacket);
    String cmd((char*) incomingPacket);
    //Serial.printf("length %d ,", cmd.length());
    //Serial.println(cmd);
    //Serial.println(cmd.substring(0,6));
    //Serial.println(cmd.substring(6));
    if (cmd.substring(0, 6) == "Start:"){
      Serial.println("Start to record:");
      filename = cmd.substring(6);
      filename2path(filename);
      mySD.openFile(SD, filepath);
      isRecording = true;           
    } else if (cmd.substring(0, 5) == "Stop:") {
      if (isRecording) {
        mySD.closeFile();
        isRecording = false;
        Serial.println("End record.");
      } else {
        Serial.println("wrong Stop command!");
      }
    } else if (cmd.substring(0, 6) == "Abort:") {
      if (isRecording) {
        mySD.closeFile();
        isRecording = false;
        Serial.println("Cancel record");
        if (filename == cmd.substring(6))
          mySD.deleteFile(SD, filepath);  
        else
          Serial.println("filename is not match!");
      } else {
        Serial.println("wrong Abort command!");
      }
    }
    else
      Serial.printf("receieved a wrong command: %s\n", incomingPacket);
  }
}
