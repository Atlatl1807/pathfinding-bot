/*------------------------------------------------------------------------------------------
 * Ver. 1 - 02/25/2024
 * This program is the sample code used for the 7th week lecture in the 2024 Spring semester. 
 * Week7: "Using LiDAR sensor through MQTT Network."
 * 
 * Based on this sample code, students should write additional commands to output robot position 
 * information using OLED Display and Serial Monitor.                      -SES IDEAs Program-

 Please note that Sample codes provided every semester contain hidden codes to prevent copying and cheating.
 Arduino Sketch code copying and sharing are strictly prohibited and will be reported as a Stevens Honor Code violation.
 -------------------------------------------------------------------------------------------*/

#include <ESP8266WiFi.h>  
#include "24s_PubSubClient.h"
#include "24s_WiFiManager.h"  
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_SH1106.h"
#include <Servo.h>
#include <Ultrasonic.h>
  
//MQTT Communication associated variables
char payload_global[100];                     
boolean flag_payload;                         

//MQTT Setting variables  
const char* mqtt_server= "192.168.0.140";               //MQTT Broker(Server) Address
const char* MQusername = "user";               //MQTT username
const char* MQpassword = "Stevens1870";               //MQTT password
const char* MQtopic    = "louis_lidar_new";               //MQTT Topic (Arena I/II)
const int mqtt_port    = 1883   ;          //MQTT TCP/IP port number 

//WiFi Setting variables
const char* ssid     = "TP-Link_9402";                 //Wi-Fi SSID (Service Set IDentifier)   
const char* password = "77556578";                 //Wi-Fi Password

// Define the DC motor control signal pins
#define leftmotorpin D4 //GPIO pin setting for leftmotor (right)
#define rightmotorpin D2 //GPIO pin setting for rightmotor (left)
Servo leftmotor; //Create servo leftmotor object to control a servo
Servo rightmotor; //Create servo rightmotor object to control a servo

//Define ultrasonic sensors trig/control signals
Ultrasonic usonic_center(D8, D5);
int distancecenter;//define variable

Ultrasonic usonic_right(D9, D6);
int distanceright;//define variable

Ultrasonic usonic_left(D10, D7);
int distanceleft;//define variable

bool goingforward = 1;
int state = 0;
bool obstacleleft = 0;
bool obstacleright = 0;
bool turnedleft = 0;
bool turnedright = 0;

//WiFi Define
WiFiClient espClient;                         
PubSubClient client(espClient);            

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SH1106 display(OLED_RESET);

static const unsigned long refreshTime = 1000; // ms
static unsigned long lastRefreshTime = 0;
int mode = 1;

void refresh() {
  display.display();
  display.clearDisplay();
}

//Function Definitions
void rotateleft(int time) //rotate left for "time" mil-sec
{
      leftmotor.write(70);
      rightmotor.write(110);
      delay(time);
      leftmotor.write(90);
      rightmotor.write(90);
}
void rotateright(int time) //rotate right for "time" mil-sec
{
      leftmotor.write(110);
      rightmotor.write(70);
      delay(time);
      leftmotor.write(90);
      rightmotor.write(90);
}
void forward() //move forward
{
      leftmotor.write(62);
      rightmotor.write(63.55);
}
void forwardadjust(int distance) //move forward slightly to the left
{     
      double adjustment = map(distance, 30, 60, 7.5, -2.5); //a higher distance will lead to a smaller adjustment
      adjustment = min(adjustment, 5.0);
      adjustment = max(adjustment, -5.0);
      leftmotor.write(62); 
      rightmotor.write(63.55+adjustment);
}
void reverse() //move backwards
{
      leftmotor.write(180);
      rightmotor.write(170);
}
void stop() //halt movement
{
      leftmotor.write(90);
      rightmotor.write(90);
}
      
void setup_wifi() { 
  delay(10);
  // We start by connecting to a Stevens WiFi network
  WiFi.begin(ssid, password);           
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");                        
  }
  randomSeed(micros());                       
}

void callback(char* topic, byte* payload, unsigned int length) {
  for (int i = 0; i < length; i++) {
    payload_global[i] = (char)payload[i];
  }
  payload_global[length] = '\0';              
  flag_payload = true;                        
}

void reconnect() {                                                                
  // Loop until we're reconnected
  while (!client.connected()) {
    // Create a random client ID
    String clientId = "ESP8266Client-";       
    clientId += String(random(0xffff), HEX);  
    // Attempt to connect                     
    if (client.connect(clientId.c_str(),MQusername,MQpassword)) {
      client.subscribe(MQtopic);             
    } else {
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

int arrayx[5] = {1600,2000,110,700};
int arrayy[5] = {130,170,150,130};

void setup() {
  Serial.begin(115200);

  leftmotor.attach(leftmotorpin); //leftmotor is attached using the leftmotorpin
  rightmotor.attach(rightmotorpin); //rightmotor is attached using the rightmotorpin

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  //display.begin(SH1106_SWITCHCAPVCC, 0x3C);
  //refresh();

  leftmotor.write(90);
  rightmotor.write(90);

  //setup_wifi();                               
  delay(3000);

  Serial.println("Wemos POWERING UP ......... ");
  //client.setServer(mqtt_server, mqtt_port);          //This 1883 is a TCP/IP port number for MQTT 
  //client.setCallback(callback); 
}

int setAngle = 0;
int myAngle = 0;

void loop() {
  //subscribe the data from MQTT server

  if (!client.connected()) {
    Serial.print("...");
    reconnect();
  }
  client.loop();                              
  
  String payload(payload_global);              
  int testCollector[10];                      
  int count = 0;
  int prevIndex, delimIndex;
    
  prevIndex = payload.indexOf('[');           
  while( (delimIndex = payload.indexOf(',', prevIndex +1) ) != -1){
    testCollector[count++] = payload.substring(prevIndex+1, delimIndex).toInt();
    prevIndex = delimIndex;
  }
  delimIndex = payload.indexOf(']');
  testCollector[count++] = payload.substring(prevIndex+1, delimIndex).toInt();
   
  int x, y; 
  Robot location x,y from MQTT subscription variable testCollector 
  x = testCollector[0];
  y = testCollector[1];
  angle = testCollector[2zxtz];

  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("Robot S-5");
  display.drawLine (0,30,128,30, WHITE);
  display.setTextSize(1);
  display.print("Location");
  display.println(String(" (") + x + ", " + y + ")");
  //display.startscrollright(0x00, 0x0F);
  //display.stopscroll();
  refresh();

  Serial.println(String("Location: ( ") + x + ", " + y + ")");

//read sensor 1
distancecenter = usonic_center.read(CM)*10; // read in MM
distanceright = usonic_right.read(CM)*10; // read in MM
distanceleft = usonic_left.read(CM)*10; // read in MM
Serial.println(distancecenter);

//setAngle = atan2(p1.y - p2.y, p1.x - p2.x) * 180 / PI

if (state == 0){ //first time sensing starts robot
  if (distancecenter<80){ 
    delay(1000);
    state = 1;
  }
} else if (state == 1) { //initial forward
    if (distancecenter<80 || distanceleft<20 || distanceright<20){ 
      stop();
      obstacleleft = 0;
      obstacleright = 0;
      turnedright = 0;
      turnedleft= 0;
      if (distanceleft<80){ 
        obstacleleft = 1;
      }
      if (distanceright<80){ 
        obstacleright = 1;
      }
      if (obstacleright && obstacleleft){
        state = 2;
      }
      else if (obstacleright || (!obstacleleft && distanceright > distanceleft)) {
        rotateleft(485);
        turnedleft = 1;
        state = 3;
      }
      else{
        rotateright(485);
        turnedright = 1;
        state =
         3;
      }
    } 
    else {
      forward();
      }
} else if (state == 2) { //reverse until direction clear
    turnedleft = 0;
    turnedright = 0;
    if (distanceleft > 80){ 
        stop();
        rotateright(485);
        state = 3;
        turnedleft = 1;
      }
    else if (distanceright > 80){ 
        stop();
        rotateleft(485);
        state = 3;
        turnedright = 1;
      }
    else{
      reverse();
    }
  }
else if (state == 3) { //forward until we can turn back
    if (turnedright && distanceleft > 80){ 
        forward();
        delay(500);
        stop();
        delay(100);
        rotateleft(485);
        state = 1;
      }
    else if (turnedleft && distanceright > 80){ 
        forward();
        delay(500);
        stop();
        delay(100);
        rotateright(485);
        state = 1;
      }
    else{
      forward();
    }
  }

  //Display location on OLED
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("Robot S-5");
  display.drawLine (0,30,128,30, WHITE);
  display.setTextSize(1);
  display.print("Location");
  display.println(String(" (") + x + ", " + y + ")");
  refresh();

  delay(100);
}
