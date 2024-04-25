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
#include <SSD1306.h>
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
#define leftmotorpin D0 //GPIO pin setting for leftmotor (right)
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
int target = 1;

int previousX = 0;
int previousY = 0;

int lastAngle = 0;
bool correctingLeft = 0;

bool goingX = 1;

bool goForward = 1;

int timesnotturnetcauseofangle = 0;

float difference;

float startdifference;


bool dontTurn = 0;

//WiFi Define
WiFiClient espClient;                         
PubSubClient client(espClient);            

SSD1306 display(0x3C, D14, D15);
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

static const unsigned long refreshTime = 1000; // ms
static unsigned long lastRefreshTime = 0;
int mode = 1;

/**
void refresh() {
  display.display();
  display.clearDisplay();
}
**/

//Function Definitions
void rotateleft(double time) //rotate left for "time" mil-sec
{
  if(time > 0.0) {
    leftmotor.write(20);
    rightmotor.write(160-1.55);
    delay(time);
    leftmotor.write(90);
    rightmotor.write(90);
  }
}
void 
rotateright(double time) //rotate right for "time" mil-sec
{
  if(time > 0.0) {
    leftmotor.write(160);
    rightmotor.write(20+1.55);
    delay(time);
    leftmotor.write(90);
    rightmotor.write(90);
  }
}
void forward() //move forward
{
      leftmotor.write(69);
      rightmotor.write(68.5);
}

void fowardslow(){
      leftmotor.write(76.5);
      rightmotor.write(77);
}

void forwardadjust(double distance) //move forward slightly to the left
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

int rotations[3] = {0, 0, 0};

void setup() {
  Serial.begin(115200);

  leftmotor.attach(leftmotorpin); //leftmotor is attached using the leftmotorpin
  rightmotor.attach(rightmotorpin); //rightmotor is attached using the rightmotorpin

  display.init();
  display.clear();
  display.drawString(0, 0, "Stevens Robot");
  display.display();
  display.flipScreenVertically();
  /** SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  **/

  leftmotor.write(90);
  rightmotor.write(90);

  setup_wifi();                               
  delay(3000);

  Serial.println("Wemos POWERING UP ......... ");
  client.setServer(mqtt_server, mqtt_port);          //This 1883 is a TCP/IP port number for MQTT 
  client.setCallback(callback); 
}

void loop() {

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
  //Robot location x,y from MQTT subscription variable testCollector 
  x = testCollector[0];
  y = testCollector[1];
int currentAngle = 0;
if((previousY == y && previousX == x) || x == 1 || y == 1){
  currentAngle = lastAngle;
  dontTurn = 1;
}else{
  currentAngle = atan2(y - previousY, x - previousX) * 180 / PI;
  dontTurn = 0;
}
int targetAngle = atan2(arrayy[target-1] - y, arrayx[target-1] - x) * 180 / PI;

bool goRight = 0;

  startdifference = (targetAngle - currentAngle);
  difference = startdifference;

  if(difference < -180.0f)
      difference += 360.0f;
  if(difference > 180.0f)
      difference -= 360.0f;

  if(difference > 0.0f){
      goRight = 0;
  }
  if(difference < 0.0f){
      goRight = 1;
  }

if(!dontTurn){
  if(abs(difference) > 150 && timesnotturnetcauseofangle < 3){
      currentAngle = lastAngle;
      dontTurn = 1;
      timesnotturnetcauseofangle++;
    if ((arrayx[target-1] - 200 <= x) && (x <= arrayx[target-1] + 200) && (arrayy[target-1] - 200 <= y) && (y <= arrayy[target-1] + 200)){
      reverse();
      delay(400);
      stop();
    } else {
      rotateleft(250);
    }
  } else{
    if (timesnotturnetcauseofangle == 3){
      rotations[0] = currentAngle;
      rotations[1] = currentAngle;
      rotations[2] = currentAngle;
    }
    timesnotturnetcauseofangle = 0;
    dontTurn = 0;
    if(rotations[0] == 0){
      rotations[0] = currentAngle;
    }else if (rotations[1] == 0 ){
      rotations[1] = currentAngle;
    }else if (rotations[2] == 0){
      rotations[2] = currentAngle;
    }
  }
}

//read sensor 1
distancecenter = usonic_center.read(CM)*10; // read in MM
distanceright = usonic_right.read(CM)*10; // read in MM
distanceleft = usonic_left.read(CM)*10; // read in MM
Serial.println(distancecenter);

if (state == 0){ //first time sensing starts robot
  if (distancecenter<80){ 
    delay(1000);
    forward();
    delay(5000);
    stop();
    state = 1;
  }
} else if (state == 1) { //initial forward
    if (target == 3 || target == 5)
    {
      if ((arrayx[target-1] - 200 <= x) && (x <= arrayx[target-1] + 200) && (arrayy[target-1] - 200 <= y) && (y <= arrayy[target-1] + 200)){
        stop();
        delay(500);
        target++;
        rotations[0] = 0;
        rotations[1] = 0;
        rotations[2] = 0;
      }
    }
    else {
      if ((arrayx[target-1] - 50 <= x) && (x <= arrayx[target-1] + 50) && (arrayy[target-1] - 50 <= y) && (y <= arrayy[target-1] + 50)){
        stop();
        delay(2000);
        target++;
        rotations[0] = 0;
        rotations[1] = 0;
        rotations[2] = 0;
      }
    }
    
    if(rotations[0] != 0 && rotations[1] != 0 && rotations[2] != 0){
      float averageangle = (rotations[0] + rotations[1] + rotations[2])/3;
      startdifference = averageangle - targetAngle;
      double adjustment = -150;
      if (abs(startdifference) >= 90){
        adjustment = map(abs(startdifference), 90, 180, 50, 100); //a smaller  distance will lead to a smaller adjustment
      }
      else if (abs(startdifference) >= 15){
        adjustment = map(abs(startdifference), 15, 90, -50, 50); //a smaller  distance will lead to a smaller adjustment
      }
      else if (abs(startdifference) > 7){
        adjustment = map(abs(startdifference), 7, 15, -100, -50);
      } else{
        adjustment = -150;
      }
      adjustment = min(adjustment, 100.0);
      adjustment = max(adjustment, -150.0);
        if(goRight){
          rotateright((150)+adjustment);
          display.clear();
          display.setFont(ArialMT_Plain_16);
          display.drawString(0, 0, "Rotating Right");
          display.drawString(0, 20, String("Amount: (") + (242.5+adjustment));
          display.display();
        }else{
          rotateleft((150)+adjustment);
          display.clear();
          display.setFont(ArialMT_Plain_16);
          display.drawString(0, 0, "Rotating Left");
          display.drawString(0, 20, String("Amount: (") + (242.5+adjustment));
          display.display();
        }
      rotations[0] = 0;
      rotations[1] = 0;
      rotations[2] = 0;
    } else {
      if (distancecenter < 45 || (target == 4 || target == 6) && distancecenter < 30 ){
          reverse();
          delay(100);
          stop();
        if(distanceright > distanceleft){
          rotateright(200);
        }else{
          rotateleft(200);
        }
        rotations[0] = 0;
        rotations[1] = 0;
        rotations[2] = 0;
      }
      else if(distanceright < 40  || (target == 4 || target == 6) && distancecenter < 20 ){
        rotateleft(140);
        rotations[0] = 0;
        rotations[1] = 0;
        rotations[2] = 0;
      } else if(distanceleft < 40  || (target == 3 || target == 5) && distancecenter < 20 ) {
        rotateright(140);
        rotations[0] = 0;
        rotations[1] = 0;
        rotations[2] = 0;
      } else{
        if ((arrayx[target-1] - 200 <= x) && (x <= arrayx[target-1] + 200) && (arrayy[target-1] - 200 <= y) && (y <= arrayy[target-1] + 200)){
          fowardslow();
        }else{
          forward();
        }
      }
    } 
}

  //Display location on OLED
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "Robot S-5");
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 20, String("Location: (") + x + ", " + y + ")");
  display.drawString(0, 30, String("Target: (") + arrayx[target-1] +  ", " + arrayy[target-1] + ")");
  display.drawString(0, 40, String("turn: (") + !dontTurn + ") dif:" + startdifference + "/" + difference);
  display.drawString(0, 50, String("Angle: (") + currentAngle + " / " + targetAngle + ")");
  display.display();

  if(x != 1 && previousX != x)
    previousX = x;
  if (y != 1 && previousY != y)
    previousY = y;
  lastAngle = currentAngle;

  delay(100);

}

