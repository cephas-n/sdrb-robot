/**********************************************************************
Project Name: Self-Driving Robot for Blind "SDRB"

Description: A self-driving robot that assists visually impaired individuals in navigating their surroundings.

Members: 
      Cephas Naweji, 
      Abel Tshimbu, 
      Eliane Nsenga, 
      Caleb Bahaya, 
      Sarah Musinde, 
      Mohammed Rashad
      
Written by: Cephas Naweji and Abel Tshimbu

Supervised by: Prof. Dr. Mehmet Kusaf

Assisted by: Assist. Assoc. Dr. Ziya dereboylu

Maintained by: Cephas

Date: Febrary 16 2023

Version: 1.0

Arduino Board: Arduino Mega 2560

Components:
  GPS Module
  Compass Module
  Bluetooth Module
  Camera Module
  Sensors: Ultrasonic, IR, ...
  SD Card Module
  Hyskylens AI Camera
  
Libraries Used:
TinyGPS++.h
QMC5883LCompass.h
HUSKYLENS.h
Servo.h
SPI.h
SD.h
SoftwareSerial.h
Wire.h

Copyright:
Cyprus International University
Cephas Naweji, Abel Tshimbu, Eliane Nsenga, Caleb Bahaya, Sarah Musinde, Mohammed Rashad

**********************************************************************/

#include <SoftwareSerial.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <QMC5883LCompass.h>
#include <SPI.h>
#include <SD.h>
#include "HUSKYLENS.h"
#include <Servo.h>


//########### ROBOT GLOBAL VARIABLES ##############
const int MINIMUM_DISTANCE = 50;  //minimum distance between the robot and an obstacle (centimeter)
const int DIRECTION_CORRECTION = 10; //compass heading correction (degrees)
const int GPS_LATITUDE_CORRECTION = 0;  // latitude correction
const int MINIMUM_DISTANCE_SIDE = 80;
const int TURN_DURATION = 1500;  // required time (milliseconds) to turn left or right the robot (45-90 deg)
const int FORWARD_DURATION = 1000;
const int TURN_FORWARD_TRANSITION = 1500;
const int WAIT_1S = 1000;   // wait  1 milliseconds
const int DISTANCE_PRECISION = 1; // cm
const int STOP_SPEED = 255;
const double DESTINATION_LAT_PRECISION = 1e-5; // degree
const double DESTINATION_LNG_PRECISION = 1e-5; // degree
const double DESTINATION_DIST_PRECISION = 20; // meters
enum Command //bluetooth command
{
  VOID = -1,
  START,
  STOP,
  TURN_LEFT,
  TURN_RIGHT,
  FORWARD,
  BACKWARD,
  DESTINATION_ACCOUNTING,
  DESTINATION_CAFE,
  DESTINATION_LIBRARY,
  DESTINATION_HOME
};

enum Direction
{
  FRONT,
  BACK,
  LEFT,
  RIGHT,
  NONE
};

unsigned int motor_speed = 255;
Command direction;  // for manual command
//########### END ROBOT GLOBAL VARIABLES ##############




//########### DESTINATIONS ##############
const double allDestinations[4][2]
{
  {35.18948170908208, 33.358761576271434}, // HOME
  {35.18941323359578, 33.360894744538975}, // location 1
  {35.18935780580859, 33.35890136244558}, // location 2
  {35.187989541698045, 33.36057458060083}, // location 3
};

int activeDestination = 0;
bool destinationIsSet = false;
bool activeDestinationDistance = 0;
double currentDestination[2]  // default: HOME
{
  allDestinations[0][0],
  allDestinations[0][1]
};
//########### END DESTINATIONS ##############




//########### PINS DEFINITION ##############
namespace Pin
{
const int START_BUTTON = 22;
const int STOP_BUTTON = 2;
const int FRONT_IR = 38;
const int RIGHT_TRIG = 49;
const int RIGHT_ECHO = A2;
const int LEFT_TRIG = 26;
const int LEFT_ECHO = A1;
const int RPWM_1 = 4;       //left
const int LPWM_1 = 5;
const int REN_1 = 8;
const int RPWM_2 = 6;       //right
const int LPWM_2 = 7;
const int REN_2 = 9;
const int BUZZER = 36;
const int GPS_TX = 11;
const int GPS_RX = 10;
const int SD_CARD = 53; // CS pin (clock)
const int LEFT_LED = 30;
const int RIGHT_LED = 32;
const int STOP_LED = 34;
const int ACTIVE_LED = 40;
};
//########### END PINS DEFINITION ##############




//########### COMPONENT STATES ##############
namespace State
{
bool obstacle = false;
int arrived = LOW;
int robot = LOW;
int motor = LOW;
int servoPosition = 180;  //store the servo position
};
//########### END COMPONENT STATES ##############




//########### NAVIGATION ENTRIES ##############
namespace NavigationEntry
{
String filename;
bool hasStarted = false;
double timeElapsed = 0;
double latitude = 0;
double longitude = 0;
int compassHeading = 0;
int destinationHeading = 0;
char steering = '-';
int obstacle = LOW;
double distance = 0;
}
//########### END NAVIGATION ENTRIES ##############




//########### OBSTACLE AVOIDANCE VARIABLES ##############
namespace Avoidance {
  long int startTime = 0;
  long int stopTime = 0;
  long int duration = 0;
  long int returnStartTime = 0;
  int retry = 0;
}
//########### END OBSTACLE AVOIDANCE VARIABLES ##############




//########### COMMUNICATION ##############
SoftwareSerial gpsSerial(Pin::GPS_TX, Pin::GPS_RX);
//########### END COMMUNICATION ##############




//########### LIBRARY INSTANCES ##############
TinyGPSPlus gps;
QMC5883LCompass compass;
Servo servo;
HUSKYLENS huskylens;
//########### END LIBRARY INSTANCES ##############




/*****************************************************************************
 *****************************************************************************
 *                                                                           *
                                  FUNCTIONS
 *                                                                           *
 *****************************************************************************
 *****************************************************************************/


/*
  Turns all the motor in forward direction
*/
void forward(const unsigned int motor_speed = motor_speed, const unsigned int duration = 0)
{
  digitalWrite(Pin::RPWM_1, HIGH);
  digitalWrite(Pin::LPWM_1, LOW);
  digitalWrite(Pin::RPWM_2, HIGH);
  digitalWrite(Pin::LPWM_2, LOW);

  analogWrite(Pin::REN_1, motor_speed);
  analogWrite(Pin::REN_2, motor_speed);

  State::motor = HIGH;
  digitalWrite(Pin::LEFT_LED, LOW);
  digitalWrite(Pin::RIGHT_LED, LOW);

  if(duration > 0) {
    delay(duration);
    digitalWrite(Pin::RPWM_1, LOW);
    digitalWrite(Pin::LPWM_1, LOW);
    digitalWrite(Pin::RPWM_2, LOW);
    digitalWrite(Pin::LPWM_2, LOW);
  }
}

/*
   Turns all the motor in backward direction
*/
void backward(const unsigned int SPEED = motor_speed)
{
  digitalWrite(Pin::RPWM_1, LOW);
  digitalWrite(Pin::LPWM_1, HIGH);
  digitalWrite(Pin::RPWM_2, LOW);
  digitalWrite(Pin::LPWM_2, HIGH);

  analogWrite(Pin::REN_1, SPEED);
  analogWrite(Pin::REN_2, SPEED);

  State::motor = HIGH;
  digitalWrite(Pin::LEFT_LED, LOW);
  digitalWrite(Pin::RIGHT_LED, LOW);
}

/*
   Turns robot to the left
   Left motors (1) OFF
   Right motors (2) ON

*/
void turnLeft(const unsigned int steeringSpeed = motor_speed, const unsigned int steeringDuration = 0)
{
  digitalWrite(Pin::RPWM_1, LOW);
  digitalWrite(Pin::LPWM_1, LOW);
  digitalWrite(Pin::RPWM_2, HIGH);
  digitalWrite(Pin::LPWM_2, LOW);

  analogWrite(Pin::REN_1, 0);
  analogWrite(Pin::REN_2, steeringSpeed);

  // indicators
  digitalWrite(Pin::LEFT_LED, HIGH);
  digitalWrite(Pin::RIGHT_LED, LOW);

  // stop steering
  if(steeringDuration > 0) {
      delay(steeringDuration);
      digitalWrite(Pin::RPWM_2, LOW);
  }
}

/*
   Turns robot to the right
   Left motors (1) OFF
   Right motors (2) ON
*/
void turnRight(const unsigned int steeringSpeed = motor_speed, const unsigned int steeringDuration = 0)
{
  digitalWrite(Pin::RPWM_1, HIGH);
  digitalWrite(Pin::LPWM_1, LOW);
  digitalWrite(Pin::RPWM_2, LOW);
  digitalWrite(Pin::LPWM_2, LOW);

  analogWrite(Pin::REN_1, steeringSpeed);
  analogWrite(Pin::REN_2, 0);

  digitalWrite(Pin::LEFT_LED, LOW);
  digitalWrite(Pin::RIGHT_LED, HIGH);

  if(steeringDuration > 0) {
      delay(steeringDuration);
      digitalWrite(Pin::RPWM_1, LOW);
  }
}

/*
   Stop all the motors
   while turning in forward direction

*/
void stopForwardMotors(const unsigned int stoppingSpeed = motor_speed)
{
  for (int Speed = stoppingSpeed; Speed >= 0 && State::motor == HIGH; Speed--)
  {
    analogWrite(Pin::REN_1, Speed);
    analogWrite(Pin::REN_2, Speed);
    delay(2);
  }

  digitalWrite(Pin::RPWM_1, LOW);
  digitalWrite(Pin::LPWM_1, LOW);
  digitalWrite(Pin::RPWM_2, LOW);
  digitalWrite(Pin::LPWM_2, LOW);

  State::motor = LOW;
  digitalWrite(Pin::LEFT_LED, LOW);
  digitalWrite(Pin::RIGHT_LED, LOW);
}

/*
   Stop all the motors
   while turning in backward direction

*/
void stopBackwardMotors(const unsigned int stoppingSpeed = motor_speed)
{
  for (int Speed = stoppingSpeed; Speed >= 0 && State::motor == HIGH; Speed--)
  {
    analogWrite(Pin::REN_1, Speed);
    analogWrite(Pin::REN_2, Speed);
    delay(2);
  }

  digitalWrite(Pin::RPWM_1, LOW);
  digitalWrite(Pin::LPWM_1, LOW);
  digitalWrite(Pin::RPWM_2, LOW);
  digitalWrite(Pin::LPWM_2, LOW);

  State::motor = LOW;
  digitalWrite(Pin::LEFT_LED, LOW);
  digitalWrite(Pin::RIGHT_LED, LOW);
}

/*
   detect obstacle in front of the robot

   This function is an ISR function that changes the value of State::obstacle
   when an obstacle is detected at MINIMUM_DISTANCE
*/
void detectObstacle()
{
//  State::obstacle = false; // TO BE REMOVED once the IR sensor is delivered

  // NORMAL DETECTION
  if(digitalRead(Pin::FRONT_IR) == LOW)
  {
    State::obstacle = true;
  }
  else{
    State::obstacle = false;
  }
  
  NavigationEntry::obstacle = State::obstacle;
}

/*
  this function runs BUZZER for 1 seconds,
  then wait 2 more seconds before return void.
  thus, allowing to verify if the obstacle is still in front of the robot every 3 seconds
*/
void runBuzzer(unsigned int duration = 500)
{
  digitalWrite(Pin::BUZZER, HIGH);
  delay(duration);
  digitalWrite(Pin::BUZZER, LOW);
  delay(2000);
}

/*
   Bluetooth command

   this function receive data from the bleutooth HC-06
   then return the corresponding command
*/
Command bluetoothCommand()
{
  char command = Serial.read();

  Serial.print("======================Bluetooth Command:");
  Serial.println(command);

  switch (command)
  {
    case '1':
      //start robot
      return START;
    case '0':
      //stop robot
      return STOP;
    case '+':
      // Increase speed
      motor_speed = motor_speed <= 240 ? motor_speed + 10 : 255;
      break;
    case '-':
      // Decrease speed
      motor_speed = motor_speed > 20 ? motor_speed - 10 : 5;
      break;
    case '3':
      // location: accounting
      activeDestination = 1;
      return DESTINATION_ACCOUNTING;
    case '4':
      // location: accounting
      activeDestination = 2;
      return DESTINATION_CAFE;
    case '5':
      // location: accounting
      activeDestination = 3;
      return DESTINATION_LIBRARY;
    case '6':
      // location: accounting
      activeDestination = 0;
      return DESTINATION_HOME;
    default:
      //invalid or no command
      return VOID;
  }
}


/*
   stopRobot

   this function stops everything and put the robot in sleep mode
*/
void stopRobot()
{
  Serial.println("Stopping...");

  //1. RESET STATES
  State::obstacle = false;
  State::robot = LOW;
//  State::arrived = false;

  //2. STOP MOTORS
  stopForwardMotors();
  stopBackwardMotors();

  //3. STOP BUZZER
  digitalWrite(Pin::BUZZER, LOW);

  //4. LED
  digitalWrite(Pin::STOP_LED, HIGH);
  digitalWrite(Pin::ACTIVE_LED, LOW);
  digitalWrite(Pin::LEFT_LED, LOW);
  digitalWrite(Pin::RIGHT_LED, LOW);

  Serial.println("Robot status: SLEEPING ......");
}

/*
   control robot state

   this function is an ISR that modify State::motor
   in order to turn off the robot
*/
void turnOffRobot()
{
  State::robot = LOW;
}

/*
   Get position

   this function return the latitude of the current position
   of the robot
*/
bool gpsLocationFound = false;
void updateGPSPosition()
{
  while (gpsSerial.available() > 0)
  {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid())
  {
    gpsLocationFound = true;
    NavigationEntry::hasStarted = true;
    Serial.print("Robot current position (latitude, longitude): ");
    Serial.print(gps.location.lat());
    Serial.print(",");
    Serial.println(gps.location.lng());

    getDistanceFromDestination();
  }
}

void getDistanceFromDestination() {
  if(gpsLocationFound) {
    const double distance = TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      currentDestination[0],
      currentDestination[1]
      );

      NavigationEntry::distance = distance;
  }
}

/*
   navigation

   this function controls the autonomous navigation of the robot,
   It returns the direction the robot should follow

*/
Direction navigation(double destination_lat, double destination_lng)
{
  updateGPSPosition();

  if (gpsLocationFound)
  {
    int destinationHeading = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), destination_lat, destination_lng);
    double destinationHeadingLow = destinationHeading - DIRECTION_CORRECTION;
    double destinationHeadingHigh = destinationHeading + DIRECTION_CORRECTION;
    int currentHeading = getCompassHeading();

    //calculate destination distance
    if(activeDestinationDistance <= 1)
    {
      activeDestinationDistance = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), destination_lat, destination_lng); //meter
    }

    // check if the user has arrived
      //HAS ALREADY ARRIVED AT DESTINATION ?
    getDistanceFromDestination();
    if(gpsLocationFound && NavigationEntry::distance <= DESTINATION_DIST_PRECISION)
    {
      Serial.println("============================== THE ROBOT HAS ARRIVED AT DESTIONATION " + String(activeDestination) + "==================");
      State::robot = LOW;
      State::arrived = true;
      runBuzzer(3000);
    }
  

    //save navigation data
    NavigationEntry::latitude = gps.location.isValid() ? gps.location.lat() : -1.00;
    NavigationEntry::longitude = gps.location.isValid() ? gps.location.lng() : -1.00;
    NavigationEntry::destinationHeading = destinationHeading;
    NavigationEntry::compassHeading = currentHeading;

    if (currentHeading >= destinationHeadingLow && currentHeading <= destinationHeadingHigh )
    {
      Serial.println("Foward");
      NavigationEntry::steering = 'f';
      return FRONT; 
    }
    if (currentHeading > destinationHeadingHigh && abs(currentHeading - destinationHeadingHigh) < 90)
    {
      Serial.println("Turn Right");
      NavigationEntry::steering = 'r';
      return RIGHT;
    }
    if (currentHeading < destinationHeadingHigh && abs(currentHeading - destinationHeadingLow) < 90)
    {
      Serial.println("Turn Left");
      NavigationEntry::steering = 'l';
      return LEFT;
    }
    if (abs(currentHeading - destinationHeadingLow) >= 90)
    {
      if((currentHeading - destinationHeadingLow)>0 && (currentHeading - destinationHeadingLow)<=180 )
      {
        while(abs(currentHeading-destinationHeadingLow)>=DIRECTION_CORRECTION )
        {
          // stop robot
          if(State::robot == LOW)
          {
            break;
          }
          
          //update heading
          currentHeading = getCompassHeading();
          destinationHeading = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), destination_lat, destination_lng);
          
          Serial.println("Turn Right");
          NavigationEntry::steering = 'r';
          turnRight();
          delay(100);
        }
        return NONE;
      }
      if((currentHeading - destinationHeadingLow)<0)
      {
         if((destinationHeadingLow-currentHeading)<=180)
         {
            while(abs(currentHeading-destinationHeadingHigh)>=DIRECTION_CORRECTION )
            {
              // stop robot
              if(State::robot == LOW)
              {
                break;
              }
              
              //update heading
              currentHeading = getCompassHeading();
              destinationHeading = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), destination_lat, destination_lng);
              
              Serial.println("Turn Left");
              NavigationEntry::steering = 'l';
              turnLeft();
              delay(100);
            }
         }
         if( (destinationHeadingLow-currentHeading)>180)
         {
            while(abs(currentHeading-destinationHeadingHigh)>=DIRECTION_CORRECTION )
            {
              // stop robot
              if(State::robot == LOW)
              {
                break;
              }
              
              //update heading
              currentHeading = getCompassHeading();
              destinationHeading = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), destination_lat, destination_lng);
              
              Serial.println("Turn Right");
              turnRight();
              NavigationEntry::steering = 'r';
              delay(100);
            }
         }
        return NONE;
      }
      else
      {
        while(abs(currentHeading-destinationHeadingLow)>=DIRECTION_CORRECTION )
        {
          // stop robot
          if(State::robot == LOW)
          {
            break;
          }
          
          //update heading
          currentHeading = getCompassHeading();
          destinationHeading = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), destination_lat, destination_lng);
          
          Serial.println("Turn Left");
          NavigationEntry::steering = 'l';
          turnLeft();
          delay(100);
        }
      return NONE;
      } 
    }
    else
    { 
        Serial.println(" NOT FOUND");
        return NONE; 
    }   
  }
}


/*
   Save Navigation Data

   this function save the navigation entries (current position, current heading,
   time elapsed, obstacle) to the SD memory card in the following format:

   entry_id,latitude,longitude,heading,time_elapsed,obstacle
*/
bool saveNavigationData()
{
  if (!NavigationEntry::hasStarted)
  {
    return false;
  }

  static long int id = 1;
  static bool filenameIsValid = false;

  const String entry = String(id) + ','
                       + String(NavigationEntry::latitude, 10) + ','
                       + String(NavigationEntry::longitude, 10) + ','
                       + String(NavigationEntry::distance, 10) + ','
                       + String(NavigationEntry::compassHeading) + ','
                       + String(NavigationEntry::destinationHeading) + ','
                       + String(NavigationEntry::steering) + ','
                       + String(millis()) + ','
                       + String(NavigationEntry::obstacle);
  id++;
  //generate filename
  const String filename = (gps.time.isValid() ? String(gps.time.value()) : "") +
                          ".csv";
  if (!filenameIsValid && filename != ".csv")
  {
    NavigationEntry::filename = filename;
    filenameIsValid = true;
    Serial.print("MISSION FILE NAME: ");
    Serial.println(filename);
  }
  else if (!filenameIsValid)
  {
//    Serial.print("MISSION FILE NAME: " + filename);
    return false;
  }

  // save data
  saveData(NavigationEntry::filename.c_str(), entry.c_str());

  return true;
}


/*
   Save Data

   this function write data to sd memory
*/
void saveData(const char *filename, const String &data)
{
  Serial.print("SD Card: Opening '");
  Serial.print(filename);
  Serial.println('\'');

  // open file
  File file = SD.open(filename, FILE_WRITE);

  if (file)
  {
    file.println(data.c_str());
    Serial.println("SD Card: data saved successfully");
  }
  else
  {
    // error
    Serial.print("SD Card: could'nt open the file '");
    Serial.print(filename);
    Serial.println('\'');
  }

  file.close();
  Serial.print("SD Card: file '");
  Serial.print(filename);
  Serial.println("' closed");
}

/*
   Load data

   this function reads data from SD memory

*/
void loadData(const char *filename, String &data)
{
  Serial.print("SD Card: Opening '");
  Serial.print(filename);
  Serial.println("' ...");

  // open file
  File file = SD.open(filename, FILE_READ);

  if (file)
  {
    Serial.print("SD Card: Reading file '");
    Serial.print(filename);
    Serial.println("'");

    while (file.available())
    {
      data += String(file.readString());
    }

    Serial.println("SD Card: data loaded successfully");
  }
  else
  {
    // error
    Serial.print("SD Card: could'nt open the file '");
    Serial.print(filename);
    Serial.println('\'');
  }

  file.close();
  Serial.print("SD Card: file '");
  Serial.print(filename);
  Serial.println("' closed" );
}

/*
   Get compass heading with respect to North

*/
double getCompassHeading()
{
  compass.setCalibration(-922, 925, -1221, 462, -591, 0);
  // Read compass values
  compass.read();

  // Return Azimuth reading
  return compass.getAzimuth();
}

/*
   Calculate distance with ultrasonic sensor

*/
// double calculateDistance(const int trig, const int echo)
// {
//   static unsigned long triggerTime = 0;

//   if(millis() - triggerTime >= 24)
//   {
//     digitalWrite(trig, LOW);
//     delayMicroseconds(2);
//     digitalWrite(trig, HIGH);
//     delayMicroseconds(10);
//     digitalWrite(trig, LOW);
//   }

//   double duration = pulseIn(echo, HIGH);
//   double distance = 0.017 * duration;

// //    Serial.print("Distance: ");
// //    Serial.println(distance);

//   return distance;
// }

/*
   Choose free side

   This function choose the free side between left and right by
   comparing how far the distances compute by both ultrasonic

   if the distances are equal or zero, the right side is return
*/
Direction chooseSide()
{
  Direction freeSide = NONE;
  if (digitalRead(Pin::RIGHT_TRIG) == HIGH) // HIGH: No obstacle, LOW: obstacle
  {
    freeSide = LEFT;
  }

  if(digitalRead(Pin::LEFT_TRIG) == HIGH)
  {
    freeSide = RIGHT;
  }

  return freeSide;
}

/*
   Is Right Side Free

   this function check whether there is an obstacle to the right hand side of
   the robot. It returns true if no obstacle is detected

*/
int isRightSideFree()
{
  return digitalRead(Pin::RIGHT_TRIG) == LOW ? 0 : 1;
}

/*
   Is Left Side Free

   this function check whether there is an obstacle to the left hand side of
   the robot. It returns true if no obstacle is detected

*/
int isLeftSideFree()
{
  const int state = digitalRead(Pin::LEFT_TRIG);
  return  state == LOW ? 0 : 1;
}

/*
   Obstacle Avoidance

   this function handle the avoidance system of the robot.
   It returns 'false' if the obstacle has be avoided or 'true' if not (in this case
   the algorithm should restart)

*/
bool obstacleAvoidance()
{
  Serial.println("avoidance started");
  while (State::obstacle == HIGH)
  {
    switch(chooseSide()) {
      case LEFT: 
        // LEFT
        turnRight(motor_speed, 1500);
        while(true){
          // advance
          forward(150, 1000);
    
          if(digitalRead(Pin::RIGHT_TRIG) == HIGH) break;
          Avoidance::duration++;
        }
    
        Serial.println("the Robot can turn left now ============");
    
        stopForwardMotors();
        
        turnLeft(motor_speed, 1500);
    
        forward(motor_speed, 1000);
        stopForwardMotors();
    
        while(true) {
          forward(150, 1000);
    
          if(digitalRead(Pin::RIGHT_TRIG) == HIGH) break;
        }
    
        stopForwardMotors();
        turnLeft(motor_speed, 1500);
    
        while(Avoidance::duration > 0) {
          forward(150, 1000);
          Avoidance::duration--;
        }

        stopForwardMotors();
        turnRight(motor_speed, 1500);
        runBuzzer();
        break;
      case RIGHT:
        turnLeft(motor_speed, 1500);
        while(true){
          // advance
          forward(150, 1000);
    
          if(digitalRead(Pin::LEFT_TRIG) == HIGH) break;
          Avoidance::duration++;
        }
    
        Serial.println("the Robot can turn right now ============");
    
        stopForwardMotors();
        
        turnRight(motor_speed, 1500);
    
        forward(motor_speed, 1000);
        stopForwardMotors();
    
        while(true) {
          forward(150, 1000);
    
          if(digitalRead(Pin::LEFT_TRIG) == HIGH) break;
        }
    
        stopForwardMotors();
        turnRight(motor_speed, 1500);
    
        while(Avoidance::duration > 0) {
          forward(150, 1000);
          Avoidance::duration--;
        }

        stopForwardMotors();
        turnLeft(motor_speed, 1500);
        runBuzzer();
        break;      
    }
    delay(60000);
    Avoidance::startTime = 0;
    Avoidance::stopTime = 0;
    Avoidance::duration = 0;
    Avoidance::returnStartTime = 0;
  }

  return false;
}


//####################### ARDUINO SETUP FUNCTIONS #######################
void setup()
{
  //1. SETUP PIN MODES
  pinMode(Pin::START_BUTTON, INPUT);
  pinMode(Pin::STOP_BUTTON, INPUT);
  pinMode(Pin::FRONT_IR, INPUT);
  pinMode(Pin::LEFT_ECHO, INPUT);
  pinMode(Pin::RIGHT_ECHO, INPUT);
  pinMode(Pin::LEFT_TRIG, INPUT);
  pinMode(Pin::RIGHT_TRIG, INPUT);
  pinMode(Pin::RPWM_1, OUTPUT);
  pinMode(Pin::LPWM_1, OUTPUT);
  pinMode(Pin::REN_1, OUTPUT);
  pinMode(Pin::RPWM_2, OUTPUT);
  pinMode(Pin::LPWM_2, OUTPUT);
  pinMode(Pin::REN_2, OUTPUT);
  pinMode(Pin::BUZZER, OUTPUT);
  pinMode(Pin::LEFT_LED, OUTPUT);
  pinMode(Pin::RIGHT_LED, OUTPUT);
  pinMode(Pin::STOP_LED, OUTPUT);
  pinMode(Pin::ACTIVE_LED, OUTPUT);

  //2. ATTACH INTERRUPT SERVICE ROUTINES
  attachInterrupt(digitalPinToInterrupt(Pin::STOP_BUTTON), turnOffRobot, RISING);

  //3. INITIALIZE SERIAL COMMUNICATION
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Serial monitor: OK");

  //5. INITIALIZE GPS COMMUNICATION
  gpsSerial.begin(9600);
  while (!gpsSerial)
  {
    Serial.println("GPS com: FAILED");
  }
  Serial.println("GPS com: OK");

  //6. COMPASS CONFIGURATION
  compass.init();
  
  //7. SD CARD MEMORY
  if (!SD.begin(Pin::SD_CARD))
  {
    Serial.println("SD card: FAILED");
  }
  else
  {
    Serial.println("SD card: READY");
  }

  //8. SERVO MOTOR CONFIGURATION
  servo.attach(3);
  servo.write(State::servoPosition);

  //OTHERS
  digitalWrite(Pin::STOP_LED, HIGH);
   digitalWrite(Pin::ACTIVE_LED, LOW);
}
//####################### END ARDUINO SETUP FUNCTIONS #######################




/*****************************************************************************
 *****************************************************************************
 *                                                                           *
                               MAIN PROGRAM LOOP
 *                                                                           *
 *****************************************************************************
 *****************************************************************************/
void loop() {
////  0. DEBUGGING
//  saveNavigationData(); //save navigation history 
//  detectObstacle();
//  if(State::obstacle)
//  {
//    digitalWrite(Pin::BUZZER, HIGH);
//  }
//  else{
//    digitalWrite(Pin::BUZZER, LOW);
//  }
//  bluetoothCommand();
//  obstacleAvoidance();
//  getDistanceFromDestination();
//  updateGPSPosition();
//  Serial.print("Heading: ");
//  Serial.println(TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), currentDestination[0], currentDestination[1]));
//  updateGPSPosition();
//  saveNavigationData();
//  Serial.println("Compass: ");
//  Serial.println(getCompassHeading());

//  Serial.print("Distance: ");
//  Serial.println(calculateDistance(Pin::FRONT_TRIG, Pin::FRONT_ECHO));
//  
  Serial.print("Distance Left: ");
  Serial.println(isLeftSideFree());
//  
  Serial.print("Distance Right: ");
  Serial.println(isRightSideFree());
//  
//  Serial.print("Obstacle: ");
//  Serial.println(detectObstacle());
//  
//  Serial.print("Choose side: ");
//  Serial.println(chooseSide());
//  
//  Serial.print("Check Left side: ");
//  Serial.println(isLeftSideFree());
//  
//  Serial.print("Check Right side: ");
//  Serial.println(isRightSideFree());
//  Serial.print("Start Button: ");
//  Serial.println(digitalRead(Pin::START_BUTTON));
//  
//  Serial.print("Stop Button: ");
//  Serial.println(digitalRead(Pin::STOP_BUTTON));
    Serial.println("=======================================DESTINATION: " + String(activeDestination));

    Serial.println("DISTANCE FROM DESTINATION: " + String(NavigationEntry::distance));
////  END DEBUGGING


  // 1. BLUETOOTH COMMAND
  if (Serial.available() > 0)
  {
    Command cmd = bluetoothCommand();
    switch (cmd)
    {
      case START:
        State::robot = HIGH;
        State::arrived = LOW;
        Serial.println("START==========================");
        delay(3000);
        break;
      case STOP:
        stopRobot();
        break;
      case DESTINATION_ACCOUNTING:
      case DESTINATION_CAFE:
      case DESTINATION_LIBRARY:
      case DESTINATION_HOME:
        Serial.println("Destination: ACCOUNTING..............");
        currentDestination[0] = allDestinations[activeDestination][0];
        currentDestination[1] = allDestinations[activeDestination][1];
        destinationIsSet = true;
        break;
      default:
        direction = cmd;
    }
  }
  // END BLUETOOTH COMMAND


  //2.  MANUAL COMMAND
  if (digitalRead(Pin::START_BUTTON) == HIGH) //start robot
  {
    State::robot = HIGH;
  }

  if (State::robot == LOW)
  {
    stopRobot();
    digitalWrite(Pin::ACTIVE_LED, LOW);
    digitalWrite(Pin::STOP_LED, HIGH);
  }
  else
  {
    digitalWrite(Pin::STOP_LED, LOW);
    digitalWrite(Pin::ACTIVE_LED, LOW);
  }
  //END MANUAL COMMAND

  // 3. ROBOT ACTIVE MODE
  if (State::robot == HIGH)
  {
    Serial.println("Robot status: WORKING");

    detectObstacle();
    
    if (!State::obstacle)
    {
      //3.1.  AUTONOMOUS NAVIGATION
       switch(navigation(currentDestination[0], currentDestination[1]))
      {
        case LEFT:
          turnLeft();
          delay(200);
          break;
        case RIGHT:
          turnRight();
          delay(200);
          break;
        case FRONT:
          forward();
          break;
          forward(100);
      }
      saveNavigationData(); //save navigation history 
      //END AUTONOMOUS NAVIGATION
    }
    else
    {
      //3.3. OBSTACLE AVOIDANCE       
      stopForwardMotors(STOP_SPEED);
      runBuzzer();
      obstacleAvoidance();
    }
  }
  // END ROBOT ACTIVE MODE
}