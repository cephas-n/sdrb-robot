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

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <QMC5883LCompass.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <string.h>
#include <ctype.h>

// Custom  Headers
#include <Pins.h>
#include <Constants.h>
#include <Locations.h>
#include <States.h>
#include <DataEntries.h>
#include <FunctionPrototypes.h>

// SDRB Library
#include <SDRB.h>


//########### DESTINATION ##############
int active_destination = 0;
bool destination_is_set = false;
int active_destination_distance = 0;
double current_destination[2]  // default: HOME
{
  allDestinations[0][0],
  allDestinations[0][1]
};
//########### END DESTINATION ##############


//########### COMMUNICATION ##############
SoftwareSerial gps_serial(Pin::GPS_TX, Pin::GPS_RX);
//########### END COMMUNICATION ##############


//########### LIBRARY INSTANCES ##############
TinyGPSPlus gps;
QMC5883LCompass compass;
//########### END LIBRARY INSTANCES ##############


MotorController *motor_controller = new MotorController();
InfraRed front_ir;
InfraRed left_ir;
InfraRed right_ir;
Led led_stop;
Led led_active;
Led led_left;
Led led_right;


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
void forward(const uint8_t motor_speed = MOTOR_SPEED, const uint16_t duration = 0)
{
  led_left.turn_off();
  led_right.turn_off();

  motor_controller->set_speed(motor_speed)
                  ->forward(duration);
}

/*
   Turns all the motor in backward direction
*/
void backward(const uint8_t motor_speed = MOTOR_SPEED, const uint16_t duration = 0)
{
  led_left.turn_on();
  led_right.turn_on();

  motor_controller->set_speed(motor_speed)
                  ->reverse(duration);
}

/*
   Turns robot to the left
   Left motors (1) OFF
   Right motors (2) ON

*/
void turn_left(const uint8_t steering_speed = MOTOR_SPEED, const uint16_t steering_duration = STEERING_DURATION)
{
  // indicators
  led_left.turn_on();
  led_right.turn_off();

  motor_controller->set_speed(steering_speed)
                  ->turn_left()
                  ->stop_after(steering_duration);
}

/*
   Turns robot to the right
   Left motors (1) OFF
   Right motors (2) ON
*/
void turn_right(const uint8_t steering_speed = MOTOR_SPEED, const uint16_t steering_duration = STEERING_DURATION)
{
  // indicators
  led_left.turn_off();
  led_right.turn_on();

  motor_controller->set_speed(steering_speed)
                  ->turn_right()
                  ->stop_after(steering_duration);
}

/*
   Stop all the motors
   while turning in forward direction

*/
void stop_forward_motors(const unsigned int stoppingSpeed = MOTOR_SPEED)
{
  led_left.turn_off();
  led_right.turn_off();

  motor_controller->stop();
}

/*
   Stop all the motors
   while turning in backward direction

*/
void stop_backward_motors(const unsigned int stoppingSpeed = MOTOR_SPEED)
{
  led_left.turn_off();
  led_right.turn_off();

  motor_controller->stop();
}

/*
  this function runs BUZZER for 1 seconds,
  then wait 2 more seconds before return void.
  thus, allowing to verify if the obstacle is still in front of the robot every 3 seconds
*/
void run_buzzer(unsigned int duration = 500)
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
Command bluetooth_command()
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
    MOTOR_SPEED = MOTOR_SPEED <= 240 ? MOTOR_SPEED + 10 : 255;
    break;
  case '-':
    // Decrease speed
    MOTOR_SPEED = MOTOR_SPEED > 20 ? MOTOR_SPEED - 10 : 5;
    break;
  case '3':
    // location: accounting
    active_destination = 1;
    return DESTINATION_ACCOUNTING;
  case '4':
    // location: accounting
    active_destination = 2;
    return DESTINATION_CAFE;
  case '5':
    // location: accounting
    active_destination = 3;
    return DESTINATION_LIBRARY;
  case '6':
    // location: accounting
    active_destination = 0;
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
void stop_robot()
{
  Serial.println("Stopping...");

  //1. RESET STATES
  State::obstacle = false;
  State::robot = LOW;
  //  State::arrived = false;

    //2. STOP MOTORS
  motor_controller->stop();

  //3. STOP BUZZER
  digitalWrite(Pin::BUZZER, LOW);

  //4. LED
  led_left.turn_off();
  led_right.turn_off();
  led_active.turn_off();
  led_stop.turn_on();

  Serial.println("Robot status: SLEEPING ......");
}

/*
   control robot state

   this function is an ISR that modify State::motor
   in order to turn off the robot
*/
void turn_off_robot()
{
  State::robot = LOW;
}

/*
   Get position

   this function return the latitude of the current position
   of the robot
*/
bool gps_location_found = false;
void update_gps_position()
{
  while (gps_serial.available() > 0)
  {
    gps.encode(gps_serial.read());
  }

  if (gps.location.isValid())
  {
    gps_location_found = true;
    NavigationEntry::hasStarted = true;
    Serial.print("Robot current position (latitude, longitude): ");
    Serial.print(gps.location.lat());
    Serial.print(",");
    Serial.println(gps.location.lng());

    get_distance_from_destination();
  }
}

void get_distance_from_destination() {
  if (gps_location_found) {
    const double distance = TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      current_destination[0],
      current_destination[1]
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
  update_gps_position();

  if (gps_location_found)
  {
    int destinationHeading = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), destination_lat, destination_lng);
    double destinationHeadingLow = destinationHeading - DIRECTION_CORRECTION;
    double destinationHeadingHigh = destinationHeading + DIRECTION_CORRECTION;
    int currentHeading = get_compass_heading();

    //calculate destination distance
    if (active_destination_distance <= 1)
    {
      active_destination_distance = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), destination_lat, destination_lng); //meter
    }

    // check if the user has arrived
      //HAS ALREADY ARRIVED AT DESTINATION ?
    get_distance_from_destination();
    if (gps_location_found && NavigationEntry::distance <= DESTINATION_DIST_PRECISION)
    {
      Serial.println("============================== THE ROBOT HAS ARRIVED AT DESTIONATION " + String(active_destination) + "==================");
      State::robot = LOW;
      State::arrived = true;
      run_buzzer(3000);
    }


    //save navigation data
    NavigationEntry::latitude = gps.location.isValid() ? gps.location.lat() : -1.00;
    NavigationEntry::longitude = gps.location.isValid() ? gps.location.lng() : -1.00;
    NavigationEntry::destinationHeading = destinationHeading;
    NavigationEntry::compassHeading = currentHeading;

    if (currentHeading >= destinationHeadingLow && currentHeading <= destinationHeadingHigh)
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
      if ((currentHeading - destinationHeadingLow) > 0 && (currentHeading - destinationHeadingLow) <= 180)
      {
        while (abs(currentHeading - destinationHeadingLow) >= DIRECTION_CORRECTION)
        {
          // stop robot
          if (State::robot == LOW)
          {
            break;
          }

          //update heading
          currentHeading = get_compass_heading();
          destinationHeading = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), destination_lat, destination_lng);

          Serial.println("Turn Right");
          NavigationEntry::steering = 'r';
          turn_right();
          delay(100);
        }
        return NONE;
      }
      if ((currentHeading - destinationHeadingLow) < 0)
      {
        if ((destinationHeadingLow - currentHeading) <= 180)
        {
          while (abs(currentHeading - destinationHeadingHigh) >= DIRECTION_CORRECTION)
          {
            // stop robot
            if (State::robot == LOW)
            {
              break;
            }

            //update heading
            currentHeading = get_compass_heading();
            destinationHeading = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), destination_lat, destination_lng);

            Serial.println("Turn Left");
            NavigationEntry::steering = 'l';
            turn_left();
            delay(100);
          }
        }
        if ((destinationHeadingLow - currentHeading) > 180)
        {
          while (abs(currentHeading - destinationHeadingHigh) >= DIRECTION_CORRECTION)
          {
            // stop robot
            if (State::robot == LOW)
            {
              break;
            }

            //update heading
            currentHeading = get_compass_heading();
            destinationHeading = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), destination_lat, destination_lng);

            Serial.println("Turn Right");
            turn_right();
            NavigationEntry::steering = 'r';
            delay(100);
          }
        }
        return NONE;
      }
      else
      {
        while (abs(currentHeading - destinationHeadingLow) >= DIRECTION_CORRECTION)
        {
          // stop robot
          if (State::robot == LOW)
          {
            break;
          }

          //update heading
          currentHeading = get_compass_heading();
          destinationHeading = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), destination_lat, destination_lng);

          Serial.println("Turn Left");
          NavigationEntry::steering = 'l';
          turn_left();
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
bool save_navigation_data()
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
  save_data(NavigationEntry::filename.c_str(), entry.c_str());

  return true;
}


/*
   Save Data

   this function write data to sd memory
*/
void save_data(const char* filename, const String& data)
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
void loadData(const char* filename, String& data)
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
  Serial.println("' closed");
}

/*
   Get compass heading with respect to North

*/
double get_compass_heading()
{
  compass.setCalibration(-922, 925, -1221, 462, -591, 0);
  // Read compass values
  compass.read();

  // Return Azimuth reading
  return compass.getAzimuth();
}

/*
   Choose free side

   This function choose the free side between left and right by
   comparing how far the distances compute by both ultrasonic

   if the distances are equal or zero, the right side is return
*/
Direction choose_side()
{
  Direction freeSide = NONE;
  if (digitalRead(Pin::RIGHT_TRIG) == HIGH) // HIGH: No obstacle, LOW: obstacle
  {
    freeSide = LEFT;
  }

  if (digitalRead(Pin::LEFT_TRIG) == HIGH)
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
    switch (choose_side()) {
    case LEFT:
      // LEFT
      turn_right(MOTOR_SPEED, 1500);
      while (true) {
        // advance
        forward(150, 1000);

        if (digitalRead(Pin::RIGHT_TRIG) == HIGH) break;
        Avoidance::duration++;
      }

      Serial.println("the Robot can turn left now ============");

      stop_forward_motors();

      turn_left(MOTOR_SPEED, 1500);

      forward(MOTOR_SPEED, 1000);
      stop_forward_motors();

      while (true) {
        forward(150, 1000);

        if (digitalRead(Pin::RIGHT_TRIG) == HIGH) break;
      }

      stop_forward_motors();
      turn_left(MOTOR_SPEED, 1500);

      while (Avoidance::duration > 0) {
        forward(150, 1000);
        Avoidance::duration--;
      }

      stop_forward_motors();
      turn_right(MOTOR_SPEED, 1500);
      run_buzzer();
      break;
    case RIGHT:
      turn_left(MOTOR_SPEED, 1500);
      while (true) {
        // advance
        forward(150, 1000);

        if (digitalRead(Pin::LEFT_TRIG) == HIGH) break;
        Avoidance::duration++;
      }

      Serial.println("the Robot can turn right now ============");

      stop_forward_motors();

      turn_right(MOTOR_SPEED, 1500);

      forward(MOTOR_SPEED, 1000);
      stop_forward_motors();

      while (true) {
        forward(150, 1000);

        if (digitalRead(Pin::LEFT_TRIG) == HIGH) break;
      }

      stop_forward_motors();
      turn_right(MOTOR_SPEED, 1500);

      while (Avoidance::duration > 0) {
        forward(150, 1000);
        Avoidance::duration--;
      }

      stop_forward_motors();
      turn_left(MOTOR_SPEED, 1500);
      run_buzzer();
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
  pinMode(Pin::BUZZER, OUTPUT);
  pinMode(Pin::LEFT_LED, OUTPUT);
  pinMode(Pin::RIGHT_LED, OUTPUT);
  pinMode(Pin::STOP_LED, OUTPUT);
  pinMode(Pin::ACTIVE_LED, OUTPUT);

  // setup motors controller
  motor_controller->setup(
    Pin::RPWM_1, Pin::LPWM_1, Pin::REN_1, 
    Pin::RPWM_2, Pin::LPWM_2, Pin::REN_2
  );

  // setup infrared sensors
  front_ir.setup(Pin::FRONT_IR);
  left_ir.setup(Pin::LEFT_TRIG);
  right_ir.setup(Pin::RIGHT_TRIG);

  // setup leds
  led_stop.setup(Pin::STOP_LED);
  led_active.setup(Pin::ACTIVE_LED);
  led_left.setup(Pin::LEFT_LED);
  led_right.setup(Pin::RIGHT_LED);

  //2. ATTACH INTERRUPT SERVICE ROUTINES
  attachInterrupt(digitalPinToInterrupt(Pin::STOP_BUTTON), turn_off_robot, RISING);

  //3. INITIALIZE SERIAL COMMUNICATION
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Serial monitor: OK");

  //5. INITIALIZE GPS COMMUNICATION
  gps_serial.begin(9600);
  while (!gps_serial)
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

  //OTHERS
  led_stop.turn_on();
  led_active.turn_off();
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
  //  save_navigation_data(); //save navigation history 
  //  detectObstacle();
  //  if(State::obstacle)
  //  {
  //    digitalWrite(Pin::BUZZER, HIGH);
  //  }
  //  else{
  //    digitalWrite(Pin::BUZZER, LOW);
  //  }
  //  bluetooth_command();
  //  obstacleAvoidance();
  //  get_distance_from_destination();
  //  update_gps_position();
  //  Serial.print("Heading: ");
  //  Serial.println(TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), current_destination[0], current_destination[1]));
  //  update_gps_position();
  //  save_navigation_data();
  //  Serial.println("Compass: ");
  //  Serial.println(get_compass_heading());

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
  //  Serial.println(choose_side());
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
  Serial.println("=======================================DESTINATION: " + String(active_destination));

  Serial.println("DISTANCE FROM DESTINATION: " + String(NavigationEntry::distance));
  ////  END DEBUGGING


    // 1. BLUETOOTH COMMAND
  if (Serial.available() > 0)
  {
    Command cmd = bluetooth_command();
    switch (cmd)
    {
      case START:
        State::robot = HIGH;
        State::arrived = LOW;
        Serial.println("START==========================");
        delay(3000);
        break;
      case STOP:
        stop_robot();
        break;
      case DESTINATION_ACCOUNTING:
      case DESTINATION_CAFE:
      case DESTINATION_LIBRARY:
      case DESTINATION_HOME:
        Serial.println("Destination: ACCOUNTING..............");
        current_destination[0] = allDestinations[active_destination][0];
        current_destination[1] = allDestinations[active_destination][1];
        destination_is_set = true;
        break;
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
    stop_robot();
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

    State::obstacle = front_ir.read();

    if (!State::obstacle)
    {
      //3.1.  AUTONOMOUS NAVIGATION
      switch (navigation(current_destination[0], current_destination[1]))
      {
      case LEFT:
        turn_left();
        delay(200);
        break;
      case RIGHT:
        turn_right();
        delay(200);
        break;
      case FRONT:
        forward();
        break;
        forward(100);
      }
      save_navigation_data(); //save navigation history 
      //END AUTONOMOUS NAVIGATION
    }
    else
    {
      //3.3. OBSTACLE AVOIDANCE       
      stop_forward_motors(STOP_SPEED);
      run_buzzer();
      obstacleAvoidance();
    }
  }
  // END ROBOT ACTIVE MODE
}


void cleanup() {
  delete motor_controller;
}