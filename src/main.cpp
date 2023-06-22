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

// Custom  Headers
#include <Vector.h>
#include <Constants.h>
#include <Path.h>
#include <Pins.h>
#include <Locations.h>
#include <States.h>
#include <DataEntries.h>
#include <FunctionPrototypes.h>
#include <utilities.h>

// SDRB Library
#include <SDRB.h>

// TSP *tsp = new TSP();
//########### DESTINATION ##############
int active_destination = 0;
bool destination_is_set = false;
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


MotorController* motor_controller = new MotorController();
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
    ->forward()
    ->stop_after(duration);
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
void turn_left(const uint8_t steering_speed = MOTOR_SPEED, const uint16_t steering_duration = 0)
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
void turn_right(const uint8_t steering_speed = MOTOR_SPEED, const uint16_t steering_duration = 0)
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
void stop_all_motors()
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
void run_buzzer(const uint16_t duration = 300)
{
  digitalWrite(Pin::BUZZER, HIGH);
  delay(duration);
  digitalWrite(Pin::BUZZER, LOW);
  delay(duration);
}

/*
   Bluetooth command

   this function receive data from the bleutooth HC-06
   then return the corresponding command
*/
Command bluetooth_command()
{
  char command = Serial.read();

  Serial.println("BLEUTOOTH CMD: " + String(command));

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

  return VOID;
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
// void update_gps_position()
// {
//   while (gps_serial.available() > 0)
//   {
//     gps.encode(gps_serial.read());
//   }

//   if (gps.location.isUpdated())
//   {
//     gps_location_found = true;
//     NavigationEntry::hasStarted = true;
//   }
// }

double get_distance_from_destination() {
  if (!gps_location_found) return 0;
  
  return TinyGPSPlus::distanceBetween(
    gps.location.lat(),
    gps.location.lng(),
    current_destination[0],
    current_destination[1]
  );
}

double get_course_to_destination() {
  if(!gps_location_found)  return 0;

  return TinyGPSPlus::courseTo(
    gps.location.lat(), 
    gps.location.lng(), 
    current_destination[0], 
    current_destination[1]
  );
}

/*
   Get position

   this function return the latitude of the current position
   of the robot
*/
void update_gps_position()
{
  while (gps_serial.available() > 0)
  {
    gps.encode(gps_serial.read());
  }

  if (gps.location.isUpdated())
  {
    gps_location_found = true;
    NavigationEntry::hasStarted = true;
  }
}

double get_gps_distance(
    const double current_lat, 
    const double current_lng,
    const double destination_lat,
    const double destination_lng
) {
  if (!gps_location_found) return 0;
  
  return TinyGPSPlus::distanceBetween(
    current_lat,
    current_lng,
    destination_lat,
    destination_lng
  );
}

double get_gps_course(
    const double current_lat, 
    const double current_lng,
    const double destination_lat,
    const double destination_lng
    ) 
{
  if(!gps_location_found)  return 0;

  return TinyGPSPlus::courseTo(
    current_lat, 
    current_lng, 
    destination_lat, 
    destination_lng
  );
}

/*
   navigation

   this function controls the autonomous navigation of the robot,
   It returns the direction the robot should follow

*/
Direction navigation(const double destination_lat, const double destination_lng)
{
  update_gps_position();
  if (gps_location_found)
  {
    int destinationHeading = get_course_to_destination();
    double destinationHeadingLow = destinationHeading - DIRECTION_CORRECTION;
    double destinationHeadingHigh = destinationHeading + DIRECTION_CORRECTION;
    int currentHeading = get_compass_heading();

    //save navigation data
    NavigationEntry::latitude = gps.location.isValid() ? gps.location.lat() : -1.00;
    NavigationEntry::longitude = gps.location.isValid() ? gps.location.lng() : -1.00;
    NavigationEntry::destinationHeading = destinationHeading;
    NavigationEntry::compassHeading = currentHeading;

    // check if the user has arrived
    //HAS ALREADY ARRIVED AT DESTINATION ?
    NavigationEntry::distance = get_distance_from_destination();
    if (gps_location_found && NavigationEntry::distance <= DESTINATION_DIST_PRECISION)
    {
      Serial.println("=======THE ROBOT HAS ARRIVED AT DESTIONATION " + String(active_destination) + "======");
      State::robot = LOW;
      State::arrived = true;

      return NONE;
    }

    Serial.print("STEERING: ");
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
          destinationHeading = get_course_to_destination();

          Serial.println("Turn Right");
          NavigationEntry::steering = 'r';

          motor_controller->set_speed(NAVIGATION_STEERING_SPEED)
              ->turn_left()
              ->stop_when([](){ return front_ir.check(); });
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
            destinationHeading = get_course_to_destination();
            Serial.println("Turn Left");
            NavigationEntry::steering = 'l';

            motor_controller->set_speed(NAVIGATION_STEERING_SPEED)
                ->turn_right()
                ->stop_when([](){ return front_ir.check(); });
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
            destinationHeading = get_course_to_destination();

            Serial.println("Turn Right");
            NavigationEntry::steering = 'r';

            motor_controller->set_speed(NAVIGATION_STEERING_SPEED)
                ->turn_left()
                ->stop_when([](){ return front_ir.check(); });
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
          destinationHeading = get_course_to_destination();

          Serial.println("Turn Left");
          NavigationEntry::steering = 'l';
          
          motor_controller->set_speed(NAVIGATION_STEERING_SPEED)
            ->turn_right()
            ->stop_when([]{ return front_ir.check(); });
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

  Serial.println("Waiting for GPS signal ...");

  return NONE;
}


/*
   Save Navigation Data

   this function save the navigation entries (current position, current heading,
   time elapsed, obstacle) to the SD memory card in the following format:

   entry_id,latitude,longitude,heading,time_elapsed,obstacle
*/
bool save_navigation_data()
{
  static uint32_t last_update = millis();

  if (!NavigationEntry::hasStarted || millis() - last_update <= LOGGER_SAMPLING_TIME)
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
  }
  else if (!filenameIsValid)
  {
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
void save_data(const String &filename, const String& data)
{
  Serial.println("SD Card: Opening '" + filename + "'");

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
    Serial.println("SD Card: could'nt open the file '" + filename);
  }

  file.close();
  Serial.println("SD Card: " + filename + " closed");
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
  Direction free_side = NONE;

  if (!left_ir.check()) {
    free_side = LEFT;
  }

  if (!right_ir.check()) {
    free_side = RIGHT;
  }

  return free_side;
}


/*
   Obstacle Avoidance

   this function handle the avoidance system of the robot.
   It returns 'false' if the obstacle has be avoided or 'true' if not (in this case
   the algorithm should restart)

*/
bool obstacle_avoidance()
{
  Serial.println("Obstacle Avoidance Started");
  
  switch(choose_side()) {
    case RIGHT:
      // step 1: turn toward direction
      turn_right(MOTOR_SPEED, STEERING_DURATION);

      // setp 2: Move forward until the left side if free 
      while (true) {
        // check if there is an obstacle in front
        if(front_ir.check()) {
          stop_all_motors();
          return false;
        }

        forward(120);
        if (!left_ir.check()) break;
        Avoidance::duration++;
      }
      stop_all_motors();

      // step 3: turn left
      turn_left(MOTOR_SPEED, STEERING_DURATION);
      // check if there is an obstacle in front
      if(front_ir.check()) {
        stop_all_motors();
        return false;
      }
      forward(120, 1.5 * FORWARD_DURATION);

      // step 4: Move forward until the left side if free 
      while (true) {
        // check if there is an obstacle in front
        if(front_ir.check()) {
          stop_all_motors();
          return false;
        }

        forward(120);
        if (!left_ir.check()) break;
      }
      stop_all_motors();

      // step 5: return to  the original path accross the obstacle
      turn_left(MOTOR_SPEED, STEERING_DURATION);
      while (Avoidance::duration > 0) {
        // check if there is an obstacle in front
        if(front_ir.check()) {
          stop_all_motors();
          return false;
        }

        forward(120);
        Avoidance::duration--;
      }
      stop_all_motors();
      turn_right(MOTOR_SPEED, STEERING_DURATION);
      break;

    case LEFT:
      // step 1: turn toward direction
      turn_left(MOTOR_SPEED, STEERING_DURATION);

      // setp 2: Move forward until the right side is free 
      while (true) {
        // check if there is an obstacle in front
        if(front_ir.check()) {
          stop_all_motors();
          return false;
        }

        forward(120);
        if (!right_ir.check()) break;
        Avoidance::duration++;
      }
      stop_all_motors();

      // step 3: turn right
      turn_right(MOTOR_SPEED, STEERING_DURATION);
      // check if there is an obstacle in front
      if(front_ir.check()) {
        stop_all_motors();
        return false;
      }
      forward(120, 1.5 * FORWARD_DURATION);

      // step 4: Move forward until the right side if free 
      while (true) {
        // check if there is an obstacle in front
        if(front_ir.check()) {
          stop_all_motors();
          return false;
        }

        forward(120);
        if (!right_ir.check()) break;
      }
      stop_all_motors();

      // step 5: return to  the original path accross the obstacle
      turn_right(MOTOR_SPEED, STEERING_DURATION);
      while (Avoidance::duration > 0) {
        // check if there is an obstacle in front
        if(front_ir.check()) {
          stop_all_motors();
          return false;
        }

        forward(120);
        Avoidance::duration--;
      }
      stop_all_motors();
      turn_left(MOTOR_SPEED, STEERING_DURATION);
      break;

    default:
      stop_all_motors();
  }

  // reset avoidance entries
  Avoidance::startTime = 0;
  Avoidance::stopTime = 0;
  Avoidance::duration = 0;
  Avoidance::returnStartTime = 0;

  return true; // obstacle avoided
}


// TSP ALGORITHM
// const int NUM_OF_DESTINATIONS = 4;

Path waypoints[NUM_OF_DESTINATIONS][NUM_OF_DESTINATIONS];
const Path *waypoints_ptr[NUM_OF_DESTINATIONS][NUM_OF_DESTINATIONS];
void calculate_waypoints(const double locations[NUM_OF_DESTINATIONS][2]) {
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            const double distance = get_gps_distance(
                locations[i][0],
                locations[i][1],
                locations[j][0],
                locations[j][1]
            );
            waypoints[i][j] = Path(i, j, distance);
            Serial.println("Distance " + String(i) + " - " + String(j) + " : " + String(waypoints[i][j].getDistance()));
        }
    } 
}

Path _path_storage[NUM_OF_DESTINATIONS];
Vector<Path> paths(_path_storage);
int path_cost = 0;
void calculate_path(int starting_point) {
    // store all vertex apart from source vertex
    int _vertex_storage[NUM_OF_DESTINATIONS];
    Vector<int> vertex(_vertex_storage);
    for (int i = 0; i < NUM_OF_DESTINATIONS; i++) {
        if (i != starting_point) {
            vertex.push_back(i);
        }
    }

    // Serial.println(vertex.size());
    // store minimum weight Hamiltonian Cycle.
    int min_path = INT_MAX;
    int last_min_path = min_path;
    do {

        // store current Path weight(cost)
        int current_pathweight = 0;
        Path _current_path_storage[NUM_OF_DESTINATIONS];
        Vector<Path> current_path(_current_path_storage); 


        // compute current path weight
        int k = starting_point;
        for (size_t i = 0; i < vertex.size(); i++) {
            current_pathweight += waypoints[k][vertex[i]].getDistance();
            current_path.push_back(waypoints[k][vertex[i]]);
            k = vertex[i];
        }
        current_pathweight += waypoints[k][starting_point].getDistance();
        current_path.push_back(waypoints[k][starting_point]);

        // update minimum
        min_path = min(min_path, current_pathweight);

        if(last_min_path != min_path) {
            for(size_t i = 0; i < 4; i++) {
                paths.push_back(current_path[i]);
            }
        }

        last_min_path = min_path;
        
    } while (next_permutation<VectorIterator<int>>(vertex.begin(), vertex.end()));

    path_cost = min_path;
}



void logger() {
  // every 5 seconds
  static uint32_t last_update = millis();
  if(millis() - last_update >= LOGGER_SAMPLING_TIME) {
    // Robot
    Serial.println("ROBOT STATUS: " + String(State::robot ? "Working" : "Sleeping zzz..."));
    Serial.println("HAS ARRIVED AT DESTINATION: " + String(State::arrived ? "YES" : "NO"));

    // Infrared sensors
    Serial.println("FRONT IR: " + String(front_ir.check() ? "detected" : "-"));
    Serial.println("LEFT IR: " + String(left_ir.check() ? "detected" : "-"));
    Serial.println("RIGHT IR: " + String(right_ir.check() ? "detected" : "-"));

    // Navigation
    Serial.println("DISTINATION: id = " + String(active_destination + 1)
                  + ", lat=" + String(current_destination[0], 6) 
                  + ",  lng=" + String(current_destination[1], 6)
                  + ",  heading=" + String(get_course_to_destination(), 2));
    Serial.println("CURRENT POSITION: lat=" + String(gps.location.lat(), 6) 
                + ",  lng=" + String(gps.location.lng(), 6)
                + ",  distance=" + String(get_distance_from_destination(), 2) + " meters");
    Serial.println("CURRENT HEADING:" + String(get_compass_heading(), 6));

    Serial.println("MISSION FILE NAME: " + String(NavigationEntry::filename));
  }
}


//####################### ARDUINO SETUP FUNCTIONS #######################
void setup()
{
  //0. SETUP PIN MODES
  pinMode(Pin::START_BUTTON, INPUT);
  pinMode(Pin::STOP_BUTTON, INPUT);
  pinMode(Pin::BUZZER, OUTPUT);

  // 1. SETUP MOTORS CONTROLLER
  motor_controller->setup(
    Pin::RPWM_1, Pin::LPWM_1, Pin::REN_1,
    Pin::RPWM_2, Pin::LPWM_2, Pin::REN_2
  );

  // 2. SETUP INFRARED SENSORS
  front_ir.setup(Pin::FRONT_IR);
  left_ir.setup(Pin::LEFT_TRIG);
  right_ir.setup(Pin::RIGHT_TRIG);

  // 3. SETUP LEDS
  led_stop.setup(Pin::STOP_LED);
  led_active.setup(Pin::ACTIVE_LED);
  led_left.setup(Pin::LEFT_LED);
  led_right.setup(Pin::RIGHT_LED);

  //4. ATTACH INTERRUPT SERVICE ROUTINES
  attachInterrupt(digitalPinToInterrupt(Pin::STOP_BUTTON), turn_off_robot, RISING);

  //5. INITIALIZE SERIAL COMMUNICATION
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Serial monitor: OK");

  //6. INITIALIZE GPS COMMUNICATION
  gps_serial.begin(9600);
  while (!gps_serial)
  {
    Serial.println("GPS com: FAILED");
  }
  Serial.println("GPS com: OK");

  //7. COMPASS CONFIGURATION
  compass.init();

  //8. SD CARD MEMORY
  if (!SD.begin(Pin::SD_CARD))
  {
    Serial.println("SD card: FAILED");
  }
  else
  {
    Serial.println("SD card: READY");
  }

  // 9. WAIT FOR VALID LOCATION
  while(!gps_location_found) {
    Serial.println("Waiting for a valid location ...");
    led_stop.blink(1, 100);

    update_gps_position();
  }

  // 10. wait 10secs then update location
  delay(2000);
  update_gps_position();
  
  // 11. CALCULATE PATH
  calculate_waypoints(allDestinations);
  calculate_path(0);
  Serial.print("Path: ");
  for(auto &path: paths) {
    Serial.print(String(path.end()) + ", ");
  }

  //OTHERS
  led_stop.turn_on();
  led_active.turn_off();

  // Serial.println(path[1].getDistance());
  // Serial.print("Path COST = ");
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
  // Obstacle Avoidance test
  // State::obstacle = front_ir.check();
  // if(State::obstacle) {
  //   obstacle_avoidance();
  // }

  // Main
  for(auto &path: paths) {

    if(path.completed) continue;
    
    bool arrived = false;
    while (!arrived)
    {
      arrived = run_robot(path);
      Serial.println("Moving from: " + String(path.start()) + " to: " + String(path.end()) + ", distance: " + String(path.getDistance()));
    }
    
    path.completed = true;

    // indicator
    run_buzzer(500);
    run_buzzer(500);
    run_buzzer(500);
  }

  Serial.println("Mission Finished!!!!!!!!!!!!!!!!");
}

int run_robot(Path &path) {
  // 0. LOGGER
  logger();

  // 0.1 CHECK IF ARRIVED AT DESTINATION
  // if(State::arrived) {
  //   return 1;
  // }

  // 1. BLUETOOTH COMMAND
  if (Serial.available() > 0)
  {
    Command cmd = bluetooth_command();
    Serial.println("BLUETOOTH COMMAND:" + String(cmd));
    switch (cmd)
    {
      case START:
        State::robot = HIGH;
        State::arrived = LOW;
        delay(3000);
        break;
      case STOP:
        stop_robot();
        break;
      case DESTINATION_ACCOUNTING:
      case DESTINATION_CAFE:
      case DESTINATION_LIBRARY:
      case DESTINATION_HOME:
        current_destination[0] = allDestinations[active_destination][0];
        current_destination[1] = allDestinations[active_destination][1];
        destination_is_set = true;
        break;
      default:
        Serial.println("Unknown command!");
    }
  }
  // END BLUETOOTH COMMAND


  //2.  MANUAL COMMAND
  if (digitalRead(Pin::START_BUTTON) == HIGH) //start robot
  {
    State::robot = HIGH;
  }
  //END MANUAL COMMAND

  // 3. ROBOT ACTIVE MODE
  if(State::arrived) 
  {
    // robot has arrived at destination
    led_stop.blink(1, 1000);
  }
  else if (State::robot == HIGH)
  {
    // start robot
    led_active.turn_on();
    led_stop.turn_off();

    State::obstacle = front_ir.check();

    if (!State::obstacle)
    {
      //3.1.  AUTONOMOUS NAVIGATION
      const Direction direction = navigation(
        allDestinations[path.end()][0], 
        allDestinations[path.end()][1]
      );
      switch (direction)
      {
        case LEFT:
          turn_left(NAVIGATION_STEERING_SPEED);
          delay(200);
          break;
        case RIGHT:
          turn_right(NAVIGATION_STEERING_SPEED);
          delay(200);
          break;
        case FRONT:
          forward();
          break;
          forward(100);
        default:
          Serial.println("Searching direction ...");
      }
      save_navigation_data(); //save navigation history 
      //END AUTONOMOUS NAVIGATION
    }
    else
    {
      //3.3. OBSTACLE AVOIDANCE       
      stop_all_motors();
      run_buzzer();
      obstacle_avoidance();
    }
    
    // END ROBOT ACTIVE MODE
  } 
  else
  {
    // stop robot
    stop_robot();
    led_active.turn_off();
    led_stop.turn_on();
  }

  // 4 CHECK IF ARRIVED AT DESTINATION
  return State::arrived;
}


void cleanup() {
  delete motor_controller;
}