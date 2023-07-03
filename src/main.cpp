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

// Custom  Headers
#include <Vector.h>
#include <Constants.h>
#include <Path.h>
#include <Pins.h>
#include <States.h>
#include <DataEntries.h>
#include <FunctionPrototypes.h>
#include <utilities.h>

// SDRB Library
#include <SDRB.h>

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
void forward(const int motor_speed = MOTOR_SPEED, const uint16_t duration = 0)
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
void backward(const int motor_speed = MOTOR_SPEED, const uint16_t duration = 0)
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
void turn_left(const int steering_speed = NAVIGATION_STEERING_SPEED, const uint16_t steering_duration = 0)
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
void turn_right(const int steering_speed = NAVIGATION_STEERING_SPEED, const uint16_t steering_duration = 0)
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
  motor_controller->set_speed(100)->stop();
  motor_controller->reverse()->stop();
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
  Serial.flush();

  Serial.println("BLEUTOOTH CMD: " + String(command));

  switch (command)
  {
  case '1':
    //start robot
    return START;
  case '0':
    //stop robot
    return STOP;
  case '2':
    return TURN_LEFT;
  case '3':
    return TURN_RIGHT;
  case '4':
    return SPEED_UP;
  case '5':
    return SPEED_DOWN;
  case '6':
    return RUN_BUZZER;
  case '7':
    return FULL_TURN;
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
      turn_right(OBSTACLE_STEERING_SPEED, STEERING_DURATION);

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
      turn_left(OBSTACLE_STEERING_SPEED, STEERING_DURATION);
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
      turn_left(OBSTACLE_STEERING_SPEED, STEERING_DURATION);
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
      turn_right(OBSTACLE_STEERING_SPEED, STEERING_DURATION);
  
    case LEFT:
      // step 1: turn toward direction
      turn_left(OBSTACLE_STEERING_SPEED, STEERING_DURATION);

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
      turn_right(OBSTACLE_STEERING_SPEED, STEERING_DURATION);
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
      turn_right(OBSTACLE_STEERING_SPEED, STEERING_DURATION);
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
      turn_left(OBSTACLE_STEERING_SPEED, STEERING_DURATION);
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
  // while (true)
  // { 
  //   // turn_left();
  // }
  
  // 1. BLUETOOTH COMMAND
  Command cmd = VOID;
  if (Serial.available() > 0)
  {
    cmd = bluetooth_command();
    if(cmd == START) {
      State::robot = HIGH;
      State::arrived = LOW;
      run_buzzer(100);
      delay(3000);
    } 
    else if(cmd == STOP) {
      stop_robot();
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
  Serial.println("Robot status" + String(State::robot));
  if (State::robot == HIGH)
  {
    // start robot
    led_active.turn_on();
    led_stop.turn_off();

    State::obstacle = front_ir.check();

    if (!State::obstacle)
    {
      //3.1.  MANUAL NAVIGATION
      switch (cmd)
      {
        case TURN_LEFT:
          turn_left(NAVIGATION_STEERING_SPEED, NAVIGATION_STEERING_DURATION);
          break;
        case TURN_RIGHT:
          turn_right(NAVIGATION_STEERING_SPEED, NAVIGATION_STEERING_DURATION);
          break;
        case SPEED_UP:
          MOTOR_SPEED += (MOTOR_SPEED < 255 ? SPEED_PARTITION : 255);
          break;
        case SPEED_DOWN:
          MOTOR_SPEED -= (MOTOR_SPEED < 255 ? SPEED_PARTITION : 255);
          break;
        case RUN_BUZZER:
          run_buzzer(200);
          break;
        case FULL_TURN:
          stop_all_motors();
          delay(2000);
          turn_right(NAVIGATION_STEERING_SPEED, 5000);
          break;
        default:
          Serial.println("Searching direction ...");
      }

      forward();
      // cmd = VOID;
      //END MANUAL NAVIGATION
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

  Serial.println("CMD ===" + String(cmd));
}


void cleanup() {
  delete motor_controller;
}