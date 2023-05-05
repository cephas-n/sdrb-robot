/**
 * @file Constants.h
 * @author your name (you@domain.com)
 * @brief ROBOT GLOBAL Constacts
 * @version 0.1
 * @date 2023-05-04
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

// Estimated amount of time (in milliseconds) require to steer the wheels by 90deg or -90deg
const unsigned STEERING_DURATION = 1000;

// Estimated amount of time (in milliseconds) require to steer the wheels by 90deg or -90deg
const unsigned STEERING_SPEED = 200;

// Time interval (in milliseconds) in which the robot will move forward, then stops
const int FORWARD_DURATION = 500;

// compass heading correction/tolerance (degrees)
const int DIRECTION_CORRECTION = 10;

// GPS latitude correction/tolerance (degrees)
const int GPS_LATITUDE_CORRECTION = 0;

// Destination distance precision from current position of the robot (degrees)
const double DESTINATION_DIST_PRECISION = 20; // meters

// The speed at which the robot will stop
const int STOP_SPEED = 255;

// Nominal speed of the robot
unsigned int MOTOR_SPEED = 255;

// Bluetooth command list
enum Command
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

// Directions list
enum Direction
{
  FRONT,
  BACK,
  LEFT,
  RIGHT,
  NONE
};

// number of motors per motor driver
const int MOTORS_PER_MOTOR_DRIVER = 2;
#endif