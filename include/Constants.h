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

 // Estimated amount of time (in milliseconds) required to steer the wheels by 90deg or -90deg
const unsigned STEERING_DURATION = 1500;

// The steering speed  of the robot (0 -> 255) during obstacle avoidance
const unsigned OBSTACLE_STEERING_SPEED = 255;

// The steering speed  of the robot (0 -> 255) during the normal operation mode
const unsigned NAVIGATION_STEERING_SPEED = 255;

// Time interval (in milliseconds) in which the robot will move forward, then stops
const int FORWARD_DURATION = 500; // Do not change it

// compass heading correction/tolerance (degrees)
const int DIRECTION_CORRECTION = 15; 

// GPS latitude correction/tolerance (degrees)
const int GPS_LATITUDE_CORRECTION = 0; // Not used

// Destination distance precision from current position of the robot (degrees)
const double DESTINATION_DIST_PRECISION = 5; // meters

// The speed at which the robot will stop
const int STOP_SPEED = 255; 

// Nominal speed of the robot (0 - 255)
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
  DESTINATION_ACCOUNTING, // same as Location 1
  DESTINATION_CAFE, // same as Location 2
  DESTINATION_LIBRARY, // same as Location 3
  DESTINATION_HOME // same as Location 0
}; // Don't change it unless you know what your doing

// Directions list
enum Direction
{
  FRONT,
  BACK,
  LEFT,
  RIGHT,
  NONE
}; // Don't Change it

// number of motors per motor driver
const int MOTORS_PER_MOTOR_DRIVER = 2; // Don't change it

// logger sampling time in milliseconds (interval at which the navigation data are saved on the SD card)
const uint16_t LOGGER_SAMPLING_TIME = 5000; // You can change if you more data

// number of destinations
const int NUM_OF_DESTINATIONS = 4;

#endif