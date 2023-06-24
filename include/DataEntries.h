/**
 * @file DataEntries.h
 * @author your name (you@domain.com)
 * @brief Data structure of entries/parameters to be saved in the SD Memmory Card
 * @version 0.1
 * @date 2023-05-04
 *
 * @copyright Copyright (c) 2023
 *
 */
 // #include "string.h"
#ifndef _DATA_ENTRIES_H_
#define _DATA_ENTRIES_H_

//NAVIGATION ENTRIES: these data are stored on the sd card
namespace NavigationEntry
{ 
  String filename;
  bool hasStarted = false;
  double timeElapsed = 0;
  double latitude = 0;
  double longitude = 0;
  int compassHeading = 0; // direction of the robot
  int destinationHeading = 0; // direction of the destination
  char steering = '-';
  int obstacle = LOW;
  double distance = 0;
  int destination = 0;
  int arrived = 0;
}
//END NAVIGATION ENTRIES


//OBSTACLE AVOIDANCE VARIABLES: these data are in the algorithm of the robot (Don't not change anything)
namespace Avoidance {
  long int startTime = 0;
  long int stopTime = 0;
  long int duration = 0;
  long int returnStartTime = 0;
  int retry = 0; // it is not used anywhere
}
//END OBSTACLE AVOIDANCE VARIABLES

#endif