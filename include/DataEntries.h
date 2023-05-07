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

//NAVIGATION ENTRIES
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
//END NAVIGATION ENTRIES


//OBSTACLE AVOIDANCE VARIABLES
namespace Avoidance {
  long int startTime = 0;
  long int stopTime = 0;
  long int duration = 0;
  long int returnStartTime = 0;
  int retry = 0;
}
//END OBSTACLE AVOIDANCE VARIABLES

#endif