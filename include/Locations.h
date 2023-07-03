/**
 * @file Locations.h
 * @author your name (you@domain.com)
 * @brief List of gps (latitude, longitude) coordinates of all locations
 * @version 0.1
 * @date 2023-05-04
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef _LOCATIONS_H_
#define _LOCATIONS_H_

const double allDestinations[4][2] {
    {35.18904238269015, 33.35917076737193}, // HOME
    {35.18863484269035, 33.359716434490174}, // location 1
    {335.188478894366696, 33.35963917480599},  // location 2
    {35.188211900005044, 33.35959488026231}, // location 3
};


/**
 * 11.a 
 * Variable where we'll copy allDestinations and replace the home locations by the current location
 * to make the calcullation of the path dynamic based on the current position
 * of the robot
 */
double final_destinations[NUM_OF_DESTINATIONS][2];


#endif