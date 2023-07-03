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
    {35.189442100721685, 33.35875282319103}, // HOME
    {35.18908767514828, 33.35923927020034}, // location 1
    {35.188874669962544, 33.3590850139744}, // location 2
    {35.188562850784194, 33.359859072164454},  // location 3
};


/**
 * 11.a 
 * Variable where we'll copy allDestinations and replace the home locations by the current location
 * to make the calcullation of the path dynamic based on the current position
 * of the robot
 */
double final_destinations[NUM_OF_DESTINATIONS][2];


#endif