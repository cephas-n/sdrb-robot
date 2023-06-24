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
    {35.18819850761783, 33.3603152440357}, // HOME
    {35.18998812751088, 33.35788895463126}, // location 1
    {35.18945976464399, 33.36261056106244},  // location 2
    {35.185269339871475, 33.35986861863806}, // location 3
};


/**
 * 11.a 
 * Variable where we'll copy allDestinations and replace the home locations by the current location
 * to make the calcullation of the path dynamic based on the current position
 * of the robot
 */
double final_destinations[NUM_OF_DESTINATIONS][2];


#endif