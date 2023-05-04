/**
 * @file States.h
 * @author your name (you@domain.com)
 * @brief All global states of the robot
 * @version 0.1
 * @date 2023-05-04
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef _GLOBAL_STATES_H_
#define _GLOBAL_STATES_H_

namespace State
{
    bool obstacle = false;
    int arrived = LOW;
    int robot = LOW;
    int motor = LOW;
};

#endif