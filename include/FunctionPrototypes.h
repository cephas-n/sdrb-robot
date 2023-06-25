/**
 * @file FunctionPrototypes.h
 * @author your name (you@domain.com)
 * @brief Function prototypes or list of all functions
 * @version 0.1
 * @date 2023-05-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef _FUNCTION_PROTOTYPES_H_
#define _FUNCTION_PROTOTYPES_H_
    double get_compass_heading();
    double get_distance_from_destination();
    void save_data(const String &filename, const String &entry);
    void turn_off_robot();
    void turn_left(const int steering_speed, const uint16_t steering_duration);
    void turn_right(const int steering_speed, const uint16_t steering_duration);
    void forward(const int motor_speed, const uint16_t duration);
    void backward(const int motor_speed, const uint16_t duration);
    void backward(const int motor_speed, const uint16_t duration);
    void stop_all_motors();
    void run_buzzer(const uint16_t duration);
    Command bluetooth_command();
    void update_gps_position();
    Direction navigation(const double destination_lat, const double destination_lng);
    bool save_navigation_data();
    double get_compass_heading();
    Direction choose_side();
    bool obstacle_avoidance();
    void logger();
    int run_robot(Path &);
#endif