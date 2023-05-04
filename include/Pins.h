#ifndef PINS_H_
#define PINS_H_

//########### PINS DEFINITION ##############
namespace Pin
{
    const int START_BUTTON = 22;
    const int STOP_BUTTON = 2;
    const int FRONT_IR = 38;
    const int RIGHT_TRIG = 49;
    const int RIGHT_ECHO = A2;
    const int LEFT_TRIG = 26;
    const int LEFT_ECHO = A1;
    const int RPWM_1 = 4;       //left
    const int LPWM_1 = 5;
    const int REN_1 = 8;
    const int RPWM_2 = 6;       //right
    const int LPWM_2 = 7;
    const int REN_2 = 9;
    const int BUZZER = 36;
    const int GPS_TX = 11;
    const int GPS_RX = 10;
    const int SD_CARD = 53; // CS pin (clock)
    const int LEFT_LED = 30;
    const int RIGHT_LED = 32;
    const int STOP_LED = 34;
    const int ACTIVE_LED = 40;
};
//########### END PINS DEFINITION ##############
#endif