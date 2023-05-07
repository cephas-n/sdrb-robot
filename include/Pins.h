#ifndef PINS_H_
#define PINS_H_

//########### PINS DEFINITION ##############
namespace Pin
{
    const uint8_t START_BUTTON = 22;
    const uint8_t STOP_BUTTON = 2;
    const uint8_t FRONT_IR = 38;
    const uint8_t RIGHT_TRIG = 49;
    const uint8_t RIGHT_ECHO = A2;
    const uint8_t LEFT_TRIG = 26;
    const uint8_t LEFT_ECHO = A1;
    const uint8_t RPWM_1 = 6;       //right
    const uint8_t LPWM_1 = 7;
    const uint8_t REN_1 = 9;
    const uint8_t RPWM_2 = 4;       //left
    const uint8_t LPWM_2 = 5;
    const uint8_t REN_2 = 8;
    const uint8_t BUZZER = 36;
    const uint8_t GPS_TX = 11;
    const uint8_t GPS_RX = 10;
    const uint8_t SD_CARD = 53; // CS pin (clock)
    const uint8_t LEFT_LED = 30;
    const uint8_t RIGHT_LED = 32;
    const uint8_t STOP_LED = 34;
    const uint8_t ACTIVE_LED = 48;
};
//########### END PINS DEFINITION ##############
#endif