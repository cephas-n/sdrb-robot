#ifndef _SDRB_H_
#define _SDRB_H_
#include <Arduino.h>
#include <Constants.h>

//Motor Class
class Motor {
  protected:
    uint8_t positive_pin;
    uint8_t negative_pin;
    uint8_t en_pin;
    int state;
  public:
    Motor() = default;
    virtual ~Motor() = default;

    virtual void setup(
      const uint8_t positive_pin, 
      const uint8_t negative_pin, 
      const uint8_t en_pin) = 0;

    Motor *forward() {
      this->state = HIGH;
      digitalWrite(this->positive_pin, HIGH);
      digitalWrite(this->negative_pin, LOW);

      return this;
    }

    Motor *reverse() {
      this->state = HIGH;
      digitalWrite(this->positive_pin, LOW);
      digitalWrite(this->negative_pin, HIGH);

      return this;
    }

    void stop() {
      this->state = LOW;
      digitalWrite(this->positive_pin, LOW);
      digitalWrite(this->negative_pin, LOW);
    }

    int get_state() {
      return this->state;
    }

};

class MotorDriver: public Motor {
  public:
    MotorDriver() = default;
    ~MotorDriver() = default;
    void setup(
      const uint8_t positive_pin, 
      const uint8_t negative_pin, 
      const uint8_t en_pin)
    {
      // setup motor
      this->positive_pin = positive_pin;
      this->negative_pin = negative_pin;

      pinMode(this->positive_pin, OUTPUT);
      pinMode(this->negative_pin, OUTPUT);

      // setup enable/pwm pin
      this->en_pin = en_pin;
      pinMode(en_pin, OUTPUT);

      // set nominal speed
      digitalWrite(en_pin, MOTOR_SPEED);
    }

    MotorDriver *set_speed(const uint8_t speed) {
      analogWrite(this->en_pin, speed);

      return this;
    }
};


class MotorController {
  protected:
    MotorDriver *driver_left = new MotorDriver();
    MotorDriver *driver_right = new MotorDriver();
  public:
    MotorController() = default;

    void setup(
      const uint8_t positive_pin_left, 
      const uint8_t negative_pin_left, 
      const uint8_t en_pin_left,
      const uint8_t positive_pin_right, 
      const uint8_t negative_pin_right, 
      const uint8_t en_pin_right
    ) {
      driver_left->setup(positive_pin_left, negative_pin_left, en_pin_left);
      driver_right->setup(positive_pin_right, negative_pin_right, en_pin_right);
    }

    MotorController *forward(
      const uint16_t duration = FORWARD_DURATION, 
      void (*in_progress)() = nullptr, 
      void (*completed)() = nullptr)
    {
      if(driver_left->get_state() == HIGH) driver_left->stop();
      if(driver_right->get_state() == HIGH) driver_right->stop();

      const long int start_time = millis();
      while(millis() - start_time <= duration) {
        if(*in_progress != nullptr) in_progress();

        driver_left->forward();
        driver_right->forward();
      }

      if(*completed != nullptr) completed();

      return this;
    }

    MotorController *reverse(
      const uint16_t duration = FORWARD_DURATION, 
      void (*in_progress)() = nullptr, 
      void (*completed)() = nullptr) 
    {
      if(driver_left->get_state() == HIGH) driver_left->stop();
      if(driver_right->get_state() == HIGH) driver_right->stop();

      const long int start_time = millis();
      while(millis() - start_time <= duration) {
        if(*in_progress != nullptr) in_progress();
        driver_left->reverse();
        driver_right->reverse();
      }

      if(*completed != nullptr) completed();

      return this;
    }

    MotorController *turn_left() {
      driver_left->stop();
      driver_right->forward();

      return this;
    }

    MotorController *turn_right() {
      driver_left->forward();
      driver_right->stop();

      return this;
    }

    MotorController *set_speed(uint8_t speed) {
      driver_left->set_speed(speed);
      driver_right->set_speed(speed);

      return this;
    }

    MotorController *stop_after(uint32_t time) {
      delay(time);
      this->stop();

      return this;
    }

    MotorController *stop_when(bool (*callback)()) {
      if(callback()) {
        digitalWrite(34, HIGH);
        this->stop(); 
      }

      return this;
    }


    void stop() {
      for(int speed = MOTOR_SPEED; speed > 100; speed--) {
        driver_left->set_speed(speed);
        driver_right->set_speed(speed);
        delay(2);
      }

      driver_left->stop();
      driver_right->stop();
    }

    // destructor
    ~MotorController() {
      delete driver_left;
      delete driver_right;
    };
};


class InfraRed {
  protected:
    uint8_t pin;
    bool last_state;
  public:
    InfraRed() = default;
    ~InfraRed() = default;
    
    void setup(uint8_t pin) {
      this->pin = pin;
      this->last_state =LOW;
      pinMode(this->pin, INPUT);
    }

    bool check() {
      const bool state = digitalRead(this->pin) == LOW ? true : false;
      this->last_state = state;
      return state;
    }

    bool get_last_state() {
      return this->last_state;
    }
};

class Led {
  protected:
    uint8_t pin;
    bool state;
  public:
    Led() = default;
    ~Led() = default;

    void setup(uint8_t pin) {
      this->pin = pin;
      this->state = LOW;

      pinMode(this->pin, OUTPUT);
    }

    void turn_on() {
      digitalWrite(this->pin, HIGH);
    }

    void turn_off() {
      digitalWrite(this->pin, LOW);
    }

    void blink(uint8_t repeat = 0, uint16_t interval = 500) {
      for(int i = repeat; i > 0; i--) {
        this->turn_on();
        delay(interval);
        this->turn_off();

        if(i > 1) delay(interval);
      }
    }
};
#endif