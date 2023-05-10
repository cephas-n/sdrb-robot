## Class documentation for SDRB.h
The header file SDRB.h contains several classes and functions for controlling a robot. In this file, we have the following classes:

Motor
MotorDriver
MotorController
InfraRed
Let's take a closer look at each of them:

Motor
The Motor class is an abstract class that represents a generic motor. It has three private attributes:

positive_pin - the pin that controls the positive direction of the motor.
negative_pin - the pin that controls the negative direction of the motor.
en_pin - the pin that controls the speed of the motor.
It also has one protected attribute:

state - indicates the state of the motor. 1 represents that it is running and 0 represents that it is stopped.
The class has five methods:

setup - an abstract method that sets up the motor.
forward - a method that makes the motor run in the forward direction.
reverse - a method that makes the motor run in the reverse direction.
stop - a method that stops the motor.
get_state - a method that returns the current state of the motor.
MotorDriver
The MotorDriver class inherits from the Motor class and implements the setup method. It has no additional attributes, but it has one additional method:

set_speed - a method that sets the speed of the motor.
MotorController
The MotorController class is responsible for controlling two motors simultaneously. It has two MotorDriver objects as private attributes:

driver_left - a pointer to a MotorDriver object that controls the left motor.
driver_right - a pointer to a MotorDriver object that controls the right motor.
The class has several methods:

setup - a method that sets up both motors.
forward - a method that makes both motors run forward for a specified duration.
reverse - a method that makes both motors run in reverse for a specified duration.
turn_left - a method that makes the robot turn left.
turn_right - a method that makes the robot turn right.
set_speed - a method that sets the speed of both motors.
stop_after - a method that stops the robot after a specified time.
stop_when - a method that stops the robot when a specified condition is met.
stop - a method that stops both motors.
InfraRed
The InfraRed class is responsible for reading the value from an infrared sensor. It has two private attributes:

pin - the pin connected to the infrared sensor.
last_state - the last state of the sensor.
The class has three methods:

setup - a method that sets up the infrared sensor.
check - a method that reads the current state of the sensor and returns true if it is on and false if it is off.
get_last_state - a method that returns the last state of the sensor.




