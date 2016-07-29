/***********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, myyerrol
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 *  * Neither the name of the myyerrol nor the names of its contributors
 *    may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ***********************************************************************

 ***********************************************************************
 *  History:
 *  <Authors>        <Date>        <Operation>
 *  myyerrol         2016.7.29     Create this file
 *
 *  Description:
 *  This .h file defines woodbox mobile car class.
 **********************************************************************/

#ifndef WOODBOX_MOBILE_CAR_H
#define WOODBOX_MOBILE_CAR_H

#include <Arduino.h>

#define SIZE            5     // Simple serial communication buffer size.
#define DIRECTION_LEFT  2     // Turn left.
#define DIRECTION_RIGHT 3     // Turn right.
#define TIMEOUT         30000 // The Ultrasonic's timeout.
#define WHEEL_DIAMETER  6.85  // The diameter of wheel.
#define WHEEL_SPACING   27.4  // The distance between the two wheels.

// This class references pololu dual vnh5019 motor driver class, and implements
// follow specific functions to achieve: Set move distance, Set Turn Radius, Set
// Turn angle, Get Distance from ultrasonic.
class WoodboxMobileCar
{
public:
    WoodboxMobileCar(void);
    WoodboxMobileCar(unsigned char motor1_ina,
                     unsigned char motor1_inb,
                     unsigned char motor1_state,
                     unsigned char motor1_cs,
                     unsigned char motor2_ina,
                     unsigned char motor2_inb,
                     unsigned char motor2_state,
                     unsigned char motor2_cs);
    void initializePin(void);
    void setMotor1Speed(int speed);
    void setMotor2Speed(int speed);
    void setMotorSpeeds(int motor1_speed, int motor2_speed);
    void setMotor1Brake(int brake);
    void setMotor2Brake(int brake);
    void setMotorBrakes(int motor1_brake, int motor2_brake);
    void setMoveDistance(int motor1_speed, int motor2_speed, float distance);
    void setTurnRadius(int motor_speed,
                       int &motor1_speed,
                       int &motor2_speed,
                       int direction,
                       float radius);
    void setTurnAngle(int motor1_speed, int motor2_speed, float angle);
    uint8_t getMotor1State(void);
    uint8_t getMotor2State(void);
    uint32_t getMotor1CurrentMilliamps(void);
    uint32_t getMotor2CurrentMilliamps(void);
    float getDistance(void);
private:
    uint8_t              motor1_ina_;
    uint8_t              motor2_ina_;
    uint8_t              motor1_inb_;
    uint8_t              motor2_inb_;
    uint8_t              motor1_state_;
    uint8_t              motor2_state_;
    uint8_t              motor1_cs_;
    uint8_t              motor2_cs_;
    uint8_t              pin_trig_;
    uint8_t              pin_echo_;
    static const uint8_t motor1_pwm_ = 9;
    static const uint8_t motor2_pwm_ = 10;
};

#endif // WOODBOX_MOBILE_CAR_H
