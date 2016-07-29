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
 *  This .cpp file implements woodbox mobile car class.
 **********************************************************************/

#include "woodbox_mobile_car.h"

WoodboxMobileCar::WoodboxMobileCar(void)
{
    motor1_ina_   = 2;
    motor1_inb_   = 4;
    motor1_state_ = 6;
    motor1_cs_    = A0;
    motor2_ina_   = 7;
    motor2_inb_   = 8;
    motor2_state_ = 12;
    motor2_cs_    = A1;
    pin_trig_     = 22;
    pin_echo_     = 23;
}

WoodboxMobileCar::WoodboxMobileCar(unsigned char motor1_ina,
                                   unsigned char motor1_inb,
                                   unsigned char motor1_state,
                                   unsigned char motor1_cs,
                                   unsigned char motor2_ina,
                                   unsigned char motor2_inb,
                                   unsigned char motor2_state,
                                   unsigned char motor2_cs)
{
    motor1_ina_   = motor1_ina;
    motor1_inb_   = motor1_inb;
    motor1_state_ = motor1_state;
    motor1_cs_    = motor1_cs;
    motor2_ina_   = motor2_ina;
    motor2_inb_   = motor2_inb;
    motor2_state_ = motor2_state;
    motor2_cs_    = motor2_cs;
}

void WoodboxMobileCar::initializePin(void)
{
    pinMode(motor1_ina_, OUTPUT);
    pinMode(motor1_inb_, OUTPUT);
    pinMode(motor1_pwm_, OUTPUT);
    pinMode(motor1_state_, INPUT);
    pinMode(motor1_cs_, INPUT);
    pinMode(motor2_ina_, OUTPUT);
    pinMode(motor2_inb_, OUTPUT);
    pinMode(motor2_pwm_, OUTPUT);
    pinMode(motor2_state_, INPUT);
    pinMode(motor2_cs_, INPUT);
    pinMode(pin_trig_, OUTPUT);
    pinMode(pin_echo_, INPUT);

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__)
    // Timer 1 configuration.
    // PWM frequency calculation.
    // 16MHz / 1 (Prescaler) / 2 (Phase-correct) / 400 (Top) = 20kHz.
    TCCR1A = 0b10100000;
    TCCR1B = 0b00010001;
    ICR1 = 400;
#endif
}

void WoodboxMobileCar::setMotor1Speed(int speed)
{
    uint8_t reverse = 0;

    if (speed < 0) {
        speed = -speed;
        reverse = 1;
    }
    if (speed > 400) {
        speed = 400;
    }

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__)
    OCR1A = speed;
#else
    // Using analogWrite function, mapping 400 to 255.
    analogWrite(motor1_pwm_, speed * 51 / 80);
#endif

    if (speed == 0) {
        digitalWrite(motor1_ina_, LOW);
        digitalWrite(motor1_inb_, LOW);
    }
    else if (reverse) {
        digitalWrite(motor1_ina_, LOW);
        digitalWrite(motor1_inb_, HIGH);
    }
    else {
        digitalWrite(motor1_ina_, HIGH);
        digitalWrite(motor1_inb_, LOW);
    }
}

void WoodboxMobileCar::setMotor2Speed(int speed)
{
    uint8_t reverse = 0;

    if (speed < 0) {
        speed = -speed;
        reverse = 1;
    }
    if (speed > 400) {
        speed = 400;
    }

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__)
    OCR1A = speed;
#else
    // Using analogWrite function, mapping 400 to 255.
    analogWrite(motor2_pwm_, speed * 51 / 80);
#endif

    if (speed == 0) {
        digitalWrite(motor2_ina_, LOW);
        digitalWrite(motor2_inb_, LOW);
    }
    else if (reverse) {
        digitalWrite(motor2_ina_, LOW);
        digitalWrite(motor2_inb_, HIGH);
    }
    else {
        digitalWrite(motor2_ina_, HIGH);
        digitalWrite(motor2_inb_, LOW);
    }
}

void WoodboxMobileCar::setMotorSpeeds(int motor1_speed, int motor2_speed)
{
    setMotor1Speed(motor1_speed);
    setMotor2Speed(motor2_speed);
}

void WoodboxMobileCar::setMotor1Brake(int brake)
{
    if (brake < 0) {
        brake = -brake;
    }
    if (brake > 400) {
        brake = 400;
    }

    digitalWrite(motor1_ina_, LOW);
    digitalWrite(motor1_inb_, LOW);

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__)
    OCR1A = brake;
#else
    // Using analogWrite function, mapping 400 to 255.
    analogWrite(motor1_pwm_, brake * 51 / 80);
#endif
}

void WoodboxMobileCar::setMotor2Brake(int brake)
{
    if (brake < 0) {
        brake = -brake;
    }
    if (brake > 400) {
        brake = 400;
    }

    digitalWrite(motor2_ina_, LOW);
    digitalWrite(motor2_inb_, LOW);

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__)
    OCR1A = brake;
#else
    // Using analogWrite function, mapping 400 to 255.
    analogWrite(motor2_pwm_, brake * 51 / 80);
#endif
}

void WoodboxMobileCar::setMotorBrakes(int motor1_brake, int motor2_brake)
{
    setMotor1Brake(motor1_brake);
    setMotor2Brake(motor2_brake);
}

void WoodboxMobileCar::setMoveDistance(int motor1_speed, int motor2_speed,
                                       float distance)
{
    float delay_time = 0.0;
    setMotorSpeeds(motor1_speed, motor2_speed);

    if ((motor1_speed == -motor2_speed) || distance == 0.0) {
        return ;
    }
    else {
        // Calculate delay time according to the distance.
        delay_time = (120.0 * distance) / (PI * WHEEL_DIAMETER *
            (motor1_speed + motor2_speed));
        delay(delay_time * 1000);
    }
}

void WoodboxMobileCar::setTurnRadius(int motor_speed,
                                     int &motor1_speed,
                                     int &motor2_speed,
                                     int direction,
                                     float radius)
{
    float radius_scale = 0.0;
    // Calculate the ratio coefficient of two wheel speeds.
    radius_scale = (2 * radius - WHEEL_SPACING) / (2 * radius + WHEEL_SPACING);

    if (radius == 0.0) {
        motor1_speed = motor_speed;
        motor2_speed = motor_speed;
        return;
    }
    // Right wheel speed greater than left.
    if (direction == DIRECTION_LEFT) {
        motor2_speed = motor_speed;
        motor1_speed = motor_speed * radius_scale;
    }
    // Left wheel speed greater than right.
    else if (direction == DIRECTION_RIGHT) {
        motor1_speed = motor_speed;
        motor2_speed = motor_speed * radius_scale;
    }
}

void WoodboxMobileCar::setTurnAngle(int motor1_speed, int motor2_speed,
                                    float angle)
{
    float delay_time = 0.0;
    setMotorSpeeds(motor1_speed, motor2_speed);

    if ((motor1_speed == motor2_speed) || angle == 0.0) {
        return ;
    }
    else {
        // Calculate delay time according to turn angle.
        delay_time = abs((angle * WHEEL_SPACING) / (3.0 * WHEEL_DIAMETER *
            (motor2_speed - motor1_speed)));
        delay(delay_time * 1000);
    }
}

uint8_t WoodboxMobileCar::getMotor1State(void)
{
    return !digitalRead(motor1_state_);
}

uint8_t WoodboxMobileCar::getMotor2State(void)
{
    return !digitalRead(motor2_state_);
}

uint32_t WoodboxMobileCar::getMotor1CurrentMilliamps(void)
{
    // 5V / 1024 ADC counts / 144 mV per A = 34 mA per count.
    return analogRead(motor1_cs_) * 34;
}

uint32_t WoodboxMobileCar::getMotor2CurrentMilliamps(void)
{
    // 5V / 1024 ADC counts / 144 mV per A = 34 mA per count.
    return analogRead(motor2_cs_) * 34;
}

float WoodboxMobileCar::getDistance(void)
{
    float duration = 0.0;
    float distance = 0.0;

    digitalWrite(pin_trig_, LOW);
    delayMicroseconds(10);
    digitalWrite(pin_trig_, HIGH);
    delayMicroseconds(10);
    digitalWrite(pin_trig_, LOW);

    duration = pulseIn(pin_echo_, HIGH, TIMEOUT);

    if (duration == 0) {
        duration = TIMEOUT;
    }

    distance = duration * 0.017;

    return distance;
}
