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
    analogWrite(motor2_pwm_, brake * 51 / 80);
#endif
}

void WoodboxMobileCar::setMotorBrakes(int motor1_brake, int motor2_brake)
{
    setMotor1Brake(motor1_brake);
    setMotor2Brake(motor2_brake);
}

void WoodboxMobileCar::setTurnAngle(int motor1_speed, int motor2_speed,
                                    float angle)
{
    float delay_time = 0.0;

    if (motor1_speed == motor2_speed) {
        setMotorSpeeds(motor1_speed, motor2_speed);
    }
    else {
        delay_time = abs((angle * WHEEL_SPACING) / (3.0 * WHEEL_DIAMETER *
        (motor2_speed - motor1_speed)));
        delay(delay_time);
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
    return analogRead(motor1_cs_) * 34;
}

uint32_t WoodboxMobileCar::getMotor2CurrentMilliamps(void)
{
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
