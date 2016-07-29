#ifndef WOODBOX_MOBILE_CAR_H
#define WOODBOX_MOBILE_CAR_H

#include <Arduino.h>

#define SIZE           5
#define DIR_LEFT       2
#define DIR_RIGHT      3
#define TIMEOUT        30000
#define WHEEL_DIAMETER 6.85
#define WHEEL_SPACING  27.4

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
