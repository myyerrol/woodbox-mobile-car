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
 *  This .ino file implements woodbox mobile car obstacle avoidance.
 **********************************************************************/

#include <woodbox_mobile_car.h>

#define DEBUG       1
#define DELAY_FIXED 1
#define DELAY       2000

int              g_flag_serial      =  false;
int              g_flag_minus       =  false;
int              g_byte             =  0;
int              g_motor_speed      =  100;
int              g_motor1_speed     = -100;
int              g_motor2_speed     =  100;
int              g_motor1_brake     =  400;
int              g_motor2_brake     =  400;
int              g_data_array[SIZE] =  {0};
float            g_distance         =  0.0;
float            g_angle            =  90;
char            *g_string           =  NULL;
String           g_serial_data      =  "";
WoodboxMobileCar g_woodbox_mobile_car;

void setup()
{
    g_woodbox_mobile_car.initializePin();
    Serial.begin(9600);
    Serial.println("Start Woodbox Mobile Car Obstacle Avoidance!");
    Serial.println("--------------------------------------------");
    Serial.println("Input follow commands to configure parameter");
    Serial.println("s [motor1_speed] [motor2_speed]: Set Speed  ");
    Serial.println("b [motor1_brake] [motor2_brake]: Set Brake  ");
    Serial.println("a [angle]:       Set Turn Angle             ");
    Serial.println("d [motor_speed]: Set Forward/Back Speed     ");
    Serial.println("--------------------------------------------");
    delay(5000);
}

void loop()
{
    // Get distance from ultrasonic.
    g_distance = g_woodbox_mobile_car.getDistance();

    // Make car turn certain angle.
    if (10 < g_distance <= 20) {
#if DELAY_FIXED
    // Turn fixed angle.
    g_woodbox_mobile_car.setMotorSpeeds(-g_motor_speed, +g_motor_speed);
    delay(DELAY);
#else
    // Turn angle that you can specify.
    g_woodbox_mobile_car.setTurnAngle(g_motor1_speed, g_motor2_speed, g_angle);
#endif
    }
    // Move back first, and then turn certain angle.
    else if (g_distance <= 10) {
        // Move back certain distance.
        g_woodbox_mobile_car.setMotorSpeeds(-g_motor_speed, -g_motor_speed);
        delay(DELAY / 2);
#if DELAY_FIXED
    // Turn fixed angle.
    g_woodbox_mobile_car.setMotorSpeeds(-g_motor_speed, +g_motor_speed);
    delay(DELAY);
#else
    // Turn angle that you can specify.
    g_woodbox_mobile_car.setTurnAngle(g_motor1_speed, g_motor2_speed, g_angle);
#endif
    }

    g_woodbox_mobile_car.setMotorSpeeds(g_motor_speed, g_motor_speed);

    // Receive multiple bytes data from serial port.
    while (Serial.available() > 0) {
        g_serial_data += char(Serial.read());
        delay(10);
        g_flag_serial = true;
    }

#if DEBUG
    Serial.print("Serial Data: ");
    Serial.println(g_serial_data);
    Serial.print("Serial Data Length: ");
    Serial.println(g_serial_data.length());
#endif

    // Convert data type from character to integer.
    if (g_flag_serial) {
        for (int i = 1, j = 0; i < g_serial_data.length(); i++) {
            if (g_serial_data[i] == ' ') {
                // Convert positive number to negative number.
                if (g_flag_minus) {
                    g_data_array[j] = -g_data_array[j];
                }
                g_flag_minus = false;
                if (i != 1) {
                    j++;
                }
            }
            else if (g_serial_data[i] == '-') {
                g_flag_minus = true;
            }
            // Make up a number from bytes.
            else {
                g_data_array[j] = g_data_array[j] * 10 +
                    (g_serial_data[i] - '0');
            }
        }

#if DEBUG
    Serial.print("parameter1: ");
    Serial.println(g_data_array[0]);
    Serial.print("parameter2: ");
    Serial.println(g_data_array[1]);
#endif

        // Modify speed of motor1 and motor2.
        if (g_serial_data[0] == 's') {
            if (g_data_array[0] >= 400) {
                g_data_array[0] = 400;
            }
            else if (g_data_array[0] <= -400) {
                g_data_array[0] = -400;
            }
            if (g_data_array[1] >= 400) {
                g_data_array[1] = 400;
            }
            else if (g_data_array[1] <= -400) {
                g_data_array[1] = -400;
            }
            if (g_data_array[0] == g_data_array[1]) {
                g_data_array[0] = -g_data_array[1];
            }
            g_motor1_speed = g_data_array[0];
            g_motor2_speed = g_data_array[1];
            Serial.print("Motor1 Speed: ");
            Serial.println(g_motor1_speed);
            Serial.print("Motor2 Speed: ");
            Serial.println(g_motor2_speed);
        }
        // Modify brake of motor1 and motor2.
        else if (g_serial_data[0] == 'b') {
            if (g_data_array[0] >= 400) {
                g_data_array[0] = 400;
            }
            else if (g_data_array[0] <= 0) {
                g_data_array[0] = 0;
            }
            if (g_data_array[1] >= 400) {
                g_data_array[1] = 400;
            }
            else if (g_data_array[1] <= 0) {
                g_data_array[1] = 0;
            }
            g_motor1_brake = g_data_array[0];
            g_motor2_brake = g_data_array[1];
            Serial.print("Motor1 Brake: ");
            Serial.println(g_motor1_brake);
            Serial.print("Motor2 Brake: ");
            Serial.println(g_motor2_brake);
        }
        // Modify angle of turn.
        else if (g_serial_data[0] == 'a') {
            if (g_data_array[0] >= 360) {
                g_data_array[0] = 360;
            }
            else if (g_data_array[0] <= 0) {
                g_data_array[0] = 0;
            }
            g_angle = g_data_array[0];
            Serial.print("Turn Angle: ");
            Serial.println(g_angle);
        }
        // Modify speed of forward/back.
        else if (g_serial_data[0] == 'd') {
            if (g_data_array[0] >= 400) {
                g_data_array[0] = 400;
            }
            else if (g_data_array[0] <= 0) {
                g_data_array[0] = 0;
            }
            g_motor_speed = g_data_array[0];
            Serial.print("Motor Forward/Back Speed: ");
            Serial.println(g_motor_speed);
        }
        // Clear up flag and buffer.
        g_flag_serial = false;
        g_serial_data = "";
        memset(g_data_array, 0, sizeof(g_data_array));
    }
}
