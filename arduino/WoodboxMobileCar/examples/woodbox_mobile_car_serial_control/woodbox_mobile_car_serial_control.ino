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
 *  This .ino file implements woodbox mobile car serial control.
 **********************************************************************/

#include <woodbox_mobile_car.h>

int               g_byte        = 0;
int               g_motor_speed = 100;
int               g_motor_brake = 400;
char             *g_string      = NULL;
WoodboxMobileCar  g_woodbox_mobile_car;

void setup()
{
    g_woodbox_mobile_car.initializePin();
    Serial.begin(9600);
    Serial.println("Start Woodbox Mobile Car Serial Control!");
    Serial.println("----------------------------------------");
    Serial.println("Input follow characters to control car  ");
    Serial.println("----------------------------------------");
    Serial.println("w: Move Forward       s: Move Back      ");
    Serial.println("a: Move Left          d: Move Right     ");
    Serial.println("u: Add Motor Speed    i: Sub Motor Speed");
    Serial.println("j: Add Motor Brake    k: Sub Motor Brake");
    Serial.println("----------------------------------------");
    delay(5000);
}

void loop()
{
    if (Serial.available()) {
        g_byte = Serial.read();
        // Move Forward.
        if (g_byte == 'w') {
            g_woodbox_mobile_car.setMotorSpeeds(+g_motor_speed,
                                                +g_motor_speed);
            g_string = "Move Forward";
        }
        // Move Back.
        else if (g_byte == 's') {
            g_woodbox_mobile_car.setMotorSpeeds(-g_motor_speed,
                                                -g_motor_speed);
            g_string = "Move Back";
        }
        // Move Left.
        else if (g_byte == 'a') {
            g_woodbox_mobile_car.setMotorSpeeds(-g_motor_speed,
                                                +g_motor_speed);
            g_string = "Move Left";
        }
        // Move Right.
        else if (g_byte == 'd') {
            g_woodbox_mobile_car.setMotorSpeeds(+g_motor_speed,
                                                -g_motor_speed);
            g_string = "Move Right";
        }
        // Add motor speed.
        else if (g_byte == 'u') {
            if (g_motor_speed >= 400) {
                g_motor_speed = 400;
            }
            else {
                g_motor_speed += 50;
            }
            Serial.print("Motor Speed: ");
            Serial.print(g_motor_speed);
            Serial.println(" r/min");
            g_string = "";
        }
        // Sub motor speed.
        else if (g_byte == 'i') {
            if (g_motor_speed <= 0) {
                g_motor_speed = 0;
            }
            else {
                g_motor_speed -= 50;
            }
            Serial.print("Motor Speed: ");
            Serial.print(g_motor_speed);
            Serial.println(" r/min");
            g_string = "";
        }
        // Add motor brake.
        else if (g_byte == 'j') {
            if (g_motor_brake >= 400) {
                g_motor_brake = 400;
            }
            else {
                g_motor_brake += 50;
            }
            Serial.print("Motor Brake: ");
            Serial.print(g_motor_brake);
            Serial.println(" r/min");
            g_string = "";
        }
        // Sub motor brake.
        else if (g_byte == 'k') {
            if (g_motor_brake <= 0) {
                g_motor_brake = 0;
            }
            else {
                g_motor_brake -= 50;
            }
            Serial.print("Motor Brake: ");
            Serial.print(g_motor_brake);
            Serial.println(" r/min");
            g_string = "";
        }
        // Stop move.
        else {
            g_woodbox_mobile_car.setMotorBrakes(g_motor_brake, g_motor_brake);
            g_string = "";
        }
        Serial.println(g_string);
    }
    g_woodbox_mobile_car.setMotorBrakes(g_motor_brake, g_motor_brake);
}
