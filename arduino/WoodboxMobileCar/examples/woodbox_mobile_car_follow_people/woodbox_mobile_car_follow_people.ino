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
 *  This .ino file implements woodbox mobile car follow people.
 **********************************************************************/

#include <woodbox_mobile_car.h>

#define DELAY 2000

int              g_byte   = 0;
char            *g_string = NULL;
WoodboxMobileCar g_woodbox_mobile_car;

void setup()
{
    g_woodbox_mobile_car.initializePin();
    Serial.begin(9600);
    Serial2.begin(115200);
    Serial.println("Start Woodbox Mobile Car Follow People!");
    Serial.println("---------------------------------------");
    Serial.println("Receive follow commands to control car ");
    Serial.println("Y/y: Move Forward        N/n: Stop Move");
    Serial.println("---------------------------------------");
    delay(5000);
}

void loop()
{
    if (Serial2.available()) {
        // Receive character from jetson tk1.
        g_byte = Serial2.read();
        // Move Forward.
        if (g_byte == 'y' || g_byte == 'Y') {
            g_woodbox_mobile_car.setMotorSpeeds(100, 100);
            delay(DELAY);
            g_string = "Move Forward";
        }
        // Stop Move.
        else if (g_byte == 'n' || g_byte == 'N') {
            g_woodbox_mobile_car.setMotorBrakes(400, 400);
            g_string = "Stop Move";
        }
        else {
            g_woodbox_mobile_car.setMotorBrakes(400, 400);
        }
        Serial.println(g_string);
    }
    g_woodbox_mobile_car.setMotorBrakes(100, 100);
}
