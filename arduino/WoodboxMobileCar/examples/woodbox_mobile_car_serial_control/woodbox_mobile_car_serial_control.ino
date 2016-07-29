#include <woodbox_mobile_car.h>

int               g_byte        = 0;
int               g_motor_speed = 100;
int               g_motor_brake = 400;
char             *g_string      = NULL;
WoodboxMobileCar g_woodbox_mobile_car;

void setup()
{
    g_woodbox_mobile_car.initializePin();
    Serial.begin(9600);
    Serial.println("Start Woodbox Mobile Car Serial Control!");
    Serial.println("----------------------------------------");
    Serial.println("Input follow characters to control car  ");
    Serial.println("----------------------------------------");
    Serial.println("w: Move Forward      s: Move Back       ");
    Serial.println("a: Move Left         d: Move Right      ");
    Serial.println("u: Add Motor Speed   i: Sub Motor Speed ");
    Serial.println("j: Add Motor Brake   k: Sub Motor Brake ");
    Serial.println("----------------------------------------");
    delay(5000);
}

void loop()
{
    if (Serial.available()) {
        g_byte = Serial.read();
        if (g_byte == 'w') {
            g_woodbox_mobile_car.setMotorSpeeds(+g_motor_speed,
                                                +g_motor_speed);
            g_string = "Move Forward";
        }
        else if (g_byte == 's') {
            g_woodbox_mobile_car.setMotorSpeeds(-g_motor_speed,
                                                -g_motor_speed);
            g_string = "Move Back";
        }
        else if (g_byte == 'a') {
            g_woodbox_mobile_car.setMotorSpeeds(-g_motor_speed,
                                                +g_motor_speed);
            g_string = "Move Left";
        }
        else if (g_byte == 'd') {
            g_woodbox_mobile_car.setMotorSpeeds(+g_motor_speed,
                                                -g_motor_speed);
            g_string = "Move Right";
        }
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
        Serial.println(g_string);
    }
    g_woodbox_mobile_car.setMotorBrakes(g_motor_brake, g_motor_brake);
}
