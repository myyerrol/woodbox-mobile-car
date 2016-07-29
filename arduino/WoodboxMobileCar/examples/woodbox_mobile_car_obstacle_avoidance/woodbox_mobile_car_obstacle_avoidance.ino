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
    g_distance = g_woodbox_mobile_car.getDistance();

    if (10 < g_distance <= 20) {
#if DELAY_FIXED
    g_woodbox_mobile_car.setMotorSpeeds(-g_motor_speed, +g_motor_speed);
    delay(DELAY);
#else
    g_woodbox_mobile_car.setTurnAngle(g_motor1_speed, g_motor2_speed, g_angle);
#endif
    }
    else if (g_distance <= 10) {
        g_woodbox_mobile_car.setMotorSpeeds(-g_motor_speed, -g_motor_speed);
        delay(DELAY / 2);
#if DELAY_FIXED
    g_woodbox_mobile_car.setMotorSpeeds(-g_motor_speed, +g_motor_speed);
    delay(DELAY);
#else
    g_woodbox_mobile_car.setTurnAngle(g_motor1_speed, g_motor2_speed, g_angle);
#endif
    }

    g_woodbox_mobile_car.setMotorSpeeds(g_motor_speed, g_motor_speed);

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

    if (g_flag_serial) {
        for (int i = 1, j = 0; i < g_serial_data.length(); i++) {
            if (g_serial_data[i] == ' ') {
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
        g_flag_serial = false;
        g_serial_data = "";
        memset(g_data_array, 0, sizeof(g_data_array));
    }
}
