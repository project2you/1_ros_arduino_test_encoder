#include <Arduino.h>
#include "PID.h"
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>

const int PIN_LEFT_INA = 4;
const int PIN_LEFT_INB = 5;
const int PIN_LEFT_PWM = 6;

const int PIN_RIGHT_INA = 8;
const int PIN_RIGHT_INB = 9;
const int PIN_RIGHT_PWM = 7;

const int PIN_ENCODE_A_MOTOR_LEFT = 2;
const int PIN_ENCODE_B_MOTOR_LEFT = 3;

const int PIN_ENCODE_A_MOTOR_RIGHT = 18;
const int PIN_ENCODE_B_MOTOR_RIGHT = 19;

long pos_left = 0, pos_right = 0;
long last_pos_left = 0, last_pos_right = 0;

// left_controller(KP = , KI = , KD =  , Loop time = )
PID left_controller(1.0, 0.1, 0.0, 0.05);

PID right_controller(1.0, 0.1, 0.0, 0.05);

ros::NodeHandle nh;
float speed_req;
float angular_speed_req;

void driveLeft(float vel) //-1.0 - 1.0
{
    if (vel > 0)
    {
        digitalWrite(PIN_LEFT_INA, HIGH);
        digitalWrite(PIN_LEFT_INB, LOW);
        analogWrite(PIN_LEFT_PWM, (int)(255.0 * vel));
    }
    else
    {
        digitalWrite(PIN_LEFT_INA, LOW);
        digitalWrite(PIN_LEFT_INB, HIGH);
        analogWrite(PIN_LEFT_PWM, (int)(255.0 * -vel));
    }
}

void driveRight(float vel) //-1.0 - 1.0
{
    if (vel > 0)
    {
        digitalWrite(PIN_RIGHT_INA, HIGH);
        digitalWrite(PIN_RIGHT_INB, LOW);
        analogWrite(PIN_RIGHT_PWM, (int)(255.0 * vel));
    }
    else
    {
        digitalWrite(PIN_RIGHT_INA, LOW);
        digitalWrite(PIN_RIGHT_INB, HIGH);
        analogWrite(PIN_RIGHT_PWM, (int)(255.0 * -vel));
    }
}

void encoderLeftMotor()
{
    if (digitalRead(PIN_ENCODE_A_MOTOR_LEFT) == digitalRead(PIN_ENCODE_B_MOTOR_LEFT))
    {
        pos_left++;
    }
    else
    {
        pos_left--;
    }
}

void encoderRightMotor()
{
    if (digitalRead(PIN_ENCODE_A_MOTOR_RIGHT) == digitalRead(PIN_ENCODE_B_MOTOR_RIGHT))
    {
        pos_right++;
    }
    else
    {
        pos_right--;
    }
}

void drive(float V, float W)
{
    float wheel_base = 0.15;
    float wheel_radius = 0.0325;
    float v_left = V - W * (wheel_base / 2);
    float v_right = V + W * (wheel_base / 2);
    float w_left = v_left / wheel_radius;
    float w_right = v_right / wheel_radius;

    left_controller.setSetPoint(w_left);
    right_controller.setSetPoint(-w_right);
}

char data[100];

void handle_cmd(const geometry_msgs::Twist &cmd_vel)
{
    speed_req = cmd_vel.linear.x;          //Extract the commanded linear speed from the message
    angular_speed_req = cmd_vel.angular.z; //Extract the commanded angular speed from the message
    drive(speed_req, angular_speed_req);
    sprintf(data, "Speed: %f, %f\n", speed_req, angular_speed_req);
    nh.loginfo(data);
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);

void setup()
{
    nh.initNode();
    
    // put your setup code here, to run once:
    pinMode(PIN_LEFT_INA, OUTPUT);
    pinMode(PIN_LEFT_INB, OUTPUT);
    pinMode(PIN_LEFT_PWM, OUTPUT);

    pinMode(PIN_RIGHT_INA, OUTPUT);
    pinMode(PIN_RIGHT_INB, OUTPUT);
    pinMode(PIN_RIGHT_PWM, OUTPUT);

    pinMode(PIN_ENCODE_A_MOTOR_LEFT, INPUT_PULLUP);
    pinMode(PIN_ENCODE_B_MOTOR_LEFT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODE_A_MOTOR_LEFT), encoderLeftMotor, RISING);

    pinMode(PIN_ENCODE_A_MOTOR_RIGHT, INPUT_PULLUP);
    pinMode(PIN_ENCODE_B_MOTOR_RIGHT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODE_A_MOTOR_RIGHT), encoderRightMotor, RISING);

    pos_left = 0;
    pos_right = 0;
    // Serial.begin(9600);
    // Serial.println("Hello World");

    //driveLeft(0.4);
    //driveRight(0.4);

    //left_controller
    left_controller.setInputLimits(-20.0, 20.0);
    left_controller.setOutputLimits(-1.0, 1.0);
    left_controller.setBias(0);
    left_controller.setMode(AUTO_MODE);
    //Motor speed 1 loop with 1 Sec =  2pi with sec = 6.28
    //left_controller.setSetPoint(-6.28); //Set Point Speed Wheel with R

    //right_controller
    right_controller.setInputLimits(-20.0, 20.0);
    right_controller.setOutputLimits(-1.0, 1.0);
    right_controller.setBias(0);
    right_controller.setMode(AUTO_MODE);
    //right_controller.setSetPoint(6.28);
    drive(0.0, 0.0);
    nh.subscribe(cmd_vel);
}

void loop()
{

    //  Serial.print(pos_left);
    //  Serial.print("\t");
    //  Serial.println(pos_right);

    // find angular velocity of left motor [rad/s]
    float temp = (pos_left - last_pos_left) / 0.05; // [pulse/sec]
    float w_left = temp * 2.0 * 3.14159 / 341.0;    // [rad/s]
    left_controller.setProcessValue(w_left);
    float output_left = left_controller.compute();
    driveLeft(output_left);
    // Serial.print(w_left);
    last_pos_left = pos_left;

    // find angular velocity of left motor [rad/s]
    float temp_right = (pos_right - last_pos_right) / 0.05; // [pulse/sec]
    float w_right = temp_right * 2.0 * 3.14159 / 341.0;     // [rad/s]
    right_controller.setProcessValue(w_right);
    float output_right = right_controller.compute();
    driveRight(output_right);
    // Serial.print("\t");
    // Serial.println(w_right);
    last_pos_right = pos_right;

    //  rad_sec = w * 2 * 3.14159 / 341
    //  Serial.print("\t");
    //  Serial.println(rad_sec);

    delay(50);
    nh.spinOnce();
}
