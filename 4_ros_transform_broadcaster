#include <Arduino.h>
#include "PID.h"
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

const int PIN_LEFT_INA = 5;
const int PIN_LEFT_INB = 4;
const int PIN_LEFT_PWM = 6;

const int PIN_RIGHT_INA = 9;
const int PIN_RIGHT_INB = 8;
const int PIN_RIGHT_PWM = 7;

//You Can Buy with : https://www.amazon.com/MITUHAKI-Motor-Gear-Encoder-Mounting/dp/B07XC63724
//DC 6 V 210 RPM Encoder MOTOR LEFT
// Black =>  Ground
// Red => Power
// Yellow => PIN 2
// Green => PIN 3
// Blue => 3.3v
const int PIN_ENCODE_A_MOTOR_LEFT = 2; // Yellow
const int PIN_ENCODE_B_MOTOR_LEFT = 3; // Green

//DC 6 V 210 RPM Encoder MOTOR LEFT
// Black =>  Ground
// Red => Power
// Yellow => PIN 19
// Green => PIN 18
// Blue => 3.3v
const int PIN_ENCODE_A_MOTOR_RIGHT = 18; // Green
const int PIN_ENCODE_B_MOTOR_RIGHT = 19; // Yellow

long pos_left = 0, pos_right = 0;
long last_pos_left = 0, last_pos_right = 0;

// left_controller(KP = , KI = , KD =  , Loop time = )
PID left_controller(1.2, 0.2, 0.0, 0.05);

PID right_controller(1.2, 0.2, 0.0, 0.05);

nav_msgs::Odometry odom_msg;
ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
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
    float wheel_base = 0.2;
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
    // sprintf(data, "Speed: %f, %f\n", speed_req, angular_speed_req);
    // nh.loginfo(data);
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", &handle_cmd);
ros::Publisher odom_pub("odom", &odom_msg);

float x = 0;
float y = 0;
float theta = 0;

unsigned long prev_time = millis();
unsigned long debug_time = millis();
void setup()
{
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    broadcaster.init(nh);
    nh.subscribe(cmd_vel);
    nh.advertise(odom_pub);

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
    // left_controller.setSetPoint(-2.0);
    //right_controller
    right_controller.setInputLimits(-20.0, 20.0);
    right_controller.setOutputLimits(-1.0, 1.0);
    right_controller.setBias(0);
    right_controller.setMode(AUTO_MODE);
    // right_controller.setSetPoint(-2.0);
    drive(0.0, 0.0);
}

float v_robot = 0, w_robot = 0;
void loop()
{
    unsigned long current_time = millis();
    if (current_time - prev_time > 50)
    {
        float dt = (current_time - prev_time) / 1000.0;
        // Serial.print(pos_left);
        // Serial.print("\t");
        // Serial.println(pos_right);

        // find angular velocity of left motor [rad/s]
        float temp = (pos_left - last_pos_left) / dt; // [pulse/sec]
        float w_left = temp * 2.0 * 3.14159 / 341.0;  // [rad/s]
        left_controller.setProcessValue(w_left);
        float output_left = left_controller.compute();
        driveLeft(output_left);
        // Serial.print(w_left);
        last_pos_left = pos_left;

        // find angular velocity of left motor [rad/s]
        float temp_right = (pos_right - last_pos_right) / dt; // [pulse/sec]
        float w_right = temp_right * 2.0 * 3.14159 / 341.0;   // [rad/s]
        right_controller.setProcessValue(w_right);
        float output_right = right_controller.compute();
        driveRight(output_right);
        // Serial.print("\t");
        // Serial.println(w_right);
        last_pos_right = pos_right;

        // Calculate Odometry
        float vl = w_left * 0.0325;
        float vr = -w_right * 0.0325;
        v_robot = (vr + vl) / 2;
        w_robot = (vr - vl) / 0.20;

        x += v_robot * cos(theta) * dt;
        y += v_robot * sin(theta) * dt;
        theta += w_robot * dt;

        // Serial.print(v_robot);
        // Serial.print("\t");
        // Serial.println(w_robot);

        prev_time = current_time;
    }
    if (current_time - debug_time > 60)
    {
        odom_msg.header.stamp = nh.now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(theta);
        odom_msg.twist.twist.linear.x = v_robot;
        odom_msg.twist.twist.angular.z = w_robot;
        odom_pub.publish(&odom_msg);

        t.header.stamp = nh.now();
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.rotation = odom_msg.pose.pose.orientation;
        broadcaster.sendTransform(t);
        // Serial.print(x);
        // Serial.print("\t");
        // Serial.print(y);
        // Serial.print("\t");
        // Serial.println(theta * 180.0 / 3.14159);
        debug_time = current_time;
    }

    // delay(50);
    nh.spinOnce();
    delay(1);
}
