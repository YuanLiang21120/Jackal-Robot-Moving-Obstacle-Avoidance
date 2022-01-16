#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <keyboard/Key.h>
#include <iostream>
#include <cmath>
#include <ctime>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include "tf/tf.h"
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <algorithm>

int key;
double acc,acc1,acc2,acc3,acc4,acc5,acc6,acc7,acc8,acc9,acc10;
double x_c, y_c, z_c, w_c;
double theta, phi, psai;
double x = 0.0;
double y = 0.0;
double goal_x = 0;
double goal_y = 18;
double pi = 3.141593;
double steer_max = pi / 2.0;
double v_max = 2.0;
double currentv = 0.0;
const double dt = 0.1;
const double kp = 0.045;
const double ki = 0.00;
const double kd = 0.00;
double output = 0;
double integral = 0.0;
double previous_error = 0.0;
double yaw = 0;
std::vector<float> scan_data(720);
int count1 = 0;
int count2 = 0;
int count3 = 0;
int count4 = 0;
int count5 = 0;
int count6 = 0;
int count7 = 0;
int count8 = 0;
int count9 = 0;
int count10 = 0;
double vl = 0.5;
int fazhi = 100;

double prev_distance = -1;
double prev_angle = -1;
double anglediff = 0;

void imuCallback(const sensor_msgs::Imu::ConstPtr& data)
{
    acc = pow(data->linear_acceleration.x, 2.0) + pow(data->linear_acceleration.y, 2.0);
}

void imuCallback1(const sensor_msgs::Imu::ConstPtr& data)
{
    acc1 = pow(data->linear_acceleration.x, 2.0) + pow(data->linear_acceleration.y, 2.0);
}

void imuCallback2(const sensor_msgs::Imu::ConstPtr& data)
{
    acc2 = pow(data->linear_acceleration.x, 2.0) + pow(data->linear_acceleration.y, 2.0);
}

void imuCallback3(const sensor_msgs::Imu::ConstPtr& data)
{
    acc3 = pow(data->linear_acceleration.x, 2.0) + pow(data->linear_acceleration.y, 2.0);
}

void imuCallback4(const sensor_msgs::Imu::ConstPtr& data)
{
    acc4 = pow(data->linear_acceleration.x, 2.0) + pow(data->linear_acceleration.y, 2.0);
}

void imuCallback5(const sensor_msgs::Imu::ConstPtr& data)
{
    acc5 = pow(data->linear_acceleration.x, 2.0) + pow(data->linear_acceleration.y, 2.0);
}

void imuCallback6(const sensor_msgs::Imu::ConstPtr& data)
{
    acc6 = pow(data->linear_acceleration.x, 2.0) + pow(data->linear_acceleration.y, 2.0);
}

void imuCallback7(const sensor_msgs::Imu::ConstPtr& data)
{
    acc7 = pow(data->linear_acceleration.x, 2.0) + pow(data->linear_acceleration.y, 2.0);
}

void imuCallback8(const sensor_msgs::Imu::ConstPtr& data)
{
    acc8 = pow(data->linear_acceleration.x, 2.0) + pow(data->linear_acceleration.y, 2.0);
}

void imuCallback9(const sensor_msgs::Imu::ConstPtr& data)
{
    acc9 = pow(data->linear_acceleration.x, 2.0) + pow(data->linear_acceleration.y, 2.0);
}

void imuCallback10(const sensor_msgs::Imu::ConstPtr& data)
{
    acc10 = pow(data->linear_acceleration.x, 2.0) + pow(data->linear_acceleration.y, 2.0);
}
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& data)
{
    scan_data = data->ranges;
}


void keyCallback(const keyboard::Key::ConstPtr& data)
{
    ROS_INFO("Key = %i", data->code);//print out the keycode
    key = data->code;
}

void odoCallback(const geometry_msgs::Pose::ConstPtr& data)
{
    x = data->position.x;
    y = data->position.y;
    yaw = tf::getYaw(data->orientation);
}
/*
void goalCallback(const geometry_msgs::Pose::ConstPtr& data)
{
    goal_x = data->position.x;
    goal_y = data->position.y;
}*/

void initialization(void) {
  integral = 0.0;
  previous_error = 0.0;
  output = 0.0;
}

void velCallback(const nav_msgs::Odometry::ConstPtr& data)
{
    currentv = data->twist.twist.linear.x;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "jackal_turn_key");
    ros::NodeHandle nh;
    ros::Publisher vel_pub, vel_pub1, vel_pub2, vel_pub3, vel_pub4, vel_pub5;
    ros::Publisher vel_pub6, vel_pub7, vel_pub8, vel_pub9, vel_pub10;


    vel_pub = nh.advertise<geometry_msgs::Twist>("/jackal0/jackal_velocity_controller/cmd_vel", 1, true);
    vel_pub1 = nh.advertise<geometry_msgs::Twist>("/jackal1/jackal_velocity_controller/cmd_vel", 1, true);
    vel_pub2 = nh.advertise<geometry_msgs::Twist>("/jackal2/jackal_velocity_controller/cmd_vel", 1, true);
    vel_pub3 = nh.advertise<geometry_msgs::Twist>("/jackal3/jackal_velocity_controller/cmd_vel", 1, true);
    vel_pub4 = nh.advertise<geometry_msgs::Twist>("/jackal4/jackal_velocity_controller/cmd_vel", 1, true);
    vel_pub5 = nh.advertise<geometry_msgs::Twist>("/jackal5/jackal_velocity_controller/cmd_vel", 1, true);
    vel_pub6 = nh.advertise<geometry_msgs::Twist>("/jackal6/jackal_velocity_controller/cmd_vel", 1, true);
    vel_pub7 = nh.advertise<geometry_msgs::Twist>("/jackal7/jackal_velocity_controller/cmd_vel", 1, true);
    vel_pub8 = nh.advertise<geometry_msgs::Twist>("/jackal8/jackal_velocity_controller/cmd_vel", 1, true);
    vel_pub9 = nh.advertise<geometry_msgs::Twist>("/jackal9/jackal_velocity_controller/cmd_vel", 1, true);
    vel_pub10 = nh.advertise<geometry_msgs::Twist>("/jackal10/jackal_velocity_controller/cmd_vel", 1, true);

    ros::Subscriber imusub = nh.subscribe("/jackal0/imu/data", 10, imuCallback);
    ros::Subscriber imusub1 = nh.subscribe("/jackal1/imu/data", 10, imuCallback1);
    ros::Subscriber imusub2 = nh.subscribe("/jackal2/imu/data", 10, imuCallback2);
    ros::Subscriber imusub3 = nh.subscribe("/jackal3/imu/data", 10, imuCallback3);
    ros::Subscriber imusub4 = nh.subscribe("/jackal4/imu/data", 10, imuCallback4);
    ros::Subscriber imusub5 = nh.subscribe("/jackal5/imu/data", 10, imuCallback5);
    ros::Subscriber imusub6 = nh.subscribe("/jackal6/imu/data", 10, imuCallback6);
    ros::Subscriber imusub7 = nh.subscribe("/jackal7/imu/data", 10, imuCallback7);
    ros::Subscriber imusub8 = nh.subscribe("/jackal8/imu/data", 10, imuCallback8);
    ros::Subscriber imusub9 = nh.subscribe("/jackal9/imu/data", 10, imuCallback9);
    ros::Subscriber imusub10 = nh.subscribe("/jackal10/imu/data", 10, imuCallback10);


    ros::Subscriber keysub = nh.subscribe("/keyboard/keydown", 10, keyCallback);
    ros::Subscriber odosub = nh.subscribe("/jackal0/global_pos", 10, odoCallback);
    ros::Subscriber scansub = nh.subscribe("/jackal0/front/scan", 10, scanCallback);
    ros::Subscriber velsub = nh.subscribe("/jackal0/odometry/local_filtered", 10, velCallback);
    //ros::Subscriber goalsub = nh.subscribe("/jackal0/goal_pos", 10, goalCallback);
    
    ros::Rate loop_rate(10);
    geometry_msgs::Twist vel, vel1, vel2, vel3, vel4, vel5, vel6, vel7, vel8, vel9, vel10;

    vel.linear.x = 0.8; 
    vel1.linear.x = 0.8; vel2.linear.x = 0.8; vel3.linear.x = 0.8; vel4.linear.x = 0.8; vel5.linear.x = 0.8;
    vel6.linear.x = 0.8; vel7.linear.x = 0.8; vel8.linear.x = 0.8; vel9.linear.x = 0.8; vel10.linear.x = 0.8;
    vel.angular.z = 0;

    // warm up
    int warm_up_count = 0;
    goal_x = 7;
    goal_y = 7;
    while (ros::ok()) {
        if (warm_up_count > 3) break;
        vel_pub.publish(vel); 
        vel_pub1.publish(vel1); vel_pub2.publish(vel2); vel_pub3.publish(vel3); vel_pub4.publish(vel4); vel_pub5.publish(vel5);
        vel_pub6.publish(vel6); vel_pub7.publish(vel7); vel_pub8.publish(vel8); vel_pub9.publish(vel9); vel_pub10.publish(vel10);
        ros::spinOnce();
        loop_rate.sleep();
        warm_up_count++;
        //
	/*prev_distance =  *std::min_element(scan_data.begin()+310, scan_data.end()-310);
        prev_angle = std::min_element(scan_data.begin(),scan_data.end()) - scan_data.begin();
        prev_angle = prev_angle/720*270-135;*/
        /*double theta_distance = atan2(goal_y - y, goal_x - x);
        double theta_error = theta_distance - yaw;
        if(theta_error > pi){
            theta_error -= 2 * pi;
        }
        else if(theta_error < -pi){
            theta_error += 2 * pi;
        }

        if (theta_error > 3 * pi / 4 || theta_error <= -3 * pi / 4) {
            mode = 1;
        }
        else {
            int middle = 359 + int(360 * (theta_error / (3 * pi / 4)));
            int left = middle + 50 < 720 ? middle + 50 : 720;
            int right = middle - 50 >= 0 ? middle - 50 : 0;
            int sum = 0;
            double temp_distance = *std::min_element(scan_data.begin() + right, scan_data.begin() + left);
            if (temp_distance < 2) {
                prev_distance = temp_distance;
                prev_angle = std::min_element(scan_data.begin() + right, scan_data.begin() + left) - (scan_data.begin() + right);
            }
        }*/
    }
    // mode 1, go to goal; mode 0, avoid obstacle
    int previous_mode = 0;
    int mode = 0;
    int mode_count = 0;
    // main ros loop
    while (ros::ok())
    {
        vel_pub.publish(vel);
        vel_pub1.publish(vel1); vel_pub2.publish(vel2); vel_pub3.publish(vel3); vel_pub4.publish(vel4); vel_pub5.publish(vel5);
        vel_pub6.publish(vel6); vel_pub7.publish(vel7); vel_pub8.publish(vel8); vel_pub9.publish(vel9); vel_pub10.publish(vel10);
        ros::spinOnce();
        loop_rate.sleep();
        // linear velocity
        /*double error = v_max - currentv;
        integral += error * dt;
        double derivative = (error - previous_error);
        output = kp * error + ki * integral + kd * derivative;
        previous_error = error;
        vel.linear.x += 0.5 * output;
        */
        vel.linear.x = 0.8;
        std::cout << "goal" << goal_x << "," << goal_y << std::endl;

        double judge_distance = *std::min_element(scan_data.begin(), scan_data.end());
	
        if (judge_distance < 0.11) continue;
        
        //jackal1        
	if((acc1 > fazhi)) {
            count1 = 1;
            vel1.linear.x = -vel1.linear.x;
        }
        if((acc2 > fazhi)) {
            count2 = 1;
            vel2.linear.x = -vel2.linear.x;
        }
        if((acc3 > fazhi)) {
            count3 = 1;
            vel3.linear.x = -vel3.linear.x;
        }
        if((acc4 > fazhi)) {
            count4 = 1;
            vel4.linear.x = -vel4.linear.x;
        }
        if((acc5 > fazhi)) {
            count5 = 1;
            vel5.linear.x = -vel5.linear.x;
        }
        if((acc6 > fazhi)) {
            count6 = 1;
            vel6.linear.x = -vel6.linear.x;
        }
        if((acc7 > fazhi)) {
            count7 = 1;
            vel7.linear.x = -vel7.linear.x;
        }
        if((acc8 > fazhi)) {
            count8 = 1;
            vel8.linear.x = -vel8.linear.x;
        }
        if((acc9 > fazhi)) {
            count9 = 1;
            vel9.linear.x = -vel9.linear.x;
        }
        if((acc10 > fazhi)) {
            count10 = 1;
            vel10.linear.x = -vel10.linear.x;
        }


    
        for (int i=0 ; i < 720; i++)
        {
            if (scan_data[i] == std::numeric_limits<float>::infinity())
            {
                scan_data[i] = 30;
            }
        }
        

        // angular
        double theta_distance = atan2(goal_y - y, goal_x - x);
        double theta_error = theta_distance - yaw;
        if(theta_error > pi){
            theta_error -= 2 * pi;
        }
        else if(theta_error < -pi){
            theta_error += 2 * pi;
        }

        if (theta_error > 3 * pi / 4 || theta_error <= - 3 * pi / 4) {
            mode = 1;
        }
        else {
            int middle = 359 + int(360 * (theta_error / (3 * pi / 4)));
            int left = middle + 50 < 720 ? middle + 50 : 720;
            int right = middle - 50 >= 0 ? middle - 50 : 0;
            int sum = 0;
            double temp_distance = *std::min_element(scan_data.begin() + right, scan_data.begin() + left);
            if (temp_distance < 2.5) {
                double current_angle = std::min_element(scan_data.begin() + right, scan_data.begin() + left) - (scan_data.begin() + right);
                if (prev_distance != -1 && prev_angle != -1 && temp_distance < prev_distance) {
                    anglediff = (current_angle-prev_angle)*270/720*pi/180;
                    double velocity = 10* sqrt(pow(temp_distance,2)+ pow(prev_distance,2) - 2*temp_distance*prev_distance*cos(anglediff));
                    if (velocity > vel.linear.x)
                    {
                        mode = 2;
                    }
                    else{
                        mode = 0;
                    }
                 }
                 else {
                     mode = 0;
                 }
                 prev_angle = current_angle;
                 prev_distance = temp_distance;
            }
            else {
                mode = 1;
                prev_angle = -1;
                prev_distance = -1;
            }
        }
        if (pow(x - goal_x, 2.0) + pow(y - goal_y, 2.0) < 2) {
            mode = 1;
            prev_angle = -1;
            prev_distance = -1;
        }
        std::cout << mode << std::endl;
        if (mode == 2) {
             std::cout << "yimezhidengdiao: " << anglediff << std::endl;
        } 

        if (mode == 1) {
vel.linear.x = 0.8; 
            double steer = 0;
            if (abs(theta_error) >= steer_max) {
                if (theta_error > 0) {
                    steer = steer_max;
                }
                else {
                    steer = -steer_max;
                }
            }
            else {
                steer = theta_error;
            }

            vel.angular.z = 2.0 * steer;
        }
        else if (mode == 0) {
            std::cout << "small distance "<< std::endl;
            std::vector<float> avg_intensity(6);
            for (int i=0 ; i < 6; i++)
            {
              avg_intensity[i] = *std::min_element(scan_data.begin()+i*80+120,scan_data.begin()+(i+1)*80+120);
            }

            int maxidx = std::max_element(avg_intensity.begin(), avg_intensity.end()) - avg_intensity.begin();
            std::cout << "max index " << maxidx << std::endl;
            vel.angular.z = 0.25* (maxidx - 2.5) * steer_max;
vel.linear.x = 0.8;
        }
        else {
            vel.angular.z = -10.0 * anglediff;
            vel.linear.x = 0.3;
            // vel.linear.x = -0.8;
        }

        std::cout << ", yaw: " << yaw << std::endl;
        std::cout << "x: " << x << ", y: " << y << std::endl;
        std::cout << "currentv: " << currentv << std::endl;

        if (pow(x - goal_x, 2.0) + pow(y - goal_y, 2.0) < 1.5) {
            break;
        }
        if (key == 99)
        {
            break;
        }
    }
    return 0;
}
