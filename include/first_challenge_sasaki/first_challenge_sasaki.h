#ifndef FIRST_CHALLENGE_SASAKI_H
#define FIRST_CHALLENGE_SASAKI_H

#include <ros/ros.h>
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/tf.h"

class FirstChallenge
{
    public:
        FirstChallenge();
        void process();

    private:
        void odometry_callback(const nav_msgs::Odometry::ConstPtr&);
        void laser_callback(const sensor_msgs::LaserScan::ConstPtr&);

        void run();
        void turn();
        double  scan_min_value();
        void show_odom();
        void show_scan();
        double  GetYaw();//クオータニオンからRPYを取得する関数

        int hz_;

        nav_msgs::Odometry odometry_;//変位
        sensor_msgs::LaserScan laser_;//観測
        roomba_500driver_meiji::RoombaCtrl cmd_vel_;//速度

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Subscriber sub_odom_;
        ros::Subscriber sub_laser_;
        ros::Publisher pub_cmd_vel_;

};

#endif
