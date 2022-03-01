#include "first_challenge_sasaki/first_challenge_sasaki.h"

FirstChallenge::FirstChallenge():private_nh_("~")
{
    private_nh_.param("hz_", hz_, {10});
    sub_odom_ = nh_.subscribe("/roomba/odometry", 100, &FirstChallenge::odometry_callback, this);
    sub_laser_ = nh_.subscribe("/scan", 100, &FirstChallenge::laser_callback, this);
    pub_cmd_vel_ = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
}

void FirstChallenge::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odometry_ = *msg;
}

void FirstChallenge::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ = *msg;
}

void FirstChallenge::run()//直進
{
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = 0.1;
    cmd_vel_.cntl.angular.z = 0.0;
    pub_cmd_vel_.publish(cmd_vel_);
    std::cout << "odom" << ": x:" << odometry_.pose.pose.position.x << " y:" <<  odometry_.pose.pose.position.y << " z:" <<  odometry_.pose.pose.position.z << std::endl;
}

void FirstChallenge::turn()
{
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = 0.0;
    cmd_vel_.cntl.angular.z = 0.1;
    pub_cmd_vel_.publish(cmd_vel_);
}

double FirstChallenge::GetYaw()
{
    double r,p,y;
    tf::Quaternion quat(odometry_.pose.pose.orientation.x, odometry_.pose.pose.orientation.y,odometry_.pose.pose.orientation.z,odometry_.pose.pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(r,p,y);
    return y;
}

void FirstChallenge::show_odom()
{
    std::cout << "odom" << ": x:" << odometry_.pose.pose.position.x << " y:" <<  odometry_.pose.pose.position.y << " z:" <<  odometry_.pose.pose.position.z << std::endl;
}

void FirstChallenge::show_scan()//センサから得られた最小値の更新
{
    float range_min = 1e6;//10の6乗
    for (int i = 0; i < laser_.ranges.size(); i++) {
        if (laser_.ranges[i] < range_min) {
            range_min = laser_.ranges[i];
        }
    }
    std::cout << "scan: min:" << range_min << std::endl;
}

double  FirstChallenge::scan_min_value()
{
    float medium_value = laser_.ranges.size()/2.0;
    int start_value = medium_value -10;
    int finish_value = medium_value+10;
    float range_min = 1e6;
    for(int i=start_value; i<finish_value;i++)
    {
        if(laser_.ranges[i] < range_min)
        {
            range_min = laser_.ranges[i];
        }
    }
    return range_min;
}

void FirstChallenge::process()
{
    ros::Rate loop_rate(hz_);

    float target_disp = 1.0;//1m移動の移動目標
    float x = 0;//初めの位置
    float y = 0;
    float dx = 0;//変位
    float dy = 0;
    float real_disp = sqrt(dx*dx + dy*dy);//移動距離
    float target_angle = 2*M_PI;//目標移動角度
    float real_angle = 0;//現在の移動角度
    float dtheta = 0;//変位
    float range_min = 1e6;//センサの得た距離
    int   count = 0;//回転のためのカウント変数開始は1停止は1

    bool first_move_judge = true;//1m移動が完了したらfalseにする
    bool second_move_judge = false;//回転移動
    bool third_move_judge = false;//センサ利用移動

    while(ros::ok())
    {
      dx =  odometry_.pose.pose.position.x  - x;
      dy =  odometry_.pose.pose.position.y  - y;
      dtheta = GetYaw() - real_angle;
      real_disp = sqrt(dx*dx+dy*dy); //移動した距離を計算
      real_angle = GetYaw();
      range_min = scan_min_value();

      if(real_disp >= target_disp )//移動距離が1m以上に到達したなら実行
      {
          first_move_judge = false;//直進はおしまい
          second_move_judge = true;//回転移動の開始
          x = odometry_.pose.pose.position.x;//回転の前に並進移動距離の更新
          y = odometry_.pose.pose.position.y;//回転の前に並進移動距離の更新
      }

      if(real_disp < target_disp && first_move_judge == true)//移動距離が1m未満なら実行
      {
          run();
      }
      if(GetYaw() > M_PI/2.0 && GetYaw()< M_PI && count == 0)
      {
          count = 1;
      }
/*
      if(real_angle >= target_angle)
      {
          second_move_judge = false;//回転移動おしまい
          third_move_judge = true;//壁まで接近
          x = odometry_.pose.pose.position.x;//回転の前に並進移動距離の更新
          y = odometry_.pose.pose.position.y;//回転の前に並進移動距離の更新
      }

      if(real_angle < target_angle && second_move_judge == true)//1周するまで回転
      {
          turn();
      }
*/
      if(GetYaw() < 0.1 && GetYaw() > -0.1 && count == 1)
      {
          second_move_judge = false;//回転移動おしまい
          third_move_judge = true;//壁まで接近
          x = odometry_.pose.pose.position.x;//回転の前に並進移動距離の更新
          y = odometry_.pose.pose.position.y;//回転の前に並進移動距離の更新

      }

      if(real_angle < target_angle && second_move_judge == true)//1周するまで回転
      {
          turn();
      }

      if(first_move_judge == false && second_move_judge == false &&  third_move_judge == true && range_min < 500)
      {
          third_move_judge = false;
      }

      if(first_move_judge == false && second_move_judge == false && third_move_judge == true && range_min > 500)//壁まで500mm以上なら実行
      {
          run();
      }

      if(first_move_judge == false && second_move_judge == false && third_move_judge == false)
      {
          break;
      }

      show_odom();
      show_scan();

      ros::spinOnce();
      loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "first_challenge");
    FirstChallenge first_challenge;
    first_challenge.process();
    ros::spin();
    return 0;
}
