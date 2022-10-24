
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <stdlib.h>
float laser_range[361] = {};
float yaw = 0.0;
float line_angle = 0.0;
float x = 0.0;
float y = 0.0;
float x2 = 4.5;
float y2 = 9;

void laser_callback(const sensor_msgs::LaserScan& msg)
{
    ROS_INFO("I heard: 1 ");
    ROS_INFO("range_0: [%f]",msg.ranges[0]);
    ROS_INFO("range_180: [%f]",msg.ranges[180]);
    ROS_INFO("range_360: [%f]",msg.ranges[360]);
    ROS_INFO("range_100: [%f]",msg.ranges[100]);

    for(int i = 0; i < 361; i++)
    {
        laser_range[i] =  msg.ranges[i];

    }

}

void odom_callback(const nav_msgs::Odometry& msg)
{   

    float x1 = msg.pose.pose.position.x;
    float y1 = msg.pose.pose.position.y;
    x = x1;
    y = y1;
    float slope = (y2-y1)/(x2-x1);
    if(x1 > 4.5)
        line_angle = atan(slope) + 3.1419;
    else 
        line_angle = atan(slope);
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y);
    double cosy_cosp = 1 - 2 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z);
    yaw = std::atan2(siny_cosp, cosy_cosp);   
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation");
    ros::NodeHandle n;
    ros::Subscriber laser_sub = n.subscribe("base_scan", 10, laser_callback);
    ros::Subscriber odom = n.subscribe("base_pose_ground_truth", 10, odom_callback);
    ros::Publisher vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    

    while (ros::ok())
    {
        geometry_msgs::Twist twist;
        ros::Rate loop_rate(10);
        float angle_diff = line_angle - yaw;
        float dist = pow(pow(x2-x,2)+pow(y2-y,2),0.5);
        float front_obst = (laser_range[170] + laser_range[180] + laser_range[190])/3;
        float wall_angle = atan((laser_range[260] * cos(0.872665) - laser_range[360])/(laser_range[260]*sin(0.872665)));
        float wall_dist = 0;
        if(laser_range[360] < 3 and laser_range[260] < 3)
        {
            wall_dist = laser_range[360]*cos(wall_angle);
        }
        ROS_INFO("wall_dist [%f] ",wall_dist);
        ROS_INFO("wall_angle [%f] ",wall_angle*180/3.142);
        ROS_INFO("front_obst [%f] ",front_obst);

        if(dist > 0.2)
        {
            
        //turn right if there is any obstacle in the front
        if(front_obst < 1.2 and laser_range[360] ==  3.0
            or (laser_range[150] < 1) or (laser_range[100] < 1.6))
        {
            for(int i = 0; i < 20; i++)
            {
                twist.linear.x = 0.0; 
                twist.angular.z = -1.0;
                vel.publish(twist);

            }
        }
        //adjust orientation parallel to wall after turning right
        if(wall_dist < 0.8 and wall_dist > 0 and laser_range[180] > 0.2)
        { 
           twist.angular.z = wall_angle ;
           twist.linear.x = 0.0;
        }

        //wall follow
        if(abs(wall_angle) < 0.3 and laser_range[260] < 3)
        {
            twist.linear.x = 1; 
            twist.angular.z = wall_angle;
        }
        else if(laser_range[360] < 1 and front_obst > 1 and laser_range[260] > 2)
        {
            for(int i = 0; i < 900; i++)
            {
                twist.linear.x = 1.5; 
                twist.angular.z = 0.0;
                vel.publish(twist);
                  ROS_INFO("front_obst [%d] ",i);
            }
        }

        //goal seek
        if(dist > 0 and front_obst > 0.7 and (wall_dist > 0.7 or wall_dist == 0))
        {
            if(angle_diff > 0.01 or angle_diff < -0.01)
            {
                twist.linear.x = 0.0; 
                twist.angular.z = angle_diff; 
            }
            else
            {
                if(dist > 0)
                {
                    twist.linear.x = 1; 
                    twist.angular.z = 0.0;
                }
                else
                {
                    twist.linear.x = 0.0; 
                    twist.angular.z = 0.0;
                }             
            }
        }

        vel.publish(twist);
          
    }
     ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();
    return 0;
}