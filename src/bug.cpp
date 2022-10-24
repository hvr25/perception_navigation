
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <stdlib.h>

class Bug {
private:
    // A ROS node
    ros::NodeHandle n;

    float laser_range[361] = {};
    
    ros::Subscriber laser_sub;
    ros::Subscriber odom;
    ros::Publisher vel;

 
public:
    Bug() 
    {
        // Initialize the node handle
        n = ros::NodeHandle("~");
        laser_sub = n.subscribe("base_scan", 1, &Bug::laser_callback, this);
        odom = n.subscribe("base_pose_ground_truth", 1, &Bug::odom_callback, this);
        vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    }


    void laser_callback(const sensor_msgs::LaserScan& msg)
    {
        ROS_INFO("I heard: 1 ");
        ROS_INFO("range_0: [%f]",msg.ranges[0]);
        ROS_INFO("range_180: [%f]",msg.ranges[180]);
        ROS_INFO("range_360: [%f]",msg.ranges[360]);

        for(int i = 0; i < 361; i++)
        {
            laser_range[i] =  msg.ranges[i];
        }
    }

    void odom_callback(const nav_msgs::Odometry& msg)
    {   
        geometry_msgs::Twist twist;
        float x1 = msg.pose.pose.position.x;
        float y1 = msg.pose.pose.position.y;
        float x2 = 4.5;
        float y2 = 9;
        float slope = (y2-y1)/(x2-x1);
        float line_angle = atan(slope);
        // yaw (z-axis rotation)
        double siny_cosp = 2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y);
        double cosy_cosp = 1 - 2 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z);
        float yaw = std::atan2(siny_cosp, cosy_cosp);
        ROS_INFO("odom:x [%f]",msg.pose.pose.position.x);
        ROS_INFO("odom:y [%f]",msg.pose.pose.position.y);
        ROS_INFO("odom:angle [%f]",yaw);
        ROS_INFO("line_angle:[%f]  ",line_angle);

        twist.linear.x = 1;
        twist.angular.z = line_angle - yaw;
        vel.publish(twist);
        ROS_INFO("I heard: 4  ");

    }
}; // end of class definition


int main(int argc, char ** argv) {
    ros::init(argc, argv, "bug");
    Bug rw;
    ros::spin();
    return 0;
}