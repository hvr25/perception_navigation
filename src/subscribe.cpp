#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <stdlib.h>


void laser_callback(const sensor_msgs::LaserScan& msg)
{

    ROS_INFO("I heard: 1 ");
    ROS_INFO("range_0: [%f]",msg.ranges[0]);
    ROS_INFO("range_180: [%f]",msg.ranges[180]);
    ROS_INFO("range_360: [%f]",msg.ranges[360]);

    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Rate r(30);
    float f = 0.0;
       
    float max_x1 = 0.0;
    float max_y1 = 0.0;
    float max_x2 = 0.0;
    float max_y2 = 0.0;
    float max_len = 0;
// %Tag(MARKER_INIT)%
    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/my_frame";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
// %EndTag(MARKER_INIT)%
    ROS_INFO("I heard: 3");
// %Tag(ID)%
    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;
// %EndTag(ID)%

// %Tag(TYPE)%
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
// %EndTag(TYPE)%

// %Tag(SCALE)%
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;
// %EndTag(SCALE)%

// %Tag(COLOR)%
    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    for(int j = 0; j < 15; j++)
    {
        int inlier_cnt = 0;
        int rand1 = rand() % 361;
        int rand2 = rand() % 361;
        float x1 = 0.0;
        float y1 = 0.0;
        float x2 = 0.0;
        float y2 = 0.0;
        ROS_INFO("for j ");
   
        if(msg.ranges[rand1] < 3.0 and msg.ranges[rand2] < 3.0)
        {
            x1 = msg.ranges[rand1] * cos((rand1 * 0.00872665)) ;
            y1 = msg.ranges[rand1] * sin((rand1 * 0.00872665)) ;
            x2 = msg.ranges[rand2] * cos((rand2 * 0.00872665)) ;
            y2 = msg.ranges[rand2] * sin((rand2 * 0.00872665)) ;

            for (int i = 0; i < msg.ranges.size(); i++)
            {
                float x0 = msg.ranges[i] * cos((i * 0.00872665)) ;
                float y0 = msg.ranges[i] * sin((i * 0.00872665)) ;
                float distance = (abs((((x2-x1)*(y1-y0))-((x1-x0)*(y2-y1)))))/(pow(pow((x2-x1),2)+pow((y2-y1),2),0.5));
                ROS_INFO("for i ");
                if(distance < 0.1)
                {
                    inlier_cnt++;
                }

                if(inlier_cnt > 100)
                {
                                       
                    geometry_msgs::Point p;
                    p.x = x1;
                    p.y = y1;
                    p.z = 0;
                    line_list.points.push_back(p);
                        p.x = x2;
                    p.y = y2;
                    line_list.points.push_back(p);
                    marker_pub.publish(line_list);
                }

       
            }

            if(inlier_cnt > 150)
            {
                float len = pow((pow(x2-x1,2))+(pow(y2-y1,2)),0.5);
                ROS_INFO("length : [%f]",len);
                if(max_len < len)
                {
                    max_x1 = x1;
                    max_x2 = x2;
                    max_y1 = y1;
                    max_y2 = y2;
                }
            }
        }
    }
           
      geometry_msgs::Point p;
      //p.x = (int32_t)i - 50;
      p.x = max_x1;
      p.y = max_y1;
      p.z = 0;

      line_list.points.push_back(p);
        p.x = max_x2;
      p.y = max_y2;
      line_list.points.push_back(p);
    marker_pub.publish(line_list);


r.sleep();

    f += 0.04;  
}


int main(int argc, char **argv)
{
 ros::init(argc, argv, "perception");
 ros::NodeHandle n;
 ros::Subscriber laser_sub;
 
  ROS_INFO("I heard: 2  ");
  laser_sub = n.subscribe("base_scan", 1, laser_callback);
 
 
 ros::spin();
  return 0;
}