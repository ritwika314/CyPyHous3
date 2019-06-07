#ifndef MONITOR_H
#define MONITOR_H

#include <cmath>

#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"


class monitorAndReport
{
public:
      monitorAndReport()
      {
          pub_ = n_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1);
          sub_ = n_.subscribe("/turtle1/pose", 10, &monitorAndReport::PoseCallback, this);
          turtle2_pos = n_.subscribe("/turtle11/pose", 10, &monitorAndReport::Pose2Callback, this);
          client_ = n_.serviceClient<std_srvs::Empty>("clear");
      }


      void Pose2Callback(const turtlesim::Pose::ConstPtr& msg)
      {
          ROS_INFO("turtle2 Location [%f, %f]", msg->x, msg->y);
          return;
      }

      void PoseCallback(const turtlesim::Pose::ConstPtr& msg)
      {
         this->agent_x = msg->x;
         this->agent_y = msg->y;
         this->agent_theta = msg->theta;
         ROS_INFO("Agent Location: [%f, %f, %f]", this->agent_x, this->agent_y, this->agent_theta);



         if(turtle2_pos.getNumPublishers() == 0)
         {
            ROS_INFO("Lost communcation with turtle 2- Returning Home...");
            if(lost_comm != 1)
            {
              gotoPt();
              this->lost_comm = 1;
            }

         }
         else{lost_comm = 0;}

          if(this->agent_x < this->boundary_x_min || this->agent_x > this->boundary_x_max || this->agent_y < this->boundary_y_min || this->agent_y > this->boundary_x_max)
          {
              ROS_INFO("Out of bounds at [%f, %f] - Returning Home...", agent_x, agent_y);
              gotoPt();
          }
          //ROS_INFO("Current Position: [%f, %f]", msg->x, msg->y );


          return;
      }

      void gotoPt()
      {
        geometry_msgs::Twist speed;
        double goal_x = 5.0;
        double goal_y = 5.0;

        ros::Rate loop_rate(5);

        while(ros::ok())
        {
            double distToTarget_x = goal_x - this->agent_x;
            double distToTarget_y = goal_y - this->agent_y;

            double angleToTarget = atan2(distToTarget_y, distToTarget_x);

            ROS_INFO("Distance to Goal: [%f, %f, %f]", distToTarget_x, distToTarget_y, angleToTarget);


            if(abs(distToTarget_x) <= 0.25 && abs(distToTarget_y) <= 0.25)
            {
                ROS_INFO("Home");
                speed.linear.x = 0.0;
                speed.angular.z = 0.0;
                std_srvs::Empty srv;
                client_.call(srv);
                break;
            }
            else if (abs(angleToTarget - this->agent_theta) > 0.3)
            {
                ROS_INFO("Angle: %f", abs(angleToTarget - this->agent_theta));
                speed.linear.x = 0.0;
                speed.angular.z = 0.3;
            }
            else
            {
              ROS_INFO("Moving");
              speed.linear.x = 0.3;
              speed.angular.z = 0.0;
            }

            pub_.publish(speed);
            ros::spinOnce();
            loop_rate.sleep();

        }

        return;
      }

    private:
      ros::NodeHandle n_;
      ros::Publisher pub_;
      ros::Subscriber sub_;
      ros::Subscriber turtle2_pos;
      ros::ServiceClient client_;

      double agent_x;
      double agent_y;
      double agent_theta;

      double boundary_x_min = 3.0;
      double boundary_x_max = 10.0;
      double boundary_y_min = 3.0;
      double boundary_y_max = 10.0;

      int lost_comm = 0;

};

#endif
