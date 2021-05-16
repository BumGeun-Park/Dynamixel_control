#include "ros/ros.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "rplidar_data/alpha.h"
#include <cstdlib>
#include "math.h"
#include "sensor_msgs/JointState.h"

#define RAD2DATA(x) ((x)*(2047/M_PI)+2048)
#define DATA2RAD(x) ((x)-2048)*(M_PI/2047)

double current;

int t = 1;
int Limit_Angle;
int pseudo_Angle;
int start = 0;

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
    client = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
    sub1 = n.subscribe("/controled_theta",1,&SubscribeAndPublish::callback1,this);
    sub2 = n.subscribe("/dynamixel_workbench/joint_states",1,&SubscribeAndPublish::callback2,this);
    }
   
    void callback1(const rplidar_data::alpha& input)
    {
      dynamixel_workbench_msgs::DynamixelCommand srv;
      std::string item_command = "";
      std::string item_addr = "Goal_Position";
      int ID = 1;

      if (Limit_Angle<2048 && Limit_Angle>3072)
	{
	  return;
	}

      if(sqrt((current-pseudo_Angle)*(current-pseudo_Angle))<20) // number 1
      {
          srv.request.command = item_command;
          srv.request.id = ID;
          srv.request.addr_name = item_addr;
          pseudo_Angle = Limit_Angle;
          srv.request.value = 4096-pseudo_Angle;
          t = 0;
      }

      if(sqrt((4096-pseudo_Angle-current)*(4096-pseudo_Angle-current))<20) // number 2
      {
          srv.request.command = item_command;
          srv.request.id = ID;
          srv.request.addr_name = item_addr;
          pseudo_Angle = Limit_Angle;
          srv.request.value = pseudo_Angle;
          t = 1;
      }

      if(sqrt((2048-current)*(2048-current))<20 & t==1 ) // number 3
      {
          double phi = atan(1 / sqrt( (1/tan(input.alpha)) * (1/tan(input.alpha)) - 1 ));
          Limit_Angle = (int)((2047/M_PI)*phi + 2048);
          if(start==0)
          {
              srv.request.command = item_command;
              srv.request.id = ID;
              srv.request.addr_name = item_addr;
              srv.request.value = Limit_Angle;
              pseudo_Angle = Limit_Angle;
              ++start;
          }
      }

      if (client.call(srv))
          {
              ROS_INFO("send ID and Position Value : %ld, %ld", (long int)srv.request.id, (long int)srv.request.value);
              ROS_INFO("receive result : %ld", (bool)srv.response.comm_result);
          }
        else
          {
              ROS_ERROR("Failed to call dynamixel_command");
          }
    }

    void callback2(const sensor_msgs::JointState& input2)
    {
        current = input2.position[0];
    }
private:
    ros::NodeHandle n;
    ros::ServiceClient client;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "write");
  SubscribeAndPublish NH;
  while(ros::ok())
    {
    ros::spinOnce();
    }
  return 0;
} 
