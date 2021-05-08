#include "ros/ros.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "rplidar_data/alpha.h"
#include <cstdlib>
#include "math.h"


class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
    client = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
    sub = n.subscribe("/controled_theta",1,&SubscribeAndPublish::callback,this);
    }
   
    void callback(const rplidar_data::alpha& input)
    {
      dynamixel_workbench_msgs::DynamixelCommand srv;
      std::string item_command = "";
      std::string item_addr = "Goal_Position";
      double phi = atan(1 / sqrt( (1/tan(input.alpha)) * (1/tan(input.alpha)) - 1 ));
      int Limit_Angle = (int)((2047/M_PI)*phi + 2048);
      if (Limit_Angle<2048 && Limit_Angle>3072)
	{
	  return;
	}
      int Angle = 0;
      int t = 2048;
      while (1)
	{
          int ID = 1;
          int value = t;
          srv.request.command = item_command;
          srv.request.id = ID;
          srv.request.addr_name = item_addr;
          srv.request.value = value;
          t++;
          if (t > Limit_Angle)
          {
              while(t > 4096-Limit_Angle)
              {
                  int value = t;
                  srv.request.command = item_command;
                  srv.request.id = ID;
                  srv.request.addr_name = item_addr;
                  srv.request.value = value;
                  t--;
              }
          }
          if(t==2048)
          {
              break;
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
private:
    ros::NodeHandle n;
    ros::ServiceClient client;
    ros::Subscriber sub;

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
