#include "ros/ros.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "rplidar_data/alpha.h"
#include <cstdlib>
#include "math.h"
#include "sensor_msgs/JointState.h"

#define RAD2DATA(x) ((x)*(2047/M_PI)+2048)
#define DATA2RAD(x) ((x)-2048)*(M_PI/2047)
#define DEG2RAD(x) ((x)*M_PI/180)

#define upper_limit DEG2RAD(-10) // 10 deg
#define lower_limit DEG2RAD(45) // 45 deg

#define PWM_Limit 885
#define item "Goal_PWM"

double current;

int error_1_1;
int error_2_1;
int error_3_1;
int error_4_1;
int error_1_2;
int error_2_2;
int error_3_2;
int error_4_2;

int error_sum1;
int error_sum2;
int error_sum3;
int error_sum4;

double current1;
double current2;
double current3;
double current4;

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
    client = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
    sub1_1 = n.subscribe("/Kp_error1",1,&SubscribeAndPublish::callback1_1,this);
    sub2_1 = n.subscribe("/Kp_error2",1,&SubscribeAndPublish::callback2_1,this);
    sub3_1 = n.subscribe("/Kp_error3",1,&SubscribeAndPublish::callback3_1,this);
    sub4_1 = n.subscribe("/Kp_error4",1,&SubscribeAndPublish::callback4_1,this);
    sub1_2 = n.subscribe("/Ki_error1",1,&SubscribeAndPublish::callback1_1,this);
    sub2_2 = n.subscribe("/Ki_error2",1,&SubscribeAndPublish::callback2_1,this);
    sub3_2 = n.subscribe("/Ki_error3",1,&SubscribeAndPublish::callback3_1,this);
    sub4_2 = n.subscribe("/Ki_error4",1,&SubscribeAndPublish::callback4_1,this);
    sub_check = n.subscribe("/dynamixel_workbench/joint_states",1,&SubscribeAndPublish::callback_check,this);
    }
   
    void callback1_1(const rplidar_data::alpha& input)
    {
      dynamixel_workbench_msgs::DynamixelCommand srv;
      error_1_1 = input.alpha;
      error_sum1 = error_1_1 + error_1_2;
      std::string item_command = "";
      std::string item_addr = item;

      if(current1 < upper_limit + DEG2RAD(2) & error_sum1 < 0)
      {
          error_sum1 = 0;
      }

      if(current1 > lower_limit - DEG2RAD(2) & error_sum1 > 0)
      {
          error_sum1 = 0;
      }

      if (error_sum1<-PWM_Limit)
        {
          error_sum1 = -PWM_Limit;
        }

      if (error_sum1>PWM_Limit)
        {
          error_sum1 = PWM_Limit;
        }

      int ID = 2;
      srv.request.command = item_command;
      srv.request.id = ID;
      srv.request.addr_name = item_addr;
      srv.request.value = error_sum1;

      if (client.call(srv))
          {
          }
        else
          {
              ROS_ERROR("Failed to call dynamixel_command");
          }
    }

    void callback1_2(const rplidar_data::alpha& input)
    {
      dynamixel_workbench_msgs::DynamixelCommand srv;
      error_1_2 = input.alpha;
      error_sum1 = error_1_1 + error_1_2;
      std::string item_command = "";
      std::string item_addr = item;

      if(current1 < upper_limit + DEG2RAD(2) & error_sum1 < 0)
      {
          error_sum1 = 0;
      }

      if(current1 > lower_limit - DEG2RAD(2) & error_sum1 > 0)
      {
          error_sum1 = 0;
      }

      if (error_sum1<-PWM_Limit)
        {
          error_sum1 = -PWM_Limit;
        }

      if (error_sum1>PWM_Limit)
        {
          error_sum1 = PWM_Limit;
        }

      int ID = 2;
      srv.request.command = item_command;
      srv.request.id = ID;
      srv.request.addr_name = item_addr;
      srv.request.value = error_sum1;

      if (client.call(srv))
          {
          }
        else
          {
              ROS_ERROR("Failed to call dynamixel_command");
          }
    }

    void callback2_1(const rplidar_data::alpha& input)
    {
      dynamixel_workbench_msgs::DynamixelCommand srv;
      error_2_1 = input.alpha;
      error_sum2 = error_2_1 + error_2_2;
      std::string item_command = "";
      std::string item_addr = item;

      if(current2 < upper_limit + DEG2RAD(2) & error_sum2 < 0)
      {
          error_sum2 = 0;
      }

      if(current2 > lower_limit - DEG2RAD(2) & error_sum2 > 0)
      {
          error_sum2 = 0;
      }

      if (error_sum2<-PWM_Limit)
        {
          error_sum2 = -PWM_Limit;
        }

      if (error_sum2>PWM_Limit)
        {
          error_sum2 = PWM_Limit;
        }

      int ID = 3;
      srv.request.command = item_command;
      srv.request.id = ID;
      srv.request.addr_name = item_addr;
      srv.request.value = error_sum2;

      if (client.call(srv))
          {
          }
        else
          {
              ROS_ERROR("Failed to call dynamixel_command");
          }
    }

    void callback2_2(const rplidar_data::alpha& input)
    {
      dynamixel_workbench_msgs::DynamixelCommand srv;
      error_2_2 = input.alpha;
      error_sum2 = error_2_1 + error_2_2;
      std::string item_command = "";
      std::string item_addr = item;

      if(current2 < upper_limit + DEG2RAD(2) & error_sum2 < 0)
      {
          error_sum2 = 0;
      }

      if(current2 > lower_limit - DEG2RAD(2) & error_sum2 > 0)
      {
          error_sum2 = 0;
      }

      if (error_sum2<-PWM_Limit)
        {
          error_sum2 = -PWM_Limit;
        }

      if (error_sum2>PWM_Limit)
        {
          error_sum2 = PWM_Limit;
        }

      int ID = 3;
      srv.request.command = item_command;
      srv.request.id = ID;
      srv.request.addr_name = item_addr;
      srv.request.value = error_sum2;

      if (client.call(srv))
          {
          }
        else
          {
              ROS_ERROR("Failed to call dynamixel_command");
          }
    }

    void callback3_1(const rplidar_data::alpha& input)
    {
      dynamixel_workbench_msgs::DynamixelCommand srv;
      error_3_1 = input.alpha;
      error_sum3 = error_3_1 + error_3_2;
      std::string item_command = "";
      std::string item_addr = item;

      if(current3 < upper_limit + DEG2RAD(2) & error_sum3 < 0)
      {
          error_sum3 = 0;
      }

      if(current3 > lower_limit - DEG2RAD(2) & error_sum3 > 0)
      {
          error_sum3 = 0;
      }

      if (error_sum3<-PWM_Limit)
        {
          error_sum3 = -PWM_Limit;
        }

      if (error_sum3>PWM_Limit)
        {
          error_sum3 = PWM_Limit;
        }

      int ID = 4;
      srv.request.command = item_command;
      srv.request.id = ID;
      srv.request.addr_name = item_addr;
      srv.request.value = error_sum3;

      if (client.call(srv))
          {
          }
        else
          {
              ROS_ERROR("Failed to call dynamixel_command");
          }
    }

    void callback3_2(const rplidar_data::alpha& input)
    {
      dynamixel_workbench_msgs::DynamixelCommand srv;
      error_3_2 = input.alpha;
      error_sum3 = error_3_1 + error_3_2;
      std::string item_command = "";
      std::string item_addr = item;

      if(current3 < upper_limit + DEG2RAD(2) & error_sum3 < 0)
      {
          error_sum3 = 0;
      }

      if(current3 > lower_limit - DEG2RAD(2) & error_sum3 > 0)
      {
          error_sum3 = 0;
      }

      if (error_sum3<-PWM_Limit)
        {
          error_sum3 = -PWM_Limit;
        }

      if (error_sum3>PWM_Limit)
        {
          error_sum3 = PWM_Limit;
        }

      int ID = 4;
      srv.request.command = item_command;
      srv.request.id = ID;
      srv.request.addr_name = item_addr;
      srv.request.value = error_sum3;

      if (client.call(srv))
          {
          }
        else
          {
              ROS_ERROR("Failed to call dynamixel_command");
          }
    }

    void callback4_1(const rplidar_data::alpha& input)
    {
      dynamixel_workbench_msgs::DynamixelCommand srv;
      error_4_1 = input.alpha;
      error_sum4 = error_4_1 + error_4_2;
      std::string item_command = "";
      std::string item_addr = item;

      if(current4 < upper_limit + DEG2RAD(2) & error_sum4 < 0)
      {
          error_sum4 = 0;
      }

      if(current4 > lower_limit - DEG2RAD(2) & error_sum4 > 0)
      {
          error_sum4 = 0;
      }

      if (error_sum4<-PWM_Limit)
        {
          error_sum4 = -PWM_Limit;
        }

      if (error_sum4>PWM_Limit)
        {
          error_sum4 = PWM_Limit;
        }

      int ID = 5;
      srv.request.command = item_command;
      srv.request.id = ID;
      srv.request.addr_name = item_addr;
      srv.request.value = error_sum4;

      if (client.call(srv))
          {
          }
        else
          {
              ROS_ERROR("Failed to call dynamixel_command");
          }
    }

    void callback4_2(const rplidar_data::alpha& input)
    {
      dynamixel_workbench_msgs::DynamixelCommand srv;
      error_4_2 = input.alpha;
      error_sum4 = error_4_1 + error_4_2;
      std::string item_command = "";
      std::string item_addr = item;

      if(current4 < upper_limit + DEG2RAD(2) & error_sum4 < 0)
      {
          error_sum4 = 0;
      }

      if(current4 > lower_limit - DEG2RAD(2) & error_sum4 > 0)
      {
          error_sum4 = 0;
      }

      if (error_sum4<-PWM_Limit)
        {
          error_sum4 = -PWM_Limit;
        }

      if (error_sum4>PWM_Limit)
        {
          error_sum4 = PWM_Limit;
        }

      int ID = 5;
      srv.request.command = item_command;
      srv.request.id = ID;
      srv.request.addr_name = item_addr;
      srv.request.value = error_sum4;

      if (client.call(srv))
          {
          }
        else
          {
              ROS_ERROR("Failed to call dynamixel_command");
          }
    }

    void callback_check(const sensor_msgs::JointState& input)
    {
        current1 = input.position[1];
        current2 = input.position[2];
        current3 = input.position[3];
        current4 = input.position[4];
    }

private:
    ros::NodeHandle n;
    ros::ServiceClient client;
    ros::Subscriber sub1_1;
    ros::Subscriber sub1_2;
    ros::Subscriber sub2_1;
    ros::Subscriber sub2_2;
    ros::Subscriber sub3_1;
    ros::Subscriber sub3_2;
    ros::Subscriber sub4_1;
    ros::Subscriber sub4_2;
    ros::Subscriber sub_check;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "leg");
  SubscribeAndPublish NH;
  while(ros::ok())
    {
    ros::spinOnce();
    }
  return 0;
} 

