#include "ros/ros.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "rplidar_data/packet.h"
#include <cstdlib>
#include "math.h"
#include "sensor_msgs/JointState.h"

#define RAD2DATA(x) ((x)*(2047/M_PI)+2048)
#define DATA2RAD(x) ((x)-2048)*(M_PI/2047)
#define DEG2RAD(x) ((x)*M_PI/180)

#define upper_limit DEG2RAD(-10) // -10 deg
#define lower_limit DEG2RAD(45) // 45 deg
#define start_angle DEG2RAD(10) //10 deg

#define PWM_Limit 885
#define item "Goal_PWM"
#define Tolerance DEG2RAD(2) //2 deg

#define scaling 40


int error[4];
double current[4];
int t = 0;

double R;
double P;

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
    client = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
    sub = n.subscribe("/feedback",1,&SubscribeAndPublish::callback1_1,this);
    sub_2 = n.subscribe("/alpha_RP",1,&SubscribeAndPublish::callback1_2,this);
    sub_check = n.subscribe("/dynamixel_workbench/joint_states",1,&SubscribeAndPublish::callback_check,this);
    }
   
    void callback1_1(const rplidar_data::packet& input)
    {
      dynamixel_workbench_msgs::DynamixelCommand srv;
      error[0] = scaling*(input.packet[0]-P-R);
      error[1] = scaling*(input.packet[1]+P-R);
      error[2] = scaling*(input.packet[2]+P+R);
      error[3] = scaling*(input.packet[3]-P+R);
      std::string item_command = "";
      std::string item_addr = item;

      if (t == 0)
      {
          for(int i = 2; i<6 ; ++i)
          {
              srv.request.command = item_command;
              srv.request.id = i;
              srv.request.addr_name = "Goal_Position";
              srv.request.value = RAD2DATA(start_angle);
          }
          ROS_INFO("********Start********");
          sleep(2);
          t++;
          return;
      }

      for(int i = 1; i<5; ++i)
      {
          if(current[i-1] < upper_limit + Tolerance & error[i-1] < 0)
            {
              error[i-1] = 0;
            }
          if(current[i-1] > lower_limit - Tolerance & error[i-1] > 0)
            {
              error[i-1] = 0;
            }
          if (error[i-1]<-PWM_Limit)
            {
              error[i-1] = -PWM_Limit;
            }
          if (error[i-1]>PWM_Limit)
            {
              error[i-1] = PWM_Limit;
            }
          int ID = i+1;
          srv.request.command = item_command;
          srv.request.id = ID;
          srv.request.addr_name = item_addr;
          srv.request.value = error[i-1];
          if (client.call(srv))
              {
              }
            else
              {
                  //ROS_ERROR("Failed to call dynamixel_command");
              }
      }
      ROS_INFO("%d,%d,%d,%d",error[0],error[1],error[2],error[3]);
    }

    void callback_check(const sensor_msgs::JointState& input)
    {
        for(int i = 0; i<4; ++i)
        {
            current[i] = input.position[i+1];
        }
    }
    void callback1_2(const rplidar_data::packet& input)
    {
      R = input.packet[0];
      P = input.packet[1];
    }

private:
    ros::NodeHandle n;
    ros::ServiceClient client;
    ros::Subscriber sub;
    ros::Subscriber sub_2;
    ros::Subscriber sub_check;
};

int main(int argc, char **argv)
{
  printf("\n");
  printf("\n");
  printf("This is leg_controller node!\n");
  printf("\n");
  printf("made by Bum Geun Park, 2021.05\n");
  printf("\n");
  printf("\x1b[31m""Input1 : /packet\n""\x1b[0m");
  printf("\n");
  printf("\x1b[31m""Input2 : /packet\n""\x1b[0m");
  printf("\n");
  printf("\x1b[31m""Input3 : /JointState\n""\x1b[0m");
  printf("\n");
  printf("\x1b[34m""Output : /packet\n""\x1b[0m");
  printf("\n");
  printf("\x1b[37m""*****PID_controller*****\n""\x1b[0m");
  ros::init(argc, argv, "leg");
  SubscribeAndPublish NH;
  while(ros::ok())
    {
    ros::spinOnce();
    }
  return 0;
} 
