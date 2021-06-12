#include "ros/ros.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "rplidar_data/alpha.h"
#include <cstdlib>
#include "math.h"
#include "sensor_msgs/JointState.h"

#define RAD2DATA(x) ((x)*(2047/M_PI)+2048)
#define DATA2RAD(x) ((x)-2048)*(M_PI/2047)
#define DEG2RAD(x) ((x)*M_PI/180)
#define Lidar_motor 1
#define center_point 2048 // 0 deg
#define motor_Limit DEG2RAD(60) // 60 deg
#define Tolerance (DEG2RAD(1.75))*(2047/M_PI)  // 1.75 deg

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
      int ID = Lidar_motor;
      srv.request.command = item_command;
      srv.request.id = ID;
      srv.request.addr_name = item_addr;

      //safe guard
      if (pseudo_Angle<center_point || pseudo_Angle>(int)RAD2DATA(motor_Limit))
        {
          ROS_INFO("Exceed motor limit!");
          srv.request.value = pseudo_Angle;
          client.call(srv);
          return;
        }
      if (RAD2DATA(current)>pseudo_Angle || RAD2DATA(current)<center_point-(pseudo_Angle-center_point))
      {
          srv.request.value = pseudo_Angle;
          client.call(srv);
      }

      //check///////////////////////////////////////////////////////////////////////////////
      printf("\x1b[34m""[checking operation]""\x1b[0m");
      ROS_INFO("[Target,current]: [%d~%d] ,%f",pseudo_Angle,center_point-(pseudo_Angle-center_point),RAD2DATA(current));
      ROS_INFO("[pseudo_Angle-center_point]: %d",pseudo_Angle-center_point);
      ROS_INFO("[RAD2DATA(DEG2RAD(30))]: %f",RAD2DATA(DEG2RAD(30)));



      //upper limit point
      if(sqrt(((int)RAD2DATA(current)-pseudo_Angle)*((int)RAD2DATA(current)-pseudo_Angle))<Tolerance) // number 1
      {
          ROS_INFO("upper limit!");
          pseudo_Angle = Limit_Angle;
          srv.request.value = center_point-(pseudo_Angle-center_point);
          ROS_INFO("go to lower limit: %f!",center_point-(pseudo_Angle-center_point));
          t = 0;
          client.call(srv);
      }

      //lower limit point
      if(sqrt((center_point-(pseudo_Angle-center_point)-(int)RAD2DATA(current))*(center_point-(pseudo_Angle-center_point)-(int)RAD2DATA(current)))<Tolerance) // number 2
      {
          ROS_INFO("lower limit!");
          srv.request.value = pseudo_Angle;
          ROS_INFO("go to lower limit: %f!",pseudo_Angle);
          t = 1;
          client.call(srv);
      }

      //center point and update limit
      if(sqrt((center_point-(int)RAD2DATA(current))*(center_point-(int)RAD2DATA(current)))<Tolerance && t==1 ) // number 3
      {
          double phi = atan(1 / sqrt( (1/tan(input.alpha)) * (1/tan(input.alpha)) - 1 ));
          Limit_Angle = (int)RAD2DATA(phi);
          if(start==0)
          {
              pseudo_Angle = Limit_Angle;
              srv.request.value = pseudo_Angle;
              ROS_INFO("go to lower limit: %f!",pseudo_Angle);
              ++start;
              client.call(srv);
          }
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
    Limit_Angle = (int)(RAD2DATA(DEG2RAD(30)));
    pseudo_Angle = (int)(RAD2DATA(DEG2RAD(30)));
    printf("\n");
    printf("\n");
    printf("This is Lidar_servo_controller node!\n");
    printf("\n");
    printf("made by Bum Geun Park, 2021.05\n");
    printf("\n");
    printf("\x1b[31m""Input1 : /alpha\n""\x1b[0m");
    printf("\n");
    printf("\x1b[31m""Input2 : /JointState\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output : /Lidar_motor_service\n""\x1b[0m");
    printf("\n");
    printf("\x1b[37m""*****Lidar_servo_controller*****\n""\x1b[0m");
    ros::init(argc, argv, "write");
    SubscribeAndPublish NH;
    while(ros::ok())
      {
        ros::spinOnce();
      }
    return 0;
} 
