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
#define start_angle DEG2RAD(17.5) //17.5 deg

#define PWM_Limit 885
#define item "Goal_PWM"
#define Tolerance DEG2RAD(0.1) // 0.1 deg
#define initiate_power 50 // power 50

#define scaling 1


int error[4];
double current[4];
int t1 = 0;
int t2 = 0;
int t3 = 0;
int t4 = 0;

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
        if (t1==0 && t2==0 && t3==0 && t4==0)
        {
            ROS_INFO("Do initiate!");
            return;
        }
        dynamixel_workbench_msgs::DynamixelCommand srv;
        std::string item_command = "";
        std::string item_addr = item;
        srv.request.command = item_command;
        srv.request.addr_name = item_addr;

        error[0] = scaling*(input.packet[0]-P-R);
        error[1] = scaling*(input.packet[1]+P-R);
        error[2] = scaling*(input.packet[2]+P+R);
        error[3] = scaling*(input.packet[3]-P+R);

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
            srv.request.id = ID;
            srv.request.value = error[i-1];
            client.call(srv);
        }
        ROS_INFO("%d,%d,%d,%d",error[0],error[1],error[2],error[3]);
    }

    void callback_check(const sensor_msgs::JointState& input)
    {
        dynamixel_workbench_msgs::DynamixelCommand srv;
        std::string item_command = "";
        std::string item_addr = item;
        srv.request.command = item_command;
        srv.request.addr_name = item_addr;

        if (t1 == 0)
        {
            ROS_INFO("1_leg");
            srv.request.id = 2;
            current[0] = input.position[1];
            ROS_INFO("current_1 :%f",current[0]);
            srv.request.value = initiate_power*abs(start_angle-current[0])/(start_angle-current[0]);
            client.call(srv);
            if(start_angle-Tolerance<current[0]&&current[0]<start_angle+Tolerance)
            {
                srv.request.value = 0;
                client.call(srv);
                t1++;
                ROS_INFO("Finish 1");
            }
            ROS_INFO("Error-> leg1: %f, leg2: %f, leg3: %f, leg4: %f",abs(start_angle-current[0]),abs(start_angle-current[1]),abs(start_angle-current[2]),abs(start_angle-current[3]));
        }

        if (t2 == 0)
        {
            ROS_INFO("2_leg");
            srv.request.id = 3;
            current[1] = input.position[2];
            ROS_INFO("current_2 :%f",current[1]);
            srv.request.value = initiate_power*abs(start_angle-current[1])/(start_angle-current[1]);
            client.call(srv);
            if(start_angle-Tolerance<current[1]&&current[1]<start_angle+Tolerance)
            {
                srv.request.value = 0;
                client.call(srv);
                t2++;
                ROS_INFO("Finish 2");
            }
            ROS_INFO("Error-> leg1: %f, leg2: %f, leg3: %f, leg4: %f",abs(start_angle-current[0]),abs(start_angle-current[1]),abs(start_angle-current[2]),abs(start_angle-current[3]));
        }

        if (t3 == 0)
        {
            ROS_INFO("3_leg");
            srv.request.id = 4;
            current[2] = input.position[3];
            ROS_INFO("current_3 :%f",current[2]);
            srv.request.value = initiate_power*abs(start_angle-current[2])/(start_angle-current[2]);
            client.call(srv);
            if(start_angle-Tolerance<current[2]&&current[2]<start_angle+Tolerance)
            {
                srv.request.value = 0;
                client.call(srv);
                t3++;
                ROS_INFO("Finish 3");
            }
            ROS_INFO("Error-> leg1: %f, leg2: %f, leg3: %f, leg4: %f",abs(start_angle-current[0]),abs(start_angle-current[1]),abs(start_angle-current[2]),abs(start_angle-current[3]));
        }

        if (t4 == 0)
        {
            ROS_INFO("4_leg");
            srv.request.id = 5;
            current[3] = input.position[4];
            ROS_INFO("current_4 :%f",current[3]);
            srv.request.value = initiate_power*abs(start_angle-current[3])/(start_angle-current[3]);
            client.call(srv);
            if(start_angle-Tolerance<current[3]&&current[3]<start_angle+Tolerance)
            {
                srv.request.value = 0;
                client.call(srv);
                t4++;
                ROS_INFO("Finish 4");
            }
            ROS_INFO("Error-> leg1: %f, leg2: %f, leg3: %f, leg4: %f",abs(start_angle-current[0]),abs(start_angle-current[1]),abs(start_angle-current[2]),abs(start_angle-current[3]));
        }

        if(t1>0 && t2>0 && t3>0 && t4>0)
        {
            for(int k = 0; k<4; ++k)
            {
                current[k] = input.position[k+1];
            }
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
    R = 0.0;
    P = 0.0;
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
    printf("\x1b[34m""Output : /Leg_motor_service\n""\x1b[0m");
    printf("\n");
    printf("\x1b[37m""*****Leg_controller*****\n""\x1b[0m");
    ros::init(argc, argv, "leg");
    SubscribeAndPublish NH;
    while(ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
} 
