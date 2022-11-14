#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <iostream>
#include <string>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>


class MyRobot_double :  public hardware_interface::RobotHW
{
public:
    MyRobot_double(std::string joint_name_0,std::string pub_topic_0, std::string sub_topic_velocity_0,std::string sub_topic_position_0,
                   std::string joint_name_1,std::string pub_topic_1, std::string sub_topic_velocity_1,std::string sub_topic_position_1)
    {        
        //Init
        ros::NodeHandle n;
        cmd_controller[0] = 0;
        cmd_controller[1] = 0;

        //Publish and subscribe in robot hardware (Switch)
        pub[0] = n.advertise<std_msgs::Float64>(pub_topic_0, 1);
        sub_feedback_velocity[0] = n.subscribe<std_msgs::Float64>(sub_topic_velocity_0, 1, &MyRobot_double::sub_callback_feedback_velocity_0, this);
        sub_feedback_position[0] = n.subscribe<std_msgs::Float64>(sub_topic_position_0, 1, &MyRobot_double::sub_callback_feedback_position_0, this);

        pub[1] = n.advertise<std_msgs::Float64>(pub_topic_1, 1);
        sub_feedback_velocity[1] = n.subscribe<std_msgs::Float64>(sub_topic_velocity_1, 1, &MyRobot_double::sub_callback_feedback_velocity_1, this);
        sub_feedback_position[1] = n.subscribe<std_msgs::Float64>(sub_topic_position_1, 1, &MyRobot_double::sub_callback_feedback_position_1, this);

        //Connect and register the joint state interface
        hardware_interface::JointStateHandle state_handle_a(joint_name_0, &pos_controller[0], &vel_controller[0], &eff_controller[0]);
        jnt_state_interface.registerHandle(state_handle_a);

        hardware_interface::JointStateHandle state_handle_b(joint_name_1, &pos_controller[1], &vel_controller[1], &eff_controller[1]);
        jnt_state_interface.registerHandle(state_handle_b);

        registerInterface(&jnt_state_interface);

        //Connect and register the joint velocity interface
        hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle(joint_name_0), &cmd_controller[0]);
        jnt_vel_interface.registerHandle(pos_handle_a);

        hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle(joint_name_1), &cmd_controller[1]);
        jnt_vel_interface.registerHandle(pos_handle_b);

        registerInterface(&jnt_vel_interface);

    }; // Setup robot
    
    // Talk to HW
    void read()
    {
        //Print status data
        /*
        std::cout<<"POS_0_controller: "<<(pos_controller[0])<<std::endl;
        std::cout<<"VEL_0_controller: "<<(vel_controller[0])<<std::endl;
        std::cout<<"EFF_0_controller: "<<(eff_controller[0])<<std::endl;
        std::cout<<"POS_1_controller: "<<(pos_controller[1])<<std::endl;
        std::cout<<"VEL_1_controller: "<<(vel_controller[1])<<std::endl;
        std::cout<<"EFF_1_controller: "<<(eff_controller[1])<<std::endl;
        */
    };
    void write(ros::Time time, ros::Duration period)
    {
        //Send command to hardware
        msg_pub[0].data = cmd_controller[0];
        msg_pub[1].data = cmd_controller[1];        

        pub[0].publish(msg_pub[0]);
        pub[1].publish(msg_pub[1]);

        //Print command data
        /*
        std::cout<<"CMD_controller_0: "<<(cmd_controller[0])<<std::endl;
        std::cout<<"MSG_cmd_0: "<<(msg_pub[0].data)<<std::endl;

        std::cout<<"CMD_controller_1: "<<(cmd_controller[1])<<std::endl;
        std::cout<<"MSG_cmd_1: "<<(msg_pub[1].data)<<std::endl<<std::endl;
        */
    };

    //Callbacks
    void sub_callback_feedback_velocity_0(const std_msgs::Float64::ConstPtr& msg )
    {
        vel_controller[0] = msg->data;
    };
    void sub_callback_feedback_velocity_1(const std_msgs::Float64::ConstPtr& msg )
    {
        vel_controller[1] = msg->data;
    };
    void sub_callback_feedback_position_0(const std_msgs::Float64::ConstPtr& msg )
    {
        pos_controller[0] = msg->data;
    };
    void sub_callback_feedback_position_1(const std_msgs::Float64::ConstPtr& msg )
    {
        pos_controller[1] = msg->data;

    };
        

private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;

    ros::NodeHandle n;

    ros::Publisher pub[2];
    std_msgs::Float64 msg_pub[2];
    ros::Subscriber sub_feedback_velocity[2];
    ros::Subscriber sub_feedback_position[2];
    
    double cmd_controller[2];
    double pos_controller[2];
    double vel_controller[2];
    double eff_controller[2]; 

       
};