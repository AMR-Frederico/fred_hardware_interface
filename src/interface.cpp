#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <fred_hardware_interface/interface_double.h>

#define loop_hz 30.0

int main(int argc, char **argv)
{
    // Setup
    ros::init(argc, argv, "Hardware_interface_node");
    ros::NodeHandle n;

    MyRobot_double robot("right_whell","/switch/right_whell_diff","/feedback/right_velocity","/feedback/right_position",
                         "left_whell" ,"/switch/left_whell_diff" ,"/feedback/left_velocity" ,"/feedback/left_position");
                        
    controller_manager::ControllerManager cm(&robot, n);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // Control loop
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(loop_hz); // 10 Hz rate
    
    while (ros::ok())
    {
        const ros::Time     time   = ros::Time::now();
        const ros::Duration period = time - prev_time;
        
        robot.read();
        cm.update(time, period);
        robot.write(time, period);
        
        rate.sleep();
    }
    return 0;
}
