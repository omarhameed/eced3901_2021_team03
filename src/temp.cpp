#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h> 
#include <actionlib/client/simple_action_client.h>
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <fstream>
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
void set_waypoint(float x, float y, float w){
    /*tell the action client that we want to spin a thread by default which is comunicated 
    through "move_base*/
    float foot = 0.3048;
    MoveBaseClient ac("move_base", true);
    //wait for the action server to come up and is ready to begin processing goals
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up"); 
        }
    move_base_msgs::MoveBaseGoal fire1;
    // we'll send a fire1 to the robot to move 1.1 meter forward from the orgin (0.3048,0.3048)
    fire1.target_pose.header.frame_id = "map"; //instead of base_link
    fire1.target_pose.header.stamp = ros::Time::now();
    /*Making the base move 1 meter forward in x-dirction and 0 in the y-dirction 
    from the orgin (0.3048) in the "map" coordinate frame.*/
    fire1.target_pose.pose.position.x = x; 
    fire1.target_pose.pose.position.y = y; 
    fire1.target_pose.pose.orientation.w = w;
    ROS_INFO("Sending goal"); 
    std::system("rostopic pub /led_input std_msgs/Float64 '0' --once");  

    // Sending the fire1 to the move_base node for processing 
    ac.sendGoal(fire1);
    // Block until the move_base node is finished processing goal 
    ac.waitForResult(); 
    // Checking if the goal is successful 
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the base moved 1 meter forward");
        std::system("rostopic pub /led_input std_msgs/Float64 '1' --once");
    } 
    else ROS_INFO("The base failed to move forward 1 meter for some reason");

}

int main(int argc, char** argv){
    int num_fires, x, y, w;
    int count=0;
    std::cout << "Emergency! Your building is on fire! \n";
    std::cout << "Please enter the number of fires in the building. The last waypoint for the exit has been set for you! \n";
    std::cin >> num_fires;
    std::cout << "Wow! Here are the number of fires in the building: " << num_fires;
    
    float fire_arr[num_fires][3];         // declaration of a new array
  
    while(count<num_fires){
        // float dimensions[3];
        std::cout << "Enter the x value of the waypoint: \n";
        std::cin >> x;
        std::cout << "Enter the y value of the waypoint: \n";
        std::cin >> y;
        std::cout << "Enter the w value of the waypoint: \n";
        std::cin >> w;
        fire_arr[count][0] = x;
        fire_arr[count][1] = y;
        fire_arr[count][2] = w;
        // fire_arr[count] = dimensions;
        count++;
    }
    ros::init(argc, argv, "simple_navigation_goals");
    int wp_count = 0;
    while(wp_count<num_fires){
        set_waypoint(fire_arr[wp_count][0], fire_arr[wp_count][1], fire_arr[wp_count][2]);
        wp_count++;
    }

    set_waypoint(2.7,0.3,1.0);
    //3.9 0.3 1 is the exit
    
    return 0; 
}
