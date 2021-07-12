
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h> 
#include <actionlib/client/simple_action_client.h>
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <fstream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals");
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
    fire1.target_pose.pose.position.x = 2.7; 
    fire1.target_pose.pose.position.y = 0.3; 
    fire1.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Sending fire1"); 
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

    move_base_msgs::MoveBaseGoal fire2;
    // turn 90 degrees ccw (x = 0.3048 + 1.1)(y = 0.3048 + 1.1 )
    fire2.target_pose.header.frame_id = "map"; 
    fire2.target_pose.header.stamp = ros::Time::now();
    fire2.target_pose.pose.position.x = 0.9; 
    fire2.target_pose.pose.position.y = 2.1; 
    fire2.target_pose.pose.orientation.w = 1;
    ROS_INFO("Sending goal");
    std::system("rostopic pub /led_input std_msgs/Float64 '0' --once");  
    ac.sendGoal(fire2);
    ac.waitForResult(); 
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the base moved 1 meter forward");
        std::system("rostopic pub /led_input std_msgs/Float64 '1' --once");
    }
    else ROS_INFO("The base failed to turn for some reason");


    move_base_msgs::MoveBaseGoal fire3;
    //turn 90 degrees ccw (x = 0.3048 )(y = 0.3048 + 1.1 )
    fire3.target_pose.header.frame_id = "map"; 
    fire3.target_pose.header.stamp = ros::Time::now();
    fire3.target_pose.pose.position.x = 3.3; 
    fire3.target_pose.pose.position.y = 3.6; 
    fire3.target_pose.pose.orientation.w = 1;
    ROS_INFO("Sending goal"); 
    std::system("rostopic pub /led_input std_msgs/Float64 '0' --once");  
    ac.sendGoal(fire3);
    ac.waitForResult(); 
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the base moved 1 meter forward");
        std::system("rostopic pub /led_input std_msgs/Float64 '1' --once");
    }
    else ROS_INFO("The base failed to turn for some reason");

    move_base_msgs::MoveBaseGoal fire4;
    //turn 90 degrees ccw (x = 0.3048)(y = 0.3048)
    fire4.target_pose.header.frame_id = "map"; 
    fire4.target_pose.header.stamp = ros::Time::now();
    fire4.target_pose.pose.position.x =3.3; 
    fire4.target_pose.pose.position.y = 1.5; 
    fire4.target_pose.pose.orientation.w = 1;
    ROS_INFO("Sending goal");
    std::system("rostopic pub /led_input std_msgs/Float64 '0' --once");  
    ac.sendGoal(fire4);
    ac.waitForResult(); 
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the base moved 1 meter forward");
        std::system("rostopic pub /led_input std_msgs/Float64 '1' --once");
    }
    else ROS_INFO("The base failed to turn for some reason");

    move_base_msgs::MoveBaseGoal exit;
    //turn 90 degrees ccw (x = 0.3048)(y = 0.3048)
    exit.target_pose.header.frame_id = "map"; 
    exit.target_pose.header.stamp = ros::Time::now();
    exit.target_pose.pose.position.x =3.9; 
    exit.target_pose.pose.position.y = 0.3; 
    exit.target_pose.pose.orientation.w = 1;
    ROS_INFO("Sending goal"); 
    std::system("rostopic pub /led_input std_msgs/Float64 '0' --once");  
    ac.sendGoal(exit);
    ac.waitForResult(); 
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO("Hooray, the base turned.");
    else ROS_INFO("The base failed to turn for some reason");
    
    
    return 0; 
}