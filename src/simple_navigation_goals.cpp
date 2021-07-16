
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h> 
#include <actionlib/client/simple_action_client.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <string>
#include <sstream>
using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
void set_waypoint(float x, float y, float w, int exit){
    /*tell the action client that we want to spin a thread by default which is comunicated 
    through "move_base*/
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

    tf2::Quaternion myQuaternion;

    myQuaternion.setRPY(0, 0, w);

    fire1.target_pose.pose.position.x = x; 
    fire1.target_pose.pose.position.y = y;
    fire1.target_pose.pose.orientation.x = myQuaternion.getX();
    fire1.target_pose.pose.orientation.y = myQuaternion.getY();
    fire1.target_pose.pose.orientation.z = myQuaternion.getZ();
    fire1.target_pose.pose.orientation.w = myQuaternion.getW();
    // fire1.target_pose.pose.orientation.w = w;
    ROS_INFO("Sending goal"); 
    std::system("rostopic pub /led_input std_msgs/Float64 '0' --once");  

    // Sending the fire1 to the move_base node for processing 
    ac.sendGoal(fire1);
    // Block until the move_base node is finished processing goal 
    ac.waitForResult(); 
    // Checking if the goal is successful 
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the base moved 1 meter forward");
        if (exit==0){
            std::system("rostopic pub /led_input std_msgs/Float64 '1' --once");
        }
    } 
    else ROS_INFO("The base failed to move forward 1 meter for some reason");

}

bool is_digits(const std::string &str) {
    return str.find_first_not_of("0123456789") == std::string::npos;
}

bool is_float( string myString ) {
    std::istringstream iss(myString);
    float f;
    iss >> noskipws >> f; // noskipws considers leading whitespace invalid
    // Check the entire string was consumed and if either failbit or badbit is set
    return iss.eof() && !iss.fail(); 
}

int num_fires_func(){
    int num_fires, error;
    std::string str_fires;
    std::cout << "Emergency! Your building is on fire!"<<endl;
    
    do{
        error=0;
        std::cout << "Please enter the number of fires in the building. The last waypoint for the exit has been set for you!"<<endl;
        // std::cin >> num_fires;
        std::getline(cin,str_fires);
        
        if(!is_digits(str_fires))
        {
            std::cout << "Invalid entry! Please try again!"<<endl;
            error=1;
            // cin.clear();
            // cin.ignore(80, '\n');
            }
        else if(str_fires==""){
            error=1;
        }
        else{
            num_fires=stoi(str_fires);

            if(num_fires==0){
                std::cout << "There are 0 fires in the building. Please try again!"<<endl;
                error=1;
            }
            else{
                std::cout << "There are "<<num_fires<<" fires in the building!"<<endl;
                error=0;
            }
            
        }    
    }while(error==1);

    return num_fires;   
}

float xy_float(const char* val, int firenum){
    std::string str_xy;
    int error;
    float xy;

    do{
        std::cout << "Enter the "<< val <<" value of waypoint "<<firenum<<endl;
        std::getline(cin,str_xy);
        if(!is_float(str_xy))
        {
            std::cout << "Invalid entry! Please try again!"<<endl;
            error=1;
            // cin.clear();
            // cin.ignore(80, '\n');
            }
        else if(str_xy == ""){
            error=1;            
        }
        else{
            xy = std::stof (str_xy);
            if (xy<0||xy>3.9624){
                cout<<"Out of bounds. Please try again!"<<endl;
                error=1;
            }
            else{
                error=0;
            }

        }
    }while (error!=0);
    return xy;
}

float w_float(const char* val, int firenum){
    std::string str_w;
    int error;
    float w;
    do{
        std::cout << "Enter the "<< val <<" value of waypoint "<<firenum<<endl;
        std::getline(cin,str_w);
        if(!is_float(str_w))
        {
            std::cout << "Invalid entry! Please try again!"<<endl;
            error=1;
            // cin.clear();
            // cin.ignore(80, '\n');
            }
        else if(str_w == ""){
            error=1;            
        }
        else{
            w = std::stof (str_w);
            error=0;
            }
    }while (error!=0);
    return w;
}

int main(int argc, char** argv){

    // int count=0;
    // int again = 0;
    // char confirm;
    // int  error;
    // float x, y, w;
    // char c;
    // std::string str_x, str_y, str_w;

    // int total_fires=num_fires_func();

    // float fire_arr[total_fires][3];         // declaration of a new array
    
    // do{
    //     // float dimensions[3];
    //     x = xy_float("x", count+1);
    //     y = xy_float("y", count+1);
    //     w = w_float("w", count+1);
    //     fire_arr[count][0] = x;
    //     fire_arr[count][1] = y;
    //     fire_arr[count][2] = w;


    //     std::cout<<"Sanity check for x in fire "<<count+1<<":"<<fire_arr[count][0]<<endl;
    //     std::cout<<"Sanity check for y in fire "<<count+1<<":"<<fire_arr[count][1]<<endl;
    //     std::cout<<"Sanity check for w in fire "<<count+1<<":"<<fire_arr[count][2]<<endl;

    //     count++;
    // }
    // while(count<total_fires);

    ros::init(argc, argv, "simple_navigation_goals");
    // int wp_count = 0;

    // do{
    //     set_waypoint(fire_arr[wp_count][0], fire_arr[wp_count][1], fire_arr[wp_count][2], 0);
    //     wp_count++;
    // }
    // while(wp_count<total_fires);
            

    // set_waypoint(1.4048,0.3048,1.0);
    // set_waypoint(1.4048,1.4048,1.0);
    // set_waypoint(0.3048,0.3048,0.0, 1);
    
    set_waypoint(2.7,0.3,0,0);
    set_waypoint(0.9,2.1,1.57,0);
    set_waypoint(3.3,3.6,0,0);
    set_waypoint(3.3,1.5,-1.57,0);
    set_waypoint(3.9,0.3,0,1);


    //3.9 0.3 1 is the exit
    
    return 0; 
}