"""
Date: May 28, 2021
Authors: Usman Kamran, Ryan Brownlee, Omar Abdel Hameed
Course: ECED 3901
Professor: Dr Vincent Sieben
Purpose:
This program enables a counterclockwise right angle triangle trajectory followed by a clockwise right angle triangle trajectory.
It is the first Design Task for ECED 3901.

Credits:
This program was repurposed from the original program created by Gabriel Urbain (gabriel.urbain@uguent.be).
The original source code can be found here: https://gist.github.com/gurbain/c833e9858dd3e5fc4e30d6b1a305667b

"""

import math
import rospy as ros
import sys
import time

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler



__author__ = "Team 03" 
__copyright__ = "Copyright 2021, ECED  3901, Dalhousie University"

__license__ = "MIT" 
__version__ = "1.0" 
__maintainer__ = "Team 03"
__email__ = "us311212@dal.ca" 
__status__ = "Education" 
__date__ = "May 28th, 2021"


class TriangleMove(object):
    """
    This class is an abstract class to control a square trajectory on the turtleBot.
    It mainly declare and subscribe to ROS topics in an elegant way.
    """

    def __init__(self):

        # Declare ROS subscribers and publishers
        self.node_name = "triangle_move" #names node
        self.odom_sub_name = "/odom" #names odom sub command
        self.vel_pub_name = "/cmd_vel" #names velocity sub command
        self.vel_pub = None 
        self.odometry_sub = None  

        # ROS params
        self.pub_rate = 0.1 #sets the rate the ROS publishes
        self.queue_size = 2 #sets the ROS que size

        # Variables containing the sensor information that can be used in the main program
        self.odom_pose = None

    def start_ros(self):

        # Create a ROS node with a name for our program
        ros.init_node(self.node_name, log_level=ros.INFO)

        # Define a callback to stop the robot when we interrupt the program (CTRL-C)
        ros.on_shutdown(self.stop_robot)

        # Create the Subscribers and Publishers
        self.odometry_sub = ros.Subscriber(self.odom_sub_name, Odometry, callback=self.__odom_ros_sub, queue_size=self.queue_size)
        self.vel_pub = ros.Publisher(self.vel_pub_name, Twist, queue_size=self.queue_size)

    def stop_robot(self):

        # Get the initial time
        self.t_init = time.time()

        # We publish for a second to be sure the robot receive the message
        while time.time() - self.t_init < 1 and not ros.is_shutdown():
            
            self.vel_ros_pub(Twist())
            time.sleep(self.pub_rate)

        sys.exit("The process has been interrupted by the user!")

    def move(self):
        """ To be surcharged in the inheriting class"""

        while not ros.is_shutdown(): 
            time.sleep(1)  # gives the program time to accept data

    def __odom_ros_sub(self, msg): #subscribes to itself to find its pose from odom

        self.odom_pose = msg.pose.pose

    def vel_ros_pub(self, msg): # publishes velocity

        self.vel_pub.publish(msg)

   



class TriangleMoveVel(TriangleMove):
    """
    This class implements a open-loop square triangular trajectory based on velocity control. HOWTO:
     - Start the sensors on the turtlebot:
            $ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch 
     - Start this node on your computer:
            $ python eced3901_dt1.py vel
    """

    def __init__(self):
        
        super(TriangleMoveVel, self).__init__()

    def go_forward(self, duration, speed):

        # Get the initial time
        self.t_init = time.time()

        # Set the velocity forward and wait (do it in a while loop to keep publishing the velocity)
        while time.time() - self.t_init < duration and not ros.is_shutdown():

            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = 0
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)

    def turn(self, duration, ang_speed):

         # Get the initial time
        self.t_init = time.time()

        # Set the velocity forward and wait 2 sec (do it in a while loop to keep publishing the velocity)
        while time.time() - self.t_init < duration and not ros.is_shutdown():

            msg = Twist()
            msg.linear.x = 0
            msg.angular.z = ang_speed
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)

class TriangleMoveOdom(TriangleMove):
    """
    This class implements a semi closed-loop square triangular trajectory based on relative position control,
    where only odometry is used. HOWTO:
     - Start the sensors on the turtlebot:
            $ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch 
     - Start this node on your computer:
            $ python eced3901_dt1.py odom
    """

    def __init__(self):


        super(TriangleMoveOdom, self).__init__()

        self.pub_rate = 0.1

    def get_z_rotation(self, orientation): #converts input quaternion coordinates into eulerian roll pitch and yaw

        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        print (roll, pitch, yaw)
        return yaw
        
    def move_of(self, d, speed=0.1): #moves the vehicle d distance at speed=0.1, speed is low to maximize accuracy

        x_init = self.odom_pose.position.x
        y_init = self.odom_pose.position.y

        print ("X_Init, Y_Init:", x_init,y_init) #prints x_init and y_init, which is the x and y position

        # Set the velocity forward until distance is reached
        while math.sqrt((self.odom_pose.position.x - x_init)**2 + \
             (self.odom_pose.position.y - y_init)**2) < d and not ros.is_shutdown():

	#prints out the distance the robot has moved out of the total distance requested
            sys.stdout.write("\r [MOVE] The robot has moved of {:.2f}".format(math.sqrt((self.odom_pose.position.x - x_init)**2 + \
            (self.odom_pose.position.y - y_init)**2)) +  "m over " + str(d) + "m")
            sys.stdout.flush()

            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = 0
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)

        sys.stdout.write("\n")

    # Convert any negative angles into their positive rad equivalents
    def convert_pose_estimate(self, pose):
        if (pose < (math.pi)/2 and pose > 0):
            return pose
        elif (pose > (math.pi)/2):
            return pose
        elif (pose < 0):
            positivePose = pose + 2*(math.pi) 
            return positivePose
        
    
    def turn_of(self, a, ang_speed=0.2):

        # Convert the orientation quaternion message to Euler angles
        a_init = self.convert_pose_estimate(self.get_z_rotation(self.odom_pose.orientation))
        print (a_init)
        print("This is our check for initial orientation: {:.2f}".format(self.get_z_rotation(self.odom_pose.orientation)))
        
        # Set the angular velocity forward until angle is reached
        while (abs(self.convert_pose_estimate(self.get_z_rotation(self.odom_pose.orientation)) - a_init)) < a and not ros.is_shutdown():
            sys.stdout.write("\r [TURN] The robot has turned of {:.2f}".format(self.get_z_rotation(self.odom_pose.orientation) - \
                a_init) + "rad over {:.2f}".format(a) + "rad")
            
            sys.stdout.flush()
            print (self.get_z_rotation(self.odom_pose.orientation) - a_init)

            msg = Twist()
            msg.angular.z = ang_speed
            msg.linear.x = 0
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)

        sys.stdout.write("\n")

    def move(self):

        # Wait that our python program has received its first messages
        while self.odom_pose is None and not ros.is_shutdown():
            time.sleep(0.1)

        # Implement main instructions
        # Counterclockwise right triangle movement implementation.
        self.move_of(0.5)
        self.turn_of(3*(math.pi)/4)
        self.move_of(math.sqrt(2)/2)
        self.turn_of(3*(math.pi)/4)
        self.move_of(0.5)
        self.turn_of(math.pi)
        self.turn_of((math.pi)/2)
        # Clockwise right triangle movement implementation.
        # self.turn_of(math.pi/2)
        self.move_of(0.5)
        self.turn_of(math.pi)
        self.turn_of((math.pi)/4)
        self.move_of(math.sqrt(2)/2)
        self.turn_of(math.pi)
        self.turn_of(math.pi)
        self.move_of(0.5)
        self.turn_of(math.pi)
        self.stop_robot()




if __name__ == '__main__':

    # Choose the example you need to run in the command line
    if len(sys.argv) > 1:

        if sys.argv[1] == "vel":
            r = TriangleMoveVel()

        elif sys.argv[1] == "odom":
            r = TriangleMoveOdom()

        else:
            sys.exit(-1)

    else:
        sys.exit(-1)

    # Listen and Publish to ROS + execute moving instruction
    r.start_ros()
    r.move()
