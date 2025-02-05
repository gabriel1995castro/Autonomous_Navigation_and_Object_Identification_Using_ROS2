#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import time


class BugNavigationRobot (Node):
    def __init__(self):
        
        super().__init__('bug_navegator')
       
        self.publish = self.create_publisher (Twist, 'cmd_vel',5)
        self.assinant = self.create_subscription (LaserScan, 'scan',self.return_lidar, 10)
        self.odom_subscription = self.create_subscription (Odometry, '/odom', self.odom_callback, 10)
        self.atu_position = np.array ([0,0])
    
        self.security_distance = 0.3
        self.conturn_obstacle = False
        self.see_obstacle = None
        self.detectd_wall = False
        
    def odom_callback (self, msg):
        
        self.atu_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ]) 
        
           
    def return_lidar (self,msg):
       
        twist = Twist()
        ranges =np.array (msg.ranges)
        min_distance = np.nanmin (ranges)
        
        if not self.conturn_obstacle :
           
            if min_distance < self.security_distance:
               
                if  self.a_wall_detected (ranges):
                    self.detectd_wall = True
                    self.rotate_comand ()
                    
                else:
                    self.dori_movement = self.atu_position.copy()
                    self.conturn_obstacle =True
          
            else:
                twist.linear.x = 0.4
                
        else:
            
            twist.linear.x = 0.2
            twist.linear.z = 0.4
            
            if self.return_on_line ():
                
                self.conturn_obstacle = False
       
        self.publish.publish(twist)
        
    def a_wall_detected  (self, ranges):
        
        obstacle_format = np.array(ranges[0:10] + ranges[-10:])
        
        return np.nanmean( obstacle_format) < self.security_distance * 2    
    
    def   rotate_comand (self):
        
        twist = Twist ()
        twist.angular.z = np.pi/6
        self.publish.publish(twist)
        time.sleep (1)
        twist.angular.z = 0.0
        self.publish.publish(twist)
        self.detectd_wall = False

    def return_on_line (self):
        
        vect_dest = self.goal -self.start_position
        vect_robot = self.atu_position - self.start_position
        
        return np.abs(np.cross(vect_dest,vect_robot)) < 0.1
    
    
def main (args = None):
        
        rclpy.init(args=args)
        node = BugNavigationRobot()
        rclpy.spin (node)
        node.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    
    main()
        
    
        
