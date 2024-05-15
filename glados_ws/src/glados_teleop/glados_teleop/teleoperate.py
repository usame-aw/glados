import math 
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.duration import Duration

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy 

from glados_teleop.communicator import Serial_Talker 

class Teleoperate(Node):
    
    def __init__(self):
        
        super().__init__('teleoperator_node')
        
        # self.talker = Serial_Talker() 
        
        self.clock = Clock()
        
        self.cmd_sub_ = self.create_subscription(
            Twist, 'cmd_vel', self.command_actuators, 10
        )   
        
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_cb, 10
        )
        
        self.lidar_tf_broadcast = TransformBroadcaster(self)

        self.rotate_lidar = False

        # Diff Drive Params
        self.b  = 0.1795
        self.r = 0.035
        self.rpm = 120
        self.max_speed = self.rpm * (1/60) * 2 * math.pi * self.r
        
        self.epsilon_vel = 0.001
        self.epsilon_omega = 0.005
        
        self.pwm_speed_ratio = 65535 / self.max_speed  

        # Lidar Params
        self.wait_rotation = Duration(seconds=0.4
                                      
                                      )
        self.wait_tf = Duration(seconds=0.1)
        # self.beta0 = # linear fitting params  
        # self.beta1 =  
        self.offset = 46
        self.zero = 4824


    def command_actuators(self, msg):
        
        if self.rotate_lidar:
            for i in range(-20, 22, 2):

                pwm_value = (218.45 * i + 32389 + 13630) / 9.4486 - self.offset
                message = "4 0 0 0 0 " + str(int(pwm_value))
                self.get_logger().info(message)

                # self.talker.send(message)
                self.clock.sleep_for(self.wait_tf)
                
                self.pub_lidar_frame(-i * math.pi / 180)
                self.clock.sleep_for(self.wait_rotation)

            # self.talker.send(f"4 0 0 0 0 {self.zero}")  # reset lidar after scanning sequence is done          
            self.rotate_lidar = False # allow movement if scanning is done 
        
        else:
            
            self.pub_lidar_frame(0.0) # cont publish lidar frame so that it does not expire
            
            # Robot ν, ω
            velocity = msg.linear.x
            omega = msg.angular.z
            
            # avoid weird pico errors
            dir_r = 1 
            dir_l = 1

            # set direction according to joystick input
            #! Note: This is a temporary solution, the final implementation will be done in the firmware
            #! Note: The epsilon values are used to prevent ν movement when left stick is moved slightly
            
            if velocity > self.epsilon_vel:
                dir_r = 1                
                dir_l = 1
            elif velocity < -self.epsilon_vel:
                dir_r = 0
                dir_l = 0   
            elif abs(velocity) <= 0.001 and omega > self.epsilon_omega:
                dir_r = 1
                dir_l = 0
            elif abs(velocity) <= 0.001 and omega < -self.epsilon_omega:
                dir_r = 0
                dir_l = 1

            # Motor ωr, ωl
            omega_r = abs( ( (velocity + omega * (self.b/2))   / self.r) * self.pwm_speed_ratio)
            omega_l = abs( ( (velocity - omega * (self.b/2))   / self.r) * self.pwm_speed_ratio)
            
            message = "3 " + str(dir_r) + " " + str(int(omega_r)) + " " + str(dir_l) + " " + str(int(omega_l)) + " 0"
            self.get_logger().info(message)
            # self.talker.send(message) 

    def pub_lidar_frame(self, theta):
        
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'rotation_point'

        t.transform.translation.x = 0.071281
        t.transform.translation.y = -0.048286
        t.transform.translation.z = 0.132819

        q = self.to_quat(0, theta, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.lidar_tf_broadcast.sendTransform(t)

    def to_quat(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q

    def joy_cb(self, joy_msg):
        
        if joy_msg.buttons[4]:
            self.rotate_lidar = True
            
    # def __del__(self):
    #     if hasattr(self, 'serial_comm') and self.serial_comm is not None:
    #         self.serial_comm.close()

def main(args=None):
    rclpy.init(args=args)
    glados_teleoperate = Teleoperate()
    rclpy.spin(glados_teleoperate)
    glados_teleoperate.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

