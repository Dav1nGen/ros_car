#!/usr/bin/env python

# -*- coding: UTF-8 -*-
import rospy 
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image,CompressedImage
import threading
from time import ctime,sleep
import os
import sys
import termios

class RobotSelfCheck():
    def __init__(self):
        rospy.init_node('RobotSelfCheck',anonymous=False)
        self.vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

        rospy.Subscriber('/jubot/avel', Int32, self.avel_sub)
        rospy.Subscriber('/jubot/bvel', Int32, self.bvel_sub)
        rospy.Subscriber('/jubot/cvel', Int32, self.cvel_sub)
        rospy.Subscriber('/jubot/dvel', Int32, self.dvel_sub)
        rospy.Subscriber('/imu', Imu, self.imu_sub)
        rospy.Subscriber('/voltage', Float32, self.voltage_sub)

        self.avel = int()
        self.bvel = int()
        self.cvel = int()
        self.dvel = int()
        self.imu_check = float()
        self.move_cmd = Twist()
        self.voltage = float()

        t = threading.Thread(target=self.start_selfCheck,args=()) 
        t.setDaemon(True)
        t.start()
        rospy.spin()

    def avel_sub(self, msg):
        self.avel = msg.data

    def bvel_sub(self, msg):
        self.bvel = msg.data

    def cvel_sub(self, msg):
        self.cvel = msg.data

    def dvel_sub(self, msg):
        self.dvel = msg.data

    def imu_sub(self, msg):
        self.imu_check = msg.linear_acceleration.z

    def voltage_sub(self, msg):
        self.voltage = msg.data

    def pub_vel(self,vel):
        self.move_cmd.linear.x = vel
        self.vel_pub.publish(self.move_cmd)
        sleep(3)
        self.move_cmd.linear.x = 0
        self.vel_pub.publish(self.move_cmd)

    def press_any_key(self,msg):
        fd = sys.stdin.fileno()
        old_ttyinfo = termios.tcgetattr(fd)
        new_ttyinfo = old_ttyinfo[:]
        new_ttyinfo[3] &= ~termios.ICANON
        new_ttyinfo[3] &= ~termios.ECHO

        sys.stdout.write(msg)
        sys.stdout.flush()
        termios.tcsetattr(fd,termios.TCSANOW,new_ttyinfo)
        os.read(fd,7)
        termios.tcsetattr(fd,termios.TCSANOW,old_ttyinfo)

    def start_selfCheck(self):
        sleep(5.0)
        print('  ')
        rospy.loginfo(" \033[1;32m*******JUBOT ROBOT SelfCheck Program*******\033[0m")

        rospy.loginfo(' \033[1;32m++++++  Battery Voltage  ++++++\033[0m')
        rospy.loginfo(' \033[1;32m++++++  Camera           ++++++\033[0m')
        rospy.loginfo(' \033[1;32m++++++  Lidar            ++++++\033[0m')
        rospy.loginfo(' \033[1;32m++++++  IMU              ++++++\033[0m')
        rospy.loginfo(' \033[1;32m++++++  Motor Forward    ++++++\033[0m')
        rospy.loginfo(' \033[1;32m++++++  Motor Backward   ++++++\033[0m')
        print('  ')
        # self.press_any_key("Press any key to \033[1;32m START!\033[0m")
        rospy.loginfo(' Press any key to \033[1;32m START!\033[0m')
        self.press_any_key(" ")
        
        print('  ')
        rospy.loginfo('\033[1;32m .......Starting Voltage Check.......\033[0m')
        rospy.wait_for_message('/voltage',Float32,3)
        if(self.voltage > 11):
            rospy.loginfo(' -----Voltage Check\033[1;32m OK!\033[0m voltage: %s---',str(self.voltage))
        else:
            rospy.loginfo(' -----Voltage Check\033[1;31m Failed!\033[0m')



        print('  ')
        rospy.loginfo('\033[1;32m .......Starting Camera Check....... \033[0m')

        try:
            rospy.wait_for_message('/camera/image_raw',Image,2)
            rospy.loginfo(' -----Raw Image Check\033[1;32m OK!\033[0m ------')
        except rospy.exceptions.ROSException:
            rospy.loginfo(' -----Raw Image Check\033[1;31m Failed!\033[0m ------')
       
        try:
            rospy.wait_for_message('/image_raw/compressed',CompressedImage,2)
            rospy.loginfo(' -----Compressed Image Check\033[1;32m OK!\033[0m ------')
        except rospy.exceptions.ROSException:
            rospy.loginfo(' -----Compressed Image Check\033[1;31m Failed!\033[0m ------')


        print('  ')
        rospy.loginfo('\033[1;32m .......Starting Rplidar Check.......\033[0m')

        try:
            rospy.wait_for_message('/scan_origin',LaserScan,3)
            rospy.loginfo(' -----Rplidar Check\033[1;32m OK!\033[0m ------')
        except rospy.exceptions.ROSException:
            rospy.loginfo(' -----Rplidar Check\033[1;31m Failed!\033[0m ------')

        
        print('  ')
        rospy.loginfo('\033[1;32m .......Starting IMU Check.......\033[0m')

        if(self.imu_check > 8.0):
            rospy.loginfo(' -----IMU Check\033[1;32m OK!\033[0m ------')
        else:
            rospy.loginfo(' -----IMU Check\033[1;31m Failed!\033[0m ------')

        print('  ')
        rospy.loginfo('\033[1;32m .......Starting Motor Check.......\033[0m')
        
        vel_t = threading.Thread(target=self.pub_vel,args=(0.5,)) 
        vel_t.setDaemon(True)
        vel_t.start()
        sleep(1.0)
        if(self.avel>0 and self.bvel>0 and self.cvel>0 and self.dvel>0):
            rospy.loginfo(' .......Four Motor Forward Check\033[1;32m Successful!\033[0m.......')
        else:
            if(self.avel>0):
                rospy.loginfo(' -----MotorA Forward Check\033[1;32m OK!\033[0m ------')
            else:
                rospy.loginfo(' -----MotorA Forward Check\033[1;31m Failed!\033[0m ------')
            if(self.bvel>0):
                rospy.loginfo(' -----MotorB Forward Check\033[1;32m OK!\033[0m ------')
            else:
                rospy.loginfo(' -----MotorB Forward Check\033[1;31m Failed!\033[0m ------')
            if(self.cvel>0):
                rospy.loginfo(' -----MotorC Forward Check\033[1;32m OK!\033[0m ------')
            else:
                rospy.loginfo(' -----MotorC Forward Check\033[1;31m Failed!\033[0m ------')
            if(self.dvel>0):
                rospy.loginfo(' -----MotorD Forward Check\033[1;32m OK!\033[0m ------')
            else:
                rospy.loginfo(' -----MotorD Forward Check\033[1;31m Failed!\033[0m ------')

        sleep(3.0)
        vel_t = threading.Thread(target=self.pub_vel,args=(-0.5,)) 
        vel_t.setDaemon(True)
        vel_t.start()
        sleep(1.0)
        if(self.avel<0 and self.bvel<0 and self.cvel<0 and self.dvel<0):
            rospy.loginfo(' .......Four Motor Backward Check\033[1;32m Successful!\033[0m.......')
        else:
            if(self.avel<0):
                rospy.loginfo(' -----MotorA Backward Check\033[1;32m OK!\033[0m ------')
            else:
                rospy.loginfo(' -----MotorA Backward Check\033[1;31m Failed!\033[0m ------')
            if(self.bvel<0):
                rospy.loginfo(' -----MotorB Backward Check\033[1;32m OK!\033[0m ------')
            else:
                rospy.loginfo(' -----MotorB Backward Check\033[1;31m Failed!\033[0m ------')
            if(self.cvel<0):
                rospy.loginfo(' -----MotorC Backward Check\033[1;32m OK!\033[0m ------')
            else:
                rospy.loginfo(' -----MotorC Backward Check\033[1;31m Failed!\033[0m ------')
            if(self.dvel<0):
                rospy.loginfo(' -----MotorD Backward Check\033[1;32m OK!\033[0m ------')
            else:
                rospy.loginfo(' -----MotorD Backward Check\033[1;31m Failed!\033[0m ------')
        
        print('  ')
        rospy.loginfo(' -----Self Check \033[1;32m Completed!\033[0m ------')
        rospy.loginfo(' -----Press [Ctrl+C] to \033[1;32m Exit!\033[0m ------')




if __name__ == '__main__':
        RobotSelfCheck()
    




        
