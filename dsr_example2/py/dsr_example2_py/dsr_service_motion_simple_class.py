#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ##
# @brief    [py example simple] motion simple test (using CDsrRobot class)
# @author   Kab Kyoum Kim (kabkyoum.kim@doosan.com)  

import rclpy
import os, sys
import threading, time
import signal

from dsr_msgs2.msg import *

sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../../common2/bin/common2/imp")) ) # get import pass : DSR_ROBOT2.py 


#--------------------------------------------------------
import DR_init
g_node = None
rclpy.init()
g_node = rclpy.create_node('dsr_service_motion_simple_class_py')
DR_init.__dsr__node = g_node
from DSR_ROBOT2 import *
#--------------------------------------------------------



def signal_handler(sig, frame):
    print('XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX signal_handler')
    print('XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX signal_handler')
    print('XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX signal_handler')
    global g_node
    publisher = g_node.create_publisher(RobotStop, 'stop', 10)

    msg = RobotStop()
    msg.stop_mode =1 

    publisher.publish(msg)
    #sys.exit(0)
    rclpy.shutdown()


def main(args=None):
    global g_node
    signal.signal(signal.SIGINT, signal_handler)

    robot = CDsrRobot()

    p1= posj(0, 0, 0, 0, 0, 0)     #joint pos
    p2= posj(0, 0, 90.0, 0, 90.0, 0)   #joint pos

    x1= posx(400.0, 500.0, 800.0, 0, 180.0, 0) #task pos
    x2= posx(400.0, 500.0, 500.0, 0, 180.0, 0) #task pos

    while rclpy.ok(): 
        res = robot.set_robot_mode(1)
        print("res= {0}".format(res))    
        robot.movej(p1, vel=100.0, acc=100.0)
        robot.movej(p2, vel=100.0, acc=100.0)
        time.sleep(1)
        movel(x1, vel=100, acc=100)
        movel(x2, vel=100, acc=100)
        time.sleep(1)

    print('XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX good-bye!')
    print('XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX good-bye!')
    print('XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX good-bye!')

if __name__ == '__main__':
    main()
