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

    #----------------------------------------------------------------
    robot = CDsrRobot()
    #----------------------------------------------------------------

    set_velx(30,20)  # set global task speed: 30(mm/sec), 20(deg/sec)
    set_accx(60,40)  # set global task accel: 60(mm/sec2), 40(deg/sec2)

    velx=[50, 50]
    accx=[100, 100]

    p1= posj(0,0,0,0,0,0)                    #joint
    p2= posj(0.0, 0.0, 90.0, 0.0, 90.0, 0.0) #joint

    x1= posx(400, 500, 800.0, 0.0, 180.0, 0.0) #task
    x2= posx(400, 500, 500.0, 0.0, 180.0, 0.0) #task

    c1 = posx(559,434.5,651.5,0,180,0)
    c2 = posx(559,434.5,251.5,0,180,0)


    q0 = posj(0,0,0,0,0,0)
    q1 = posj(10, -10, 20, -30, 10, 20)
    q2 = posj(25, 0, 10, -50, 20, 40) 
    q3 = posj(50, 50, 50, 50, 50, 50) 
    q4 = posj(30, 10, 30, -20, 10, 60)
    q5 = posj(20, 20, 40, 20, 0, 90)
    qlist = [q0, q1, q2, q3, q4, q5]

    x1 = posx(600, 600, 600, 0, 175, 0)
    x2 = posx(600, 750, 600, 0, 175, 0)
    x3 = posx(150, 600, 450, 0, 175, 0)
    x4 = posx(-300, 300, 300, 0, 175, 0)
    x5 = posx(-200, 700, 500, 0, 175, 0)
    x6 = posx(600, 600, 400, 0, 175, 0)
    xlist = [x1, x2, x3, x4, x5, x6]


    X1 =  posx(370, 670, 650, 0, 180, 0)
    X1a = posx(370, 670, 400, 0, 180, 0)
    X1a2= posx(370, 545, 400, 0, 180, 0)
    X1b = posx(370, 595, 400, 0, 180, 0)
    X1b2= posx(370, 670, 400, 0, 180, 0)
    X1c = posx(370, 420, 150, 0, 180, 0)
    X1c2= posx(370, 545, 150, 0, 180, 0)
    X1d = posx(370, 670, 275, 0, 180, 0)
    X1d2= posx(370, 795, 150, 0, 180, 0)


    seg11 = posb(DR_LINE, X1, radius=20)
    seg12 = posb(DR_CIRCLE, X1a, X1a2, radius=21)
    seg14 = posb(DR_LINE, X1b2, radius=20)
    seg15 = posb(DR_CIRCLE, X1c, X1c2, radius=22)
    seg16 = posb(DR_CIRCLE, X1d, X1d2, radius=23)
    b_list1 = [seg11, seg12, seg14, seg15, seg16] 


    while rclpy.ok(): 
        # move joint    
        robot.movej(p2, vel=100, acc=100)
        print("------------> movej OK")    
        time.sleep(1)

        # move joint task    
        robot.movejx(x1, vel=30, acc=60, sol=0)
        print("------------> movejx OK")    
        time.sleep(1)

        # move line    
        robot.movel(x2, velx, accx)
        print("------------> movel OK")    

        # move circle    
        robot.movec(c1, c2, velx, accx)
        print("------------> movec OK")    

        # move spilne joint    
        robot.movesj(qlist, vel=100, acc=100)
        print("------------> movesj OK")    

        # move spilne task    
        robot.movesx(xlist, vel=100, acc=100)
        print("------------> movesx OK")    

        # move spiral    
        robot.move_spiral(rev=9.5,rmax=20.0,lmax=50.0,time=20.0,axis=DR_AXIS_Z,ref=DR_TOOL)
        print("------------> move_spiral OK")    

        # move periodic    
        robot.move_periodic(amp =[10,0,0,0,30,0], period=1.0, atime=0.2, repeat=5, ref=DR_TOOL)
        print("------------> move_periodic OK")    

        # move Blending    
        robot.moveb(b_list1, vel=150, acc=250, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        print("------------> moveb OK")    

    print('XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX good-bye!')
    print('XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX good-bye!')
    print('XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX good-bye!')

if __name__ == '__main__':
    main()
