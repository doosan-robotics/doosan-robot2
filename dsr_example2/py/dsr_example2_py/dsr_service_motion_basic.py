#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ##
# @brief    [py example simple] motion basic test for doosan robot
# @author   Kab Kyoum Kim (kabkyoum.kim@doosan.com)  

import rclpy
import sys
import time 
import signal

from dsr_msgs2.msg import *
from dsr_msgs2.srv import *

g_node = None
rclpy.init()
g_node = rclpy.create_node('dsr_service_motion_basic_py')


def set_robot_mode(robot_mode):
    global g_node

    srv = g_node.create_client(SetRobotMode, 'system/set_robot_mode')

    req = SetRobotMode.Request()
    req.robot_mode = robot_mode

    future = srv.call_async(req)
    rclpy.spin_until_future_complete(g_node, future)

    result = future.result()
    if result == None:
        ret = -1
    else:
        ret = 0 if (result.success == True) else -1             
    return ret

def movej(pos, vel=None, acc=None, time=0.0, radius=0.0, mod= 0, ra=0):
    global g_node

    srv = g_node.create_client(MoveJoint, 'motion/move_joint')

    req = MoveJoint.Request()

    req.pos         = [float(x) for x in pos]
    req.vel         = float(vel)
    req.acc         = float(acc)
    req.time        = float(time)
    req.radius      = float(radius)
    req.mode        = int(mod)
    req.blend_type  = int(ra)
    req.sync_type   = 0

    future = srv.call_async(req)
    rclpy.spin_until_future_complete(g_node, future)

    try:
        result = future.result()
    except Exception as e:
        g_node.get_logger().info('movej Service call failed %r' % (e,))
    else:
        if result == None:
            ret = -1    
        else:        
            ret = 0 if (result.success == True) else -1             

    return ret

def movel(pos, vel=None, acc=None, time=0.0, radius=0.0, ref=0, mod= 0, ra=0):
    global g_node

    srv = g_node.create_client(MoveLine, 'motion/move_line')

    req = MoveLine.Request()

    req.pos         = [float(x) for x in pos] 
    req.vel         = [float(x) for x in vel]
    req.acc         = [float(x) for x in acc]
    req.time        = float(time)
    req.radius      = float(radius)
    req.ref         = int(ref)
    req.mode        = int(mod)
    req.blend_type  = int(ra)
    req.sync_type   = 0

    future = srv.call_async(req)
    rclpy.spin_until_future_complete(g_node, future)

    try:
        result = future.result()
    except Exception as e:
        g_node.get_logger().info('movel Service call failed %r' % (e,))
    else:
        if result == None:
            ret = -1    
        else:        
            ret = 0 if (result.success == True) else -1             

    return ret

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

    p1= [0,0,0,0,0,0]                       #joint pos
    p2= [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]    #joint pos

    x1= [400, 500, 800.0, 0.0, 180.0, 0.0]  #task pos
    x2= [400, 500, 500.0, 0.0, 180.0, 0.0]  #task pos

    while rclpy.ok():    
        res = set_robot_mode(1)
        print("res= {0}".format(res))    

        # move joint   
        movej(p1, vel=100.0, acc=100.0)
        movej(p2, vel=100.0, acc=100.0)
        time.sleep(1)

        # move line    
        movel(x1, vel=[100.0, 100], acc=[100.0, 100])
        movel(x2, vel=[100.0, 100], acc=[100.0, 100])
        time.sleep(1)


    print('XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX good-bye!')
    print('XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX good-bye!')
    print('XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX good-bye!')


if __name__ == '__main__':
    main()