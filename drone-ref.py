# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2016 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects to one crazyflie (check the address at the top
and update it to your crazyflie address) and send a sequence of setpoints,
one every 5 seconds.

This example is intended to work with the Loco Positioning System in TWR TOA
mode. It aims at documenting how to set the Crazyflie in position control mode
and how to send setpoints.

EDITED:
Modified program for ball following application. Features made:
1. initial flying routine to ensure vertical take off
2. ball object to get ball's position in the field
3. path planning 
4. socket communication to be sent to any process 
"""
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from ballpos.ballPosition import ball

# additional import
import math
from cfSocket import cfSocket as cs

# add communication object
#MatlabCom = cs()

# estimated drone position and yaw to pass as global variable
x = 0
y = 0
z = 0
yaw = 0

# estimated drone initial position and yaw as relative motion
x_init = 0 
y_init = 0
z_init = 0
yaw_init = 0

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M'
#uri  = 'usb://0'

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')
    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001
    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:           
            data = log_entry[1]
            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)
            # print("{} {} {}".
            #        format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    wait_for_position_estimator(cf)

def position_callback(timestamp, data, logconf):
    # set to global to be sent to server (if needed)
    global x, y, z, yaw
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    yaw = data['stabilizer.yaw']
    # global MatlabCom
    # MatlabCom.sendDronePose([int(i*100) for i in [x,y,z,yaw]])
    print('pos: ({}, {}, {}) yaw: ({})'.format(x, y, z, yaw))


def start_position_printing(scf):
    # This function will also used to initialize 
    # initial position global: x_init, y_init, z_init, yaw_init
    log_conf = LogConfig(name='Position', period_in_ms=500)#500
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')
    log_conf.add_variable('stabilizer.yaw', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()


def run_sequence(scf, sequence):
    cf = scf.cf
    
    for position in sequence:
        print('Setting position {}'.format(position))
        for i in range(200):
            fly(cf,position) # edited

    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


def fly(cf, position):
    cf.commander.send_position_setpoint(position[0],
                                        position[1],
                                        position[2],
                                        position[3])
    time.sleep(0.02) # 0.1


def init_fly(scf):
    # To be able to fly stably in the beginning, 
    # the drone need to increase height only (hence no any roll,yaw,pitch)
    cf = scf.cf
    global x_init, y_init, z_init, yaw_init
    x_init, y_init, z_init, yaw_init = [x, y, z, yaw]
    print('pos_init: ({}, {}, {}) yaw: ({})'.format(x_init, y_init, z_init, yaw_init))
    height_init = 0.2
    z_init += height_init

    for i in range(200):
	    fly(cf,[x_init, y_init, z_init, yaw_init])    

def ball_frame_to_drone_frame(ball_pos):
    # transform the ball frame to drone frame 
    # depends on drone localizing system
    xb, yb = ball_pos
    xb, yb = [-yb/1000 + 1 , -xb/1000]
    return [xb, yb]

def ignore_ball_pos(xb, yb):
    # return condition to ignore ball location
    # depends on drone localizing system
    return xb < 0 or yb > 0 or xb > 0.9 or yb < -1.4

def set_point(ball_pos):
    # return setpoint (simple path planning)
    xb, yb = ball_frame_to_drone_frame(ball_pos) 
    print(xb,yb)
    if ignore_ball_pos(xb, yb):
        setpoint = [x, y, z_init, yaw_init]
    else:
        dy = yb
        dyaw = 0
        setpoint = [x_init+xb, y_init + dy, z_init, yaw_init]
    return setpoint

def set_point2(ball_pos):
    # return setpoint (path planning 2)
    r = 0.15     # closest range drone from the ball
    R = 0.3     # farthest range drone from the ball
    Ra = 0.2    # initial set range drone from the ball

    xb, yb     = ball_frame_to_drone_frame(ball_pos)
    xd_0, yd_0 = [x-x_init, y-y_init] # depends on drone localizing
    xbd, ybd   = [xb - xd_0, yb - yd_0]
    
    dtheta = math.atan2(ybd,xbd)    
    D = math.sqrt(xbd**2 + ybd**2)

    dx = xd_0
    dy = yd_0
    onlyRotate = True
    
    if D > R or D < r:
        # check if ball not in range
        onlyRotate = False
    
    if (D-Ra < 0.02 and D-Ra > -0.02) or ignore_ball_pos(xb,yb):
        # check if ball already in range
        onlyRotate = True

    if not onlyRotate:
        dx = xb - Ra*math.cos(dtheta)
        dy = yb - Ra*math.sin(dtheta)
    
    dtheta = math.degrees(dtheta)
    return [x_init + dx, y_init + dy, z_init, yaw_init + dtheta]


if __name__ == '__main__':
    url = 'http://192.168.1.77:4747/video'#16
    a = ball(cam=url,stream ='on')
    cflib.crtp.init_drivers(enable_debug_driver=False)
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        reset_estimator(scf)
        start_position_printing(scf)
        time.sleep(0.01)
        init_fly(scf)
        
        cf = scf.cf

        for i in range(5000):
            setpoint = set_point2(a.getBallPos())
            fly(cf,setpoint)
            
        cf.commander.send_stop_setpoint()
        '''
        seq = [[x_init,y_init, z_init, yaw_init+300],
            [x_init,y_init, z_init, yaw_init+60],
            [x_init,y_init, z_init, yaw_init],
            [x_init,y_init, z_init-0.05, yaw_init]]
        run_sequence(scf, seq)
        '''
        time.sleep(0.1)