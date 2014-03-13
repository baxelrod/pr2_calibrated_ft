#!/usr/bin/python
import roslib

import sys

from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import geometry_msgs.msg
import sensor_msgs.msg 

import actionlib
import rospy
import tf

from time import sleep
import threading

import scipy.optimize
import scipy.interpolate
import numpy as np
import matplotlib.pyplot as plt

from TopicLogger import TopicLogger, Joint_Logger, FT_Logger
from SineFit import SineFit, SineFitFactory
            
class Arm:
    def __init__(self, arm_name):
        #arm_name should be l_arm or r_arm
        self.name = arm_name
        self.jta = actionlib.SimpleActionClient('/'+arm_name+'_controller/joint_trajectory_action', JointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')
        self.last_angles = None
        
    def move(self, angles):
        goal = JointTrajectoryGoal()
        char = self.name[0] #either 'r' or 'l'
        goal.trajectory.joint_names = [char+'_shoulder_pan_joint',
                                       char+'_shoulder_lift_joint',
                                       char+'_upper_arm_roll_joint',
                                       char+'_elbow_flex_joint',
                                       char+'_forearm_roll_joint',
                                       char+'_wrist_flex_joint',
                                       char+'_wrist_roll_joint']
        point = JointTrajectoryPoint()
        
        new_angles = list(angles)
        
        #replace angles that have None with last commanded joint position from this Arm class.
        for i,ang in enumerate(angles):
            if ang is None:
                assert self.last_angles is not None
                new_angles[i] = self.last_angles[i]

        self.last_angles = new_angles
        point.positions = new_angles

        point.time_from_start = rospy.Duration(3)
        goal.trajectory.points.append(point)
        self.jta.send_goal_and_wait(goal)

class Joint_FT_Log():
    def __init__(self,move):
        #start the joint and force torque sensor loggers, call "move", then stop the loggers
        #then do the proper time interpolation to align the joint and ft data

        rospy.loginfo('start logging')
        jl = Joint_Logger(6000, [  'r_shoulder_pan_joint', 
                                    'r_shoulder_lift_joint', 
                                    'r_upper_arm_roll_joint',
                                    'r_forearm_roll_joint', 
                                    'r_elbow_flex_joint', 
                                    'r_wrist_flex_joint', 
                                    'r_wrist_roll_joint',]  )
                                    
        self.internal_joint_names = [   'shoulder_pan', 
                                        'shoulder_lift', 
                                        'upper_roll', 
                                        'lower_roll', 
                                        'elbow_flex', 
                                        'wrist_flex', 
                                        'wrist_roll',]                                    
        jl.block_until_start()
        sleep(1) # make sure we can interpolate joint data
        l = FT_Logger(60000)
        l.block_until_start()

        move()  #move the arm around
        
        l.stop()
        l.block_until_done()
        sleep(1) # make sure we can interpolate joint data
        jl.stop_after(l.time[l.idx-1]) #keep logging until we've logged joint data past the last force-torque measurement
        jl.block_until_done()
        
        #truncate timeseries
        l.ft = l.ft[:l.idx,:]
        l.time = l.time[:l.idx]
        jl.pos = jl.pos[:jl.idx,:]
        jl.time = jl.time[:jl.idx]

        offset = min(l.time[0],jl.time[0])
        l.time = l.time - offset
        jl.time = jl.time - offset

        self.joint_logger = jl
        self.ft_logger = l

        
        #the interpolator below will generate an exception if there is an attempt to extrapolate.
        
        if not jl.time[0] < l.time[0]:
            print 'joint logger started late:', jl.time[0] - l.time[0],
            return
        if not jl.time[-1] > l.time[-1]:
            print 'joint logger ended early:' , l.time[-1] - jl.time[-1],
            return
            
        self.time_aligned = False
        
    def time_align(self):
        #joints are published separatly from ft sensor. interpolate joint values at the times of the ft messages
        #we do this not immediately after collecting the data because the interpolator throws an exception
        #if there if we ask for extrapolation  (good), but we want to at least save the data beforehand.
        
        self.joint_interpolator = {}
        self.pos = np.zeros((len(self.ft_logger.time), len(self.joint_logger.joints)),dtype=np.float64)

        for (i,joint) in enumerate(self.joint_logger.joints):
            interpolator = scipy.interpolate.interp1d(self.joint_logger.time,self.joint_logger.pos[:,i]) 
            self.pos[:,i] = interpolator(self.ft_logger.time)
            self.joint_interpolator[joint] = interpolator            
        
        self.time_aligned = True
        
    def get(self,name):
        ft_names = ['fx','fy','fz','tx','ty','tz']
        joint_names = self.internal_joint_names
        
        if name in ft_names:
            return self.ft_logger.ft[:,  ft_names.index(name)]
        elif name in joint_names:
            if not self.time_aligned:
                self.time_align()
                
            return self.pos[:,  joint_names.index(name)]
        else:
            raise ValueError('Unknown name: %s'%(name))

"""
pushing on X+/Y+/Z+ (markings on the sensor) increases fx/fy/fz (values in the message)
pushing on gripper, toward arm, decreases fz.
pulling on gripper, away from arm, increases fz
(the hypothetical Z+ label is toward the robot. Z- is away from the robot.)

the force is measured in a left-handed coordinate system. The positive axis of the measurement frames are the centered at the sensor and point out in the X-,Y-,Z- (markings on the sensor) direction 
"""

def get_z():
    f = FT_Logger(30000)
    f.block_until_done()
    return np.mean(f.ft[:,2])
    
if __name__ == '__main__':
    rospy.init_node('b')

def do_it():
    arm = Arm('r_arm')
    
    rospy.loginfo('arm initialized')    
    #assert False

    def wrist_move(roll,flex):
        arm.move([0.0, 0.0, 0.0, np.deg2rad(-55), np.pi, np.deg2rad(-55)+flex, roll ] )

    wrist_move(0,0)
    
    def wrist_action_roll():
        for i in [ 1, 2, 3, 4]:
            wrist_move(np.deg2rad(179)*i,0)
    
    wrist_roll_forward = Joint_FT_Log(wrist_action_roll)

    def wrist_action_roll():
            for i in [ 4, 3, 2, 1]:
                wrist_move(np.deg2rad(179)*i,0)
                
    wrist_roll_backward = Joint_FT_Log(wrist_action_roll)

    def wrap(ang):
        return ( ang + np.pi) % (2 * np.pi ) - np.pi
        
        
    roll_angle_forward = wrap ( wrist_roll_forward.get('wrist_roll') )
    roll_angle_backward = wrap ( wrist_roll_backward.get('wrist_roll') )
    
    plt.plot( roll_angle_forward , wrist_roll_forward.get('fx') , '.',label='x force')
    plt.plot( roll_angle_forward , wrist_roll_forward.get('fy') , '.',label='y force')

    plt.plot( roll_angle_forward , wrist_roll_forward.get('tx') , '.',label='x torque')
    plt.plot( roll_angle_forward , wrist_roll_forward.get('ty') , '.',label='y torque')
    
    plt.xlabel('wrist roll (rads)')
    plt.ylabel('force/torque (raw)')
    plt.title('empty gripper')
    plt.legend()
    
    plt.figure()
    plt.title('back and forth')
    plt.plot( roll_angle_forward , wrist_roll_forward.get('fx') , 'r.',label='x force forward')
    plt.plot( roll_angle_backward , wrist_roll_backward.get('fx') , 'g.',label='x force backward')    
    plt.legend()
        

    sff = SineFitFactory(sys.argv[1] if len(sys.argv) >= 2 else None)

    cw_xfit = sff.create_sine_fit(wrap(wrist_roll_forward.get('wrist_roll')), wrist_roll_forward.get('fx'), 'cw_xfit')
    cw_xfit.fit()
    
    ccw_xfit = sff.create_sine_fit(wrap(wrist_roll_backward.get('wrist_roll')), wrist_roll_backward.get('fx'), 'ccw_xfit')
    ccw_xfit.fit()

    cw_yfit = sff.create_sine_fit(wrap(wrist_roll_forward.get('wrist_roll')), wrist_roll_forward.get('fy'), 'cw_yfit')
    cw_yfit.fit()
    
    ccw_yfit = sff.create_sine_fit(wrap(wrist_roll_backward.get('wrist_roll')), wrist_roll_backward.get('fy'), 'ccw_yfit')
    ccw_yfit.fit()

    plt.figure()    

    ax1 = plt.subplot(2,1,1)    
    cw_xfit.plot_data(color='m')
    cw_xfit.plot_fit(color='r')
    
    ccw_xfit.plot_data(color='c')
    ccw_xfit.plot_fit(color='b')
    plt.ylabel('force x')
    plt.title('Wrist Roll')
    
    plt.subplot(2,1,2,sharex=ax1)
    cw_yfit.plot_data(color='m')
    cw_yfit.plot_fit(color='r')
    
    ccw_yfit.plot_data(color='c')
    ccw_yfit.plot_fit(color='b')
    plt.ylabel('force y')
    
    fx0_roll = ( cw_xfit.zerocross() + ccw_xfit.zerocross() ) / 2.0
    fy0_roll = ( cw_yfit.zerocross() + ccw_yfit.zerocross() ) / 2.0

    wrist_move(fx0_roll,0)  #x has zero force, y has most negative force (Y- on FT is pointing up)   
    wrist_move(fx0_roll,np.deg2rad(-50))

    def wrist_action_flex():
        for i in [-50,0,50,0,-50]:
            wrist_move(fx0_roll,np.deg2rad(i))

    wrist_flex = Joint_FT_Log(wrist_action_flex)

    wrist_flex_fz = sff.create_sine_fit(wrist_flex.get('wrist_flex'),wrist_flex.get('fz'), 'wrist_flex_fz')
    wrist_flex_fy = sff.create_sine_fit(wrist_flex.get('wrist_flex'),wrist_flex.get('fy'), 'wrist_flex_fy')

    wrist_flex_fz.fit(fix_freq=1.0)
    wrist_flex_fy.fit(fix_freq=1.0)
    
    plt.figure()

    
    ax1 = plt.subplot(2,1,1)
    wrist_flex_fz.plot_data();
    wrist_flex_fz.plot_fit();
    plt.ylabel('fz')
    
    plt.title('Wrist Flex')
    
    plt.subplot(2,1,2,sharex=ax1)
    wrist_flex_fy.plot_data();
    wrist_flex_fy.plot_fit();
    plt.ylabel('fy')

    fz0_flex = wrist_flex_fz.zerocross()-np.pi
    
    wrist_move(fx0_roll,np.deg2rad(55) + fz0_flex)  #z has zero force

    dr = np.deg2rad
    
    arm.move([dr(0), dr(-20), dr(-180), dr(-20), dr(180), dr(0), dr(0)] ) #nominally the same wrist orientation as we started
    
    arm.move([dr(0), dr(-20), dr(-180), dr(-20), fx0_roll, dr(0), dr(0)] )
    
    def move():
        for i in [20, 0, -20, -40, -60, -80, -100, -80, -60, -40, -20, 0]:
            arm.move([dr(0), dr(-20), dr(-180), dr(-20 + i), fx0_roll, dr(0), dr(0)] )

    arm.move([dr(0), dr(-20), dr(-180), dr(0), fx0_roll, dr(0), dr(0)] )
    elbow_flex = Joint_FT_Log(move)

    plt.figure()
    plt.title('Elbow Flex')
    
    elbow_flex_fz = sff.create_sine_fit(elbow_flex.get('elbow_flex'),elbow_flex.get('fz'), 'elbow_flex_fz')
    elbow_flex_fz.fit(fix_freq=1.0,fix_bias=wrist_flex_fz.bias)
    elbow_flex_fz.plot_data()
    elbow_flex_fz.plot_fit()
    
    fz0_elbow = elbow_flex_fz.zerocross() -np.pi    
    plt.axvline(fz0_elbow)
    
    def wrist_move_flex2up():
        for i in [0, -25, -50, -75, -100, -110, -90, -70, -50, -25, 0]:
            arm.move([dr(0), dr(-20), dr(-180), fz0_elbow, dr(-180), dr(i), dr(0)] )

    def wrist_move_flex2down():
        for i in [0, -25, -50, -75, -100, -110, -90, -70, -50, -25, 0]:
            arm.move([dr(0), dr(-20), dr(-180), fz0_elbow, dr(0), dr(i), dr(0)] )
            
            
    arm.move([dr(0), dr(-20), dr(-180), fz0_elbow, dr(-180), dr(0), dr(0)] )
    wrist_flex2up = Joint_FT_Log(wrist_move_flex2up)

    arm.move([dr(0), dr(-20), dr(-180), fz0_elbow, dr(0), dr(0), dr(0)] )
    wrist_flex2down = Joint_FT_Log(wrist_move_flex2down)
    
    wrist_flex2up_fz = sff.create_sine_fit(wrist_flex2up.get('wrist_flex'),wrist_flex2up.get('fz'), 'wrist_flex2up_fz')
    wrist_flex2up_fz.fit(fix_freq=1.0)
    
    wrist_flex2down_fz = sff.create_sine_fit(wrist_flex2down.get('wrist_flex'),wrist_flex2down.get('fz'), 'wrist_flex2down_fz')
    wrist_flex2down_fz.fit(fix_freq=1.0)
    
    plt.figure()
    plt.subplot(2,1,1)
    wrist_flex2up_fz.plot_data()
    wrist_flex2up_fz.plot_fit()
    plt.ylabel('fz')
    plt.title('Wrist Flex arm up')
        
    plt.subplot(2,1,2)
    wrist_flex2down_fz.plot_data()
    wrist_flex2down_fz.plot_fit()
    plt.ylabel('fz')
    plt.title('Wrist Flex arm down')
    
    wrist_flex_up_fzmax = wrist_flex2up_fz.zerocross() - dr(90)
    wrist_flex_down_fzmax = wrist_flex2down_fz.zerocross() + dr(90)
    
    elbow_error = (wrist_flex_up_fzmax - wrist_flex_down_fzmax)
    wrist_flex_fzmax = (wrist_flex_up_fzmax + wrist_flex_down_fzmax) / 2
    
#    arm.move([dr(0), dr(-20), dr(-180), fz0_elbow, dr(-180), wrist_flex_up_fzmax, dr(0)] )        #z axis aligned with gravity
#    arm.move([dr(0), dr(-20), dr(-180), fz0_elbow, dr(0), wrist_flex_down_fzmax, dr(0)] )        #z axis aligned with gravity

    fz0_elbow_new = fz0_elbow - elbow_error/2
    
    def move_z_elbow_roll():
        for i in [0,1,2,3,4,3,2,1,0]:
           arm.move([dr(0), dr(-20), dr(-180), fz0_elbow_new, dr(0+179*i), wrist_flex_fzmax, fx0_roll] ) #as rolls, fy should be fixed

    arm.move([dr(0), dr(-20), dr(-180), fz0_elbow_new, dr(0), wrist_flex_fzmax, fx0_roll] )        #z axis aligned with gravity
    elbow_roll = Joint_FT_Log(move_z_elbow_roll)
    
    elbow_roll_fz = sff.create_sine_fit(wrap(elbow_roll.get('lower_roll')),elbow_roll.get('fz'), 'elbow_roll_fz')
    elbow_roll_fz.fit(fix_freq=1.0)

    elbow_roll_fx = sff.create_sine_fit(wrap(elbow_roll.get('lower_roll')),elbow_roll.get('fx'), 'elbow_roll_fx')
    elbow_roll_fx.fit(fix_freq=1.0)

    plt.figure()
    plt.gcf().suptitle('elbow roll')
        
    plt.subplot(2,1,1)
    elbow_roll_fz.plot_data()
    elbow_roll_fz.plot_fit()
    plt.ylabel('fz')
        
    plt.subplot(2,1,2)
    elbow_roll_fx.plot_data()
    elbow_roll_fx.plot_fit()

    plt.ylabel('fx')
    

        
    if False:
        #outdated since not using TF right now
        rpy = np.zeros((len(l.data),3),np.float64)
        
        import PyKDL as kdl
        
        rot0 = kdl.Rotation.Quaternion( *tuple(l.data[0,0:4]) ) 
        rot0inv = rot0.Inverse()
        
        for i in range(len(l.data)):
            rot = kdl.Rotation.Quaternion( *tuple(l.data[i,0:4]) ) 
            rpy[i,:] = (rot0inv * rot).GetRPY() #roll pitch yaw 
            
            
