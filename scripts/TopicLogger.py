from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import geometry_msgs.msg
import sensor_msgs.msg 

import numpy as np
import threading

import rospy

class TopicLogger:
    def __init__(self,topic_name,message_type,log_length,subscribe_buffer_length=2):
        """
        subscribe_buffer_length not correct
        """
        
        self.log_length = log_length
        self.time = np.zeros((self.log_length,),np.uint64)
        self.idx = 0
        
        self.done = False
        self.unregister_on_finish = True
        self._send_stop = False
        
        self.last_message = None
        
        self.subscriber = rospy.Subscriber(topic_name, message_type, self.handler, buff_size=subscribe_buffer_length)
        
        self.done_lock = threading.Lock() #this lock remains acquired until logging is done
        self.done_lock.acquire()
        self.start_lock = threading.Lock() #this lock remains acquired until logging is started
        self.start_lock.acquire()
        self.first_run = True              #gets set to False the first time the handler is run
        
    def block_until_done(self):
        self.done_lock.acquire()
        self.done_lock.release()

    def block_until_start(self):
        self.start_lock.acquire()
        self.start_lock.release()
        
    def stop(self):
        self._send_stop = True #is this threadsafe?
        
    def handler(self,message):
        #this handler logs time, manages unsubscribing, manipulates self.idx and calls another handler that may do other things in subclassed version
        if self.done:
            if self.unregister_on_finish:
                raise AssertionError("handler got called, but should have been unsubscribed from topic")
            return
            
        if self.idx == self.log_length or self._send_stop:
            #this block runs exactly once
            self.done = True
            if self.unregister_on_finish:
                self.subscriber.unregister()
            self.done_lock.release()
            return

        self.last_message = message
        self.time[self.idx] = message.header.stamp.to_nsec()
        self.handler_specific(message)
        self.idx += 1
        
        if self.first_run:
            self.start_lock.release()
            self.first_run = False
                  
    def handler_specific(self,message):
        pass

class FT_Logger(TopicLogger):
    def __init__(self,log_length,subscribe_buffer_length=2):
        self.ft = np.zeros((log_length,6),np.float64)
        
        TopicLogger.__init__(self,"/ft/r_gripper_motor", geometry_msgs.msg.WrenchStamped,
            log_length=log_length,subscribe_buffer_length=subscribe_buffer_length)
        
    def handler_specific(self,message):
        f = message.wrench.force
        t = message.wrench.torque                 
        self.ft[self.idx,:] = (f.x,f.y,f.z) + (t.x, t.y, t.z)        

class Joint_Logger(TopicLogger):
    def __init__(self,log_length,joints):
        """
        joints is an iterable of joint names
        """
        
        self.joints = joints
        self.pos = np.zeros((log_length,len(self.joints)),np.float64)
        TopicLogger.__init__(self,"joint_states",sensor_msgs.msg.JointState,
            log_length=log_length,subscribe_buffer_length=2)

    def handler_specific(self,message):
        for (i,joint) in enumerate(self.joints):
            try:
                joint_idx = message.name.index(joint) #Shlemiel the painter's algorithm
                v = message.position[joint_idx]
            except ValueError:
                rospy.loginfo("Joint %s missing from log entry %i"%(joint,self.idx) )
                v = np.NaN

            self.pos[self.idx,i] = v
