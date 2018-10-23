#!/usr/bin/env python
import rospy
import math
import time
import copy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseArray
vel_X=0
vel_Y=0
vel_Z=0
error_x=[]
error_y=[]
error_z=[]
j=0
current_position=[0,0,0]
whypoints=[ (4.25, 4.05, 11), (-4.0, 3.25, 11), (-3.8, -4.2, 11),(4.25, -4.2, 11), (0, 0, 11)]
def final_position():
        global j
        while(j<5):
            if((error_x<0.2)and(error_y<0.2)and(error_z<1.5)):
                j+=1
            else:
                destination=copy.copy(whypoints[j])
                print "target ", j
                return destination

msg=[]
def name_callback(msg):
        global current_position
        X= msg.poses[0].position.x
        Y= msg.poses[0].position.y
        Z= msg.poses[0].position.z
        current_position=[X,Y,Z]
        return

    #current_position=[X,Y,Z]

def go(*args):

    msg=Twist()
    msg.linear.x=vel_X
    msg.linear.y=vel_Y
    msg.linear.z=vel_Z
    msg.angular.x=0
    msg.angular.y=0
    msg.angular.z=0
    pub_motion.publish(msg)
    print "velocity",msg
    print "vel_X",vel_X
    print "vel_Y",vel_Y
    print "vel_Z",vel_Z
class PID:


	def __init__ (self, P, I, D, Derivator, Integrator, Integrator_max, Integrator_min):

		self.Kp=P
		self.Ki=I
		self.Kd=D
		self.Derivator=Derivator
		self.Integrator=Integrator
		self.Integrator_max=Integrator_max
		self.Integrator_min=Integrator_min
		self.set_point=[0.0,0.0,0.0]                             # destination
		self.error=0.0


	def update(self,current_value=[]):
		global error_x
		global error_y
		global error_z
		error_x= self.set_point [0]- current_value[0]
		error_y= self.set_point [1]- current_value[1]
		error_z= self.set_point [2]- current_value[2]
		self.error= ((error_x**2)+(error_y**2)+(error_z**2))**(0.5)

		self.P_value = self.Kp * self.error
		self.D_value = self.Kd * ( self.error - self.Derivator)
		self.Derivator = self.error

		self.Integrator = self.Integrator + self.error

		if self.Integrator > self.Integrator_max:
			self.Integrator = self.Integrator_max
		elif self.Integrator < self.Integrator_min:
			self.Integrator = self.Integrator_min
		self.I_value = self.Integrator * self.Ki

		PID = self.P_value + self.I_value + self.D_value
		print "distination ",self.set_point
		print "current_position",current_value
		vel_X = PID*error_x
		vel_Y = PID*error_y
		vel_Z = PID*error_z

		return PID ,vel_X ,vel_Y, vel_Z,current_value

	def setPoint(self,set_point=[]):
		self.set_point = set_point
		self.Integrator=0
		self.Derivator=0



	def setIntegrator(self, Integrator):
		self.Integrator = Integrator

	def setDerivator(self, Derivator):
		self.Derivator = Derivator

	def setKp(self,P):
		self.Kp=P

	def setKi(self,I):
		self.Ki=I

	def setKd(self,D):
		self.Kd=D

	def getPoint(self):
		return self.set_point

	def getError(self):
		return self.error

	def getIntegrator(self):
		return self.Integrator

	def getDerivator(self):
		return self.Derivator


if __name__ == '__main__':
    rospy.init_node('ISSU_MOTION')
	
    while (1):

        sub=rospy.Subscriber('/whycon/poses' ,PoseArray , name_callback, queue_size=50)
        rospy.sleep(.05)   # IMPORTANT GLOBAL
        pub_motion= rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        a=PID(1.5,0.1,1.0,0,0,500,-500)
        a.setPoint(final_position())
        vel,vel_X,vel_Y,vel_Z,current_value= a.update(current_position)
        go()
		
