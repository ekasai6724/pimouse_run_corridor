#!/usr/bin/env python
# coding:utf8
import rospy,copy,math
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues

class WallAround():
	def __init__(self):
		self.cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
		
		self.sensor_values = LightSensorValues()
		rospy.Subscriber('/lightsensors', LightSensorValues, self.callback)

	def callback(self, messages):
		self.sensor_values = messages

	def get_freq(self):
		f = rospy.get_param('lightsensors_freq', 10)
		try:
			if f <= 0.0:
				raise Exception()
		except:
			rospy.logerr("Value error: lightsensors_freq")
			sys.exit(1)
		return f

	def wall_front(self, ls):
		return ls.left_forward > 50 or ls.right_forward > 50

	def too_right(self, ls):
		return ls.right_side > 50

	def too_left(self, ls):
		return ls.left_side > 50
	
	def vel_accel(self, vel, target, accel):
		vtmp = vel
		if vtmp < target:
			vtmp += accel
			if vtmp > target: vtmp = target
		else:
			vtmp -= accel
			if vtmp < target: vtmp = target
		
		return vtmp
		
	def run(self):
		freq = self.get_freq()
		rate = rospy.Rate(freq)
		data = Twist()

		vl = 0.0
		va = 0.0
		vl_tgt = 0.3
		va_tgt = 0.0
		while not rospy.is_shutdown():
			if self.wall_front(self.sensor_values):
				vl_tgt = 0.05
				va_tgt = -math.pi
			else:
				vl_tgt = 0.3
				e = 50 - self.sensor_values.left_side
				va_tgt = e * 2.0 * math.pi / 180.0
			
			vl = self.vel_accel(vl, vl_tgt, 0.02)
			va = self.vel_accel(va, va_tgt, 0.5)
			data.linear.x = vl
			data.angular.z = va

			self.cmd_vel.publish(data)
			rate.sleep()

if __name__ == '__main__':
	rospy.init_node('wall_around')
	rospy.wait_for_service('/motor_on')
	rospy.wait_for_service('/motor_off')
	rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
	rospy.ServiceProxy('/motor_on',Trigger).call()
	WallAround().run()
