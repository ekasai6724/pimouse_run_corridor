#!/usr/bin/env python
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

	def run(self):
		freq = self.get_freq()
		rate = rospy.Rate(freq)
		data = Twist()

		data.linear.x = 0.3
		data.angular.z = 0.0
		while not rospy.is_shutdown():
			if self.wall_front(self.sensor_values):
				data.angular.z = -math.pi
			elif self.too_right(self.sensor_values):
				data.angular.z = math.pi
			elif self.too_left(self.sensor_values):
				data.angular.z = -math.pi
			else:
				e = 50 - self.sensor_values.left_side
				data.angular.z = e * math.pi / 180.0

			self.cmd_vel.publish(data)
			rate.sleep()

if __name__ == '__main__':
	rospy.init_node('wall_around')
	rospy.wait_for_service('/motor_on')
	rospy.wait_for_service('/motor_off')
	rospy.on_shutdown(rospy.ServiceProxy('/motor_off',Trigger).call)
	rospy.ServiceProxy('/motor_on',Trigger).call()
	WallAround().run()
