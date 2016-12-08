#!/usr/bin/env python
import rospy
from std_msgs.msg import String
class test1(object):
	def __init__(self):
		self.testpub = rospy.Publisher('chatter', String, queue_size=10)
	   	rospy.init_node('talker', anonymous=True)
		rate = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
			hello_str = "hello world %s" % rospy.get_time()
			#rospy.loginfo(hello_str)
			self.testpub.publish(hello_str)
			rate.sleep()
			#rospy.spin()
class test2(object):
	def __init__(self):
	   	#rospy.init_node('listener', anonymous=True)

   		self.testsub = rospy.Subscriber("chatter", String, self.cb)

    		# spin() simply keeps python from exiting until this node is stopped


	def cb(self,val):
		print 'h'
		rospy.loginfo(rospy.get_caller_id() + "I heard %s", val.data)

def main():
	f = test2()
	t = test1()



if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
