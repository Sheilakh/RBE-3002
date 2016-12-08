import rospy, tf
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion
import math
from Queue import PriorityQueue

from Robot import *
from MyMap import *

def set_goal(pose, arg):
	print "set_goal"
	arg.set_goal(pose)

def map_callback(data, arg):
	#print "map_callback"
	arg.update_map(data)

def init_robot(vel_pub, path_pub, wall_pub, v = False):
	if v:
		print"initialize robot"
	_map = MyMap()
	robot = Robot(vel_pub, path_pub, wall_pub, _map, v)
	return robot

#rospy.Timer(rospy.Duration(1), dummy_cb)
#def dummy_cb(e):
#	print "dummy"

def main():
	rospy.init_node('lab5')
	vel_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)
	path_pub = rospy.Publisher("/path", GridCells, queue_size=1)
	wall_pub = rospy.Publisher("/walls", GridCells, queue_size=1)

	robot = init_robot(vel_pub, path_pub, wall_pub, v = True)

	map_sub = rospy.Subscriber("/map", OccupancyGrid, map_callback, (robot))
	goal_sub = rospy.Subscriber('/move_base_simple/goal2', PoseStamped, set_goal, (robot))
	rospy.sleep(2)

	rospy.Timer(rospy.Duration(0.1), robot.odom_timer_cb)
	rospy.Timer(rospy.Duration(1), robot.wall_pub_timer_cb)
	rospy.Timer(rospy.Duration(1), robot.path_pub_timer_cb)
	#print("here")
	while(not rospy.is_shutdown()):
		robot.execute()
		rospy.sleep(1)

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
