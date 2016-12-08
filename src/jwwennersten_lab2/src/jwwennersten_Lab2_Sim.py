#!/usr/bin/env python

import rospy, tf, math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion
# Add additional imports for each of the message types used

# To make publishing easier, just give linear and angular velocity
def pubTwist(lin, ang):
    global pub
    msg = Twist()
    msg.linear.x = lin
    msg.angular.z = ang
    pub.publish(msg)

# Drive directly to a Nav Goal from rviz
def navToPose(goal):
    global pose
    goalPose = goal.pose
    
    # Calculate the angle and distance to the goal
    deltaX = goalPose.position.x - pose.position.x
    deltaY = goalPose.position.y - pose.position.y
    theta = math.degrees(math.atan2(deltaY, deltaX)) # Global frame
    dist = math.sqrt(deltaX**2 + deltaY**2)
    
    # Convert goal orientation from quaternion to a yaw angle
    quat = (
        goalPose.orientation.x,
        goalPose.orientation.y,
        goalPose.orientation.z,
        goalPose.orientation.w)
    euler = euler_from_quaternion(quat)
    goalAngle = math.degrees(euler[2]) # Yaw

    # Rotate until facing the goal
    print "spin!"
    rotateTo(theta, 0.25)
    rospy.sleep(0.5)

    # Drive straight until on goal
    print "move!"
    driveStraight(0.25, dist)
    rospy.sleep(0.5)

    # Rotate until facing proper direction
    print "spin!"
    rotateTo(goalAngle, 0.25)
    rospy.sleep(0.5)

    print "done"



#This function sequentially calls methods to perform a predefined trajectory.
def executeTrajectory():
    # Drive straight 60 cm
    driveStraight(0.2, 0.6)
    # Rotate right 90 deg
    rotate(-90, 0.5)
    # Drive straight 45 cm
    driveStraight(0.2, 0.45)
    # Rotate left 135 deg
    rotate(135, 0.5)



#This function accepts two wheel velocities (m/s) and a time interval (s).
def spinWheels(vL, vR, time):
    global pub
    
    # Calculate linear and angular velocity from wheel velocities
    wheelbase = 0.25 # meters
    omega = (vR - vL) / wheelbase
    if vL == vR: # Don't want to divide by zero, driving straight
        vel = vL
    else:
        radius = (wheelbase * (vL + vR)) / (2 * (vR - vL))
        vel = radius * omega
    
    # Create messages for driving (linX and angZ velocities) and stopping (zeros)
    driveMsg = Twist()
    driveMsg.linear.x = vel
    driveMsg.angular.z = omega
    stopMsg = Twist()
    stopMsg.linear.x = 0
    stopMsg.angular.z = 0
    
    # Record the current time
    now = rospy.Time.now().secs
    # Send the drive message until the desired time has passed
    while (rospy.Time.now().secs - now <= time and not rospy.is_shutdown()):
        pub.publish(driveMsg)
    # Send the stop message
    pub.publish(stopMsg)



#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    global pose
    
    # Record initial position
    initX = pose.position.x
    initY = pose.position.y
    atTarget = False
    
    # Drive (twist(speed,0)) until distance has passed, based on current and initial positions
    while not atTarget and not rospy.is_shutdown():
        if not (pose.position.x == 0 and pose.position.y == 0): # Account for occasional 0 position
            currentX = pose.position.x
            currentY = pose.position.y
        currentDist = math.sqrt((currentX - initX)**2 + (currentY - initY)**2)
        if currentDist >= distance:
            atTarget = True
            pubTwist(0,0)
        else:
            pubTwist(speed,0)
            rospy.sleep(0.15)
    
    # Stop
    pubTwist(0,0)



#Accepts a relative angle and makes the robot rotate by that much. (-180 to 180)
def rotate(theta, speed):
    global pose
    
    if theta < -180 or theta > 180:
        print "Keep angle within -180 to 180 please."
    
    # Record initial orientation
    initAng = math.degrees(pose.orientation.z)
    atTarget = False
    
    # Make sure rotating by smallest angle
    if theta > 180:
        theta = -360 + theta
    elif theta < -180:
        theta = 360 + theta
    
    # Determine rotation direction
    if theta > 0:
        angVel = speed
    elif theta < 0:
        angVel = -speed
    else:
        atTarget = True
    
    # Rotate (twist(0,speed)) until proper angle reached, based on current and initial rotations
    while not atTarget and not rospy.is_shutdown():
        currentAng = math.degrees(pose.orientation.z)
        traveled = currentAng - initAng
        
        # Account for wraparound
        if traveled > 180:
            traveled = -360 + traveled
        elif traveled < -180:
            traveled = 360 + traveled
            
        if abs(traveled) > abs(theta):
            atTarget = True
            pubTwist(0,0)
        else:
            pubTwist(0,angVel)
            rospy.sleep(0.15)
    
    # Stop
    pubTwist(0,0)



#Accepts a global angle and makes the robot rotate to it. (-180 to 180)
def rotateTo(angle, speed):
    global pose
    
    initAng = math.degrees(pose.orientation.z)
    theta = angle - initAng
    # Make sure rotating by smallest angle
    if theta > 180:
        theta = -360 + theta
    elif theta < -180:
        theta = 360 + theta
    rotate(theta, speed)



#(Extra credit) This function works the same as rotate how ever it does not publish linear velocities.
# def driveArc(radius, speed, angle):
    # Calculate the drive time for the arc
    # Calculate the wheel speeds needed to achieve the desired turning radius and time
    # Call spinWheels with the calculated wheel speeds and time
    
    # pass  # Delete this 'pass' once implemented



#Bumper Event Callback function
def readBumper(msg):
    if msg.bumper == 1 and msg.state == 1:
        # STOP
        #pubTwist(0,0)
        # Back up
        #driveStraight(-0.2, 0.25)
        # Turn around or rotate 90 deg or something
        #rotate(0.5, 90)
        # Go back to normal
        
        # Trigger executeTrajectory()
        executeTrajectory()



# TF update timer, keep track of current location
# Start the timer with the following line of code:
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def tfCallback(event):
    global pose
    global theta
    pose = Pose()

    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
    pose.position.x = position[0]
    pose.position.y = position[1]
    
    q = [orientation[0], orientation[1], orientation[2], orientation[3]] # quaternion nonsense
    roll, pitch, yaw = euler_from_quaternion(q)
    pose.orientation.z = yaw



# This is the program's main function
if __name__ == '__main__':
    rospy.init_node('jwwennersten_Lab2_node')

    # Globals, write "global <variable_name>" to gain access
    global pub
    global pose
    global odom_tf
    global odom_list

    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 10) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    rviz_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, navToPose, queue_size=1) # Callback function to handle rviz nav goals

    odom_list = tf.TransformListener() # Use this object to get the robot's Odometry
    rospy.Timer(rospy.Duration(.01), tfCallback) # Timer for updating current robot location

    rospy.sleep(1) # Use this command to make the program wait for some seconds

    print "Starting Lab 2"

    # Make the robot do stuff...
    
    # Demo spinWheels()
    print "Demonstrating spinWheels()"
    rospy.sleep(0.5)
    spinWheels(0.2, 0.2, 1.0)
    rospy.sleep(0.5)
    spinWheels(0.1, -0.1, 1.0)
    rospy.sleep(0.5)
    spinWheels(-0.15, -0.3, 1.0)
    rospy.sleep(1)
    
    # Demo driveStraight()
    print "Demonstrating driveStraight()"
    rospy.sleep(0.5)
    driveStraight(0.2, 0.5)
    rospy.sleep(1)
    
    # Demo rotate()
    print "Demonstrating rotate()"
    rospy.sleep(0.5)
    rotate(45, 0.3)
    rospy.sleep(0.5)
    rotate(-90, 0.3)
    rospy.sleep(0.5)
    rotate(45, 0.3)
    rospy.sleep(1)
    
    rospy.spin()

    print "Lab 2 complete!"
