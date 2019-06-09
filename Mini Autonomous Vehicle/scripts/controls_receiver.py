#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16MultiArray as PWM

from actuator import PWMController
from config import *

# Array positions in data.data
# data = [right/left, up/down]

STEERING = 0
THROTTLE = 1

steering_controller = PWMController(channel=STEERING_CHANNEL)
throttle_controller = PWMController(channel=THROTTLE_CHANNEL)

slower_downer = 0

def listener():
    """Receives (listens for) controls from the ROS topic that sends controls"""
    
    rospy.init_node("actuator", anonymous=True) # Create subscriber node named actuator
    
    print("Running actuator subscriber")
    # Subscribe to "controls" topic
    # Receives messages of type std_msgs.msg.String
    # callback() is called when a message is received
    rospy.Subscriber("controls", PWM, callback)

    # Keeps Python from exiting until node is stopped
    rospy.spin()    

def callback(data):
    global slower_downer
    rospy.loginfo(rospy.get_caller_id() + " Data received: %s", data)
    
    new_throttle = data.data[THROTTLE]
    new_steering = data.data[STEERING]

    slower_downer += 1
    slower_downer %= 10
    print("slower downer %d" % slower_downer)
    if (slower_downer == 0):
        throttle_controller.set_pwm(THROTTLE_STOPPED_PWM)
    elif (new_throttle < THROTTLE_STOPPED_PWM+THROTTLE_RANGE and new_throttle > THROTTLE_STOPPED_PWM-THROTTLE_RANGE):
        print("new throttle %d" % new_throttle)
        throttle_controller.set_pwm(new_throttle)
    
    print("new steering %d" % new_steering)
    steering_controller.set_pwm(new_steering)


if __name__ == '__main__':
    listener()
