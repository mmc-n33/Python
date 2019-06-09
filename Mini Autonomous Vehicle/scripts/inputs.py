#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import curses
from curses import wrapper
from config import *
from std_msgs.msg import Int16MultiArray

# get the curses screen window
#screen = curses.initscr()

# turn off input echoing
#curses.noecho()

# respond to keys immediately (don't wait for enter)
#curses.cbreak()

# map arrow keys to special values
#screen.keypad(True)

def talker(screen):
    pub = rospy.Publisher('controls', Int16MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pwm_input = Int16MultiArray()
        pwm_input.data = [0,0]
        try:
            while True:
                #tpwm_input = [1,2,3,4]
                char = screen.getch()
                #rospy.loginfo(pwm_input)
                #pub.publish(pwm_input)
                #rate.sleep()
                if char == ord('q'):
                    break
                elif char == curses.KEY_RIGHT:
                    # print doesn't work with curses, use addstr instead
                    pwm_input.data[0] += 1
                elif char == curses.KEY_LEFT:
                    screen.addstr(0, 0, 'left ')
                    pwm_input.data[0] -= 1
                elif char == curses.KEY_UP:
                    screen.addstr(0, 0, 'right')
                    pwm_input.data[1] += 1
                elif char == curses.KEY_DOWN:
                    pwm_input.data[1] -= 1
                #char = screen.getch()
                rospy.loginfo(pwm_input)
                pub.publish(pwm_input)
                rate.sleep()
                    
        finally:
            # shut down cleanly
            curses.nocbreak();
            screen.keypad(0);
            curses.echo()
            curses.endwin()
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(tpwm_input)
        #pub.publish(tpwm_input)
        #rate.sleep()

if __name__ == '__main__':
    try:
        wrapper(talker)
    except rospy.ROSInterruptException:
        curses.nocbreak();
        screen.keypad(0);
        curses.echo()
        curses.endwin()
        pass
