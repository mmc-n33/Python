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
import time
from config import *
from std_msgs.msg import Int16MultiArray

# get the curses screen window
screen = curses.initscr()

# turn off input echoing
curses.noecho()

# respond to keys immediately (don't wait for enter)
curses.cbreak()

# map arrow keys to special values
screen.keypad(True)

safety_counter = 0
no_response_max_time = 5000 # milliseconds

#def timeout():
#    safety_counter += 1
#    screen.addstr(3, 65, str(safety_counter))
#    if safety_counter > no_response_max_time:
#        pwm_input.data = [steering_neutral,throttle_neutral]
#        screen.addstr(3, 0, '<<<<<<<<<<<< TIMEOUT REACHED, RESETTING TO NEUTRAL >>>>>>>>>>>')
#        safety_counter = 0
    
def talker():
    pub = rospy.Publisher('controls', Int16MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    steering_neutral = round((STEERING_RIGHT_PWM + STEERING_LEFT_PWM)/2.0)
    throttle_neutral = THROTTLE_STOPPED_PWM + 10
    throttle_max = THROTTLE_STOPPED_PWM + THROTTLE_RANGE
    throttle_min = THROTTLE_STOPPED_PWM - THROTTLE_RANGE
    steering_scale = 15
    steering_string_range = round((STEERING_RIGHT_PWM - STEERING_LEFT_PWM)/(2*steering_scale))
    while not rospy.is_shutdown():
        pwm_input = Int16MultiArray()
        pwm_input.data = [steering_neutral,throttle_neutral]
        screen.addstr(0, 0, \
                      'Receiving Inputs Now\n'\
                      'DO NOT PRESS CTRL-C TO QUIT\n'\
                      'Must press q instead\n'\
                      'Currently outputting:'\
                      '[Steering PWM, Throttle PWM] = \n\n')
        screen.addstr(3, 52, str(pwm_input.data))
        screen.addstr(5, 0, 'Steering Angle:')
        try:
            while True:
                screen.timeout(no_response_max_time)                                                         
                char = screen.getch()
                if char == ord('q'):
                    return
                elif char == curses.KEY_RIGHT:
                    # print doesn't work with curses, use addstr instead
                    if pwm_input.data[0] >= STEERING_RIGHT_PWM:
                        # limit the steering max value (right side)
                        pwm_input.data[0] = STEERING_RIGHT_PWM
                    else:
                        pwm_input.data[0] += steering_scale
                    screen.addstr(3, 0, \
                                  'Currently outputting:'\
                                  '[Steering PWM, Throttle PWM] = ')
                    screen.addstr(3, 52, str(pwm_input.data))
                    steering_angle = round((pwm_input.data[0] - steering_neutral)/steering_scale)
                    whitespace_range = steering_string_range - abs(steering_angle)
                    whitespace = " "*whitespace_range
                    if steering_angle < 0:
                        steering_angle = -steering_angle
                        anglespace = '<'*steering_angle
                        display_angle = '|' + whitespace + anglespace + 'O' + " "*steering_string_range + '|'
                        screen.addstr(5,15, display_angle)
                    elif steering_angle > 0:
                        anglespace = '>'*steering_angle
                        display_angle = '|' + " "*steering_string_range + 'O' + anglespace + whitespace + '|'
                        screen.addstr(5,15, display_angle)
                    else:
                        display_angle = '|' + " "*steering_string_range + 'O' + " "*steering_string_range +\
                                        '|'
                        screen.addstr(5,15, display_angle)
                
                elif char == curses.KEY_LEFT:
                    if pwm_input.data[0] <= STEERING_LEFT_PWM:
                        # limit the steering max value (left side)
                        pwm_input.data[0] = STEERING_LEFT_PWM
                    else:
                        pwm_input.data[0] -= steering_scale
                    screen.addstr(3, 0, \
                                  'Currently outputting:'\
                                  '[Steering PWM, Throttle PWM] = ')
                    screen.addstr(3, 52, str(pwm_input.data))
                    steering_angle = round((pwm_input.data[0] - steering_neutral)/steering_scale)
                    whitespace_range = steering_string_range - abs(steering_angle)
                    whitespace = " "*whitespace_range
                    if steering_angle < 0:
                        steering_angle = -steering_angle
                        anglespace = '<'*steering_angle
                        display_angle = '|' + whitespace + anglespace + 'O' + " "*steering_string_range + '|'
                        screen.addstr(5,15, display_angle)
                    elif steering_angle > 0:
                        anglespace = '>'*steering_angle
                        display_angle = '|' + " "*steering_string_range + 'O' + anglespace + whitespace + '|'
                        screen.addstr(5,15, display_angle)
                    else:
                        display_angle = '|' + " "*steering_string_range + 'O' + " "*steering_string_range +\
                                        '|'
                        screen.addstr(5,15, display_angle)

                elif char == curses.KEY_UP:
                    if pwm_input.data[1] >= throttle_max:
                        # limit the steering max value
                        pwm_input.data[1] = throttle_max
                    else:
                        pwm_input.data[1] += 1
                    screen.addstr(3, 0, \
                                  'Currently outputting:'\
                                  '[Steering PWM, Throttle PWM] = ')
                    screen.addstr(3, 52, str(pwm_input.data))

                elif char == curses.KEY_DOWN:
                    if pwm_input.data[1] <= throttle_min:
                        # limit the steering max value
                        pwm_input.data[1] = throttle_min
                    else:
                        pwm_input.data[1] -= 1
                    screen.addstr(3, 0, \
                                  'Currently outputting:'\
                                  '[Steering PWM, Throttle PWM] = ')
                    screen.addstr(3, 52, str(pwm_input.data))
                        
                elif char == ord(' '):
                    # hard stop -- hitting spacebar resets steering/throttle to neutral 
                    pwm_input.data[1] = throttle_neutral
                    screen.addstr(3, 0, \
                                  'HARD STOP TRIGGERED: RESET TO NEUTRAL THROTTLE                     ')

                elif char == -1:
                    # Increase the safety counter, in case connection is lost this will trigger a stop
                    pwm_input.data = [steering_neutral,throttle_neutral]
                    screen.addstr(3, 0, '<<<<<<<<<<<<<< TIMEOUT REACHED, RESET TO NEUTRAL >>>>>>>>>>>>>  ')
                            
                    
                #if char == curses.KEY_RIGHT and char == curses.KEY_UP:
                #    pwm_input.data[0] += 1
                #    pwm_input.data[1] += 1
                #char = screen.getch()
                #rospy.logerr(pwm_input)
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
        talker()
    except rospy.ROSInterruptException:
        curses.nocbreak();
        screen.keypad(0);
        curses.echo()
        curses.endwin()
        pass
