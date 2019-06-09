import robot
import odrive.core
import time
import math
import sympy
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt


from sympy import symbols
from sympy import sin, cos, asin, acos, atan, pi, sqrt, diff
from sympy.utilities.lambdify import lambdify
from sympy import Matrix

import IPython
from IPython import get_ipython
get_ipython().run_line_magic('matplotlib', 'inline')


#%matplotlib inline
import matplotlib
#import numpy as np
#import matplotlib.pyplot as plt


# Make a new leg variable which is a robot.Leg class

#leg = robot.Leg(False)


left_pos = 0
right_pos = 0

pos = [0,3*pi/4]
slpt = [.3,.3]
vel = [1, 1]

#leg.set_joint_pos(right_pos, left_pos)

for i in range(10):
    ind = i%2
    
    tim = slpt[ind]
    velo = vel[ind] 
    left_pos = pi - pos[ind]
    right_pos = pos[ind]
    leg.set_joint_pos(right_pos, left_pos,velo,velo)
    time.sleep(tim)
leg.move_home()

print('pos',leg.joint_0_pos, leg.joint_1_pos)

