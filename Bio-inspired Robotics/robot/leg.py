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


l1 =  7.3  
l2 = 14.3
l_base = 8
pi = np.pi
encoder2rad = 2048 * 4
theta0_sym, theta1_sym, alpha0_sym, alpha1_sym = symbols('theta0_sym theta1_sym alpha0_sym alpha1_sym', real=True)

class Leg:
        
    def __init__(self, simulate = True):  
        
        self.simulate = simulate
        
        self.t0 = list()
        self.t1 = list()
        self.a0 = list()
        self.a1 = list()
        
        if self.simulate == False:
            self.drv = self.connect_to_controller()
            self.m0 = self.drv.motor0  
            self.m1 = self.drv.motor1
        else:
            self.drv = None

        
        # current positions
        self.initial_done = 0
        
        self.joint_0_pos, self.joint_1_pos = self.get_joint_pos()
        
         # home angles
        self.joint_0_home, self.joint_1_home = self.set_home()
        
        self.initial_done = 1

        # We will compute the jacobian and inverse just once in the class initialization.
        # This will be done symbolically so that we can use the inverse without having
        # to recompute it every time
        
        self.Jacobian = self.compute_jacobian()
        self.Jacobian_inv = self.Jacobian.inv
        

    def connect_to_controller(self):
        drv = odrive.core.find_any(consider_usb=True, consider_serial=False)

        if drv is None:
            print('No controller found')
        else:
            print('Connected!')
        return drv

    ###
    ### Motion functions
    ###
    
    def get_joint_pos(self):
        # if simulating exit function
        if self.simulate == True:
            m0_pos = 2
            m1_pos = 1.14
        else:
            m0_pos = self.m0.encoder.pll_pos*(2*pi)/encoder2rad+2
            m1_pos = self.m1.encoder.pll_pos*(2*pi)/encoder2rad+1.14
            
        if self.initial_done == 1:
            m0_pos = self.joint_0_pos
            m1_pos = self.joint_1_pos
            
        #print('Get joint pos',m0_pos, m1_pos)
        return (m0_pos, m1_pos)

        
    def set_home(self):
        """
        This function updates the home locations of the motors so that 
        all move commands we execute are relative to this location. 
        """
        # if simulating exit function
        if self.simulate == True:
            return (0,0)
        else:
            m0_pos, m1_pos = self.get_joint_pos()
            print('set_home',m0_pos, m1_pos)
            return(m0_pos,m1_pos) # the home is (0,0) but we will put the two legs as (0,pi) deg at the beginning


    def set_joint_pos(self, theta0, theta1, vel0=1, vel1=1, curr0=1, curr1=1):
        """
        Set the joint positions in units of deg, and with respect to the joint homes.
        We have the option of passing the velocity and current feedforward terms.
        """
        # if simulating exit function
        if self.simulate == True:
            
            self.joint_0_pos = theta0
            self.joint_1_pos = theta1
            
            #print('set the = ',theta0+theta1)
            
            return
        else:
            self.m0.set_pos_setpoint((theta0-self.joint_0_home)/(2*pi)*encoder2rad,vel0,curr0)
            self.m1.set_pos_setpoint((theta1-self.joint_1_home)/(2*pi)*encoder2rad,vel1,curr1)
            
            self.joint_0_pos = theta0
            self.joint_1_pos = theta1
            return


    def move_home(self):
        """
        Move the motors to the home position
        """
        # if simulating exit function
        if self.simulate == True:
            self.joint_0_pos = self.joing_0_home
            self.joint_1_pos = self.joing_1_home
            return
        else:
            self.set_joint_pos(2,1,14)
            return


    def set_foot_pos(self, x, y):
        """
        Move the foot to position x, y. This function will call the inverse kinematics 
        solver and then call set_joint_pos with the appropriate angles
        """
        
        theta_0, theta_1 = self.inverse_kinematics(x, y)
        self.set_joint_pos(theta_0, theta_1, vel0=0, vel1=0, curr0=0, curr1=0)


    def move_trajectory(self, tt, xx, yy):
        """
        Move the foot over a cyclic trajectory to positions xx, yy in time tt. 
        This will repeatedly call the set_foot_pos function to the new foot 
        location specified by the trajectory.
        """
        self.t0 = list()
        self.t1 = list()
        self.a0 = list()
        self.a1 = list()
        
        for i in range(tt):
            print(i)
            the0, the1 = self.inverse_kinematics(xx[i],yy[i])
            print(the0, the1)
            self.t0.append(the0)
            self.t1.append(the1)
            alp1, alp2 = self.compute_internal_angles(the0, the1)
            self.a0.append(alp1)
            self.a1.append(alp2)
            self.set_joint_pos(the0,the1)
            
        np.savetxt('jiao', (self.t0, self.t1))
        if self.simulate == True:
            return (self.t0, self.t1, self.a0, self.a1)
        else:
            for i in range(tt):
                self.set_joint_pos(self.t0[i],self.t1[i])
                
            return (self.t0, self.t1, self.a0, self.a1)
    ###
    ### Leg geometry functions
    ###

    def compute_internal_angles(self, theta_0, theta_1):
        
        # compute length of the virtual link
        d = sqrt(l_base**2 + l1**2 - 2*l_base*l1*cos(theta_0))
        
        # compute increment
        beta = -asin(l1/d*sin(theta_0))
        
        # define input with respect to increment angle
        #q1 = theta_1 - beta
        # define coefficient matrices
        A_q1 = 2*l1*l2*cos(theta_1) + 2*l2*d*cos(beta)
        B_q1 = 2*l1*l2*sin(theta_1) + 2*l2*d*sin(beta)
        C_q1 = -l1**2-d**2-2*d*l1*cos(theta_1-beta)
        # compute internal angles
        alpha_1 = atan(B_q1/A_q1) + acos(C_q1/sqrt(A_q1**2 + B_q1**2))
        alpha_0 = acos((l1*cos(theta_1-beta) + l2*cos(alpha_1-beta)+d)/l2) + beta

        return (alpha_0, alpha_1)
    


    def compute_jacobian(self):
        """
        This function implements the symbolic solution to the Jacobian.
        """

        # initiate the symbolic variables
        
        alpha0_sym, alpha1_sym = self.compute_internal_angles(theta0_sym, theta1_sym)

        # form FK matrix
        x = l_base/2 + l1*cos(theta0_sym) + l2*cos(alpha0_sym)
        y = l1*sin(theta1_sym) + l2*sin(alpha1_sym)
        
#        y = l1*sin(theta0_sym) + l2*sin(alpha0_sym)
        
        # solve for Jacobian
        Jacobian = Matrix([[diff(x, theta0_sym), diff(x, theta1_sym)], [diff(y, theta0_sym), diff(y, theta1_sym)]])
        
        #J = FK.jacobian(theta0_sym, theta1_sym)
        return(Jacobian) 
        

    def inverse_kinematics(self, x_target, y_target):
        """
        This function will compute the required theta_0 and theta_1 angles to position the 
        foot to the point x, y. We will use an iterative solver to determine the angles.
        """
        # initial guess on angles
        #if theta_current is []:
        t0 = self.joint_0_pos
        t1 = self.joint_1_pos
        theta_current = np.array([t0, t1])
        theta_current = np.reshape(theta_current, (2,1))
        
        
        # define increment and threshold
        incre = 2e-1
        epsilon = 1e-1
        count = 0
        # iterate to solve
        while True:
            count = count +1
            if count > 100:
                incre = 5e-3
            # compute forward kinematics
            alpha_0, alpha_1 = self.compute_internal_angles(theta_current[0], theta_current[1])
            
            x = l_base/2 + l1*cos(theta_current[1]) + l2*cos(alpha_1)
            y = l1*sin(theta_current[1]) + l2*sin(alpha_1)
            
            #y = l1*sin(theta_current[0]) + l2*sin(alpha_0)
            
            # compute error
            error = Matrix([(x_target - x), (y_target - y)])
            
            #print(error.norm())
            
            # when to stop
            if error.norm() < epsilon: 
                break
            
            # update angles
            J_lambda = self.Jacobian.subs({theta0_sym: theta_current[0], theta1_sym: theta_current[1]})
            # J_mid = J_lambda(theta_current[0],theta_current[1])
            J_lambda = sympy.N(J_lambda)
            # J_lambda = float(J_lambda)
            J_inv_lambda = J_lambda.pinv()
            theta_current = incre*J_inv_lambda*error + theta_current
        
        return (theta_current[0], theta_current[1])
    

   
    