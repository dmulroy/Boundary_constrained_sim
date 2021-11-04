# -*- coding: utf-8 -*-
"""
Created on Sat Feb 27 10:42:52 2021

@author: dmulr
"""
"""
This module contains many controllers for Sphero(s) Mini/Bolt to be used
with ROS.
"""

###################################################
from numpy import zeros, ndarray
import numpy as np
from math import atan2, sqrt, pi
from time import time
from time import sleep
import datetime
import rospy
from std_msgs.msg import String
from pid_controller.pid import PID
from .Spheros import SpheroMini, SpheroBolt, \
    MultiSpheroMini, MultiSpheroBolt
from scipy import stats
from scipy.interpolate import RegularGridInterpolator
import statistics
from scipy.optimize import curve_fit
from scipy.interpolate import Rbf
from scipy import signal

###################################################


###################################################
# ------------------- Classes ------------------- #
###################################################


###################################################

###################################################


###################################################

###################################################


###################################################
class SpheroController:
    """PID controller of a single Sphero.

    Parameters
    ----------
    single_sphero : SpheroMini or SpheroBolt or int
        The Sphero instance to be controlled. If an int,
        this is the controller for a simulation.
    kp : float, optional
        Proportional gain of the PID.
    ki : float, optional
        Integral gain of the PID.
    kd : float, optional
        Derivative gain of the PID.
    is_alone : bool, optional
        Set to False if the controller is used in a swarm (multiple controller).

    Attributes
    ----------
    sphero : SpheroMini or SpheroBolt or None
        The Sphero instance that is controlled.
    at_id : int
        The Apriltag ID of the Sphero.
    x : int
        Current position in x.
    y : int
        Current position in y.
    tar_x : int
        Target position in x.
    tar_y : int
        Target position in y.
    _ulim : int
        Ultimate speed limit for the Sphero.
    _freq : float
        Maximum frequency to send command to the Sphero via Bluetooth.
    speed_ctl : PID
        The speed PID controller.
    last : float
        The time at which data was last sent to the Sphero via Bluetooth.
    """

    def __init__(self, single_sphero, kp=1., ki=0., kd=0., is_alone=True, method='pot'):
        """Constructor of the SpheroController class.

        Parameters
        ----------
        single_sphero : SpheroMini or SpheroBolt or int
            The Sphero instance to be controlled. If an int,
            this is the controller for a simulation.
        kp : float, optional
            Proportional gain of the PID.
        ki : float, optional
            Integral gain of the PID.
        kd : float, optional
            Derivative gain of the PID.
        is_alone : bool, optional
            Set to False if the controller is used in a swarm (multiple controller).

        Yields
        ------
        SpheroController
            A PID controller for a single Sphero Mini/Bolt.
        """

        self.x = self.y = self.tar_x = self.tar_y = 0
        self._ulim = 255
        self._freq = 5
        # Check if this is a simulated Sphero.
        if type(single_sphero) is int:
            self.sphero = None
            self.at_id = single_sphero
        else:
            self.sphero = single_sphero
            self.at_id = single_sphero.id

        # Initialize the PID speed controller.
        self.speed_ctl = PID(p=kp, i=ki, d=kd)
        self.speed_ctl.target = 0
        # Set the last data sending time as now.
        self.last = time()
        self.PX=0
        self.PY=0
        self.check=0
        self.tstart=time()
        # If this is not a part of a swarm, start ROS
        if is_alone:
            rospy.init_node("Sphero_" + str(self.at_id), anonymous=False)
            rospy.Subscriber("target", String, self.ros_set_target, queue_size=2)
            rospy.Subscriber("state", String, self.ros_callback, queue_size=2)
            rospy.spin()
        self.control_method = method
        self.A = kp
        self.alpha = ki
        self.beta = kd
        self.t_k_1 = 0


        # mode 1: shapes
        # mode 2: point field
        # mode 3: grasping
        self.modes = 1
        if self.modes == 1:
            self.res = 2  # resolution
            self.b = 150  # field length
            self.px = 0
            self.py = 0
            self.shape = 'Circle'  # what shape do you want
            self.r1 = 60# outer radius
            self.r2 = self.r1 - 5 # inner radius
            self.nb = 12  # number of bots
            self.R = 1
            if self.shape == 'Square':
                self.nr = [0, 10, 20, 30]  # rings for additional points outer field 
                self.nr2 = np.arange(0, self.r1 / 2, 3)  # rings for additional points inner  field 

            if self.shape == 'Triangle':
                self.nr = [0, 10, 20]  # rings for additional points outer field
                self.nr2 = [0, 5, 10]  # rings for additional points inner field

            if self.shape == 'Circle':
                self.nr = [0, 10, 20,30]  # rings for additional points outer field
                self.nr2 = [0, 5,10]

            self.xmin = -self.b  # set x min value
            self.xmax = self.b  # set x max value
            self.ymin = -self.b  # set y min value
            self.ymax = self.b  # set y max value
            self.xcount = int(round((self.xmax - self.xmin) / self.res))  # find out how many numbers in the x
            self.ycount = int(round((self.ymax - self.ymin) / self.res))  # find out how many numbers in the y
            self.txp = np.linspace(self.xmin, self.xmax, self.xcount)  # set up range for x
            self.typ = np.linspace(self.ymin, self.ymax, self.ycount)  # set up range for y
            self.xxt, self.yyt = np.meshgrid(self.txp, self.typ)  # create mesh grid



            (self.x, self.y, self.z, self.x2, self.y2,self.z2) = self.Points_for_shape()  # create points for the field

            self.rbf = Rbf(self.x, self.y, self.z,function='thin_plate')  # create radial basis function 1 outer field

            self.rbf2 = Rbf(self.x2, self.y2, self.z2,function='thin_plate')  # create radial basis function inner field
            self.zz1 = self.rbf(self.xxt, self.yyt)  # create pouints for rbf
            self.zz2 = self.rbf2(self.xxt, self.yyt)  # create pouints for rbf
            self.zz1 = self.zz1 / np.max(self.zz1)  # normalize the first field
            self.zz1 = np.maximum(0, self.zz1)  # maximize values so it floors anything negative
            self.zz2 = self.zz2 / np.max(self.zz2)  # normalize inner ring
            self.zz2 = self.zz2  # switch the signs
            self.zz2 = np.maximum(0, self.zz2)  # flat line it so negative values are zero

            self.zz = self.zz1 + self.zz2  # add the fields up
            (self.fy, self.fx) = np.gradient(self.zz)  # find gradient



        if self.mode==1:
            self.PotField = self.controller(self.px, self.py, self.zz, self.fx, self.fy, self.b, self.res,self.tstart,vmax=100)
            self.pot_loop = 0

        else:
            self.PotField = self.controller2(self.px, self.py, self.tstart, vmax=255)
            self.pot_loop = 0
            
    def Points_for_shape(self):
        # Square
        if self.shape == 'Square':
            (self.x, self.y, self.z, self.x2, self.y2, self.z2) = self.points_square()
        # Traingle 
        if self.shape == 'Triangle':
            #print('started')
            (self.x, self.y, self.z, self.x2, self.y2, self.z2) = self.points_triangle()
            #print('ended')
        if self.shape == 'Circle':
            (self.x, self.y, self.z, self.x2, self.y2, self.z2) = self.points_Circle()

        return (self.x, self.y, self.z, self.x2, self.y2, self.z2)


    def points_Circle(self):
    # I hate this system
        # x points
        theta=np.linspace(0,2*np.pi,30)
        xt = np.cos(theta)
        yt = np.sin(theta)
        self.x = np.zeros(len(self.nr) * len(xt))  # empty x array
        self.y = np.zeros(len(self.nr) * len(xt))  # empty y array
        self.z = np.zeros(len(self.nr) * len(xt))  # empty z array

        # fill points for field
        for j in range(len(self.nr)):
            for i in range(len(xt)):
                nt = self.nr[j]  # ring displacement
                self.x[i + len(xt) * j] = (nt + self.r1) * xt[i]  # fill x positions
                self.y[i + len(xt) * j] = (nt + self.r1) * yt[i]  # fill y positions
                if j==0:
                    self.z[i + len(xt) * j] = 0  # fill z positions
                else:
                    self.z[i + len(xt) * j] = j  # fill z positions

        self.x2 = np.zeros(len(self.nr2) * len(xt))  # empty x2 arrays
        self.y2 = np.zeros(len(self.nr2) * len(xt))  # empty y2 arrays
        self.z2 = np.zeros(len(self.nr2) * len(xt))  # empty z2 arrays

        for j in range(len(self.nr2)):
            for i in range(len(xt)):
                self.x2[i + len(xt) * j] = (self.r2 - self.nr2[j]) * xt[i]  # fill x2 ring
                self.y2[i + len(xt) * j] = (self.r2 - self.nr2[j]) * yt[i]  # fill y2 ring
                if j == 0:
                    self.z2[i + len(xt) * j] = 0  # fill z positions
                else:
                    self.z2[i + len(xt) * j] = j  # fill z positions
        return (self.x, self.y, self.z, self.x2, self.y2, self.z2)


    def points_square(self):
        # x points
        xt = np.array([1, 1, 1, 1, 1, .75, .5, .25, 0, -.25, -.5, -.75, -1, -1, -1, -1, -1, -1, -1, -1, -1, -.75, -.5, -.25,0, .25, .5, .75, 1, 1, 1, 1])
        # y points
        yt = np.array([0, .25, .5, .75, 1, 1, 1, 1, 1, 1, 1, 1, 1, .75, .5, .25, 0, -.25, -.5, -.75, -1, -1, -1, -1, -1, -1,-1, -1, -1, -.75, -.5, -.25])
        self.x = np.zeros(len(self.nr) * len(xt))  # empty x array
        self.y = np.zeros(len(self.nr) * len(xt))  # empty y array
        self.z = np.zeros(len(self.nr) * len(xt))  # empty z array

        # fill points for field
        for j in range(len(self.nr)):
            for i in range(len(xt)):
                nt = self.nr[j]  # ring displacement
                self.x[i + len(xt) * j] = (nt + self.r2) * xt[i]  # fill x positions
                self.y[i + len(xt) * j] = (nt + self.r2) * yt[i]  # fill y positions
                self.z[i + len(xt) * j] = nt  # fill z positions
        self.x2 = np.zeros(len(self.nr2) * len(xt))  # empty x2 arrays
        self.y2 = np.zeros(len(self.nr2) * len(xt))  # empty y2 arrays
        self.z2 = np.zeros(len(self.nr2) * len(xt))  # empty z2 arrays
        for j in range(len(self.nr2)):
            for i in range(len(xt)):
                self.x2[i + len(xt) * j] = (self.r1 - self.nr2[j]) * xt[i]  # fill x2 ring
                self.y2[i + len(xt) * j] = (self.r1 - self.nr2[j]) * yt[i]  # fill y2 ring
                self.z2[i + len(xt) * j] = -abs(j)  # fill z2 ring
        return (self.x, self.y, self.z, self.x2, self.y2, self.z2)

    def points_triangle(self):
        n = 3  # number of sides
        theta = np.linspace(0, 2 * np.pi, 30)  # number of points for zero contour
        r = np.cos(np.pi / n) / np.cos((theta % (2 * np.pi / n)) - (np.pi / n))  # radius of contour
        xt = r * np.cos(theta)  # zero iso contour points x
        yt = r * np.sin(theta)  # zero iso conout points y
        self.x = np.zeros(len(self.nr) * len(xt))  # empty x array
        self.y = np.zeros(len(self.nr) * len(xt))  # empty y array
        self.z = np.zeros(len(self.nr) * len(xt))  # empty z array
        for j in range(len(self.nr)):
            for i in range(len(xt)):
                self.x[i + len(xt) * j] = (self.nr[j] + self.r1) * xt[i]  # fill x points
                self.y[i + len(xt) * j] = (self.nr[j] + self.r1) * yt[i]  # fill y points
                if j == 0:
                    self.z[i + len(xt) * j] = 0  # fill z points
                else:
                    self.z[i + len(xt) * j] = j  # fill z points
        self.x2 = np.zeros(len(self.nr2) * len(xt))  # empty array for x2 points
        self.y2 = np.zeros(len(self.nr2) * len(xt))  # empty array for y2 points
        self.z2 = np.zeros(len(self.nr2) * len(xt))  # empty array for z2 points
        for j in range(len(self.nr2)):
            for i in range(len(xt)):
                self.x2[i + len(xt) * j] = (self.r2 - self.nr2[j]) * xt[i]  # fill x2 points
                self.y2[i + len(xt) * j] = (self.r2 - self.nr2[j]) * yt[i]  # fill y2 points
                if j == 0:
                    self.z2[i + len(xt) * j] = 0  # fill z2 points
                else:
                    self.z2[i + len(xt) * j] = j
        return (self.x, self.y, self.z, self.x2, self.y2, self.z2)




    class controller:
        def __init__(self, px, py, zz, fx, fy, b, res, tstart,vmax):

            self.px = px  # center x
            self.py = py# center y
            self.zz = zz  # grid of field
            self.fx = fx  # grid of gradient x
            self.fy = fy  # grid of gradient y
            self.b = b  # how far left or right
            self.res = res  # resolution
            self.vmax = vmax
            self.alpha = self.vmax  # max velocity
            self.beta = 0  # dampning term
            self.A = .001  # constant for checking error
            self.tstart=tstart
            self.rr=5

            # Settting up the grid of the potential field
            self.xmin = self.px - self.b  # minimum x value based on center
            self.xmax = self.px + self.b  # maximum x value based on center
            self.ymin = self.py - self.b  # minimum y value based on center
            self.ymax = self.py + self.b  # maximum y value based on center
            self.xcount = int(round((self.xmax - self.xmin) / self.res))  # find out how many numbers in the x
            self.ycount = int(round((self.ymax - self.ymin) / self.res))  # find out how many numbers in the y
            self.xp = np.linspace(self.xmin, self.xmax, self.xcount)  # set up range for x
            self.yp = np.linspace(self.ymin, self.ymax, self.ycount)  # set up range for y
            self.f = RegularGridInterpolator((self.yp, self.xp), self.zz**2)  # interpolated function for field
            self.fny = RegularGridInterpolator((self.yp, self.xp),self.fy)  # interpolated function  x gradient of field
            self.fnx = RegularGridInterpolator((self.yp, self.xp),self.fx)  # interpolated function y gradient of field


        def out_vel1(self, x, y, vx, vy):
            ''' Controller for shape formation '''
            #A = np.round(self.f((y, x)), 4)  # check error
            A=self.f((y, x))
            print('Error= ', A)  # print error 

            # if the error is low enough then it doesnt move 
            #if A < self.A:
            #    V = 0
            #    VX = 0
            #    VY = 0
            #    theta = 0
            #    print('gucci')
            #else:
            VX = self.fnx((y, x))  # x velocity direction
            VY = self.fny((y, x))  # y veloicty direction

            mag = np.sqrt(VY ** 2 + VX ** 2)  # magnitude
            VY = VY / mag  # normalize  Y
            VX = VX / mag  # normalize X
            Vx = -self.alpha * VX  # - self.beta * vx
            Vy = -self.alpha * VY  # - self.beta * vy
            V = np.sqrt(Vx ** 2 + Vy ** 2)
            theta = np.nan_to_num(180 * np.arctan2(Vy, Vx) / np.pi) % 360  # set heading angle
            tt = abs(self.tstart - time())
            ang2 = signal.square(2 * np.pi * .2 * tt)*45
            theta=theta + ang2
            return (V, theta)

        def tanh(self,t):
            p=2
            return((np.exp(p*t)-1)/(np.exp(p*t)+1))

        def out_vel_2(self, x, y, vx, vy):
            ''' Controller for  grasping'''
            if abs(self.tstart-time())>20:
                tt=abs(self.tstart-time())-20
                r=(1-self.tanh(tt))*self.rr*20+(self.tanh(tt))*self.rr
            else:
                r=20*self.rr
            print('radius: ',np.round(r,2))
            VY = self.Fy(x, y,r)
            VX = self.Fx(x, y,r)
            mag = np.sqrt(VY ** 2 + VX ** 2)  # magnitude
            VY = VY / mag  # normalize  Y
            VX = VX / mag  # normalize X
            Vx = -self.alpha * VX # - self.beta * vx
            Vy = -self.alpha * VY  # - self.beta * vy
            V = np.sqrt(Vx ** 2 + Vy ** 2)

            print('V= ',V)
            theta = np.nan_to_num(180 * np.arctan2(Vy, Vx) / np.pi) % 360  # set heading angle
            # print(self.r1,self.r2)
            return (V, theta)

        def out_vel_3(self, x, y, vx, vy):
            ''' Controller for  moving around'''
            VY = self.Fy2(x, y,self.px,self.py)
            VX = self.Fx2(x, y,self.px,self.py)
            mag = np.sqrt(VY ** 2 + VX ** 2)  # magnitude
            VY = VY / mag  # normalize  Y
            VX = VX / mag  # normalize X
            Vx = -self.alpha * VX # - self.beta * vx
            Vy = -self.alpha * VY  # - self.beta * vy
            V = np.sqrt(Vx ** 2 + Vy ** 2)
            theta = np.nan_to_num(180 * np.arctan2(Vy, Vx) / np.pi) % 360  # set heading angle
            # print(self.r1,self.r2)
            return (V, theta)



    class controller2:
        def __init__(self, px, py,tstart,vmax,mode):

            self.px = px  # center x
            self.py = py# center y
            self.vmax = vmax
            self.alpha = self.vmax  # max velocity
            self.beta = 0  # dampning term
            self.tstart=tstart
            self.rr=2
            self.mode=mode
            
            
        def F(self, x, y, px, py, a, c):
            Y = (y - py)
            X = (x - px)
            d = np.sqrt((X ** 2) / (a ** 2) + (Y ** 2) / (c ** 2))
            return (d**2 * np.log(d))  

        def Fx(self, x, y, px, py, a, c):
            Y = (y - py)
            X = (x - px)
            d = np.sqrt((X ** 2) / (a ** 2) + (Y ** 2) / (c ** 2))
            return (((2 * x - 2 * px) / a ** 2) * np.log(d) + (2 * x - 2 * px) / 2 * (a) ** 2)


        def Fy(self, x, y, px, py, a, c):
            Y = (y - py)
            X = (x - px)
            d = np.sqrt((X ** 2) / (a ** 2) + (Y ** 2) / (c ** 2))
            return (((2 * y - 2 * py) / c ** 2) * np.log(d) + (2 * y - 2 * py) / 2 * (c) ** 2)


        def F2x(self, x, y, px, py, a, c):
            return (self.F(x, y, px, py, a, c) * self.Fx(x, y, px, py, a, c))


        def F2y(self, x, y, px, py, a, c):
            return (self.F(x, y, px, py, a, c) * self.Fy(x, y, px, py, a, c))


        
        def Fx2(self,x,y,px,py):
            return((x-px)/(np.sqrt((x-px)**2 + (y-py)**2)))

        def Fy2(self,x,y,px,py):
            return((y-py)/(np.sqrt((x-px)**2 + (y-py)**2)))


        def tanh(self,t):
            p=.5
            return((np.exp(p*t)-1)/(np.exp(p*t)+1))



        def out_vel2(self, x, y, vx, vy):
            ''' Controller for  grasping'''
            tt = abs(self.tstart - time())
            if tt>10:
                #ttt=tt-10
                px=self.px
                r = self.rr
                r2 = self.rr
                #r2=self.rr
                #r=(1-self.tanh(ttt))*self.rr*20+(self.tanh(ttt))*self.rr
                self.alpha=255
            else:
                r = self.rr
                r2 = self.rr*30
                self.alpha = 255

            VY = self.F2y(x, y,px,self.py,r2,r)
            VX = self.F2x(x, y,px,self.py,r2,r)
            mag = np.sqrt(VY ** 2 + VX ** 2)  # magnitude
            VY = VY / mag  # normalize  Y
            VX = VX / mag  # normalize X
            Vx = -self.alpha * VX # - self.beta * vx
            Vy = -self.alpha * VY  # - self.beta * vy
            V = np.sqrt(Vx ** 2 + Vy ** 2)
            theta = np.nan_to_num(180 * np.arctan2(Vy, Vx) / np.pi) % 360  # set heading angle
            theta=theta
            # print(self.r1,self.r2)
            return (V, theta)

        def out_vel3(self, x, y, vx, vy):
            ''' Controller for  moving around'''
            VY = self.Fy2(x, y,self.px,self.py)
            VX = self.Fx2(x, y,self.px,self.py)
            mag = np.sqrt(VY ** 2 + VX ** 2)  # magnitude
            VY = VY / mag  # normalize  Y
            VX = VX / mag  # normalize X
            Vx = -self.alpha * VX # - self.beta * vx
            Vy = -self.alpha * VY  # - self.beta * vy
            V = np.sqrt(Vx ** 2 + Vy ** 2)
            theta = np.nan_to_num(180 * np.arctan2(Vy, Vx) / np.pi) % 360  # set heading angle
            # print(self.r1,self.r2)

            return (V, theta)

        def out_vel4(self, x, y, vx, vy):
            ''' Controller for tunneling'''
            VY = self.Fy2(x, y,self.px,self.py)
            VX = self.Fx2(x, y,self.px,self.py)
            mag = np.sqrt(VY ** 2 + VX ** 2)  # magnitude
            VY = VY / mag  # normalize  Y
            VX = VX / mag  # normalize X
            Vx = -self.alpha * VX # - self.beta * vx
            Vy = -self.alpha * VY  # - self.beta * vy
            V = np.sqrt(Vx ** 2 + Vy ** 2)
            theta = np.nan_to_num(180 * np.arctan2(Vy, Vx) / np.pi) % 360  # set heading angle

            tt = abs(self.tstart - time())
            ang2 = signal.square(2 * np.pi * .2 * tt)*45
            # print(self.r1,self.r2)
            theta=theta + ang2
            return (V, theta)


    def get_error_length(self):
        """Method to get the length between the current position and the target position.

        Returns
        -------
        float
            The length between the current position and the target position.
        """

        dx = self.tar_x - self.x
        dy = self.tar_y - self.y
        return sqrt(dx ** 2 + dy ** 2)

    def get_error_angle(self):
        """Method to get the orientation of the target relative to the current position.

        Returns
        -------
        float
            The orientation of the target relative to the current position.
        """

        dx = self.tar_x - self.x
        dy = self.tar_y - self.y
        ang = 180 * atan2(dy, dx) / pi
        return ang % 360

    def ros_set_target(self, data):
        """Method used to update the closed loop target positions. Used with a ROS Subscriber.

        Parameters
        ----------
        data : String
            Message published on the /target topic containing the position of the target tag.
        """
        # print("at_id", self.at_id)
        data_dict = eval(data.data)
        for key in data_dict:
            # print("data_dict:", data_dict)
            self.tar_x = data_dict[key][0]
            self.tar_y = data_dict[key][1]
            print("target_key", key)

    def ros_callback(self, data):
        """Method used to run an iteration of the control loop. Used with a ROS Subscriber.

        Parameters
        ----------
        data : String
            Message published on the /state topic containing the position of all tags.
        """

        data_dict = eval(data.data)

        vx = (data_dict[str(self.at_id)][0] - self.x) / (time() - self.t_k_1)
        vy = (data_dict[str(self.at_id)][1] - self.y) / (time() - self.t_k_1)

        # Set the current positions
        self.x = data_dict[str(self.at_id)][0]
        self.y = data_dict[str(self.at_id)][1]
        self.t_k_1 = time()
        X = []
        Y = []

        for idx, i in data_dict.items():
            X = np.append(X, data_dict[str(idx)][0])
            Y = np.append(Y, data_dict[str(idx)][1])

        X = np.append(X, data_dict[str(idx)][0])
        Y = np.append(Y, data_dict[str(idx)][1])
        # Potential field controller
        ################################################################################################################
        px = self.tar_x
        py = self.tar_y

        tt=abs(self.tstart-time())

        self.PotField = self.controller2(px, py, self.tstart, vmax=255)
        mag, ang = self.PotField.out_vel2(self.x, self.y, 0, 0)

        if self.control_method == 'pot':
            head = int(ang)
            speed =int(mag)

        elif self.control_method == 'pid':
            head = int(self.get_error_angle())
            speed = max(min(int(self.speed_ctl(feedback=self.get_error_length())), self.ulim), 0)


        print("\n Bot#",self.at_id, "Target pos:",np.round(self.tar_x,2), np.round(self.tar_y,2),"Bot pos:",np.round(data_dict[str(self.at_id)][0],2), np.round(data_dict[str(self.at_id)][1],2), "speed_pot:", speed,"time: ",np.round(tt,3))


        # Send it to the sphero
        freq = 1.0 / (time() - self.last)
        #print('freq:', self.freq)
        #print('other freq',freq)
        if freq <= self.freq:
            self.last = time()

            if self.sphero is None:
                print("\r[{0}] ----> {1:3d}° at {2} % speed.".format(self.at_id, head, int(100 * speed / 255)))
            else:
                self.sphero.send(speed=speed, heading=head)

    @property
    def ulim(self):
        return self._ulim

    @ulim.setter
    def ulim(self, lim):
        self._ulim = max(0, min(lim, 255))

    @property
    def freq(self):
        return self._freq

    @freq.setter
    def freq(self, f):
        self._freq = max(0., min(f, 20.))


###################################################


###################################################
class MultiSpheroController:
    """PID controller of a multiple Spheros.

    Parameters
    ----------
    multi_sphero : MultiSpheroMini or MultiSpheroBolt or tuple of int
        The Spheros swarm instance to be controlled. If a tuple,
        this is the controllers for a simulation.
    kp : float, optional
        Proportional gain of the PID.
    ki : float, optional
        Integral gain of the PID.
    kd : float, optional
        Derivative gain of the PID.

    Attributes
    ----------
    spheros : ndarray of SpheroMini or ndarray of SpheroBolt or tuple of int
        An numpy array of the SpheroMini/SpheroBolt or of the Apriltag IDs if this is a simulation.
    num : int
        Number of Sphero in the swarm.
    ids : tuple of int
        A tuple containing the Apriltag ID of the Spheros.
    swarm : MultiSpheroMini or MultiSpheroBolt or None
        The Sphero swarm instance or None if this is a simulation.
    controllers : ndarray of SpheroController
        An numpy array of the SpheroController.
    last : float
        The time at which data was last received from the /state topic.
    last_tar : float
        The time at which data was last received from the /target topic.
    freq_tar : float
        Frequency at which the /target topic receives messages.
    """

    def __init__(self, multi_sphero, kp=1., ki=0., kd=0., method='pot'):
        """Constructor of the SpheroController class.

        Parameters
        ----------
        multi_sphero : MultiSpheroMini or MultiSpheroBolt or tuple of int
            The Spheros swarm instance to be controlled. If a tuple,
            this is the controllers for a simulation.
        kp : float, optional
            Proportional gain of the PID.
        ki : float, optional
            Integral gain of the PID.
        kd : float, optional
            Derivative gain of the PID.

        Yields
        ------
        MultiSpheroController
            A PID controller for multiple Sphero Mini/Bolt.
        """

        if type(multi_sphero) is tuple:
            self.spheros = self.ids = multi_sphero
            self.num = len(multi_sphero)
            self.swarm = None
        else:
            self.swarm = multi_sphero
            self.spheros = self.swarm.spheros
            self.num = self.swarm.num
            self.ids = self.swarm.ids

        self.controllers = zeros(self.num, dtype=SpheroController)
        for i in range(self.num):
            self.controllers[i] = SpheroController(self.spheros[i], kp, ki, kd, is_alone=False, method=method)

        self.last = self.last_tar = time()
        self.freq_tar = 0
        print("Frequencies :\n| Apriltags |  Target  | Timeout Error |")

        rospy.init_node("Spheros" + str(self.ids[0]), anonymous=False)
        rospy.Subscriber("target", String, self.ros_set_target, queue_size=2)
        rospy.Subscriber("state", String, self.ros_callback, queue_size=2)
        rospy.spin()

    def __del__(self):
        """ Destructor of the SpheroController class. """

        print("\b\b  \n")

    def ros_set_target(self, data):
        """Method used to update the closed loop target positions. Used with a ROS Subscriber.

        Parameters
        ----------
        data : String
            Message published on the /target topic containing the position of the target tag.
        """

        self.freq_tar = 1. / (time() - self.last_tar)
        self.last_tar = time()
        print("target data:", data)
        for ctl in self.controllers:
            ctl.ros_set_target(data)

    def ros_callback(self, data):
        """Method used to run an iteration of the control loop. Used with a ROS Subscriber.

        Parameters
        ----------
        data : String
            Message published on the /state topic containing the position of all tags.
        """

        freq = 1. / (time() - self.last)

        self.last = time()
        print(
            "\r| {0:2.2f} Hz  | {1:2.2f} Hz | {2} Errors    |      ".format(freq, self.freq_tar, self.get_sum_error()),
            end='')
        for ctl in self.controllers:
            ctl.ros_callback(data)

    def get_sum_error(self):
        """ Method the total number of"""

        if type(self.spheros[0]) is SpheroMini:
            return "N/A"
        else:
            val = 0
            for s in self.spheros:
                val += s.toerr

            return val
###################################################
