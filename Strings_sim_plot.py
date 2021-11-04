# -*- coding: utf-8 -*-
"""
Created on Tue May 12 12:50:52 2020

@author: dmulr
"""

import os
import csv
import timeit
import numpy as np
import math as math
import Strings_objects as sim_obj
import Strings_config as cf
from Strings_sim_plot_objects import *
sim='19_05_2021_19_16_40' # name of file 
nb=30
n=[41, 51, 31, 36, 20, 21, 9, 5]
start=timeit.default_timer()
Rb=1.148012668/3
control_type="pot_field_grab"
#path='C:/Users/dmulr/OneDrive/Documents/soro_chrono/python/Pychrono/Strings/Strings_final/'+'robot_data_'+sim+'/'
direct=os.path.dirname(__file__)
path=direct+'/robot_data_'+sim+'/'
phi=robot_plots(sim,control_type,path,nb,n,cf.radius,cf.radius2,cf.height,ball_radius=Rb)   
#end=timeit.default_timer()
#print('Time plotting: ',(end-start)/60,'mins')


