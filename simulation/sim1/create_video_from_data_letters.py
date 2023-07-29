# -*- coding: utf-8 -*-
"""
Created on Mon Mar  8 10:14:09 2021

@author: dmulr
"""

import os
import csv
import timeit
import numpy as np
import math as math
import Strings_objects as sim_obj
#import Strings_config as cf
from Strings_sim_plot_objects import *
print('issue')
nb=60
ni=515
err=0.0
radius=0.038
radius2=0.05
R=0.967094387
Rb=1
mode='nmax'
sim='23_05_2021_14_00_57'
width=5
tim=[6,12,18,24,32,38]
single_entry=False
if not os.path.exists(sim):
    os.mkdir(sim)
#filename='F:/soro_chrono/python/Pychrono/Strings/Strings_final/'+sim+'/robot_data_'+sim+'/bot_position.csv'
#filename2='F:/soro_chrono/python/Pychrono/Strings/Strings_final/'+sim+'/robot_data_'+sim+'/particle_position.csv'

filename='F:/soro_chrono/python/Pychrono/Strings/Strings_final/robot_data_'+sim+'/bot_position.csv'
filename2='F:/soro_chrono/python/Pychrono/Strings/Strings_final/robot_data_'+sim+'/particle_position.csv'
filename3='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/Desfield/'+sim+'.npz'
filename4='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/radius/R'+sim+'.npz'
result_dir='F:/soro_chrono/python/Pychrono/Strings/Strings_final/'
phi=create_videos_irr(filename,filename2,filename3,filename4,None,None,None,None,None,result_dir,nb,ni,mode,R,radius,radius2,sim,width,err,tim,Rb)
phi.sort_data()
#phi.create_images_nmax()

#29
#59
#89
#119
#159
#189
#219
#phi.create_images_nmax_letter_1_letter(219)
if single_entry==False:
    
    phi.create_images_nmax_letter()
    directory='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/'+sim+'_video'
    export='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/'+sim+'field'
    file='jpg'
    create_video(sim,directory,export,file)    

else:
#29
#59
#89
#119
#159
#189
#219
    entry=219
    phi.create_images_nmax_letter_snap_shot(entry)  


# directory='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/'+sim+'_video'
# export='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/'+sim+'field'
# file='jpg'
# create_video(sim,directory,export,file)