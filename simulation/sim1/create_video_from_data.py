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
import Strings_config as cf
from Strings_sim_plot_objects import *

nb=30
ni=404
err=0.0
radius=0.038
radius2=0.04
R=0.967094387
mode='nmax'
#sim='09_06_2021_11_52_06' #oval
#sim='09_06_2021_12_03_20' # square
sim='09_06_2021_12_16_39' # triangle

#sim = '09_06_2021_11_52_06'

if not os.path.exists(sim):
    os.mkdir(sim)
    

# 04_04_2021_12_30_10
# 04_04_2021_12_30_17
single_entry=False

width=1.75
tim=[10,20]
Rb=1
#ext="F:/Soro_chrono/python/Pychrono/Strings/Strings_final/_results/_single_shapes/09_06_2021_12_16_39_triangle"
#ext="F:/Soro_chrono/python/Pychrono/Strings/Strings_final/_results/_single_shapes/09_06_2021_11_52_06_oval"
#ext="F:/Soro_chrono/python/Pychrono/Strings/Strings_final/_results/_single_shapes/09_06_2021_12_03_20_square"
#filename='F:/soro_chrono/python/Pychrono/Strings/Strings_final/'+sim+'/robot_data_'+sim+'/bot_position.csv'
#filename2='F:/soro_chrono/python/Pychrono/Strings/Strings_final/'+sim+'/robot_data_'+sim+'/particle_position.csv'

# filename=ext+'/robot_data_'+sim+'/bot_position.csv'
# filename2=ext+'/robot_data_'+sim+'/particle_position.csv'
# filename3='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/Desfield/'+sim+'.npz'
# filename4='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/radius/R'+sim+'.npz'

filename='F:/soro_chrono/python/Pychrono/Strings/Strings_final/robot_data_'+sim+'/bot_position.csv'
filename2='F:/soro_chrono/python/Pychrono/Strings/Strings_final/robot_data_'+sim+'/particle_position.csv'
filename3='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/Desfield/'+sim+'.npz'
filename4='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/radius/R'+sim+'.npz'
filename9='F:/soro_chrono/python/Pychrono/Strings/Strings_final/robot_data_'+sim+'/membrane_position.csv'
result_dir='F:/soro_chrono/python/Pychrono/Strings/Strings_final/'


phi=create_videos_irr(filename,filename2,filename3,filename4,None,None,None,None,None,result_dir,nb,ni,mode,R,radius,radius2,sim,width,err,tim,Rb)
phi.sort_data()
if single_entry==False:
    
    phi.create_images_nmax_1shape()


    directory='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/'+sim+'_video'
    export='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/'+sim+'field'
    file='jpg'
    create_video(sim,directory,export,file)
else:
    entry=24
    titl="Square"
    xticks = np.linspace(-1.0, 1.0,3,endpoint=True)
    #xticks=[-1.0,0,1.5]
    yticks=[-1.0,0,1.0]
    #yticks=[-1,0,1.5]
    #xticks = np.linspace(-1.5,1.5,3,endpoint=True)
    phi.create_images_nmax_snapshot_1shape(entry,titl,xticks,yticks)