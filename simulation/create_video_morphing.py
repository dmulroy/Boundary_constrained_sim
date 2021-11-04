# -*- coding: utf-8 -*-
"""
Created on Fri Jun 18 11:16:49 2021

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

sim='24_07_2021_19_44_30'



if not os.path.exists(sim):
    os.mkdir(sim)
    

single_video=False
width=1.75
tim=[10,20]
Rb=1



filename='F:/soro_chrono/python/Pychrono/Strings/Strings_final/robot_data_'+sim+'/bot_position.csv'
filename2='F:/soro_chrono/python/Pychrono/Strings/Strings_final/robot_data_'+sim+'/particle_position.csv'
filename3='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/Desfield/'+sim+'.npz'
filename4='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/radius/R'+sim+'.npz'
result_dir='F:/soro_chrono/python/Pychrono/Strings/Strings_final/'

phi=create_videos_irr(filename,filename2,filename3,filename4,None,None,None,None,None,result_dir,nb,ni,mode,R,radius,radius2,sim,width,err,tim,Rb)
phi.sort_data()

if single_video==False:
    phi.create_images_morphing()



    directory='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/'+sim+'_video'
    export='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/'+sim+'field'
    file='jpg'
    create_video(sim,directory,export,file)
    
else:
    # 0 
    # 199
    # 399
    # 598
    entry=399
    xticks = np.linspace(-1.5, 1.5,3,endpoint=True)
    yticks = xticks
    phi.create_images_morphing_snap_shot(entry,xticks,yticks)