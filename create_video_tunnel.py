# -*- coding: utf-8 -*-
"""
Created on Mon May 31 15:23:55 2021

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
sim='18_09_2021_16_39_42'
#sim='18_09_2021_14_01_23'
width=20
tim=[10,20]

single_entry=False
if not os.path.exists(sim):
    os.mkdir(sim)
#filename='F:/soro_chrono/python/Pychrono/Strings/Strings_final/'+sim+'/robot_data_'+sim+'/bot_position.csv'
#filename2='F:/soro_chrono/python/Pychrono/Strings/Strings_final/'+sim+'/robot_data_'+sim+'/particle_position.csv'


direct=os.path.dirname(__file__)
path=direct+'/robot_data_'+sim+'/'

# bot position
filename=path+'bot_position.csv'

# particle positions
filename2=path+'particle_position.csv'

# desfield
filename3=direct+'/Desfield/'+sim+'.npz'

#radius
filename4=direct+'/radius/R'+sim+'.npz'

# control forces applied
filename7=path+'/Force_controller.csv'

# membrane position
filename9=path+'/membrane_position.csv'

# result directory where we add 
result_dir=path

# create video
phi=create_videos_irr(filename,filename2,None,filename4,None,None,filename7,None,filename9,result_dir,nb,ni,mode,R,radius,radius2,sim,width,err,tim,None)

# sort data
phi.sort_data()

if single_entry==False:
    phi.create_images_nmax_tunnel()
    directory=direct+'/_tunnel_video'
    export=direct+'/tunnel'
    file='jpg'
    create_video(sim,directory,export,file)

else:
    #phi.create_images_nmax_tunnel_entry_2(15)
    entry=[0,360,480]
    phi.create_images_nmax_tunnel_entry(entry)