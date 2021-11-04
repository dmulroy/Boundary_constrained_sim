# -*- coding: utf-8 -*-
"""
Created on Thu Jun  3 10:02:41 2021

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

radius=.038
radius2=.04
Rb=.382671
#Rb=0.14166912118906336
#Rb=.5*np.pi*Rb

width=4 # width of window
height=cf.height # height of bot
sim="18_08_2021_16_26_24" # sim name
err=0 # additional spacing for interior

# geom options 
#circle .382671
# square  .5*np.pi*self.Rb
# triangle .4627



geom="circle" # geometry of the ball
 
single_entry=True # if its a single entry or video


# create path
if not os.path.exists(sim):
    os.mkdir(sim)
    


direct=os.path.dirname(__file__)
file=direct+'/robot_data_'+sim+'/'



### File direction for specific data ###
filename1=file+"bot_position.csv"# name for robot
filename2=file+"particle_position.csv" # name for interior
filename3="F:/Soro_chrono/python/Pychrono/Strings/Strings_final/radius/R"+sim+".npz" # name for radius data
filename4=file+"ballx.csv" # ball x position
filename5=file+"ballz.csv" # ball z position
filename6=file+"Force_controller.csv"  # force controllers   
filename7=file+"ballFb.csv" # pull force of ball   
filename8=file+"x contact force.csv" # x contact force         
filename9=file+"y contact force.csv" # y contact force         
filename10=file+"z contact force.csv" # y contact force
filename11=file+"AN.csv" # AN
filename12=file+"BN.csv" # BN
filename13=file+"x contact points.csv" # x contact point       
filename14=file+"y contact points.csv" # y contact point    
filename15=file+"z contact points.csv" # z contact point
filename16=file+"nc.csv" # nc       
filename17=file+"bphi.csv" # phi angle of ball
filename18=file+"btheta.csv" # theta angle of ball
filename19=file+"bpsi.csv" # psi angle of ball
filename20=file+"AID.csv" # contact ID A
filename21=file+"BID.csv" # contact ID B
filename22=file+"bot_TotalForces.csv" # bot total force 
filename23=file+'/membrane_position.csv'
filename24=file+'/particle_angles.csv'

result_dir=direct+'/'

if not os.path.isdir(result_dir):
    os.makedirs(result_dir)


phi=grasping_analysis(filename1,filename2,filename3,filename4,filename5,filename6,filename7,filename8,filename9,filename10,filename11,filename12,filename13,filename14,filename15,filename16,filename17,filename18,filename19,filename20,filename21,filename22,filename23,filename24,result_dir,nb,ni,radius,radius2,Rb,sim,width,err,height) 
phi.sort_data()  
phi.sort_data_forces()

if single_entry==False:
    phi.create_images_ball(geom)
    directory=result_dir+sim+'_video_'
    export=result_dir+sim+'field_'
    file='jpg'
    create_video(sim,directory,export,file)
    
else:
    phi.ball_forces_entry(25,geom)



