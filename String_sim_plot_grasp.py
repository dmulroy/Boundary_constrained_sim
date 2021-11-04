# -*- coding: utf-8 -*-
"""
Created on Sun Feb 21 11:35:52 2021

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
n=404
radius=.038
radius2=.04
height=.1
Rb=.38

########## OLD METHOD ####################
# triangle
# path1='15_06_2021_19_40_27'
# path2='15_06_2021_19_40_38'
# path3='15_06_2021_19_40_47'
# circle
# path1='12_06_2021_09_58_55'
# path2='12_06_2021_09_59_03'
# path3='12_06_2021_09_59_09'
# square
# path1='12_06_2021_10_01_57'
# path2='12_06_2021_10_02_05'
# path3='12_06_2021_10_02_15'
############NEW METHIOD########################

# circle
path1 = "18_08_2021_16_26_17"
path2 = "18_08_2021_16_26_24"
path3 = "18_08_2021_16_26_31"

# # square
# path1 = "18_08_2021_17_11_56"
# path2 = "18_08_2021_17_12_02"
# path3 = "18_08_2021_17_12_14"

# triangle
# path1 = "18_08_2021_17_39_41"
# path2 = "18_08_2021_17_39_47"
# path3 = "18_08_2021_17_39_53"


# path1 = "25_08_2021_10_33_36"
# path2 = "25_08_2021_10_33_29"
# path3 = "25_08_2021_10_33_18"


name1=r'$\alpha$=20 (N)'
name2=r'$\alpha$=40 (N)'
name3=r'$\alpha$=60 (N)'


name=[name1,name2,name3]
path=[path1,path2,path3]


phi=compare_grasp(name,path,n,nb,radius,radius2,height,Rb)

yticks=[0.0,0.2,0.4,0.6,0.8,1]
#yticks=[1.7,1.8,1.9,2.0,2.1,2.2,2.3]
xticks=[0,200,400,600,800]
titl='Circle'
phi.Pull_force_vs_position(titl,xticks,yticks)
