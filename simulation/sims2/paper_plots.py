# -*- coding: utf-8 -*-
"""
Created on Sat Aug 13 15:10:42 2022

@author: dmulr
"""

import warnings
warnings.filterwarnings("ignore")
import pychrono.core as chrono
import timeit
import numpy as np
start=timeit.default_timer()
import objects4 as sim_obj
import random
import os
import csv
import glob
from IPython.display import HTML
import matplotlib.pyplot as plt

path = os.path.dirname(__file__)
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)

#entry=[0,100,109,180]
d=4
snap_shot=False
membrane=False
dxmin=-4
dxmax=2
dymin=-3
dymax=3

wxmin=dxmin
wxmax=dxmax
wymin=dymin
wymax=dymax
Psi=sim_obj.R_functions(name,path)  
sim_data=sim_obj.import_data(name,path,dxmin,dxmax,dymin,dymax,Psi)
fsx=1.
fsy=1.
sim_data.create_frames_u3_only_robot(membrane,wxmin,wxmax,wymin,wymax)

xticks=[-4,-2,0,2]
yticks=[-3,0,3]

sim_data.create_frames_u3_snapshots(membrane,wxmin,wxmax,wymin,wymax,fsx,fsy,xticks,yticks,entry)



