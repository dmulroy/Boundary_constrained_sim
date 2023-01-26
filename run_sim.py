# -*- coding: utf-8 -*-
"""
Created on Thu Jul 22 12:12:32 2021

@author: dmulr
"""

import pychrono.core as chrono
import timeit
start=timeit.default_timer()
#import objects2 as sim_obj
import objects as sim_obj
#from config import *
import random
import os
import csv
import glob

data_path="F:/data/"

# Create system
chrono.SetChronoDataPath(data_path)
my_system = chrono.ChSystemNSC() 
my_system.SetSolverType(chrono.ChSolver.Type_PSSOR)
my_system.Set_G_acc(chrono.ChVectorD(0,-9.81, 0)) 
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0000001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.000001)


path = os.path.dirname(__file__)
path=path+"/Experiments/"
os.chdir(path)
files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)
name = files[-1]


# Create the floor
Floor=sim_obj.floor(name,my_system,path)
body_floor=Floor.body_floor
(my_system)=Floor.return_enviroment()

# Create the robots
bots = sim_obj.robots(name,my_system,body_floor,path)
 
# Create the interiors
interior = sim_obj.Interiors(name,my_system,body_floor,path)

# Create object to be chases

ball=sim_obj.Ball(name,my_system,body_floor,path)

# Report contact class
my_rep = sim_obj.MyReportContactCallback()

# Potential Fields
Psi=sim_obj.Potential_fields(name)

controls=sim_obj.controller(name,my_system,bots,Psi,path,ball,interior=interior)

# Create simulation
sim = sim_obj.simulate(name,my_system,bots,interior,controls,my_rep,path,ball)

# Run the simulation 
sim.simulate()

# export data
data_export=sim_obj.export_data(my_system,bots,controls,interior,ball,sim,Psi,my_rep,path,name)
data_export.export_data()