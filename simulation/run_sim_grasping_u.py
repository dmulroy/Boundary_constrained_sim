 # -*- coding: utf-8 -*-
"""
Created on Thu Jul 22 12:12:32 2021

@author: dmulr
""" 
import pychrono as chrono
import timeit
start=timeit.default_timer()
#import objects_2 as sim_obj
import objects4 as sim_obj
#from config import *
import numpy as np
import random
import os
import csv
import glob

 

data_path="F:/data/"

# Create system
chrono.SetChronoDataPath(data_path)
my_system = chrono.ChSystemNSC() 
my_system.SetSolverType(chrono.ChSolver.Type_PSSOR)
#my_system.SetSolverMaxIterations(70)

my_system.Set_G_acc(chrono.ChVectorD(0,-9.81, 0)) 
#chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
#chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.00001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.00001)

path = os.path.dirname(__file__)
path=path+"/Experiments/grasping_u/"
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
Ball=sim_obj.Ball(name,my_system,body_floor,path)
forceb=Ball.forceb
# Report contact class
my_rep = sim_obj.MyReportContactCallback()

# Potential Fields
Psi=sim_obj.R_functions(name,path)

controls=sim_obj.controller(name,my_system,bots,Psi,Ball,path)

# Create simulation
sim = sim_obj.simulate(name,my_system,bots,interior,Ball,controls,my_rep,path,Psi)

# Run the simulation 
sim.simulate()
print("simulation run:",np.round((sim.sim_end-sim.sim_start)/60,2)," minutes")
#export data
data_export=sim_obj.export_data(my_system,bots,controls,interior,Ball,sim,Psi,my_rep,path,name)
#%%  
sim_export_start=timeit.default_timer()  
data_export.export_data()
    
sim_export_end=timeit.default_timer()
print("export_time:",np.round((sim_export_end-sim_export_start)/60,2)," minutes")
print("name =",str(name))

