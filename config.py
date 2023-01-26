# -*- coding: utf-8 -*-
"""
Created on Wed Jul 21 14:14:49 2021

@author: dmulr
"""

import os
import pathlib
from shutil import copyfile
import csv
import time
import numpy as np
from datetime import datetime
from shutil import copyfile

#### SIMULATION MODES ####
dimension = '2D' #2D: 2D sim   3D: 3D sim
dt = 0.001 # time step 
time_end = 30
save_rate = 200 #save every n number of steps
visual = 'pov'
pwm = 255
fixed = False
xcenter = 0
zcenter = 0.0

#### Control Modes ####
'''
Control modes:
shape formation: Control mode specific to shape formation
shape_morphing: morphing 
target_chasing
'''
 
control_mode = "grasping"
#### GEOMETRIES ####
"""
circle:
square:
wrench
"""

#### CONVERSION VARIABLES ####
'''
cm: convert_dist = 100
meters: convert_dist = 1

kg: convert mass = 1
grams: convert mass = 1000

'''
convert_dist = 1 # if its meters or cm
convert_mass = 1 # if its grams or kg


#### ROBOT VARIABLES #####
nb = 30 # number of bots
bot_mass = .200  # mass of bot kg 
bot_geom = 'cylinder'
bot_width = 0.09525  #  bot width  [m] 
bot_height = 0.09525   # bot height    [m]
#bot_height = bot_width
membrane_type = 1
skin_width = 0.035 # diameter of membrane particles
ns = 5# number of skin particles 
membrane_width = ns*skin_width # cloth width [m]
spring_stiffness =60 # spring stiffness
spring_damping = 0 # spring damping 
theta = 2*np.pi/nb 
cord_length = membrane_width + bot_width*np.cos(theta/2) # Cord length between bot centers
R = np.sqrt((cord_length**2)/(2*(1-np.cos(theta)))) # radius [m]
#R = 0.5
#R = 0.4
membrane_density = 2000
#print(R)

ns=7
#### INTERIOR PROPERTIES ####
'''
"bi_dispersion_ring"
"bidispersion"
"monodispersion"
"bi_dispersion_uniform_ring"
experiment_shape
tunnel0
tunnel1
tunnel2
'''



particle_mass = 0.01 # kg 
particle_width = 0.0762 # meters
particle_width2 = 0.1016 # meters
particle_height = bot_height # meters
particle_geom = 'cylinder'
interior_mode = "tunnel0" # interior particle mode
scale_radius = 1 # scale the radius (percentage the R is scale down by )
offset_radius = 0

#### FLOOR PARAMETERS ####
floor_length=100 # Length of the body floor
floor_height=particle_height     # height of the body floor


#### ENVIROMENT PARAMETERS ####
lateralFriction = 0.1 # friction
spinningFriction = 0.1# spinning friction
rollingFriction = 0.1 # rolling friction
dampingterm = 0.001 # damping 
Ctr = 0.000000 # tangent compliane
Cr = 0.0000000 # compliance
Crr = 0.000000 # rolling compliance
Csr = 0.000000 # sliding compliance


Ctp = 0.000000 # tangent compliane
Cp = 0.0000000 # compliance
Crp = 0.000000 # rolling compliance
Csp = 0.000000 # sliding compliance

#### CONTROL PARAMETERS ####
if control_mode=="shape_formation":
    geometry = 'triangle'
    delta = 1 # spacing between the 
    d = 75
    if geometry == "oval":
        a = 40
        b = 0.7*40
        epsilon = 1
        npoints = 100
        step = 2
        gap = 5
        
    if geometry == "square":
        Rshape = 30
        epsilon = 6
        npoints = 100
        step = 2
        gap = 5
        
    if geometry == "triangle":
        Rshape = 27
        epsilon = 5
        npoints = 90
        step = 3
        epsilon = 5
        gap = 5
        
    scale = 1
    alpha = 0.45
    beta = 0

##### SSHAPE MORPHING #####
if control_mode=="shape_morphing":
    geometries=["circle","oval","square","triangle"]
    tcut=[10,20,30]    
    delta = 1 # spacing between the 
    d = 175   
    p = 0.5
    scale = 1
    alpha = 2
    beta = 0

    # circle
    a1 = R*100
    b1 = R*100
    epsilon1 = 1
    npoints1 = 100
    step1 = 2
    gap1 = 5


    # oval
    a2 = R*1.15*100
    b2 = 0.8*R*100
    epsilon2 = 1
    npoints2 = 100
    step2 = 2
    gap2 = 5
        
    # square
    Rshape3 = 0.83*R*100
    epsilon3 = 6
    npoints3 = 100
    step3 = 2
    gap3 = 1
    
    # triangle
    Rshape4 = 0.75*R*100
    epsilon4 = 5
    npoints4 = 90
    step4 = 3
    epsilon4 = 5
    gap4 = 1 
    
    
    

if control_mode=="grasping":
    d = 1
    ball_geometry="triangle"
    ball_radius=R*.3
    ballx=R+ball_radius+.3
    ballz=0 
    
    p = 10
    tcut = 2
    
    a1 = .01
    b1 = 1
    
    a2 = .01
    b2 = .01 
    
    

    
    xc1 = ballx+ball_radius+1
    yc1 = 0 
    
    xc2 = ballx+ball_radius+.1
    yc2 = 0
    
    
    alpha1 = 1.0
    alpha2 = 2
    beta1 = 0    
    beta2 = 0   
    
    tcut1 = 3
    tcut2 = 5
    tcut3=8
    

if control_mode=="target_chasing":
    d = 1
    xc = -12
    zc = -3 
    alpha = .5
    beta = 0    
    
    
#### SAVE SIMULATION ####
now = datetime.now()
dt_string = now.strftime("%d_%m_%Y_%H_%M_%S")
mainDirectory = "F:/Soro_chrono/python/Pychrono/Strings/Strings_soro/"
savefile = mainDirectory +'Experiments/'+ dt_string
os.makedirs(savefile, exist_ok=True)
txtFile = savefile+'/Parameters.csv'
    




##### export as npy file
envParams = {}
envParams['dt'] = dt
envParams['time_end'] = time_end
envParams['save_rate'] = save_rate
envParams['convert_dist'] = convert_dist
envParams['convert_mass'] = convert_mass
envParams['visual'] = visual
envParams['xcenter'] = xcenter
envParams['zcenter'] = zcenter
envParams['fixed'] = fixed
# control mode
envParams['control_mode'] = control_mode
envParams['pwm'] = pwm



if control_mode=="grasping":
    envParams["p"] = p
    envParams["tcut"] = tcut
    
    envParams["a1"] = a1
    envParams["b1"] = b1
    
    envParams["a2"] = a2
    envParams["b2"] = b2    
    
    envParams["xc1"] = xc1
    envParams["yc1"] = yc1
    
    envParams["xc2"] = xc2
    envParams["yc2"] = yc2  
    
    envParams["tcut1"] = tcut1
    envParams["tcut2"] = tcut2    
    envParams["tcut3"] = tcut3 
    
    envParams["alpha1"] = alpha1
    envParams["alpha2"] = alpha2
    
    envParams["beta1"] = beta1   
    envParams["beta2"] = beta2     
    envParams["ball_geometry"] = ball_geometry
    envParams["ball_radius"] = ball_radius
    envParams["ballx"] = ballx
    envParams["ballz"] = ballz        
    
    
    
    
if control_mode=='shape_formation':
    envParams['geometry'] = geometry
    envParams["gap"] = gap
    envParams["epsilon"] = epsilon
    envParams["npoints"] = npoints
    envParams["step"] = step
    envParams['alpha'] = alpha
    envParams['beta'] = beta
    envParams['d'] = d  
    envParams['delta'] = delta
    
    if geometry == "square":
       envParams["Rshape"] = Rshape
       
    if geometry == "triangle":
       envParams["Rshape"] = Rshape 
       
    if geometry == "oval":
        envParams["a"] = a
        envParams["b"] = b

if control_mode=='shape_morphing':
    envParams["geometries"] = geometries  
    envParams["delta"] = delta # spacing between the 
    envParams["d"] = d   
    envParams["tcut"] = tcut
    envParams["p"] = p
    
    envParams["scale"] = scale
    envParams["alpha"] =alpha
    envParams["beta"] = beta
    
    # circle
    envParams["a1"] = a1
    envParams["b1"] = b1
    envParams["epsilon1"] = epsilon1
    envParams["npoints1"] = npoints1
    envParams["step1"] = step1
    envParams["gap1"] = gap1    
    
    # oval
    envParams["a2"] = a2
    envParams["b2"] = b2
    envParams["epsilon2"] = epsilon2
    envParams["npoints2"] = npoints2
    envParams["step2"] = step2
    envParams["gap2"] = gap2
        
    # square
    envParams["Rshape3"] = Rshape3
    envParams["epsilon3"] = epsilon3
    envParams["npoints3"] = npoints3
    envParams["step3"] = step3
    envParams["gap3"] = gap3
    
    # triangle
    envParams["Rshape4"] = Rshape4
    envParams["epsilon4"] = epsilon4
    envParams["npoints4"] = npoints4
    envParams["step4"] = step4
    envParams["epsilon4"] = epsilon4
    envParams["gap4"] = gap4 


if control_mode=="target_chasing":
    envParams["xc"] = xc
    envParams["zc"] = zc
    envParams["alpha"] = alpha  
    envParams["beta"] = beta

        

# Robot Parameters
envParams['nb'] = nb 
envParams['bot_mass'] = bot_mass
envParams['bot_geom'] = bot_geom
envParams['bot_width'] = bot_width
envParams['bot_height'] = bot_height
envParams['membrane_width'] = membrane_width
envParams['skin_width'] = skin_width
envParams['membrane_type'] = membrane_type
envParams['cord_length'] = cord_length
envParams['spring_stiffness'] = spring_stiffness
envParams['spring_damping'] = spring_damping
envParams['ns'] = ns
envParams['R'] = R
envParams['membrane_density']=membrane_density
# Particle Parameters  
envParams['interior_mode'] = interior_mode
envParams['particle_mass'] = particle_mass
envParams['particle_width'] = particle_width
envParams['particle_width2'] = particle_width2
envParams['particle_height'] = particle_height 
envParams['particle_geom'] = particle_geom
envParams['offset_radius'] = offset_radius
envParams['scale_radius'] = scale_radius
# floor parameters
envParams['floor_length'] = floor_length
envParams['floor_height'] = floor_height



# Physical Paramters
envParams['lateralFriction'] = lateralFriction
envParams['spinningFriction'] = spinningFriction
envParams['rollingFriction'] = rollingFriction
envParams['dampingterm'] = dampingterm
envParams['Ctr'] = Ctr
envParams['Cr'] = Cr
envParams['Crr'] = Crr
envParams['Csr'] = Csr

envParams['Ctp'] = Ctp
envParams['Cp'] = Cp
envParams['Crp'] = Crp
envParams['Csp'] = Csp


envParams['number_parameters'] = len(envParams)

np.save(savefile+'/Parameters.npy',envParams)


copyfile(__file__,savefile+"/"+'config.py')


#data=np.load(savefile+'/Parameters.npy',allow_pickle=True)
#data=data.tolist()