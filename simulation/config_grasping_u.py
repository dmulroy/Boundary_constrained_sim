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
dt = 0.002 # time step 
time_end = 20
#time_end = 25
#time_end = 40
save_rate = 50 #save every n number of steps
visual = 'pov'

#xcenter = 2.25
xcenter = 0
zcenter = 0

#### Control Modes ####
'''
Control modes:
shape formation: Control mode specific to shape formation
shape morphing linear: linear shape morphing 
shape_morphing: transfinte morphing 
'''
 
control_mode = "grasping_explore"
control_mode = "grasping"
control_mode = "grasping_u"
#control_mode = "grasping_epsilon"
#control_mode = "Verify"
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
bot_height = 0.09525/2 # bot height    [m]
membrane_type = 1
skin_width = 0.035# diameter of membrane particles
ns = 7# number of skin particles 
membrane_width = ns*skin_width # cloth width [m]
spring_stiffness = 50 # spring stiffness
spring_damping = 0 # spring damping 
theta = 2*np.pi/nb 
cord_length = membrane_width + bot_width*np.cos(theta/2) # Cord length between bot centers
print(np.pi*((nb*cord_length/np.pi)/2)**2)
R = np.sqrt((cord_length**2)/(2*(1-np.cos(theta)))) # radius [m]
print(np.pi*(R)**2)
membrane_density = 2000

ns=7



#### INTERIOR PROPERTIES ####
'''

"bi_dispersion_ring"
"bidispersion"
"monodispersion"
"bi_dispersion_uniform_ring"
'''



particle_mass = 0.01 # kg 
particle_width = 0.0762 # meters
particle_width2 = 0.1016 # meters
#particle_width = 0.1016*np.sqrt(2) # meters
particle_height = bot_height # meters
particle_geom = 'cylinder'
interior_mode = "bidispersion" # interior particle mode
#interior_mode = "Verify"
scale_radius = 1.01
offset_radius = .78

Rbar = scale_radius*R
#### FLOOR PARAMETERS ####
floor_length=10 # Length of the body floor
floor_height=1     # height of the body floor


#### ENVIROMENT PARAMETERS ####
lateralFriction = 0.2
spinningFriction = 0.1
rollingFriction = 0.1
dampingterm = 0.00001
#dampingterm = 0
Ct=0
C=0
Cr=0
Cs=0
restituition=0.05
# Ct=0
# C=0
# Cr=0
# Cs=0
#Ct=0
#C=0
#Cr=0
#Cs=0



#### CONTROL MODE -- SHAPE FORMATION ####
if control_mode=="shape_formation":
    geometry = 'pacman'
    if geometry == "circle":
        a = 1
        b = 1
        
    if geometry == 'pacman':
        a = 0.82
        b = 0.82
        
        
    else:
        a=0.82
        b=0.82      
    scale = 1
    alpha = 0.75
    beta = 0

#### CONTROL MODE -- MORPHING ####
if control_mode=="shape_morphing":
    geometry1 = 'circle'
    geometry2 = 'wrench'
    if geometry1 == "circle":
        a = 0
        b = 0
    
    if geometry2 == "circle":
        a = 0
        b = 0    
    
    scale1 = R
    scale2 = 1
    p = 0.5
    alpha = 1.0
    beta = 0

#### CONTROL MODE -- GRASPING EXPLORE ####
if control_mode=="grasping_explore":
    ball_geometry = "circle"
     
    # ball_geometry = "square" 
    ball_geometry = "import"
    
    #ball_geometry = "triangle"
    # circle
    br=.36
    if ball_geometry=="circle":
        ball_radius=br
    # square     
    if ball_geometry=="square":	
        
        br=(2*br)*np.pi/4
        ball_radius=br/2	
        
    # triangle
    if ball_geometry=="triangle":   
        br=br
        ball_radius=br
    
    if ball_geometry=="import":
        ball_radius=2
    #ball_radius = R*0.3

    #Rr1=1.25
    #Rr2=ball_radius
    theta=np.pi
    ballx = 0.0
    ballz = 0     
    ball_mass = 5
    a1 = .01*ball_radius
    b1 = 5*ball_radius
    increment = np.pi/6
    
    const=.01
    
    a2 = const
    b2 = const
    
    xc1 = ballz
    yc1 = ballx
    
    xc2 = ballx
    yc2 = ballz
    
    particle_mix = True
    
    xcenter = -(ball_radius+R+.2)
    zcenter = 0 
    
    
    Rr1=0
    Rr2=abs(xcenter)
    
    
    #Rr1=0
    #Rr2=2.25
    #xcenter = 2.25
    #xcenter = 0
    #zcenter = 0
    
    # tcut1 = 5
    # tcut2 = 10
    # tcut3 = 15
    
    tcut1 = 5
    tcut2 = 13
    tcut3 = 21
    

    # alpha1 = 2.25
    # alpha2 = 2.25 
    alpha1 = 1.75
    alpha2 = 1.75
    beta = 0
    
#### CONTROL MODE -- GRASPING_U #### 
if control_mode=="grasping_u":
    
    ball_geometry = "circle"
    ball_geometry = "square"
    ball_geometry = "c_shape"
    br=0.36
    br2=0.36
    if ball_geometry=="circle":
        ball_radius=br
        print("perimeter object=",str(2*br*np.pi))

        
    if ball_geometry=="square":	
        
        br=(2*br)*np.pi/4
        ball_radius=br/2	 
        #print("perimeter object=",str(2*br*np.pi))
    if ball_geometry=="c_shape":	
        
        br=(2*br)*np.pi/4
        
        ball_radius=br/2
        w=4*ball_radius
        l=2*ball_radius
        t=.1
    
    p=0.07
    width_grasp = 0.9
    lengtho_grasp = .36*2
    #atilda=np.round(np.pi * ((cord_length * nb / np.pi) / 2)**2,2)
    #print("atilda=",atilda)
    pack=1
    a=4.5
    atilda=a/pack
    print("atilda=",atilda)
    length_grasp = atilda/(2*width_grasp) - lengtho_grasp/2
    print("length_grasp=",length_grasp)
    
    #print("Area_act=",np.round(np.pi*R**2,2))
    
    #print("Asquare=",np.round(width_grasp*(2*length_grasp+lengtho_grasp),2))
    #length_grasp = 3.25
    #lengtho_grasp = 0.5
    floor_friction = .03
    particle_mix = False
    ball_fixed=False
    ballx = 0
    ballz = 0 
    ball_mass = 20
    fb_rate=1*dt
    update_rate = 1
    rho_ = 10
    rtilda = 1.5*br2
    #xcenter, -2.207794544559873
    #xcenter, -2.2850512057367918
    xcenter = -(ball_radius+R+.3)
    zcenter = 0 
    
    xcenter_grasp = -3*br2
    ycenter_grasp = zcenter
    
    xcenter_grasp2 = ballx
    ycenter_grasp2 = ballz
    
    
    xc1 = ballx
    yc1 = ballz
    
    xc2 = ballx+rtilda
    yc2 = ballz
    
    tcut1 = 45
    tcut2 = 30
    tcut3 = 150
    alpha1 = 2.5
    alpha2 = 1
    beta = 0
    error = .01
#### CONTROL MODE -- GRASPING ####
if control_mode=="grasping":
    
    ball_geometry = "circle"
     
    ##ball_geometry = "square" 
    
    #ball_geometry = "triangle"
    
   # ball_geometry = "c_shape"
    #ball_geometry = "import"
    #ball_geometry = "circle"
    # circle
    br=.36
    br=0.36
    if ball_geometry=="circle":
        ball_radius=br
    # square     
    if ball_geometry=="square":	
        
        br=(2*br)*np.pi/4
        ball_radius=br/2	
        
    # triangle
    if ball_geometry=="triangle":   
        br=br
        ball_radius=br
    
    if ball_geometry=="import":
        ball_radius=2.25
        
        
        # square     
    if ball_geometry=="c_shape":	
        
        br=(2*br)*np.pi/4
        ball_radius=br/2
        w=2*ball_radius
        l=3*ball_radius
        t=.2
        
    #ball_radius = R*0.3
    particle_mix = True
    ballx = 0
    ballz = 0 
    ball_mass = 5
    const=.01
    a1 = const
    b1 = const
    #b1 = 5*l
    fb_rate=1*dt
    const=.01
    a2 = const
    b2 = const
    
    xcenter = -(ball_radius+R+.1)
    zcenter = 0 
    
    xc1 = (ballx)
    yc1 = ballz

    xc2 = (ballx)
    yc2 = ballz
    #xc1=3.1176326916504427
    #yc1=0
    #xc2=2.1176326916504427
    #yc2=0
    #xc2=ballx+1.1*ball_radius
    #yc2=ballz
    
    #xc2 = ballx #- (w/2)-(t/2)
    #xc2 = -1.39
    #xc2=(-w/2)+(t/2)
    #yc2 = ballz
    
    tcut1 = 2
    tcut2 = 6
    tcut3 = 15
    alpha1 = 2.25
    alpha2 = 2.25
    beta = 0
    
#### CONTROL MODE -- GRASPING EPSILON ####
if control_mode=="grasping_epsilon":
    
    ball_geometry = "circle"
     
    #ball_geometry = "square" 
    
    #ball_geometry = "triangle"
    # circle
    br = .36
    if ball_geometry=="circle":
        ball_radius=br
    # square     
    if ball_geometry=="square":	
        
        br=(2*br)*np.pi/4
        ball_radius=br/2	
        
    # triangle
    if ball_geometry=="triangle":   
        br=br
        ball_radius=br
    
    if ball_geometry=="import":
        ball_radius=3
    #ball_radius = R*0.3
    particle_mix = True
    ball_fixed = True
    ballx = ball_radius+R+.3
    ballz = 0 
    ball_mass = 5
    a1 = .01*ball_radius
    b1 = 5*ball_radius
    floor_friction = .01
    fb_rate = 1*dt
    const = .01
    a2 = const
    b2 = const
    
    
    
    xc1 = ballx+1
    yc1 = ballz
    
    xc2 = ballx
    yc2 = ballz
    
    tcut1 = 2
    tcut2 = 10
    tcut3 = 15
    alpha1 = 3
    alpha2 = 3
    beta = 0 
    
    
#### SAVE SIMULATION ####
now = datetime.now()
dt_string = now.strftime("%d_%m_%Y_%H_%M_%S")
#mainDirectory = "F:/Soro_chrono/python/Pychrono/Strings/String_grasping/"
mainDirectory =os.path.abspath(os.getcwd())
savefile = mainDirectory +'/Experiments/'+'grasping_u/'+ dt_string
os.makedirs(savefile, exist_ok=True)
txtFile = savefile+'/Parameters.csv'
    




##### export as npy file
envParams = {}
envParams['dt_string'] = dt_string
envParams['dt'] = dt
envParams['time_end'] = time_end
envParams['save_rate'] = save_rate
envParams['convert_dist'] = convert_dist
envParams['convert_mass'] = convert_mass
envParams['visual'] = visual
envParams['xcenter'] = xcenter
envParams['zcenter'] = zcenter
envParams['restituition'] = restituition
# control mode
envParams['control_mode'] = control_mode
envParams['alpha1'] = alpha1
envParams['alpha2'] = alpha2
envParams['beta'] = beta

if control_mode=='shape_formation':
    envParams['geometry'] = geometry
    envParams['scale'] = scale
    envParams['a'] = a
    envParams['b'] = b
    
    
if control_mode=='shape_morphing':
    envParams['geometry1'] = geometry1
    envParams['geometry2'] = geometry2
    envParams['scale1'] = scale1   
    envParams['scale2'] = scale2
    envParams['p'] = p
    if geometry1=='circle' or geometry2=='circle':
        envParams['a'] = a
        envParams['b'] = b
        
if control_mode=="grasping" or control_mode=="grasping_epsilon":
    envParams['a1'] = a1
    envParams['b1'] = b1    
    envParams['a2'] = a2
    envParams['b2'] = b2

    envParams['xc1'] = xc1
    envParams['yc1'] = yc1

    envParams['xc2'] = xc2
    envParams['yc2'] = yc2    

    envParams['tcut1'] = tcut1
    envParams['tcut2'] = tcut2
    envParams['tcut3'] = tcut3   
    
    envParams['ballx'] = ballx
    envParams['ballz'] = ballz 
    envParams['fb_rate'] = fb_rate
    envParams['ball_geometry'] = ball_geometry
    envParams['ball_radius'] = ball_radius
    envParams['ball_mass'] = ball_mass
    envParams['ball_fixed'] = ball_fixed
    envParams['floor_friction'] = floor_friction
    if ball_geometry=="c_shape":
        envParams['w']=w
        envParams['l']=l
        envParams['t']=t
        
        
        
if control_mode=="grasping_u":   
    envParams['ball_fixed'] = ball_fixed
    envParams['width_grasp'] = width_grasp
    envParams['length_grasp'] = length_grasp
    envParams['lengtho_grasp'] = lengtho_grasp
    envParams['xcenter_grasp'] = xcenter_grasp
    envParams['ycenter_grasp'] = ycenter_grasp
    
    envParams['xcenter_grasp2'] = xcenter_grasp2
    envParams['ycenter_grasp2'] = ycenter_grasp2
    
    envParams['p'] = p
    envParams['tcut1'] = tcut1
    envParams['tcut2'] = tcut2
    envParams['tcut3'] = tcut3   
    envParams['update_rate'] = update_rate
    envParams['ballx'] = ballx
    envParams['ballz'] = ballz 
    
    envParams['ball_geometry'] = ball_geometry
    envParams['ball_radius'] = ball_radius
    envParams['ball_mass'] = ball_mass   
    envParams['floor_friction'] = floor_friction
    envParams['ball_fixed'] = ball_fixed
    envParams['xc1'] = xc1
    envParams['yc1'] = yc1

    envParams['xc2'] = xc2
    envParams['yc2'] = yc2 
    envParams['rho_'] = rho_
    envParams['error'] = error
    envParams['rtilda'] = rtilda
    if ball_geometry=="c_shape":
        envParams['w']=w
        envParams['l']=l
        envParams['t']=t
if control_mode=="grasping_explore":
    envParams['a1'] = a1
    envParams['b1'] = b1    

    envParams['a2'] = a2
    envParams['b2'] = b2

    envParams['xc1'] = xc1
    envParams['yc1'] = yc1

    envParams['xc2'] = xc2
    envParams['yc2'] = yc2    
    envParams['increment'] = increment
    envParams['tcut1'] = tcut1
    envParams['tcut2'] = tcut2
    envParams['tcut3'] = tcut3   
    
    envParams['ballx'] = ballx
    envParams['ballz'] = ballz 
    
    envParams['ball_geometry'] = ball_geometry
    envParams['ball_radius'] = ball_radius
    envParams['ball_mass'] = ball_mass    
    envParams['Rr1'] = Rr1
    envParams['Rr2'] = Rr2
    envParams['theta'] = theta
    if ball_geometry=="c_shape":
        envParams['w']=w
        envParams['l']=l
        envParams['t']=t
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
envParams['particle_mix'] = particle_mix
envParams['offset_radius'] = offset_radius
envParams['scale_radius'] = scale_radius
# floor parameters
envParams['floor_length'] = floor_length
envParams['floor_height'] = floor_height
envParams['Rbar'] = Rbar


# Physical Paramters
envParams['lateralFriction'] = lateralFriction
envParams['spinningFriction'] = spinningFriction
envParams['rollingFriction'] = rollingFriction
envParams['dampingterm'] = dampingterm
envParams['Ct'] = Ct
envParams['C'] = C
envParams['Cr'] = Cr
envParams['Cs'] = Cs
envParams['number_parameters'] = len(envParams)

np.save(savefile+'/Parameters.npy',envParams)


copyfile(__file__,savefile+"/"+'config.py')


#data=np.load(savefile+'/Parameters.npy',allow_pickle=True)
#data=data.tolist()