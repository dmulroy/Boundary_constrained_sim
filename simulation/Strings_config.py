# -*- coding: utf-8 -*-
"""
Created on Mon Apr 27 19:56:00 2020

@author: dmulr
"""
# import libraries
import numpy as np
import Strings_objects as sim_obj
from scipy.interpolate import interp1d
from datetime import datetime
import os
import sys


# timer (used to time how long the config file takes)
now = datetime.now()
# create name of sim based on  (day, month, year hour minute second)
dt_string = now.strftime("%d_%m_%Y_%H_%M_%S")

# In[Variables for all bots]
### General parameters ###
visual="irr" # visualization
sim= dt_string# sim number 
fixed=False # is it fixed
obj=[] # empty file to ave all objects
data_path="F:/data/" # data file for other visuals

##### control type #####
control_type="shape_form"
env_mode = None # create tunnel 

# Control type:
'''
"shape_form"
"shape_form_analytic"
"pot_field_grab"
"GRASP"
"image_warp"
"import_field_letters"
"tunnel"
"tunnel_verify"
"target_verify"
"grasp_verify"
'''



##### interior modes #####
mode="nonhnmax"

# Interior generation modes:
'''
'empty':Nothing inside
'nmax' :customize the ring sizes
'nonhnmax'  max number of interiors with different diamters
anger
verify
target_verify
tunnel_verify
grasp_verify
'''


##### time #####
tstep=.001  # time step
tend=323# time end

##### Friction #####
mu_f=.4# friction
mu_b=.01    # dampning
mu_r=.01    # rolling friction
mu_s=.01   # SPinning fiction

##### compliance #####
Ct=.00000 # tangent compliane
C=.000000 # compliance
Cr=.00000 # rolling compliance
Cs=.00000 # sliding compliance

##### Robot #####       
mr=.3       # mass
nb=30 #number of robots
height=.1  # height of cylinder
radius=.038 # diameter of cylinder and robots
volume=np.pi*height*(radius)**2   # calculate volume
rowr=mr/volume # calculate density of robot
skind=.03 # diamter of memebrane particles
rationM=4# how many membrane particles between each bot
#R=(2*diameter*nb/(np.pi*2))+.26
R=(skind*(rationM))/(np.sin(np.pi/nb))
#=(.18)/(np.sin(np.pi/nb))

#R=.51

rationM=7
### Geometry of robot ###
geom="cylinder"

'''
square: Robots will be in a square shape
cylinder: Robots will be a cylinder
shere: robots will be a sphere
'''

##### Interior particles #####

mp=.02 # mass 
#radius2=.75 # diameter of cylinder and robots
radius2=.04
volume2=np.pi*height*(radius2)**2   # calculate volume
rowp=mp/volume2 # density



#(n,Area)=sim_obj.MaxValues(R-17.5*radius2,radius,radius2,nb,mode)
#(n,Area)=sim_obj.MaxValues(R-3.5*radius2,radius,radius2,nb,mode)
(n,Area)=sim_obj.MaxValues(R,radius,radius2,nb,mode)
R = R
##### Spring ##### 
k=1000# spring constant (bots)
rl=0 # resting length
type_spring='var'

##### Floor #####
length=1000 # Length of the body floor
tall=1     # height of the body floor


##### pwm #####
pwm=255# 0-255
w=5 # freqency
tn=(pwm/255)/w  # pulse length so out of .2 seconds it runs and applies for for tn seconds 


# In[Shape form]

''' This sets up the variables neede for forming a shape '''
if control_type=="shape_form": 
    tpull=0        # time to pull omn the object  not needed 
    xc=0        # center of robot x
    zc=0         # center of robot z
    ##### pot field ######
    """types of fields
    "Analytic fields:
    numeric fields: fields created numerically
    region: region based fields 
    'point_field' : point field 
    'importstar' this imports a star shape potential field 
    """
    shape="Circle"
    alpha=1# controller gain
    beta=0           # damping term
    b=5          # range of field 
    res=0.05
    px=0             # ball location x
    py=0           # ball location y
    gapw=0 # size of gap if tunneling
    env_mode=None # create tunnel 
    # potential field
    # triangle R=1.3R
    # Square R=.85R
    phi=sim_obj.Shape_fields(1.2*R,px,py,b,res,shape,sim)
    gapw=0 # size of gap if tunneling
    env_mode=None # create tunnel 
    # arguemnts
    args=(phi,alpha,beta,b) # create arguement list 
    X=[]
    Z=[]     
 
    
 
# In[Analytic Fields]
''' This sets up the variables neede for forming a analytic shape '''
if control_type=="shape_form_analytic": 
    tpull=0        # time to pull omn the object  not needed 
    xc=0        # center of robot x
    zc=0         # center of robot z
    
    a=.58*R # radii 1
    c=1*R # radii 2
    theta=0 # rotation 
    res=0.1 # resolution of the field 
    
    alpha=20   # controller gain
    beta=0           # damping term
    b=5          # range of field 
    res=0.1
    px=0             # ball location x
    py=0           # ball location y
    gapw=0 # size of gap if tunneling
    env_mode=None # create tunnel 
    # potential field    
    phi=sim_obj.analytic_field(a,c,px,py,theta,b,res)
    gapw=0 # size of gap if tunneling
    env_mode=None # create tunnel 
    # arguements #
    args=(phi,alpha,beta,b) # create arguement list 
     

# In[pot_field_grab]
''' variables for grasping with no morphing '''
if control_type=="pot_field_grab": 
    #const=0.6010980273469619/2 #square
    const= .2313 #triangle 
    tpull=20        # time to pull omn the object
    xc=0            # center of robot x
    zc=0            # center of robot z
    # Ball variables #####
    mb=1 # mass of ball
    Rb=R/3 # radius of ball
    volume3=np.pi*height*(Rb)**2   # calculate volume
    rowb=mb/volume3 # density
    px=1.5*R          # ball location x
    py=0        # ball location y    
    xball=py # center of ball x
    zball=px # center of ball y

    a=1.1*const # radii 1
    c=1.1*const # radii 2
    theta=0 # rotation 
    res=0.1 # resolution of the field
    alpha=60 # controller gain
    beta=0          # damping term
    b=20          # range of field 

    Rd=R 
    X=[]
    Z=[]
    
    phi=sim_obj.analytic_field(a,c,px,py,theta,b,res)
    gapw=0 # size of gap if tunneling
    env_mode=None # set = to "tunnel" if you want thre to be a tunnel 
    # arguements 
    args=(phi,alpha,beta,b)
    

# In[GRASP]
'''  GRASP with morphing '''
if control_type=="GRASP": 
    const=0.6010980273469619/2 #square
    #const= .2313 #triangle 
    tpull=20        # time to pull omn the object
    xc=0            # center of robot x
    zc=0            # center of robot z
    # Ball variables #####
    mb=1# mass of ball
    Rb=R/3# radius of ball
    volume3=np.pi*height*(Rb)**2   # calculate volume
    rowb=mb/volume3 # density
    px=4*R          # ball location x
    py=0        # ball location y    
    xball=py # center of ball x
    zball=1.5*R # center of ball y
    a=1.1*const# radii 1
    c=1.1*const # radii 2
    alpha=2# controller gain
    alpha2=2
    beta=0          # damping term
    b=20          # range of field 
    
    Rd=R
    X=[]
    Z=[]
    #phi=sim_obj.GRASPING_FIELD(a,c,px,py,Rb,zball,xball)
    phi=sim_obj.GRASPING_FIELD(a,c,px,py,Rb,zball,xball,alpha2)
    gapw=0 # size of gap if tunneling
    env_mode=None # set = to "tunnel" if you want thre to be a tunnel 
    # arguements 
    args=(phi,alpha,beta,b)


# In[Image warping ]
''' This sets up the variables neede for forming a shape '''
if control_type=="image_warp": 
    tpull=0        # time to pull omn the object  not needed 
    xc=0        # center of robot x
    zc=0         # center of robot z
    alpha=100  # controller gain
    beta=0           # damping term
    b=100        # range of field 
    res=1
    px=0             # ball location x
    py=0           # ball location y
    gapw=0 # size of gap if tunneling
    env_mode=None # create tunnel 
    Rd=R
    shape=['Circle','EllipseL','Triangle','Square']
    # potential field
    phi=sim_obj.image_warping(px,py,b,res,shape,sim,a=.75*Rd,c=1.2*Rd,theta=0,R=0.85*Rd,R2=1.5*R)
    gapw=0 # size of gap if tunneling
    env_mode=None # create tunnel 
    #arguements 
    args=(phi,alpha,beta,b) # create arguement list     
    X=[]
    Z=[]
    
 # In[Import field letters]
''' Letter morphing '''
if control_type=="import_field_letters": 
    tpull=20        # time to pull omn the object
    xc=0           # center of robot x
    zc=0            # center of robot z
    alpha=150          # controller gain
    beta=.1         # damping term
    field='JAM4' # name of field 
    # potential field
    phi=sim_obj.imported_fields_letters(field,sim)
    env_mode=None # create tunnel 
    gapw=3
    px=0 # center of field x
    py=0 # center of field y
    #arguements
    args=(phi,alpha,beta)
    X=[]
    Z=[]
    gapw=1
       
    
    
# In[Tunnel]

''' Running tunneling simulation '''
if control_type=="tunnel": 
    tpull=20        # time to pull omn the object
    xc=1.5         # center of robot x
    zc=-4            # center of robot z
    a=.58*R # radii 1
    c=1.2*R # radii 2

    theta = 0
    alpha=30      # controller gain
    beta=0          # damping term
    b=20             # range of field 
    Rd=.1            # radius of zero contour
    px = -2.5  # goal location x 
    py=-15     # goal location y
    res=1         # res
    # potential field
    phi=sim_obj.point_field(px,py,res,b) 
    
    #phi2=sim_obj.analytic_field(a,c,xc,zc,theta,b,res)
    #phi=[phi2,phi1]
    env_mode='import_tunnel' # create tunnel 
    gapw=1
    X=[]
    Z=[]
    #arguements 
    args=(phi,alpha,beta,b)      
    
# In[Tunnel Verify]

''' Tunnel verify simulation '''  
if control_type=="tunnel_verify": 
    tpull=20        # time to pull omn the object
    xc=1.5         # center of robot x
    zc=-4            # center of robot z
    a=.58*R # radii 1
    c=1.2*R # radii 2

    theta = 0
    alpha=3      # controller gain
    beta=0          # damping term
    b=20             # range of field 
    Rd=.1            # radius of zero contour
    #px=-2.5            # goal location x
    px = -2.5
    py=-15  
    res=1          # goal location y
    # potential field
    phi=sim_obj.point_field(px,py,res,b) 
    #phi2=sim_obj.analytic_field(a,c,xc,zc,theta,b,res)
    #phi=[phi2,phi1]
    env_mode='verify_tunnel' # create tunnel 
    gapw=1
    X=[]
    Z=[]
    #arguements 
    args=(phi,alpha,beta,b)   
    
   
    
# In[Target Verify]
''' Target chasing verification simulation '''
if control_type=="target_verify": 
    tpull = 20        # time to pull omn the object
    # Ball variables #####
    mb=1# mass of ball
    Rb=.07# radius of ball
    volume3=np.pi*height*(Rb)**2   # calculate volume
    rowb=mb/volume3 # density
    xball=0 # center of ball x
    zball=0 # center of ball y
    xc = 0         # center of robot x
    zc = 0           # center of robot z
    alpha = 20     # controller gain
    beta = 0          # damping term
    b = 20             # range of field 
    px = 0            # goal location x
    py = 0  
    # potential field
    phi = sim_obj.source_field(px,py)
    env_mode = None # create tunnel 
    gapw = 1
    X = []
    Z = []
    #arguements 
    args = (phi,alpha,beta,b)     
    
    

# In[Grasp Verify]
''' variables for the grasp verify simulation '''
if control_type=="grasp_verify": 
    const=0.6010980273469619/2 #square
    #const= .2313 #triangle 
    tpull=20        # time to pull omn the object
    xc=0           # center of robot x
    zc=0            # center of robot z
    # Ball variables #####
    mb=100# mass of ball
    Rb=.07# radius of ball
    volume3=np.pi*height*(Rb)**2   # calculate volume
    rowb=mb/volume3 # density

    xball=-1
    zball=-0.04    
    py=(xball-.5)
    px=-0.04       # ball location y    
    c=.02
    a=3*Rb
    alpha=10# controller gain
    alpha2=10
    beta=3         # damping term
    b=20          # range of field 
    Rd=R
    X=[]
    Z=[]
    #phi=sim_obj.GRASPING_FIELD(a,c,px,py,Rb,zball,xball)
    phi1 = sim_obj.source_field(px,py)
    phi2=sim_obj.GRASPING_FIELD(a,c,px,py,Rb/2,zball,xball,alpha2)
    phi=[phi1,phi2]
    gapw=0 # size of gap if tunneling
    env_mode=None # set = to "tunnel" if you want thre to be a tunnel 
    # arguements 
    args=(phi,alpha,beta,b)
    

# In[SAVE VARIABLES]
position=True # save bot position
velocity=True# save bot velocity
forces=False  # save bot forces
control_force=False # save all contact forces
contact_positions=False

# interior particle postions, velocity and forces
if mode!='empty':
    particle_position=True
    particle_vel=True
    particle_force=True
else:
    particle_position=True
    particle_vel=True
    particle_force=True

# ball information
if control_type=='pot_field_grab' or "GRASP" or 'Verify':
    ball_data=True
else:
    ball_data=False

    


if control_type=='shape_form':
    shaped=True
    error=True
else:
    shaped=False
    error=False
    
if control_type=='image_warp':
    shaped=False
    error=True
else:
    shaped=False
    error=False
save_data=[position,velocity,forces,control_force,contact_positions,particle_position,particle_vel,particle_force,ball_data,shaped]        

