# -*- coding: utf-8 -*-
"""
Created on Wed Jul 21 14:14:49 2021

@author: dmulr
"""


import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
 
import numpy as np
from numpy import savetxt
import random
import os
import csv
import glob
from csv import writer
import timeit
import cv2
from os.path import exists
from IPython.display import HTML
import matplotlib.pyplot as plt
from matplotlib import animation
import matplotlib.font_manager as fm
import matplotlib.patches as patches
import matplotlib
from matplotlib import colors as colors
from matplotlib.pyplot import cm
from matplotlib.patches import RegularPolygon
from shutil import copyfile
from scipy.spatial import ConvexHull
from scipy.interpolate import RegularGridInterpolator
from sympy import Plane, Point3D
#from sympy import *
from scipy.spatial import Delaunay
#import random
from scipy import signal
num = random.random()
print(num)
try:
    import pyhull.convex_hull as cvh
except:
    logging.warning('Failed to import pyhull')
try:
    import cvxopt as cvx
except:
    logging.warning('Failed to import cvx')
    
import itertools    
###############################################################################
class robots:
    def __init__(self,name,my_system,body_floor,path):
        
        self.name=name # name of the simulation run 
        self.my_system = my_system # total system
        self.body_floor = body_floor # body floor object
        
        #### Imported Variables ####
        #self.path=path # file 
        self.mainDirectory = path # location of the simulation folder
        
        copyfile(__file__,self.mainDirectory+self.name+"/"+'objects.py') # create a copy of this file 
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True) # load the parameters from the simulation file
        
        self.parameters=parameters.tolist()
        
        self.convert_dist=self.parameters['convert_dist'] 
        self.convert_mass=self.parameters['convert_mass']
        
        self.xcenter = self.parameters['xcenter']
        self.zcenter = self.parameters['zcenter']
        
        self.nb=self.parameters['nb'] # number of bots
        self.bot_mass = self.convert_mass*self.parameters['bot_mass'] 
        self.bot_width = self.convert_dist*self.parameters['bot_width']
        self.bot_height = self.convert_dist*self.parameters['bot_height']
        self.bot_geom = self.parameters['bot_geom']
        self.R = self.parameters['R']
        self.Rbar = self.parameters['Rbar']
        self.bot_volume=np.pi*self.bot_height*(self.bot_width/2)**2   # calculate volume
        self.membrane_width = self.convert_dist*self.parameters['membrane_width']
        self.membrane_type = self.parameters['membrane_type']
        self.skin_width = self.parameters['skin_width']
        self.ns = self.parameters['ns']
        self.spring_stiffness = self.parameters['spring_stiffness'] 
        self.spring_damping = self.parameters['spring_damping'] 
        
        
        #### Material properties
        self.lateralFriction = self.parameters['lateralFriction']
        self.spinningFriction = self.parameters['spinningFriction']
        self.rollingFriction = self.parameters['rollingFriction']
        self.dampingterm = self.parameters['dampingterm']
        self.Compliance_tangent = self.parameters['Ct'] 
        self.Compliance = self.parameters['C'] 
        self.membrane_density = self.parameters['membrane_density']
        self.restituition = self.parameters['restituition']
        
        #### Calculated variables ######
        self.bot_density=self.bot_mass/self.bot_volume # calculate density of robot 
        
        #### create material properties
        self.bot_material = self.Material(self.lateralFriction)       
        self.membrane_material = self.Material(self.lateralFriction)
        
        
        self.countm = 0 #counter which is used to fill arrays for the data of the membrane units
        self.rl = 0
        self.fixed = False #if the object is fixed in space or not
        
        #### Colors
        self.col_y = chrono.ChColorAsset(); self.col_y.SetColor(chrono.ChColor(1, 1, 0))       # Yellow
        self.col_b = chrono.ChColorAsset(); self.col_b.SetColor(chrono.ChColor(0, 0, 1))       # Blue
        self.col_g = chrono.ChColorAsset(); self.col_g.SetColor(chrono.ChColor(0, 1, 0))       # Green
        self.col_p = chrono.ChColorAsset(); self.col_p.SetColor(chrono.ChColor(0.44, .11, 52)) # Purple
        self.col_w = chrono.ChColorAsset(); self.col_p.SetColor(chrono.ChColor(.8, .8, .8)) # Purple
        
        
        #### Empty list of  variables
        self.skinM = []
        self.bots = []
        self.force = []
        self.Springs = []
        
        # empty list of bot postions
        self.bot_xposition = {}
        self.bot_yposition = {}
        self.bot_zposition = {}
        
        # empty list of bot velocityies
        self.bot_xvelocity = {}
        self.bot_yvelocity = {}
        self.bot_zvelocity = {}
        
        # empty list of total bot forces, contact, applied forces, membrnae forces
        self.bot_xForcetotal = {}
        self.bot_yForcetotal = {}
        self.bot_zForcetotal = {}
        
        
        self.bot_xForcecontact = {}
        self.bot_yForcecontact = {}
        self.bot_zForcecontact = {}
        
        # empty list of the membrane positions
        self.skin_xposition = {} # x position
        self.skin_yposition = {} # y position
        self.skin_zposition = {} # z position
        
        # empty list of contact forces for the membrane
        self.skin_x_contact_forces = {} # x force
        self.skin_y_contact_forces = {} # y force
        self.skin_z_contact_forces = {} # z force
                  
        # empty list of total forces, contact forces, spring forces etc              
        self.skin_x_total_forces = {} # x force
        self.skin_y_total_forces = {} # y force
        self.skin_z_total_forces = {} # z force
                                      

        ##### fill the lists with empty arrays 
        for i in range(self.nb):
            
            # positions
            self.bot_xposition["bot_xposition{0}".format(i)]=[]  #x position
            self.bot_yposition["bot_yposition{0}".format(i)]=[]  # y position
            self.bot_zposition["bot_zposition{0}".format(i)]=[]  # z position 
            
            
            # velocities
            self.bot_xvelocity["bot_xvelocity{0}".format(i)]=[]  # x velocity
            self.bot_yvelocity["bot_yvelocity{0}".format(i)]=[]  # y velocity
            self.bot_zvelocity["bot_zvelocity{0}".format(i)]=[]  # z velocity
            
            
            # total forces
            self.bot_xForcetotal["bot_xForcetotal{0}".format(i)]=[]  # Force x
            self.bot_yForcetotal["bot_yForcetotal{0}".format(i)]=[]  # Force y
            self.bot_zForcetotal["bot_zForcetotal{0}".format(i)]=[]  # force z
            
            
            # contact forces
            self.bot_xForcecontact["bot_xForcecontact{0}".format(i)]=[]  # Force x
            self.bot_yForcecontact["bot_yForcecontact{0}".format(i)]=[]  # Force y
            self.bot_zForcecontact["bot_zForcecontact{0}".format(i)]=[]  # force z            
        
        
        
            #### Create robots and membrane
            # postion 
            theta=i*2*np.pi/self.nb # set angle
            x = self.Rbar*np.cos(theta)+self.xcenter  # set x positon 
            y = self.bot_height/2                     # set y position 
            z = self.Rbar*np.sin(theta)+self.zcenter  # set z position 
            # create body
            ##### geometry of the robots to be cylinders
            if self.bot_geom=="cylinder":
                # create the object
                bot = chrono.ChBodyEasyCylinder(self.bot_width/2, self.bot_height,self.bot_density,True,True)
                bot.SetPos(chrono.ChVectorD(x,y,z)) # set position
                bot.SetName("bot"+str(i)) # set name (important for contact tracking)
                bot.SetId(i)              # set id   (impoortant for contact tracking )  
                bot.SetMaterialSurface(self.bot_material)  # set material 

                rotation1 = chrono.ChQuaternionD() # rotate the robots about y axis 
                rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0)) 
                bot.SetRot(rotation1) 
                
                
                ##### Add force components to each bot 
                
                # x forces 
                botforcex = chrono.ChForce()  # create it 
                bot.AddForce(botforcex) # apply it to bot object
                botforcex.SetMode(chrono.ChForce.FORCE) # set the mode 
                botforcex.SetDir(chrono.VECT_X) # set direction 
                self.force.append(botforcex) # add to force list
                
                # y forces    
                botforcey = chrono.ChForce() # create it 
                bot.AddForce(botforcey) # apply it to bot object
                botforcey.SetMode(chrono.ChForce.FORCE) # set the mode
                botforcey.SetDir(chrono.VECT_Y) # set direction 
                self.force.append(botforcey) # add to force list
                
                # z forces            
                botforcez = chrono.ChForce() # create it 
                bot.AddForce(botforcez) # apply it to bot object
                botforcez.SetMode(chrono.ChForce.FORCE) # set the mode
                botforcez.SetDir(chrono.VECT_Z) # set direction 
                self.force.append(botforcez) # add to force list
                
                
                
                # set max speed (Not always needed but it helps)
                bot.SetMaxSpeed(2)
                bot.SetLimitSpeed(False)
                
            
                # make the color blue 
                bot.AddAsset(self.col_b)
                entry=[25,24,25,26,27,28,29,0,1,2,3,4,5,6,7]
                entry=[0,2,5,8,11,14,17,20,23,26]
                if i in entry:
                    bot.AddAsset(self.col_p)
                # # zeroth robot so we know which is which in the videos
                # if i==0:   
                #     entry=[25,24,25,26,27,28,29,0,1,2,3,4,5,6,7]
                # if i==10:
                #     bot.AddAsset(self.col_p)
                # if i==20:
                #     bot.AddAsset(self.col_p)    
                # if i==30:
                #     bot.AddAsset(self.col_p)                  
                #if i>=20 and i<=25:
                    #bot.AddAsset(self.col_w)
                #if i==22:   
                    #bot.AddAsset(self.col_y)  
                
                # set collision
                bot.SetCollide(True)
                
                
                # set fixed
                bot.SetBodyFixed(self.fixed)
                
                # link to floor
                #pt=chrono.ChLinkMatePlane()
                #pt.Initialize(self.body_floor,bot,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
                
                # add link to the system 
                #self.my_system.AddLink(pt)
                
                # add bot to series of array 
                self.my_system.Add(bot) # add bot to system 
                self.bots.append(bot) # add bot to bot array 
                
                
            
    
            
            
            
            #### Create membrane elements
            b_ang=2*np.pi/self.nb                   # angle between centers of bots
            o_ang=np.arctan((self.bot_width*.66)/self.Rbar)   # angle offset for radius of bot
            p_ang=np.arctan((self.skin_width*0.8)/self.Rbar)           # angle offset for radius of skin particle
            
            
            # Between this bot and last bot
            if i>=1 and i<self.nb:
                for j in range(1,self.ns+1,1):
                    # Initial postion of each particle
                    theta=i*b_ang + j*(b_ang - o_ang -  p_ang)/(self.ns) + p_ang 
                    x=self.Rbar*np.cos(theta)+self.xcenter # x position 
                    y=self.bot_height/2              # y position 
                    z=self.Rbar*np.sin(theta)+self.zcenter # z position  
                    
                    # create them and set position
                    #skinm = chrono.ChBodyEasySphere(self.skind/2,self.skinrho,True,True)
                    skinm = chrono.ChBodyEasyCylinder(self.skin_width/2, .95*self.bot_height,self.membrane_density,True,True) # create particle
                    skinm.SetPos(chrono.ChVectorD(x,y,z)) # set position 
                    skinm.SetMaterialSurface(self.membrane_material) # add material 
                    skinm.SetNoGyroTorque(True) # no gyro toruqe 
                    skinm.SetName('skin'+str(i)) # create name 
                    skinm.SetId(i) # set id 
                    # link to floor
                    #pt=chrono.ChLinkMatePlane()
                    #pt.Initialize(self.body_floor,skinm,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
                    # add link to the system 
                    #self.my_system.AddLink(pt)
                    
                    # rotate them bout y axis
                    rotation1 = chrono.ChQuaternionD()
                    rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0));  
                    skinm.SetRot(rotation1)
                    
                    # Fill the empty positions
                    
                    self.skin_xposition["skin_xposition{0}".format(self.countm)]=[]  #x position
                    self.skin_yposition["skin_yposition{0}".format(self.countm)]=[]  # y position
                    self.skin_zposition["skin_zposition{0}".format(self.countm)]=[]  # z position
                    
                    self.skin_x_contact_forces["skin_x_contact_forces{0}".format(self.countm)]=[]  #x position
                    self.skin_y_contact_forces["skin_y_contact_forces{0}".format(self.countm)]=[]  # y position
                    self.skin_z_contact_forces["skin_z_contact_forces{0}".format(self.countm)]=[]  # z position
                                            
                    self.skin_x_total_forces["skin_x_total_forces{0}".format(self.countm)]=[]  #x position
                    self.skin_y_total_forces["skin_y_total_forces{0}".format(self.countm)]=[]  # y position
                    self.skin_z_total_forces["skin_z_total_forces{0}".format(self.countm)]=[]  # z position
                                                  
                    self.countm=self.countm+1 # add countm 
                    # Attach springs if more then one was created    
                    if j>1:
                        ground=chrono.ChLinkSpring() # create spring 1
                        p1=0; p2=self.skin_width/2 # points where each spring is attatched 
                        p3=0; p4=-self.skin_width/2  
                        h=self.bot_height/5 # height of the membrane elements
                        
                        # spring 1
                        ground.Initialize(self.skinM[-1], skinm,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),True) # link spring to particles
                        ground.Set_SpringK(self.spring_stiffness) # set spring constant
                        ground.Set_SpringR(self.spring_damping) # set damping constant
                        ground.Set_SpringRestLength(self.rl) # set resting length 
                        ground.AddAsset(self.col_p) # add color 
                        ground.AddAsset(chrono.ChPointPointSpring(.01,80,15)) # create visual 
                        self.my_system.AddLink(ground) # add spring to system 
                        self.Springs.append(ground)
                        
                        # spring 2
                        ground1=chrono.ChLinkSpring() # create spring 2
                        ground1.Initialize(self.skinM[-1], skinm,True,chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),True) # link spring to particles 
                        ground1.Set_SpringK(self.spring_stiffness) # set spring constant
                        ground1.Set_SpringR(self.spring_damping) # set damping 
                        ground1.AddAsset(self.col_p) # create color 
                        ground1.AddAsset(chrono.ChPointPointSpring(.01,80,15)) # create visual 
                        self.my_system.AddLink(ground1) # add to the system                                 
                        self.Springs.append(ground1)
                        
                    #Link to cylinder 
                    if j==1:
                        skinm.AddAsset(self.col_p) # add color
                        glue=chrono.ChLinkMateFix() # cretae fix constraint
                        glue.Initialize(skinm,self.bots[i]) # fix object to bot
                        self.my_system.AddLink(glue) # add to system 
                        # Link last particle with this bot
                        if i>=2:
                            glue=chrono.ChLinkMateFix() # create the constraint 
                            glue.Initialize(self.skinM[-1],self.bots[-1]) # add constraint 
                            self.my_system.AddLink(glue) # add to the system 
                    if j==self.ns:
                        skinm.AddAsset(self.col_p)
                    self.my_system.Add(skinm)
                    self.skinM.append(skinm)
            
            # Between this bot and first bot
            if i==self.nb-1:
                for j in range(1,self.ns+1,1):
                    # Initial postion of each particle
                    theta=(i+1)*b_ang + j*(b_ang - o_ang - p_ang)/(self.ns) + p_ang
                    x=self.Rbar*np.cos(theta)+self.xcenter
                    y=self.bot_height/2
                    z=self.Rbar*np.sin(theta)+self.zcenter
                    
                    self.skin_xposition["skin_xposition{0}".format(self.countm)]=[]  #x position
                    self.skin_yposition["skin_yposition{0}".format(self.countm)]=[]  # y position
                    self.skin_zposition["skin_zposition{0}".format(self.countm)]=[]  # z position
                    
                    self.skin_x_contact_forces["skin_x_contact_forces{0}".format(self.countm)]=[]  #x position
                    self.skin_y_contact_forces["skin_y_contact_forces{0}".format(self.countm)]=[]  # y position
                    self.skin_z_contact_forces["skin_z_contact_forces{0}".format(self.countm)]=[]  # z position
                                            
                    self.skin_x_total_forces["skin_x_total_forces{0}".format(self.countm)]=[]  #x position
                    self.skin_y_total_forces["skin_y_total_forces{0}".format(self.countm)]=[]  # y position
                    self.skin_z_total_forces["skin_z_total_forces{0}".format(self.countm)]=[]  # z position  
                    self.countm=self.countm+1
                    
                    # Create particles
                    #skinm = chrono.ChBodyEasySphere(self.skind/2,self.skinrho,True,True)
                    skinm = chrono.ChBodyEasyCylinder(self.skin_width/2, .95*self.bot_height,self.membrane_density,True,True)
                    skinm.SetPos(chrono.ChVectorD(x,y,z))
                    skinm.SetMaterialSurface(self.membrane_material)
                    skinm.SetNoGyroTorque(True)
                    skinm.SetName("skin"+str(i))
                    skinm.SetId(i)
                    # rotate them
                    rotation1 = chrono.ChQuaternionD()
                    rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0));  
                    skinm.SetRot(rotation1)
                    #pt=chrono.ChLinkMatePlane()
                    #pt.Initialize(self.body_floor,skinm,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
                    #self.my_system.AddLink(pt)
                    # Attach springs    
                    if j>1:
                        ground=chrono.ChLinkSpring()
                        p1=0; p2=self.skin_width/2
                        p3=0; p4=-self.skin_width/2
                        h=self.bot_height/5
                        
                        # spring 1
                        ground.Initialize(self.skinM[-1], skinm,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),True)
                        ground.Set_SpringK(self.spring_stiffness)
                        ground.Set_SpringR(self.spring_damping)
                        ground.Set_SpringRestLength(self.rl)
                        ground.AddAsset(self.col_y)
                        ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                        self.my_system.AddLink(ground)
                        self.Springs.append(ground)
                        
                        # spring 2
                        ground1=chrono.ChLinkSpring()
                        ground1.Initialize(self.skinM[-1], skinm,True,chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),True)
                        ground1.Set_SpringK(self.spring_stiffness)
                        ground1.Set_SpringR(self.spring_damping)
                        ground.Set_SpringRestLength(self.rl) # set resting length 
                        ground1.AddAsset(self.col_y)
                        ground1.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                        self.my_system.AddLink(ground1)  
                        self.Springs.append(ground1)
                    #Link to cylinder
                    if j==1:
                        skinm.AddAsset(self.col_p)
                        glue=chrono.ChLinkMateFix()
                        glue.Initialize(skinm,self.bots[0])
                        self.my_system.AddLink(glue)
                        glue=chrono.ChLinkMateFix()
                        glue.Initialize(self.skinM[-1],self.bots[0])
                        self.my_system.AddLink(glue)
                     
                    if j==self.ns:
                        skinm.AddAsset(self.col_p)
                        glue=chrono.ChLinkMateFix()
                        glue.Initialize(skinm,self.bots[1])
                        self.my_system.AddLink(glue)
                        
                    self.my_system.Add(skinm)
                    self.skinM.append(skinm)               
                
               
                
               
                
    def Material(self,lateralFriction):
        ''' Function that creates material object '''
        material = chrono.ChMaterialSurfaceNSC() # create material object
        material.SetFriction(lateralFriction) # set friction properties
        material.SetDampingF(self.dampingterm) # set damping properties
        material.SetCompliance(self.Compliance) # set compliance property
        material.SetComplianceT(self.Compliance_tangent) # set tangential property
        material.SetRollingFriction(self.rollingFriction)
        material.SetSpinningFriction(self.spinningFriction)
        material.SetRestitution(self.restituition)
        # material.SetComplianceRolling(Cr)
        # material.SetComplianceSpinning(Cs)
        return (material)
    
 
    # return system (Helps with adding to the data extractor, simulaor, and controllers)
    def return_system(self):
        ''' Return system, springs, bots, obj, force '''
        return(self.my_system,self.Springs,self.bots,self.force)

    # save position data
    def save_data_position(self):
        ''' Save position of each bot '''
        for i in range(self.nb):
            self.bot_xposition["bot_xposition"+str(i)].append(self.bots[i].GetPos().x)
            self.bot_yposition["bot_yposition"+str(i)].append(self.bots[i].GetPos().y)
            self.bot_zposition["bot_zposition"+str(i)].append(self.bots[i].GetPos().z)
            
        for i in range(self.countm):
            self.skin_xposition['skin_xposition'+str(i)].append(self.skinM[i].GetPos().x)
            self.skin_yposition['skin_yposition'+str(i)].append(self.skinM[i].GetPos().y)
            self.skin_zposition['skin_zposition'+str(i)].append(self.skinM[i].GetPos().z)
            
                
    def save_data_forces(self):
        ''' Save position of each bot '''
                 
        for i in range(self.countm):
            self.skin_x_contact_forces["skin_x_contact_forces"+str(i)].append(self.skinM[i].GetContactForce().x)
            self.skin_y_contact_forces["skin_y_contact_forces"+str(i)].append(self.skinM[i].GetContactForce().y)
            self.skin_z_contact_forces["skin_z_contact_forces"+str(i)].append(self.skinM[i].GetContactForce().z)
                       
            self.skin_x_total_forces["skin_x_total_forces"+str(i)].append(self.skinM[i].Get_Xforce().x)
            self.skin_y_total_forces["skin_y_total_forces"+str(i)].append(self.skinM[i].Get_Xforce().y)
            self.skin_z_total_forces["skin_z_total_forces"+str(i)].append(self.skinM[i].Get_Xforce().z)
    
    
    # save velocity data
    def save_data_velocity(self):
        ''' save velocity of each bot '''
        for i in range(self.nb):
            self.bot_xvelocity["bot_xvelocity"+str(i)].append(self.bots[i].GetPos_dt().x)
            self.bot_yvelocity["bot_yvelocity"+str(i)].append(self.bots[i].GetPos_dt().y)
            self.bot_zvelocity["bot_zvelocity"+str(i)].append(self.bots[i].GetPos_dt().z)
            

    # save force data            
    def save_data_Forces(self):
        ''' Save total force on each bot '''
        for i in range(self.nb):
            self.bot_xForcetotal["bot_xForcetotal"+str(i)].append(self.bots[i].Get_Xforce().x)
            self.bot_yForcetotal["bot_yForcetotal"+str(i)].append(self.bots[i].Get_Xforce().y)
            self.bot_zForcetotal["bot_zForcetotal"+str(i)].append(self.bots[i].Get_Xforce().z)
    
    def save_data_Forces_contact(self):
        ''' Save total force on each bot '''
        for i in range(self.nb):
            self.bot_xForcecontact["bot_xForcecontact"+str(i)].append(self.bots[i].GetContactForce().x)
            self.bot_yForcecontact["bot_yForcecontact"+str(i)].append(self.bots[i].GetContactForce().y)
            self.bot_zForcecontact["bot_zForcecontact"+str(i)].append(self.bots[i].GetContactForce().z)
            
            
    # return position data
    def return_position_data(self):
        ''' return the dictionary of each bot '''
        return(self.bot_xposition,self.bot_yposition,self.bot_zposition)
    
    # return position membrane data
    def return_position_membrane_data(self):
        ''' return the dictionary of each pmembrane particle '''
        return(self.skin_xposition,self.skin_yposition,self.skin_zposition)  
      
    # return velocity data
    def return_velocity_data(self):
        ''' return the diction of each bot velocity '''
        return(self.bot_xvelocity,self.bot_yvelocity,self.bot_zvelocity)
        
    # return force data    
    def return_total_force(self):
        ''' return dictionary of each bot forces '''
        return(self.bot_xForcetotal,self.bot_yForcetotal,self.bot_zForcetotal,)
    
    
    def return_skin_forces(self):
        ''' Return the skin forces '''
        return(self.skin_x_contact_forces,
               self.skin_y_contact_forces,
               self.skin_z_contact_forces,
               self.skin_x_total_forces,
               self.skin_y_total_forces,
               self.skin_z_total_forces)


    # return force data    
    def return_force_data_contact(self):
        ''' return dictionary of each bot forces '''
        return(self.bot_xForcecontact,self.bot_yForcecontact,self.bot_zForcecontact)
    
class Interiors:
    def __init__(self,name,my_system,body_floor,path):
        self.name=name # name of the simulation 
        self.my_system = my_system # the system object
        self.body_floor = body_floor # the floor object
        self.mainDirectory = path # path to the simulation run file 
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True) # import parameter
        self.parameters=parameters.tolist()
        
        #### import parameters
        self.convert_dist=self.parameters['convert_dist']
        self.convert_mass=self.parameters['convert_mass']

        self.xcenter = self.parameters['xcenter']
        self.zcenter = self.parameters['zcenter']
        
        self.nb=self.parameters['nb']
        self.bot_width = self.convert_dist*self.parameters['bot_width']
        self.particle_mass = self.convert_mass*self.parameters['particle_mass']
        self.particle_width = self.convert_dist*self.parameters['particle_width']
        self.particle_width2 = self.convert_dist*self.parameters['particle_width2']
        self.particle_height = self.convert_dist*self.parameters['particle_height']
        self.particle_geom = self.parameters['particle_geom']
        self.scale_radius = self.parameters['scale_radius']
        self.offset_radius = self.parameters['offset_radius']
        self.R =  self.convert_dist*self.parameters['R']
        self.R = self.R * self.offset_radius
        self.Rbar=self.parameters['Rbar']
        self.interior_mode = self.parameters['interior_mode']
        self.offset_radius=self.parameters['offset_radius']
        self.particle_mix=self.parameters['particle_mix']
        
        #### material properties
        self.lateralFriction = self.parameters['lateralFriction']
        self.spinningFriction = self.parameters['spinningFriction']
        self.rollingFriction = self.parameters['rollingFriction']
        self.dampingterm = self.parameters['dampingterm']
        self.Compliance_tangent = self.parameters['Ct'] 
        self.Compliance = self.parameters['C'] 
        self.restituition = self.parameters['restituition']
        
        #### calculated properties
        self.particle_volume=np.pi*self.particle_height*(self.particle_width/2)**2   # calculate volume
        self.particle_density=self.particle_mass/self.particle_volume # calculate density of robot 
        self.control_mode = self.parameters['control_mode']
        self.fixed = False
        
        #### find the number of particles and the respective ring radii
        (self.n,self.Ri) = self.MaxValues2()

        self.total_particles = np.sum(self.n) # find total particles
        
        #### create particle material
        self.particle_material = self.Material(self.lateralFriction)
        
        
        self.diff = self.Rbar - self.Ri[0] - self.bot_width/2 - self.particle_width2/2 
        
        #### save parameters
        self.parameters['n'] = self.n
        self.parameters['total_particles'] = self.total_particles
        self.parameters['Ri'] = self.Ri
        self.parameters["diff"] = self.diff
        self.number_parameters = self.parameters['number_parameters']
        #print(len(self.parameters),self.number_parameters+4)
        if len(self.parameters)==self.number_parameters+4:
            #print('save')
            np.save(self.mainDirectory+self.name+'/Parameters.npy',self.parameters)
        else:
             print('not save')
        
        
        with open(self.mainDirectory+name+"/Parameters.csv", 'w') as f:
            for key in self.parameters.keys():
                f.write("%s, %s\n" % (key, self.parameters[key]))
        


        #### Create empty lists 
        
        # Empty list of position of the particles
        self.particle_xposition = {} # x position
        self.particle_yposition = {} # y position
        self.particle_zposition = {} # z position
        
        # empty list of velocity of the particles
        self.particle_xvelocity = {} # x velocity
        self.particle_yvelocity = {} # y velocity
        self.particle_zvelocity = {} # z velocity
        
        # empty list of particle total forces
        self.particle_xForcetotal = {} # x forces
        self.particle_yForcetotal = {} # y forces
        self.particle_zForcetotal = {} # z forces
        
        # empty list of contact forces
        self.particle_xForcecontact = {} # x forces
        self.particle_yForcecontact = {} # y forces
        self.particle_zForcecontact = {} # z forces       
        
        
        self.particles = []
        self.Rm = []
        self.Area = 0
        self.N1=0
        self.N2=0
        
        # colors
        self.col_y = chrono.ChColorAsset(); self.col_y.SetColor(chrono.ChColor(1, 1, 0))       # Yellow
        self.col_b = chrono.ChColorAsset(); self.col_b.SetColor(chrono.ChColor(0, 0, 1))       # Blue
        self.col_g = chrono.ChColorAsset(); self.col_g.SetColor(chrono.ChColor(0, 1, 0))       # Green
        self.col_r = chrono.ChColorAsset(); self.col_r.SetColor(chrono.ChColor(1, 0, 0))       # Green
        self.col_p = chrono.ChColorAsset(); self.col_p.SetColor(chrono.ChColor(0.44, .11, 52)) # Purple        
        


        #### mono-dispersion 
        if self.interior_mode=="monodispersion":  
            count=0
            for i in range(self.n.size):
                if i%2==0:
                    con='b'
                    const=0
                else:
                    con='a'
                    const=0
                    
                    
                self.radius2=self.particle_width/2 - self.offset_radius
                
                #R2=self.radius2*self.n[i]/(np.pi) + const   
                R2=self.Ri[i]+self.diff
                thetahat=random.uniform(-np.pi/12, np.pi/12)
                for j in range(self.n[i]):
                     
                    self.particle_xposition["particle_xposition{0}".format(count)]=[] # x position  
                    self.particle_yposition["particle_yposition{0}".format(count)]=[] # y position 
                    self.particle_zposition["particle_zposition{0}".format(count)]=[] # z position 
                    
                    
                    self.particle_xvelocity["particle_xvelocity{0}".format(count)]=[] # x velocity
                    self.particle_yvelocity["particle_yvelocity{0}".format(count)]=[] # y velocity
                    self.particle_zvelocity["particle_zvelocity{0}".format(count)]=[] # z velocity
                    

                    count=count+1
                    
                    # position
                    x = R2*np.cos(j*2*np.pi/self.n[i])+self.xcenter # x position 
                    y = .5*self.particle_height                         # y position 
                    z = R2*np.sin(j*2*np.pi/self.n[i])+self.zcenter # z position 
                    
                    xp=np.cos(thetahat)*x - np.sin(thetahat)*z
                    zp=np.sin(thetahat)*x + np.cos(thetahat)*z
                    # create granular
                    gran = chrono.ChBodyEasyCylinder(self.radius2, self.particle_height,self.particle_density,True,True)
                    gran.SetMaterialSurface(self.particle_material) # add material 
                    gran.SetPos(chrono.ChVectorD(x,y,z))
                    gran.SetName("grana"+str(i))                   # set name 
                    gran.SetId(i)                          # add id 
                    gran.SetCollide(True)                  # create collision   
                    gran.SetBodyFixed(self.fixed)          # Add body fixed 
    
                    # add color
                    gran.AddAsset(self.col_r)
                    
                    # mate to floor
                    pt=chrono.ChLinkMatePlane() 
                    pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
                    self.my_system.AddLink(pt)
                    
                    # set speed limit ( Helps but not always needed)
                    #gran.SetMaxSpeed(2)
                    #gran.SetLimitSpeed(False)
                    
                    # add to system
                    self.my_system.Add(gran) # add object to system 
                    self.particles.append(gran) 
                    
            np.savez(self.mainDirectory+self.name+'/Radii'+self.name+'.npz',Rm=self.Rm)   


        #### bidispersion (This one is used 99 percent of the time)
        if self.interior_mode=="bidispersion": 
            count=0
            
            for i in range(len(self.n)): # create ring
                if i%2==0:
                    self.radius2 = (self.particle_width2/2)                    
                    con = 'b'
                    const = 0
                else:
                    self.radius2 = self.particle_width/2 
                    con = 'a'
                    const = 0
                    
                # radii of the ith particle ring
                R2=self.Ri[i]+self.diff
                
                #### fill the ring
                for j in range(self.n[i]): 
                    
                    # empty arrays of variables
                    self.particle_xposition["particle_xposition{0}".format(count)]=[] # x position  
                    self.particle_yposition["particle_yposition{0}".format(count)]=[] # y position 
                    self.particle_zposition["particle_zposition{0}".format(count)]=[] # z position 
                    
                    
                    self.particle_xvelocity["particle_xvelocity{0}".format(count)]=[] # x velocity
                    self.particle_yvelocity["particle_yvelocity{0}".format(count)]=[] # y velocity
                    self.particle_zvelocity["particle_zvelocity{0}".format(count)]=[] # z velocity 
                    
                    self.particle_xForcetotal["particle_xForcetotal{0}".format(count)]=[]
                    self.particle_yForcetotal["particle_yForcetotal{0}".format(count)]=[]
                    self.particle_zForcetotal["particle_zForcetotal{0}".format(count)]=[]
                    
                    self.particle_xForcecontact["particle_xForcecontact{0}".format(count)]=[]
                    self.particle_yForcecontact["particle_yForcecontact{0}".format(count)]=[]
                    self.particle_zForcecontact["particle_zForcecontact{0}".format(count)]=[]
                    
                    
                    # positions of particles
                    x = R2*np.cos(j*2*np.pi/self.n[i]) # x position
                    y = .5*self.particle_height  # y position
                    z = R2*np.sin(j*2*np.pi/self.n[i]) # z position

                    # if there is a offset of the particle radii
                    thetahat=random.uniform(-np.pi/12, np.pi/12)
                    if self.particle_mix==True:
                        
                        xp=np.cos(thetahat)*z - np.sin(thetahat)*x
                        zp=np.sin(thetahat)*z + np.cos(thetahat)*x
                        xp=xp+self.xcenter
                        zp=zp+self.zcenter
                    # if there is not a offset of the particle radii
                    if self.particle_mix==False:
                        xp=x+self.xcenter
                        zp=z+self.zcenter
                    
                    
                    
                    self.Rm.append(self.radius2) # fill the radii of the particle radius
                    self.Area = self.Area + (np.pi)*(self.radius2)**2 # total area of the particles 
                    # ring number 
                    if i%2==0:
                        con = 'b'
                        const = 0
                        self.N1 = 1+self.N1
                    else: 
                        con = 'a'
                        const = 0
                        self.N2 = 1+self.N2  
                        
                    # create body
                    gran = chrono.ChBody()
                    gran = chrono.ChBodyEasyCylinder(self.radius2 , self.particle_height ,self.particle_density,True,True)
                    gran.SetPos(chrono.ChVectorD(xp,y,zp))
                    gran.SetMaterialSurface(self.particle_material)
                    gran.SetName('gran'+str(con)+str(count))
                    gran.SetId(i)
                    gran.SetCollide(True)
                    gran.SetBodyFixed(self.fixed)
                    count=count+1
                    # alternate colors on rings so one is red the other is green
                    if i%2==0:
                    # add color
                        col_r = chrono.ChColorAsset()
                        col_r.SetColor(chrono.ChColor(1, 0, 0))
                        gran.AddAsset(col_r)
                    else:
                        col_r = chrono.ChColorAsset()
                        col_r.SetColor(chrono.ChColor(0, 1, 0))
                        gran.AddAsset(col_r)  
                    
                    # set speed limit ( Helps but not always needed)
                    # gran.SetMaxSpeed(2)
                    # gran.SetLimitSpeed(True)
                    
                    # link to plane    
                    #pt=chrono.ChLinkMatePlane()
                    #pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
                    #self.my_system.AddLink(pt)
                    
                    # add to system
                    self.my_system.Add(gran) # add object to system 
                    self.particles.append(gran) 
                    
                    
        np.savez(self.mainDirectory+self.name+'/Radii'+self.name+'.npz',Rm=self.Rm)     
        self.parameters['Area'] = self.Area
        self.parameters['N1(smaller)'] = self.N1
        self.parameters['N2(larger)'] = self.N2
        
        np.save(self.mainDirectory+self.name+'/Parameters.npy',self.parameters)
        
        
        with open(self.mainDirectory+name+"/Parameters.csv", 'w') as f:
            for key in self.parameters.keys():
                f.write("%s, %s\n" % (key, self.parameters[key]))             




    def Material(self,lateralFriction):
        ''' Function that creates material object '''
        material = chrono.ChMaterialSurfaceNSC() # create material object
        material.SetFriction(lateralFriction) # set friction properties
        material.SetDampingF(self.dampingterm) # set damping properties
        material.SetCompliance(self.Compliance) # set compliance property
        material.SetComplianceT(self.Compliance_tangent) # set tangential property
        material.SetRestitution(self.restituition)
        material.SetRollingFriction(self.rollingFriction)
        material.SetSpinningFriction(self.spinningFriction)
        return (material)
        
        
        
        
    def MaxValues(self):
        ''' Function to create the particle rings and and number of particles per ring '''
        if self.interior_mode=='empty':
            N=[]
            R=0
        radius=self.bot_width/2
        radius2=self.particle_width/2 
        
        if self.interior_mode=="bidispersion":            
            Rin=self.R-radius
            S=2*radius2+2*radius
            P=int(Rin/S)
            Ri=np.zeros(2*P)

            N=[]
            for i in range(P):
                Ri[2*i]=Rin-1.33*radius2-i*S
                Ri[2*i+1]=Rin-(2*1.33*radius2+radius2)-i*S
    

            for i in range(P):
                N.append(int((np.pi*2*Ri[2*i])/(2*1.33*radius2)))
                N.append(int((np.pi*2*Ri[2*i+1])/(2*radius2)))

        if self.interior_mode=="monodispersion":
            Rin=self.R-radius-radius2
            ngrans1=int(Rin/(2*radius2))
            Ri=np.zeros((1,ngrans1))
            ni=np.zeros((1,ngrans1))
            radii=Rin-(2*radius2)
            for i in range(ngrans1):
                remainder=((2*radius2))*i
                Ri[:,i]=radii-remainder
                ni[:,i]=np.floor(((Ri[:,i]*np.pi)/radius2))
            n=np.asarray(ni,dtype=int)
            N=n[0]
            
        if self.interior_mode=="bi_dispersion_ring":
            Rin=self.R-radius
            ngrans1=int(Rin/(2*radius2))
            Ri=np.zeros((1,ngrans1))
            ni=np.zeros((1,ngrans1))
            radii=Rin-(2*radius2)
            for i in range(ngrans1):
                remainder=((2*radius2))*i
                Ri[:,i]=radii-remainder
                ni[:,i]=np.floor(((Ri[:,i]*np.pi)/radius2))
            n=np.asarray(ni,dtype=int)
            N=n[0]           
           
            
        if self.interior_mode=="bi_dispersion_uniform_ring":
            Rin=self.R-radius
            ngrans1=int(Rin/(2*radius2))
            Ri=np.zeros((1,ngrans1))
            ni=np.zeros((1,ngrans1))
            radii=Rin-(2*radius2)
            for i in range(ngrans1):
                remainder=((2*radius2))*i
                Ri[:,i]=radii-remainder
                ni[:,i]=np.floor(((Ri[:,i]*np.pi)/radius2))
            n=np.asarray(ni,dtype=int)
            N=n[0]           
            
            
        return(N,Ri.flatten())
    
    
    def MaxValues2(self):
        ''' Function to create the particle rings and and number of particles per ring '''
        if self.interior_mode=='empty':
            N=[]
            Ri=[]
        
        radius=self.bot_width/2
        radius2=(self.particle_width/2) 
        radius3=(self.particle_width2/2) 
            
        if self.interior_mode=="bidispersion":            
            Rin=self.R-radius
            S=2*radius2+2*radius3
            P=int(Rin/S)
            Ri=np.zeros(2*P)
            N=[]
            
            # create ring radii
            for i in range(P):
                Ri[2*i]=Rin-radius3-i*S # i ring                
                Ri[2*i+1]=Rin-(2*radius3+radius2)-i*S # i+1 ring
    
            # number of particles per ring
            for i in range(P):
                N.append(int((np.pi*2*Ri[2*i])/(2*radius3))) # i ring
                N.append(int((np.pi*2*Ri[2*i+1])/(2*radius2))) # i+1 ring
            
        return(N,Ri)
     
    
    
    # return system (Helps with adding to the data extractor, simulaor, and controllers)
    def return_system(self):
        ''' Return system, springs, bots, obj, force '''
        return(self.my_system,self.particles)

    # save position data
    def save_data_position(self):
        ''' Save position of each bot '''
        for i in range(self.total_particles):
            self.particle_xposition["particle_xposition"+str(i)].append(self.particles[i].GetPos().x)
            self.particle_yposition["particle_yposition"+str(i)].append(self.particles[i].GetPos().y)
            self.particle_zposition["particle_zposition"+str(i)].append(self.particles[i].GetPos().z)
            

            
    # save velocity data
    def save_data_velocity(self):
        ''' save velocity of each bot '''
        for i in range(self.total_particles):
            self.particle_xvelocity["particle_xvelocity"+str(i)].append(self.particles[i].GetPos_dt().x)
            self.particle_yvelocity["particle_yvelocity"+str(i)].append(self.particles[i].GetPos_dt().y)
            self.particle_zvelocity["particle_zvelocity"+str(i)].append(self.particles[i].GetPos_dt().z)
            
    def save_data_Forces(self):
        ''' Save total force on each bot '''
        for i in range(self.total_particles):
            self.particle_xForcetotal["particle_xForcetotal"+str(i)].append(self.particles[i].Get_Xforce().x)
            self.particle_yForcetotal["particle_yForcetotal"+str(i)].append(self.particles[i].Get_Xforce().y)
            self.particle_zForcetotal["particle_zForcetotal"+str(i)].append(self.particles[i].Get_Xforce().z)
    
    def save_data_Forces_contact(self):
        ''' Save total force on each bot '''
        for i in range(self.total_particles):
            self.particle_xForcecontact["particle_xForcecontact"+str(i)].append(self.particles[i].GetContactForce().x)
            self.particle_yForcecontact["particle_yForcecontact"+str(i)].append(self.particles[i].GetContactForce().y)
            self.particle_zForcecontact["particle_zForcecontact"+str(i)].append(self.particles[i].GetContactForce().z)
                     
          
    # return position data
    def return_position_data(self):
        ''' return the dictionary of each bot '''
        return(self.particle_xposition,self.particle_yposition,self.particle_zposition)
    

      
    # return velocity data
    def return_velocity_data(self):
        ''' return the diction of each bot velocity '''
        return(self.particle_xvelocity,self.particle_yvelocity,self.particle_zvelocity)
        
    # return force data    
    def return_force_data(self):
        ''' return dictionary of each bot forces '''
        return(self.particle_xForcetotal,self.particle_yForcetotal,self.particle_zForcetotal)
  
    # return force data    
    def return_force_data_contact(self):
        ''' return dictionary of each bot forces '''
        return(self.particle_xForcecontact,self.particle_yForcecontact,self.particle_zForcecontact)
      
        
class floor:
    def __init__(self,name,my_system,path):
        self.name=name
        self.my_system = my_system
        self.mainDirectory = path
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters=parameters.tolist()
        self.convert_dist=self.parameters['convert_dist']
        self.convert_mass=self.parameters['convert_mass']
        self.floor_height = self.parameters['floor_height']
        self.floor_length = self.parameters['floor_length']
        self.bot_height = self.convert_dist*self.parameters['bot_height']
        self.lateralFriction = self.parameters['lateralFriction']
        self.spinningFriction = self.parameters['spinningFriction']
        self.rollingFriction = self.parameters['rollingFriction']
        self.dampingterm = self.parameters['dampingterm']
        self.Compliance_tangent = self.parameters['Ct'] 
        self.Compliance = self.parameters['C'] 
        self.floor_friction = self.parameters['floor_friction']
        self.restituition = self.parameters['restituition']
        self.control_mode = self.parameters['control_mode']
        if self.control_mode=="target_chasing":
            self.tunnel_geom = self.parameters['tunnel_geom']
        self.material1 = self.Material(self.floor_friction)
        self.material2 = self.Material(0)
        # self.body_floor = chrono.ChBody()
        # self.body_floor.SetName('floor')
        # self.body_floor.SetBodyFixed(True)
        # self.body_floor.SetPos(chrono.ChVectorD(0, -self.floor_height, 0 ))
        # self.body_floor.SetMaterialSurface(self.material1)
        # self.body_floor.GetCollisionModel().ClearModel()
        # self.body_floor.GetCollisionModel().AddBox(self.floor_length, self.floor_height, self.floor_length) # hemi sizes
        # self.body_floor.GetCollisionModel().BuildModel()       
        # self.body_floor.SetCollide(True)
        # body_floor_shape = chrono.ChBoxShape()
        # body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(self.floor_length, self.floor_height, self.floor_length)
        # self.body_floor.GetAssets().push_back(body_floor_shape)
        # col_k = chrono.ChColorAsset()
        # col_k.SetColor(chrono.ChColor(0, 0, 0))
        # self.body_floor.AddAsset(col_k)
        # self.my_system.Add(self.body_floor)
        

        
        
        #### bottom floor
        self.body_floor = chrono.ChBody()
        self.body_floor.SetName('floor')
        self.body_floor.SetBodyFixed(True)
        self.body_floor.SetPos(chrono.ChVectorD(0, -self.floor_height, 0 ))
        self.body_floor.SetMaterialSurface(self.material1)
        self.body_floor.GetCollisionModel().ClearModel()
        self.body_floor.GetCollisionModel().AddBox(self.floor_length, self.floor_height, self.floor_length) # hemi sizes
        self.body_floor.GetCollisionModel().BuildModel()       
        self.body_floor.SetCollide(True)
        body_floor_shape = chrono.ChBoxShape()
        body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(self.floor_length, self.floor_height, self.floor_length)
        self.body_floor.GetAssets().push_back(body_floor_shape)
        col_k = chrono.ChColorAsset()
        col_k.SetColor(chrono.ChColor(0, 0, 0))
        self.body_floor.AddAsset(col_k)
        self.my_system.Add(self.body_floor)
        
 
        #### top floor used to keep particles inside
        self.body_floor2 = chrono.ChBody()
        self.body_floor2.SetName('floor2')
        self.body_floor2.SetBodyFixed(True)
        self.body_floor2.SetPos(chrono.ChVectorD(0, self.floor_height+self.bot_height+.001, 0 ))
        self.body_floor2.SetMaterialSurface(self.material2)
        self.body_floor2.GetCollisionModel().ClearModel()
        self.body_floor2.GetCollisionModel().AddBox(self.floor_length, self.floor_height, self.floor_length) # hemi sizes
        self.body_floor2.GetCollisionModel().BuildModel()       
        self.body_floor2.SetCollide(True)
        self.my_system.Add(self.body_floor2)
        
        #### Target chasing control mode
        # creates tunnel
        if self.control_mode=="target_chasing":
            
            if self.tunnel_geom=='tanh':
                wall = chrono.ChBody()
                path="C:/soro/python/Pychrono/Strings/String_grasping/object_file/"
                mesh_for_visualization = chrono.ChTriangleMeshConnected()
                mesh_for_visualization.LoadWavefrontMesh(path+'wall_3.obj')
                mesh_for_visualization.Transform(chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(1))
                visualization_shape = chrono.ChTriangleMeshShape()
                visualization_shape.SetMesh(mesh_for_visualization)
                wall.AddAsset(visualization_shape)
                mesh_for_collision = chrono.ChTriangleMeshConnected()
                mesh_for_collision.LoadWavefrontMesh(path+'wall_3.obj')
                # Optionally: you can scale/shrink/rotate the mesh using this:
                mesh_for_collision.Transform(chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(1))
                wall.GetCollisionModel().ClearModel()
                wall.GetCollisionModel().AddTriangleMesh(
                mesh_for_collision, # the mesh 
                False,  # is it static?
                False)  # is it convex?
                wall.SetMaterialSurface(self.material1)
                wall.GetCollisionModel().BuildModel()        
                wall.SetPos(chrono.ChVectorD(0,-3.2,0))
                wall.SetMass(16)
                wall.SetInertiaXX(chrono.ChVectorD(0.270,0.400,0.427))
                wall.SetInertiaXY(chrono.ChVectorD(0.057,0.037,-0.062))            
                wall.SetBodyFixed(True)
                col_y = chrono.ChColorAsset() # apply color
                col_y.SetColor(chrono.ChColor(1, 1, 0))
                wall.AddAsset(col_y)
                wall.SetCollide(True) # set the collision mode
                self.my_system.Add(wall)         
    
            elif self.tunnel_geom=='sin':
                wall = chrono.ChBody()
                # Attach a visualization shape .
                path="C:/soro/python/Pychrono/Strings/String_grasping/object_file/"
                mesh_for_visualization = chrono.ChTriangleMeshConnected()
                mesh_for_visualization.LoadWavefrontMesh(path+'wall_7.obj')
                mesh_for_visualization.Transform(chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(1))
                visualization_shape = chrono.ChTriangleMeshShape()
                visualization_shape.SetMesh(mesh_for_visualization)
                wall.AddAsset(visualization_shape)
        
                mesh_for_collision = chrono.ChTriangleMeshConnected()
                mesh_for_collision.LoadWavefrontMesh(path+'wall_7.obj')
                mesh_for_collision.Transform(chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(1))
                wall.GetCollisionModel().ClearModel()
                wall.GetCollisionModel().AddTriangleMesh(
                mesh_for_collision, # the mesh 
                False,  # is it static?
                False)  # is it convex?
                wall.GetCollisionModel().BuildModel()        
                wall.SetPos(chrono.ChVectorD(0,-3.2,0))
                wall.SetMass(16)
                wall.SetInertiaXX(chrono.ChVectorD(0.270,0.400,0.427))
                wall.SetInertiaXY(chrono.ChVectorD(0.057,0.037,-0.062))            
                wall.SetBodyFixed(True)
                col_y = chrono.ChColorAsset() # apply color
                col_y.SetColor(chrono.ChColor(1, 1, 0))
                wall.AddAsset(col_y)
                wall.SetCollide(True) # set the collision mode
                self.my_system.Add(wall)    
                
    
    def Material(self,lateralFriction):
        ''' Function that creates material object '''
        material = chrono.ChMaterialSurfaceNSC() # create material object
        material.SetFriction(lateralFriction) # set friction properties
        material.SetDampingF(self.dampingterm) # set damping properties
        material.SetCompliance(self.Compliance) # set compliance property
        material.SetComplianceT(self.Compliance_tangent) # set tangential property
        material.SetRestitution(self.restituition)
        material.SetRollingFriction(self.rollingFriction)
        material.SetSpinningFriction(self.spinningFriction)
        return (material)    
    
    
    def return_enviroment(self):
        ''' Return the object my_system '''
        return(self.my_system)
    
class Ball:
    def __init__(self,name,my_system,body_floor,path):
        self.name=name # name of simulation 
        self.my_system = my_system # my system object
        self.body_floor = body_floor # my body floor object
        self.mainDirectory = path # path to the simulation file 
        
        copyfile(__file__,self.mainDirectory+self.name+"/"+'objects.py')
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters=parameters.tolist()
        
        
        #### Imported Variables 
        self.convert_dist=self.parameters['convert_dist']
        self.convert_mass=self.parameters['convert_mass']
        self.height=self.convert_dist*self.parameters['bot_height']
        self.y = self.convert_dist*self.parameters['bot_height']/2
        self.x =self.parameters['ballx']
        self.z =self.parameters['ballz']
        
        #### material properties
        self.lateralFriction = self.parameters['lateralFriction']
        self.spinningFriction = self.parameters['spinningFriction']
        self.rollingFriction = self.parameters['rollingFriction']
        self.dampingterm = self.parameters['dampingterm']
        self.Compliance_tangent = self.parameters['Ct'] 
        self.Compliance = self.parameters['Cr'] 
        self.restituition = self.parameters['restituition']
        self.material=self.Material(self.lateralFriction)
        self.geom = self.parameters['ball_geometry']
        self.fixed=self.parameters['ball_fixed']
        
        # additional properties if the ball object is c shape
        if self.geom=="c_shape":
            self.w=self.parameters['w']
            self.l=self.parameters['l']
            self.t=self.parameters['t']
            
        # additional properties if the ball object is rectangle
        if self.geom =="rectangle":
            self.ball_length = self.parameters['ball_length']  
        
        self.radius = self.parameters['ball_radius']
        self.mb= self.parameters['ball_mass']
        
        # calulate properties
        volume3=np.pi*self.height*(self.radius)**2   # calculate volume
        self.rho=self.mb/volume3 # density
        
        
        #### empty arrarys
        self.balls=[] # array containing the ball object
        self.obj=[]
        
        self.forceb=[] # empty array containing the forces
        
        # ball position
        self.bx={} #x position
        self.bz={} #z position
        
        # ball velocity
        self.bvx={} # x velocity
        self.bvz={} # z velocity
        
        # applied force
        self.bFx={} # x force
        self.bFy={} # y force
        self.bFz={} # z force
        
        # total force
        self.bFtx={} # x force
        self.bFty={} # y force
        self.bFtz={} # z force
        
        # quaterion (rotation orientation)
        self.bq0 = {}
        self.bq1 = {}
        self.bq2 = {}
        self.bq3 = {}        
        
        # empty lists for pull test
        self.Fb={} # force
        self.PX={} # x position
        self.PY={} # z position
        self.TIME={} # time
        
        
        #### fill empty lists
        self.bx["ballx"]=[]
        self.bz["ballz"]=[] 
        
        self.bvx["ballvx"]=[]        
        self.bvz["ballvz"]=[]
        
        self.bFx["ballfx"]=[]
        self.bFy["ballfy"]=[]
        self.bFz["ballfz"]=[]
        
        
        self.bFtx["ballftx"]=[]
        self.bFty["ballfty"]=[]        
        self.bFtz["ballftz"]=[]
        
        self.bq0["bq0"] = []
        self.bq1["bq1"] = []
        self.bq2["bq2"] = []
        self.bq3["bq3"] = [] 
        
        self.Fb["Fb"]=[]
        self.PX["PX"]=[]
        self.PY["PY"]=[]
        self.TIME["TIME"]=[]
        
        z2x = chrono.ChQuaternionD()
        z2x.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 1, 0))
        z2y = chrono.ChQuaternionD()
        z2y.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 0, 1))
        
        #### colors
        self.col_y = chrono.ChColorAsset(); self.col_y.SetColor(chrono.ChColor(1, 1, 0))       # Yellow
        self.col_b = chrono.ChColorAsset(); self.col_b.SetColor(chrono.ChColor(0, 0, 1))       # Blue
        self.col_g = chrono.ChColorAsset(); self.col_g.SetColor(chrono.ChColor(0, 1, 0))       # Green
        self.col_p = chrono.ChColorAsset(); self.col_p.SetColor(chrono.ChColor(0.44, .11, 52)) # Purple
        self.col_w = chrono.ChColorAsset(); self.col_p.SetColor(chrono.ChColor(.8, .8, .8)) # Purple
        
        self.check1=0
        #### circle
        if self.geom=='circle':
            #Create ball
            ball = chrono.ChBody() # create ball object
            ball = chrono.ChBodyEasyCylinder(self.radius, self.height,self.rho,True,True) # specify properties and make it a cylinder
            ball.SetPos(chrono.ChVectorD(self.x,self.y,self.z)) # set position
            ball.SetName("ball") # give it a name
            ball.SetId(10000) # ball id
            ball.SetCollide(True) # set the collision mode
            ball.SetBodyFixed(self.fixed) # set if its fixed
            ball.SetMaterialSurface(self.material) # add material
            
            # this is a visual mechanism for irrlecht
            body_floor_texture = chrono.ChTexture() 
            body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'bluwhite.png')
            ball.GetAssets().push_back(body_floor_texture)

            
            # pt=chrono.ChLinkMatePlane()
            # pt.Initialize(self.body_floor,ball,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
            # self.my_system.AddLink(pt) 
            #pt=chrono.ChLinkMatePlane() 
            #pt.Initialize(self.body_floor,ball,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
            #append the constraint and the ball to array of objects and system
            
            # make its movement only in the x direction 
            prismatic_ground_ball = chrono.ChLinkLockPrismatic()
            prismatic_ground_ball.SetName("prismatic_ground_ball")
            prismatic_ground_ball.Initialize(self.body_floor, ball, chrono.ChCoordsysD(chrono.ChVectorD(self.x, self.y, self.z), chrono.Q_ROTATE_X_TO_Z))
            
            self.my_system.Add(prismatic_ground_ball) 
            self.obj.append(ball)
            self.balls.append(ball) 
            self.my_system.Add(ball) 
            
            # set external force
            myforcez = chrono.ChForce() # create it 
            ball.AddForce(myforcez) # apply it to bot object
            myforcez.SetMode(chrono.ChForce.FORCE) # set the mode
            myforcez.SetDir(chrono.VECT_Z) # set direction 
            myforcez.SetVpoint(chrono.ChVectorD(0,0.05,0))
            self.forceb.append(myforcez) # add to force list
            
        #### square  
        elif self.geom=="square":
            const=self.radius*2
            ball = chrono.ChBody() # create ball object
            ball = chrono.ChBodyEasyBox(const,self.height,const,self.rho,True,True) # create object # specify properties and make it a cylinder
            ball.SetPos(chrono.ChVectorD(self.x,self.y,self.z)) # set position
            
            # rotate the object if desired
            #rotation1 = chrono.ChQuaternionD() # rotate the robots about y axis 
            #rotation1.Q_from_AngAxis(-np.pi/4, chrono.ChVectorD(0, 1, 0)) 
            #ball.SetRot(rotation1)
                
            ball.SetName("ball") # give it a name
            ball.SetId(10000) # set id number
            ball.SetCollide(True) # set the collision mode
            ball.SetBodyFixed(self.fixed) # set if its fixed

            # lock movement to x direction
            prismatic_ground_ball = chrono.ChLinkLockPrismatic()
            prismatic_ground_ball.SetName("prismatic_ground_ball")
            prismatic_ground_ball.Initialize(self.body_floor, ball, chrono.ChCoordsysD(chrono.ChVectorD(self.x, self.y, self.z), chrono.Q_ROTATE_X_TO_Z))
            
            self.my_system.Add(prismatic_ground_ball) 
            self.obj.append(ball)
            self.balls.append(ball) 
            self.my_system.Add(ball) 
            # set position
            myforcez = chrono.ChForce() # create it 
            ball.AddForce(myforcez) # apply it to bot object
            myforcez.SetMode(chrono.ChForce.FORCE) # set the mode
            myforcez.SetDir(chrono.VECT_Z) # set direction 
            myforcez.SetVpoint(chrono.ChVectorD(0,0.05,0))
            self.forceb.append(myforcez) # add to force list
            
        #### triangle    
        if self.geom=="triangle":     
            
            const=self.radius*2*np.pi/3
            r=const*np.sqrt(3)/3
            
            
            #r = .04/.5236
            x1=0
            y1=-r
            x2=r*np.cos(7*np.pi/6)
            y2=-r*np.sin(7*np.pi/6)
            x3=r*np.cos(11*np.pi/6)
            y3=-r*np.sin(11*np.pi/6)
            pt_vect = chrono.vector_ChVectorD()
            #const=.4627
            
            # creates top
            pt_vect.push_back(chrono.ChVectorD(y1,self.height/2,x1))
            pt_vect.push_back(chrono.ChVectorD(y2,self.height/2,x2))
            pt_vect.push_back(chrono.ChVectorD(y3,self.height/2,x3))
            
            # create bottom
            pt_vect.push_back(chrono.ChVectorD(y1,-self.height/2,x1))
            pt_vect.push_back(chrono.ChVectorD(y2,-self.height/2,x2))
            pt_vect.push_back(chrono.ChVectorD(y3,-self.height/2,x3))            

            ball=chrono.ChBodyEasyConvexHull(pt_vect,self.rho,True,True)   
            
            ball.SetPos(chrono.ChVectorD(self.x,self.y,self.z)) # set positon 
            ball.SetName("ball") # give it a name
            ball.SetCollide(True) # set the collision mode
            ball.SetBodyFixed(True) # set if its fixed
            
            # create texture for irrlecht
            body_floor_texture = chrono.ChTexture()
            body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'bluwhite.png')
            ball.GetAssets().push_back(body_floor_texture)
            
            # set x direction to only move in that axis
            prismatic_ground_ball = chrono.ChLinkLockPrismatic()
            prismatic_ground_ball.SetName("prismatic_ground_ball")
            prismatic_ground_ball.Initialize(self.body_floor, ball, chrono.ChCoordsysD(chrono.ChVectorD(self.x, self.y, self.z), chrono.Q_ROTATE_X_TO_Z))
            self.my_system.Add(prismatic_ground_ball) 
            
            
            ball.GetCollisionModel().ClearModel()
            ball.GetCollisionModel().AddConvexHull(pt_vect)
            ball.GetCollisionModel().BuildModel() 
            
            
        #### C shape    
        if self.geom=="c_shape":          
            height=self.height   
            #w=2*self.radius
            #l=2*self.radius
           # t=.2*w
            box1_x=(-self.w/2)+(self.t/2)+self.z
            box1_y=self.y
            box1_z=self.x
            volume=(self.l - 2*self.t)*self.t*height
            rho=(self.mb/3)/volume
            box1 = chrono.ChBody() # create ball object
            box1 = chrono.ChBodyEasyBox(self.t,height,self.l - 2*self.t,rho,True,True) # create object # specify properties and make it a cylinder
            box1.SetPos(chrono.ChVectorD(box1_x,box1_y,box1_z))
            box1.SetName("ball") # give it a name
            box1.SetId(10000) 
            box1.SetCollide(True) # set the collision mode
            box1.SetBodyFixed(self.fixed) # set if its fixed
            box1.AddAsset(self.col_y)
            box1.SetMaterialSurface(self.material) # apply material
            prismatic_ground_ball = chrono.ChLinkLockPrismatic()
            prismatic_ground_ball.SetName("prismatic_ground_ball")
            prismatic_ground_ball.Initialize(self.body_floor, box1, chrono.ChCoordsysD(chrono.ChVectorD(self.x, self.y, self.z), chrono.Q_ROTATE_X_TO_Z))
            self.my_system.Add(prismatic_ground_ball) 
            self.obj.append(box1)
            self.balls.append(box1) 
            self.my_system.Add(box1) 
            
            


            box2_x=self.x
            box2_y=self.y
            box2_z = (self.l-self.t)/2 + self.z
            box2 = chrono.ChBody() # create ball object
            volume=self.w*self.t*height
            rho=(self.mb/3)/volume
            box2 = chrono.ChBodyEasyBox(self.w,height,self.t,rho,True,True) # create object # specify properties and make it a cylinder
            box2.SetPos(chrono.ChVectorD(box2_x,box2_y,box2_z))
            box2.SetName("ball") # give it a name
            box2.SetId(10000) 
            box2.SetCollide(True) # set the collision mode
            box2.SetBodyFixed(self.fixed) # set if its fixed
            box2.AddAsset(self.col_g)
            box2.SetMaterialSurface(self.material) # apply material
            prismatic_ground_ball = chrono.ChLinkLockPrismatic()
            prismatic_ground_ball.SetName("prismatic_ground_ball")
            prismatic_ground_ball.Initialize(self.body_floor, box2, chrono.ChCoordsysD(chrono.ChVectorD(self.x, self.y, self.z), chrono.Q_ROTATE_X_TO_Z))
            self.my_system.Add(prismatic_ground_ball) 
            self.obj.append(box2)
            self.balls.append(box2) 
            self.my_system.Add(box2)
            glue=chrono.ChLinkMateFix() # cretae fix constraint
            glue.Initialize(box1,box2) # fix object to bot
            self.my_system.AddLink(glue) # add to system 
            
            
            box3_x=self.x
            box3_y=self.y
            box3_z = -(self.l-self.t)/2 + self.z
            box3 = chrono.ChBody() # create ball object
            volume=self.w*self.t*height
            rho=(self.mb/3)/volume
            box3 = chrono.ChBodyEasyBox(self.w,height,self.t,rho,True,True) # create object # specify properties and make it a cylinder
            box3.SetPos(chrono.ChVectorD(box3_x,box3_y,box3_z))
            box3.SetName("ball") # give it a name
            box3.SetId(10000) 
            box3.SetCollide(True) # set the collision mode
            box3.SetBodyFixed(self.fixed) # set if its fixed
            box3.AddAsset(self.col_g)
            box3.SetMaterialSurface(self.material) # apply material
            prismatic_ground_ball = chrono.ChLinkLockPrismatic()
            prismatic_ground_ball.SetName("prismatic_ground_ball")
            prismatic_ground_ball.Initialize(self.body_floor, box3, chrono.ChCoordsysD(chrono.ChVectorD(self.x, self.y, self.z), chrono.Q_ROTATE_X_TO_Z))
            self.my_system.Add(prismatic_ground_ball) 
            self.obj.append(box3)
            self.balls.append(box3) 
            self.my_system.Add(box3)
            glue=chrono.ChLinkMateFix() # cretae fix constraint
            glue.Initialize(box1,box3) # fix object to bot
            self.my_system.AddLink(glue) # add to system 
            
            #glue=chrono.ChLinkMateFix() # cretae fix constraint
            ##glue.Initialize(ball,box1) # fix object to bot
            #self.my_system.AddLink(glue) # add to system             
        
        #### import    
        if self.geom=="import":          
  
            height=self.height
            R=1.25
            # a=0.504
            # b=0.37
            # w1=.75
            # w2=1.26
            # Rr=np.sqrt(R**2 - (w1)**2)
            
            nsides=50
            pt_vect = chrono.vector_ChVectorD()
            theta=np.linspace(3*np.pi/2 - np.pi/2 + .415,(2*np.pi + 3*np.pi/2) - .415 - np.pi/2,nsides,endpoint=True)
            
            x=[]
            y=[]
            for ii in range(len(theta)):
                x.append(R*np.cos(theta[ii]))
                y.append(R*np.sin(theta[ii]))
            
            
            #box1_width=1.007974825964066
            #box1_length=box1_width/4
            #box1_height=height
            #p#rint(box1_length)
            #print(box1_width)
            for i in range(len(x)):
                pt_vect.push_back(chrono.ChVectorD(x[i],-height/2,y[i]))
            
            for i in range(len(x)):
                pt_vect.push_back(chrono.ChVectorD(x[i],height/2,y[i]))
                
            ball=chrono.ChBodyEasyConvexHull(pt_vect,self.rho,True,True)   
            ball.SetPos(chrono.ChVectorD(0,height/2,0))
            ball.SetCollide(True) # set the collision mode
            ball.SetBodyFixed(True) # set if its fixed
            ball.SetName("ball") # give it a name
            ball.SetId(10000) 
            ball.GetCollisionModel().ClearModel()
            ball.GetCollisionModel().AddConvexHull(pt_vect)
            ball.GetCollisionModel().BuildModel()
            
            prismatic_ground_ball = chrono.ChLinkLockPrismatic()
            prismatic_ground_ball.SetName("prismatic_ground_ball")
            prismatic_ground_ball.Initialize(self.body_floor, ball, chrono.ChCoordsysD(chrono.ChVectorD(self.x, self.y, self.z), chrono.Q_ROTATE_X_TO_Z))
            self.my_system.Add(prismatic_ground_ball) 
            #self.my_system.Add(ball) 
            ball.SetMaterialSurface(self.material) # apply material
            self.obj.append(ball)
            self.balls.append(ball) 
            self.my_system.Add(ball) 


            box1_width=1.007974825964066
            box1_length=box1_width/2
            box1_height=height
            
            Rr=np.sqrt(R**2 - (box1_width/2)**2)
            box1_x=-(Rr + box1_length/2)
            box1_y=height/2
            box1_z=0 

            
            box2_width=1.5
            box2_length=.25
            
            box2_x=box1_x - box1_length/2 - box2_length/2
            box2_z=0 


            box1 = chrono.ChBody() # create ball object
            box1 = chrono.ChBodyEasyBox(box1_length,box1_height,box1_width,self.rho,True,True) # create object # specify properties and make it a cylinder
            box1.SetPos(chrono.ChVectorD(box1_x,box1_y,box1_z))
            box1.SetName("ball") # give it a name
            box1.SetId(10000) 
            box1.SetCollide(True) # set the collision mode
            box1.SetBodyFixed(True) # set if its fixed
            box1.AddAsset(self.col_y)
            box1.SetMaterialSurface(self.material) # apply material
            prismatic_ground_ball = chrono.ChLinkLockPrismatic()
            prismatic_ground_ball.SetName("prismatic_ground_ball")
            prismatic_ground_ball.Initialize(self.body_floor, box1, chrono.ChCoordsysD(chrono.ChVectorD(self.x, self.y, self.z), chrono.Q_ROTATE_X_TO_Z))
            self.my_system.Add(prismatic_ground_ball) 
            self.obj.append(box1)
            self.balls.append(box1) 
            self.my_system.Add(box1)    
            glue=chrono.ChLinkMateFix() # cretae fix constraint
            glue.Initialize(ball,box1) # fix object to bot
            self.my_system.AddLink(glue) # add to system 
            
            box2 = chrono.ChBody() # create ball object
            box2 = chrono.ChBodyEasyBox(box2_length,box1_height,box2_width,self.rho,True,True) # create object # specify properties and make it a cylinder
            box2.SetPos(chrono.ChVectorD(box2_x,box1_y,box2_z))
            box2.SetName("ball") # give it a name
            box2.SetId(10000) 
            box2.SetCollide(True) # set the collision mode
            box2.SetBodyFixed(True) # set if its fixed
            box2.AddAsset(self.col_y)
            box2.SetMaterialSurface(self.material) # apply material
            prismatic_ground_ball = chrono.ChLinkLockPrismatic()
            prismatic_ground_ball.SetName("prismatic_ground_ball")
            prismatic_ground_ball.Initialize(self.body_floor, box2, chrono.ChCoordsysD(chrono.ChVectorD(self.x, self.y, self.z), chrono.Q_ROTATE_X_TO_Z))
            self.my_system.Add(prismatic_ground_ball) 
            self.obj.append(box2)
            self.balls.append(box2) 
            self.my_system.Add(box2)    
            glue=chrono.ChLinkMateFix() # cretae fix constraint
            glue.Initialize(box1,box2) # fix object to bot
            self.my_system.AddLink(glue) # add to system 
                       
            
            # box2_width=a
            # box2_length=1.5
            # box2_height=height
            # box2_x=-(Rr + a + b/2 - self.x) 
            # box2_y=height/2
            # box2_z=0 + self.z
            
            # box2 = chrono.ChBody() # create ball object
            # box2 = chrono.ChBodyEasyBox(box2_width/2,box2_height,box2_length,self.rho,True,True) # create object # specify properties and make it a cylinder
            # box2.SetPos(chrono.ChVectorD(box2_x,box2_y,box2_z))
            # box2.SetCollide(True) # set the collision mode
            # box2.SetBodyFixed(True) # set if its fixed
            # box2.SetName("ball") # give it a name
            # box2.SetId(10000) 
            # box2.AddAsset(self.col_g)
            # box2.SetMaterialSurface(self.material) # apply material
            # prismatic_ground_ball = chrono.ChLinkLockPrismatic()
            # prismatic_ground_ball.SetName("prismatic_ground_ball")
            # prismatic_ground_ball.Initialize(self.body_floor, box2, chrono.ChCoordsysD(chrono.ChVectorD(self.x, self.y, self.z), chrono.Q_ROTATE_X_TO_Z))
            # self.my_system.Add(prismatic_ground_ball) 
            # self.obj.append(box2)
            # self.balls.append(box2) 
            # self.my_system.Add(box2)    
            
            # glue=chrono.ChLinkMateFix() # cretae fix constraint
            # glue.Initialize(box1,box2) # fix object to bot
            # self.my_system.AddLink(glue) # add to system 
        
            
    def Material(self,lateralFriction):
        ''' Function that creates material object '''
        material = chrono.ChMaterialSurfaceNSC() # create material object
        material.SetFriction(lateralFriction) # set friction properties
        #material.SetDampingF(self.dampingterm) # set damping properties
        material.SetCompliance(self.Compliance) # set compliance property
        material.SetComplianceT(self.Compliance_tangent) # set tangential property
        material.SetRollingFriction(self.rollingFriction)
        material.SetSpinningFriction(self.spinningFriction)
        material.SetRestitution(self.restituition)
        # material.SetComplianceRolling(Cr)
        # material.SetComplianceSpinning(Cs)
        return (material)            
        
            
        
    def save_data_position(self):
        ''' Function that saves the positon of the ball '''
        #self.bx["ballx"].append(self.balls[0].GetPos().x) # x postion 
        #self.bz["ballz"].append(self.balls[0].GetPos().z) # z postion 
        #tempx=0
        #tempz=0
        #if self.geom=="c_shape":
            #self.bx["ballx"].append((self.balls[0].GetPos().x+self.balls[1].GetPos().x+self.balls[2].GetPos().x)/3)    
            #self.bz["ballz"].append((self.balls[0].GetPos().z+self.balls[1].GetPos().z+self.balls[2].GetPos().z)/3)
           
        #else:
        self.bx["ballx"].append(self.balls[0].GetPos().x) # x postion 
        self.bz["ballz"].append(self.balls[0].GetPos().z) # z postion     
        
    def save_data_velocity(self):
        ''' save the velocity of the ball '''
        self.bvx["ballvx"].append(self.balls[0].GetPos_dt().x) # x velocity
        self.bvz["ballvz"].append(self.balls[0].GetPos_dt().z)# z velocity 
        
    def save_contact_force(self):
        ''' save contact forces  '''
        self.bFx["ballvx"].append(self.balls[0].GetContactForce().x) # x contact force
        self.bFy["ballvy"].append(self.balls[0].GetContactForce().y) # y contact force
        self.bFz["ballvz"].append(self.balls[0].GetContactForce().z) # z contact force
    
    def save_total_force(self):
        ''' save contact forces  '''
        self.bFtx["ballvx"].append(self.balls[0].Get_Xforce().x) # x contact force
        self.bFty["ballvy"].append(self.balls[0].Get_Xforce().y) # y contact force
        self.bFtz["ballvz"].append(self.balls[0].Get_Xforce().z) # z contact force
    
    def save_angle_data(self):
        ''' save angle in quarterions '''
        temp=self.balls[0].GetRot()  
        q0=temp.e0
        q1=temp.e1
        q2=temp.e2
        q3=temp.e3
        self.bq0["bq0"].append(q0)
        self.bq1["bq1"].append(q1)
        self.bq2["bq2"].append(q2)
        self.bq3["bq3"].append(q3)                  
    
    
    def return_angle_data(self):
        ''' return orientation quaterions of the ball'''
        return(self.bq0,self.bq1,self.bq2,self.bq3)
    
    def return_position_data(self):
        ''' return velocity data '''
        return(self.bx,self.bz)   

    def return_velocity_data(self):
        ''' return velocity data '''
        return(self.bvx,self.bvz)    

    # return system    
    def return_system(self):
        ''' return system  '''
        return(self.my_system,self.balls)      
    
    def return_ball_contact_forces(self):
        ''' return contact forces '''
        return(self.bFx,self.bFy,self.bFz)
    
    def return_ball_total_forces(self):
        ''' return total forces '''
        return(self.bFtx,self.bFty,self.bFtz)
    
class simulate:
    def __init__(self,name,my_system,bots,particles,Ball,controller,my_rep,path,Psi):
        self.name=name # name of simulation
        self.my_system = my_system # system object
        self.Psi = Psi # R-function object
        self.Bots = bots # bots object
        self.particles = particles # particles object
        self.controller = controller # controller class
        self.my_rep = my_rep # contact class
        self.ball=Ball # ball class
        
        #### Imported Variables 
        self.mainDirectory = path
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters=parameters.tolist()
        self.convert_dist=self.parameters['convert_dist']
        self.convert_mass=self.parameters['convert_mass']
        self.control_mode=self.parameters['control_mode']
        self.xcenter = self.parameters['xcenter']
        self.zcenter = self.parameters['zcenter']
        self.visual = self.parameters['visual'] # visualization time
        self.dt = self.parameters['dt'] # time step
        self.time_end = self.parameters['time_end'] # end of simuation time
        self.save_rate = self.parameters['save_rate'] # rate the system saves data
        
        # If grasping import additional variables
        if self.control_mode=="grasping" or self.control_mode=="grasping_u":
            self.geom=self.parameters['ball_geometry']
            self.ball_radius = self.parameters['ball_radius']
            self.ballx_ = self.parameters['ballx'] 
            self.ballz_ = self.parameters['ballz']
            self.tcut1 = self.parameters['tcut1']
            self.tcut2 = self.parameters['tcut2']
            self.tcut3 = self.parameters['tcut3']
            self.trig1 = 0
            self.rho=0
            self.k=0
            self.m=8
            if self.geom=="c_shape":
                self.w=self.parameters['w']
                self.l=self.parameters['l']
                self.t=self.parameters['t'] 



        ###### Predefined variables ######
        self.myapplication=[]
        self.epoch = 0 # epoc used intermnally
        
        # irrlecht specific variables
        self.camx = 0 # x positon of camera 
        self.camy = 5 # y positon of camera 
        self.camz = 0 # z positon of camera 
        self.camy_height = 2
        self.save_video = False
        self.Trip=False
        
        
        self.sim_start=timeit.default_timer() # start simulation time 


        #### Empty Arrays 
        self.THETA=[] # For searching, this is the beta angle of the syste, 
        self.Rr_=[] # for grasp searching, this is the radius from the center of the object to the point of the field 
        
        self.time = [] # time empty array
        self.time_contact = [] # contact time  empty array
        self.number_contacts = [] # number of contacts
        
        
        self.EPSILON4 = [] # epsilon metric 
        self.Qcm = [] # Qcm metric 
        self.calcultation_time  = [] # time to calcuate the metric
        self.calc_type=[] # type of calculation done for metric
        
        # center of contact points
        self.centroidx = [] # x position
        self.centroidz = [] # z position
         

        # contact positions
        self.Contact_points_x = [] # contact points x
        self.Contact_points_y = [] # contact points y
        self.Contact_points_z = [] # contact points z
        
        # contact forces
        self.Contact_force_x = [] # x forces
        self.Contact_force_y = [] # y forces
        self.Contact_force_z = [] # z forces
        
        # contact forces 2
        self.Contact_force_x2 = [] # x forces
        self.Contact_force_y2 = [] # y forces
        self.Contact_force_z2 = [] # z forces
        
        self.bodiesA = [] # name of the contact bodies A
        self.bodiesB = [] # name of the contact bodies B
        self.bodiesA_ID = [] # ID of the contact bodies A
        self.bodiesB_ID = [] # ID of the contact bodies B
        
        # x direction empty array vector
        self.dir_xx = []
        self.dir_xy = []
        self.dir_xz = []
        
        # y direction empty array vector
        self.dir_yx = []
        self.dir_yy = []
        self.dir_yz = []
        
        # z direction empty array vector
        self.dir_zx = []
        self.dir_zy = []
        self.dir_zz = []   
        
        #### Contact arrays used to calculate epsilon metric 
        self.time_contact_bot = [] # contact time  empty array for robots
        self.number_contacts_bot = [] # number of contacts for bots
        
        # contact position 
        self.Contact_points_x_bot = []
        self.Contact_points_y_bot = []
        self.Contact_points_z_bot = []
                
        # contact forces global 
        self.Contact_force_x_bot = []
        self.Contact_force_y_bot = []
        self.Contact_force_z_bot = []
                
        # cotnact forces local frame 
        self.Contact_force_x2_bot = []
        self.Contact_force_y2_bot = []
        self.Contact_force_z2_bot = []
        
        # object name 
        self.bodiesA_bot = []
        self.bodiesB_bot = []
        
        # object id
        self.bodiesA_ID_bot = []
        self.bodiesB_ID_bot = []

        # x direction of contact frame global
        self.dir_xx_bot = []
        self.dir_xy_bot = []
        self.dir_xz_bot = []
          
        # y direction of contact frame global
        self.dir_yx_bot = []
        self.dir_yy_bot = []
        self.dir_yz_bot = []
               
        # z direction of contact frame global
        self.dir_zx_bot = []
        self.dir_zy_bot = []
        self.dir_zz_bot = []
        
        self.i=0
        
        #### empty lists of contact info for ball
        self.Force_x_contact_ball_ = {} # contact forces for ball x
        self.Force_z_contact_ball_ = {} # contact forces for ball z   
        
        self.position_x_contact_ball_ = {} # contact position for ball x
        self.position_z_contact_ball_ = {} # contact position for ball z               
    

        self.dir_xx_contact_ball_ = {} # contact position for ball x
        self.dir_xz_contact_ball_ = {} # contact position for ball x
    
        self.dir_zx_contact_ball_ = {} # contact position for ball x
        self.dir_zz_contact_ball_ = {} # contact position for ball x
            
        self.x0b_=0
        self.y0b_=0
        
    # simulate the robot
    def simulate(self):
        #### Irrrlecnt
        #  Create an Irrlicht application to visualize the system
        if self.visual=="irr":
            self.myapplication = chronoirr.ChIrrApp(self.my_system, self.name , chronoirr.dimension2du(800,600))
            self.myapplication.AddTypicalSky()
            self.myapplication.AddTypicalLogo()
            self.myapplication.AddTypicalCamera(chronoirr.vector3df(self.camx,self.camy,self.camz),chronoirr.vector3df(self.camx,0,self.camz))
            self.myapplication.SetSymbolscale(.002)
            self.myapplication.SetShowInfos(True)
            self.myapplication.SetPaused(self.Trip)
            self.myapplication.AddTypicalLights()
            self.myapplication.DrawAll               
            self.myapplication.AssetBindAll()
            self.myapplication.AssetUpdateAll()
            self.myapplication.AddShadowAll()
            self.count=0
            self.myapplication.SetTimestep(self.dt)
            self.myapplication.SetTryRealtime(False)
            
            #Run the sim
            while(self.myapplication.GetDevice().run()):
                self.myapplication.BeginScene()
                self.myapplication.DrawAll()
                self.myapplication.DoStep()
                self.controller.run_controller() # run controller
                self.controller.get_position() # get position for controller
                self.my_rep.ResetList() # clear contact points list
                self.save_contacts()  # save contact points list
                time=np.round(self.my_system.GetChTime(),3)
                
                if self.control_mode=="grasping_explore":
                    if self.controller.t>=self.tcut3:
                         print("time change")
                         self.controller.t=0
                    else:
                         self.controller.t=self.controller.t+self.dt
                         print("not time")

                if self.control_mode=="shape_morphing":
                    ft=np.round(self.controller.Psi.tanh(np.round(self.my_system.GetChTime(),3)),3)
                
                    print('time='+str(time), 'f(t)='+str(ft),end='\n')
                elif self.control_mode=="grasping_u":
                    ft=np.round(self.controller.Psi.tanh(np.round(self.my_system.GetChTime(),3)-self.controller.Psi.tcut1),3)
                
                    print('time='+str(time), 'f(t)='+str(ft),end='\n')
                else:
                    print('time='+str(time))
                
                self.myapplication.EndScene()
                self.save_parameters() # save parameters
                
                self.epoch = self.epoch + 1 # next epoch
                self.my_rep.ResetList() # reset contact list
                
                # aaa=len(self.Bots.bots)
                # cam_x=0.33*(self.Bots.bots[0].GetPos().x + self.Bots.bots[int(aaa/3)].GetPos().x + self.Bots.bots[int(2*aaa/3)].GetPos().x)
                # cam_y=0.33*(self.Bots.bots[0].GetPos().y + self.Bots.bots[int(aaa/3)].GetPos().y + self.Bots.bots[int(2*aaa/3)].GetPos().y)
                # cam_z=0.33*(self.Bots.bots[0].GetPos().z + self.Bots.bots[int(aaa/3)].GetPos().z + self.Bots.bots[int(2*aaa/3)].GetPos().z)
                # self.myapplication.GetSceneManager().getActiveCamera().setPosition(chronoirr.vector3df(cam_x,cam_y+self.camy_height,cam_z))
                # self.myapplication.GetSceneManager().getActiveCamera().setTarget(chronoirr.vector3df(cam_x,cam_y,cam_z))
                # self.myapplication.SetVideoframeSave(self.save_video)
                # self.myapplication.SetVideoframeSaveInterval(round(1/(self.dt*60)))
                
                # Close the simulation if time ends
                if self.my_system.GetChTime()> self.time_end :
                    self.myapplication.GetDevice().closeDevice()
                    
            self.sim_end=timeit.default_timer() # save simulation time end
            self.parameters['sime_time']=(self.sim_end-self.sim_start)/60 # find total sim time
            self.number_parameters = self.parameters['number_parameters']
            self.parameters['rho']=self.rho # save rho variable used for grasping 
            
        #### pov   
        if self.visual=="pov": 
            while (self.my_system.GetChTime() < self.time_end): 
                self.my_system.DoStepDynamics(self.dt) # input time step
                self.controller.run_controller() # run controller
                self.controller.get_position() # acquire position 
                self.my_rep.ResetList() # reset contact list
                self.save_contacts() # save contact list
                
                if self.control_mode=="grasping_explore":
                    if self.controller.t>=self.tcut3:
                        print("time change")
                        self.controller.t=0
                    else:
                        self.controller.t=self.controller.t+self.dt
                        print("not time")
    
        
    
    
                time=np.round(self.my_system.GetChTime(),3)
                
                if self.control_mode=="shape_morphing":
                    ft=np.round(self.controller.Psi.tanh(np.round(self.my_system.GetChTime(),3)),3)
                
                    print('time='+str(time), 'f(t)='+str(ft),end='\n')
        
                elif self.control_mode=="grasping_u":
                    if self.controller.trig1_==0:
                          print('time='+str(time), 'phase = '+str(self.controller.trig1_+1),end='\n')  
                    elif self.controller.trig1_!=0:
                        print('time='+str(time), 'phase = '+str(self.controller.trig1_+1),end='\n')
                    else:
                        print('time='+str(time))
                else:
                    print('time='+str(time))
                    
                self.save_parameters() # save parameters
                
                self.epoch = self.epoch + 1 # next epoch
                self.my_rep.ResetList() # reset contact list
                
            self.sim_end=timeit.default_timer()
            self.parameters['sime_time']=(self.sim_end-self.sim_start)/60
            self.number_parameters = self.parameters['number_parameters']
            self.parameters['rho']=self.rho

        np.save(self.mainDirectory+self.name+'/Parameters.npy',self.parameters)

        with open(self.mainDirectory+self.name+"/Parameters.csv", 'w') as f:
            for key in self.parameters.keys():
                f.write("%s, %s\n" % (key, self.parameters[key]))

    def find_contact_forces(self):
        '''This function for extacting contact forces without regards for grasping'''
        # empty arrays for current variables
        self.Force_x_contact_ball_=[]
        self.Force_z_contact_ball_=[]
            
        self.position_x_contact_ball_=[]
        self.position_z_contact_ball_=[]
            
        self.dir_xx_contact_ball_=[]
        self.dir_xz_contact_ball_=[]
            
        self.dir_zx_contact_ball_=[]
        self.dir_zz_contact_ball_=[]      
        
        self.x0b,self.y0b=self.ball.balls[0].GetPos().x,self.ball.balls[0].GetPos().z

        difx=self.x0b_-self.ballx_ 
        dify=self.y0b_-self.ballz_

        if self.ball.check1==0:
            print("ball_check")
            self.ball.check1=1
            self.x0b_=self.x0b
            self.y0b_=self.y0b
            
        for j in range(int(self.number_contacts[-1])):
            temp1=self.bodiesA_bot[-1][j]
            temp2=self.bodiesB_bot[-1][j]  
            
            #### Contact forces for ball        
            if temp1[0:4]=="ball" or temp2[0:4]=="ball" :

                if temp1[0:4]=="ball":  # if object 1 is the ball
                    self.Force_x_contact_ball_.append(self.Contact_force_x2_bot[-1][j])
                    self.Force_z_contact_ball_.append(self.Contact_force_z2_bot[-1][j])
                        
                    self.position_x_contact_ball_.append(self.Contact_points_x_bot[-1][j])
                    self.position_z_contact_ball_.append(self.Contact_points_z_bot[-1][j])  
                        
                    self.dir_xx_contact_ball_.append(self.dir_xx_bot[-1][j])
                    self.dir_xz_contact_ball_.append(self.dir_xz_bot[-1][j]) 
                        
                    self.dir_zx_contact_ball_.append(self.dir_zx_bot[-1][j])
                    self.dir_zz_contact_ball_.append(self.dir_zz_bot[-1][j])                    
                    
                if temp2[0:4]=="ball": # if object 2 is the ball
                    self.Force_x_contact_ball_.append(self.Contact_force_x2_bot[-1][j])
                    self.Force_z_contact_ball_.append(self.Contact_force_z2_bot[-1][j])
                        
                    self.position_x_contact_ball_.append(self.Contact_points_x_bot[-1][j])
                    self.position_z_contact_ball_.append(self.Contact_points_z_bot[-1][j])  
                        
                    self.dir_xx_contact_ball_.append(self.dir_xx_bot[-1][j])
                    self.dir_xz_contact_ball_.append(self.dir_xz_bot[-1][j]) 
                        
                    self.dir_zx_contact_ball_.append(self.dir_zx_bot[-1][j])
                    self.dir_zz_contact_ball_.append(self.dir_zz_bot[-1][j])                      
                
                #### finding rho 
                if self.trig1==0:
                    self.trig1=1
                    if self.geom=="c_shape":
                        bx=self.ballx_ # center of object x
                        bz=self.ballz_ # center of object z
                    else:
                        bx=self.ball.balls[0].GetPos().x # center of object x
                        bz=self.ball.balls[0].GetPos().z # center of object z
                    # calculate rho
                    self.rho=np.sqrt((self.Contact_points_x_bot[-1][j]-bx)**2 + (self.Contact_points_z_bot[-1][j]-bz)**2)

                    self.parameters['rho']=self.rho # save rho 
           
            
           
        #### Properties used for R-function used for grasping
        if self.geom=="square":
            const=self.ball_radius*2-.01
            rx=const
            ry=const
            w=rx/2
            h=ry/2                    
            x__=[w,-w,-w,w,w]
            y__=[h,h,-h,-h,h]
            _x_=[]
            _y_=[]
            t=np.pi/4
            (segments)=self.create_segment(x__,y__) 
            const_=self.ball_radius*2
            xb,yb=0-const_/2,0 - const_/2
            x0_=xb
            y0_=yb


        if self.geom=="triangle":
           const=self.ball_radius*2*np.pi/3
           r=const*np.sqrt(3)/3 -.01
           x1=-r
           y1=0
           x2=-r*np.cos(2*np.pi/3)
           y2=r*np.sin(2*np.pi/3)
           x3=-r*np.cos(4*np.pi/3)
           y3=r*np.sin(4*np.pi/3)
           x__ = [x1,x2,x3,x1]
           y__ = [y1,y2,y3,y1]
           (segments)=self.create_segment(x__,y__)

        if self.geom=="c_shape":
            w=self.w/2
            l=self.l/2
            t=self.t
            x__=[w,-w,-w,w,w,-w+t,-w+t,w,w]
            y__=[-l,-l,l,l,l-t,l-t,-l+t,-l+t,-l]
            d=.02
            r=np.sqrt(d**2 / 2)
            x__=[w-r,-w+r,-w+r,w-r,w-r,-w+t-r,-w+t-r,w-r,w-r]
            y__=[-l+r,-l+r,l-r,l-r,l-t+r,l-t+r,-l+t-r,-l+t-r,-l+r]
            (self.segments)=self.create_segment(x__,y__)
            
        if self.geom=="import":
            R=1.25
            nsides=50
            Rbar=0.99*R
            diff=R - Rbar
            box1_width=1.007974825964066
            box1_length=box1_width/2
            thetabar=np.arcsin(((box1_width/2)-diff)/(Rbar))
            theta=np.linspace(3*np.pi/2 + thetabar,(2*np.pi + 3*np.pi/2) - thetabar,100,endpoint=True)
            xp=[]
            yp=[]
            
            for iij in range(len(theta)):
                xp.append(Rbar*np.cos(theta[iij]))
                yp.append(Rbar*np.sin(theta[iij]))

            box1_length=box1_width/2
            box2_width=1.5
            Rr=np.sqrt(R**2 - (box1_width/2)**2)
        
            box1_x=-(Rr + box1_length/2)
            box1_z=0 
            box2_width=1.5
            box2_length=.25
            box2_x=box1_x - box1_length/2 - box2_length/2
            box2_z=0 
            
            xp.append(-box1_width/2 +diff)
            yp.append(box1_x - box1_length/2 - diff)
            xp.append(-box2_width/2 + diff)
            yp.append(box1_x - box1_length/2 - diff)
            xp.append(-box2_width/2 +diff)
            yp.append(box1_x - box1_length/2 - (box2_length-diff))
            xp.append(box2_width/2 - diff)
            yp.append(box1_x - box1_length/2 - (box2_length-diff))
            xp.append(box2_width/2 - diff)
            yp.append(box1_x - box1_length/2 - diff)
            xp.append(box1_width/2 -diff)
            yp.append(box1_x - box1_length/2 - diff)
            xp.append(Rbar*np.cos(theta[0]))
            yp.append(Rbar*np.sin(theta[0]))
            (self.segments)=self.create_segment(yp,xp)            
        
        
        #### Empty arrays
        X=[]
        Y=[]
        theta=[]
        
        temp_position_x = []
        temp_position_z = []
        
        temp_force_x = []
        temp_force_z = []  
        
        temp_vx = []
        temp_vy = []
        
        temp_c1 = []
        temp_c2 = []
        
        temp_id = []
        temp_theta = []
        temp_frames=[]
        temp_offset_theta=[]
        temp_wrenches=[]
        temp_wrenches_norm=[]
        temp_n=[]
        frames = np.zeros((len(self.position_x_contact_ball_),3)) # position orientation
        
        Vx=np.zeros((len(self.position_x_contact_ball_),2)) # x frame 
        Vy=np.zeros((len(self.position_x_contact_ball_),2)) # y frame
        
        C1=np.zeros((2,len(self.position_x_contact_ball_))) # positive cone
        C2=np.zeros((2,len(self.position_x_contact_ball_))) # negative cone   
        
        #### Calculate Qcm metric
        cx=np.sum(self.position_x_contact_ball_)/len(self.position_x_contact_ball_)
        cy=np.sum(self.position_z_contact_ball_)/len(self.position_z_contact_ball_)
        self.centroidx.append(cx) # append to x array
        self.centroidz.append(cy) # append to z array
        self.Qcm.append(np.sqrt((cx-self.x0b)**2 +(cy-self.y0b)**2))
        
        for ii in range(len(self.position_x_contact_ball_)):
            x0 = self.position_x_contact_ball_[ii] # x contact position
            y0 = self.position_z_contact_ball_[ii] # z position 
            theta1=np.arctan2(.2,1) # friction angle  
            
            if self.geom=="square":
                # Contact frame 
                Fx1=self.PHIDX(x0-self.x0b,y0-self.y0b,segments) # x direction 
                Fy1=self.PHIDY(x0-self.x0b,y0-self.y0b,segments) # y direction 
                mag=np.sqrt(Fx1**2 + Fy1**2) # magnitude
                
                # normalize
                Fx1=-Fx1/mag # x direction
                Fy1=-Fy1/mag # y direction
                
                F_t=np.array([Fx1,Fy1]) # stack in array
                t=self.angle(Fx1, Fy1)-np.pi/2 # orientation of contact frame
                
                X.append(x0-self.x0b) # save position x of frame
                Y.append(y0-self.y0b) # save position y of frame
                theta.append(t) #save angle of orientation of frame
                
                # frame empty array which is used to calc metric
                frames[ii,0]=x0-self.x0b # x 
                frames[ii,1]=y0-self.y0b # y 
                frames[ii,2]=t # orientation
                mag=1
                
                
            elif self.geom=="triangle":
                Fx1=self.PHIDX(x0-self.x0b,y0-self.y0b,segments)
                Fy1=self.PHIDY(x0-self.x0b,y0-self.y0b,segments)
                mag=np.sqrt(Fx1**2 + Fy1**2)
                Fx1=-Fx1/mag
                Fy1=-Fy1/mag
                F_t=np.array([Fx1,Fy1])
                X.append(x0-self.x0b)
                Y.append(y0-self.y0b)
                t=self.angle(Fx1, Fy1)-np.pi/2
                theta.append(t)
                frames[ii,0]=x0-self.x0b
                frames[ii,1]=y0-self.y0b
                frames[ii,2]=t
                #mag=np.sqrt(F_contact_ballx_entry[j]**2 + F_contact_ballz_entry[j]**2)
                mag=1  
                
            elif self.geom=="c_shape":
                difx=self.x0b_-self.ballx_
                dify=self.y0b_-self.ballz_
                Fx1=self.PHIDX(x0-(self.x0b-difx),y0-(self.y0b-dify),self.segments)
                Fy1=self.PHIDY(x0-(self.x0b-difx),y0-(self.y0b-dify),self.segments)
                mag=np.sqrt(Fx1**2 + Fy1**2)
                Fx1=-Fx1/mag
                Fy1=-Fy1/mag
                F_t=np.array([Fx1,Fy1])
                X.append(x0-(self.x0b-difx))
                Y.append(y0-(self.y0b-dify))
                t=self.angle(Fx1, Fy1)-np.pi/2
                theta.append(t)
                frames[ii,0]=x0-(self.x0b-difx)
                frames[ii,1]=y0-(self.y0b-dify)
                frames[ii,2]=t
                #mag=np.sqrt(F_contact_ballx_entry[j]**2 + F_contact_ballz_entry[j]**2)
                mag=1   
                
            elif self.geom=="circle":
                Fx1,Fy1=(self.x0b-x0),(self.y0b-y0)
                mag=np.sqrt(Fx1**2 + Fy1**2)
                Fx1=Fx1/mag
                Fy1=Fy1/mag
                F_t=np.array([Fx1,Fy1])
                X.append(x0-self.x0b)
                Y.append(y0-self.y0b)
                t=self.angle(Fx1, Fy1)-np.pi/2
                theta.append(t)
                frames[ii,0]=x0-self.x0b
                frames[ii,1]=y0-self.y0b
                frames[ii,2]=t

            elif self.geom=="import":
                Fx1=self.PHIDX(x0-self.x0b,y0-self.y0b,self.segments)
                Fy1=self.PHIDY(x0-self.x0b,y0-self.y0b,self.segments)
                mag=np.sqrt(Fx1**2 + Fy1**2)
                Fx1=-Fx1/mag
                Fy1=-Fy1/mag
                F_t=np.array([Fx1,Fy1])
                X.append(x0-self.x0b)
                Y.append(y0-self.y0b)
                t=self.angle(Fx1, Fy1)-np.pi/2
                theta.append(t)
                frames[ii,0]=x0-self.x0b
                frames[ii,1]=y0-self.y0b
                frames[ii,2]=t
                mag=1     
                
            mag=np.sqrt(self.Force_x_contact_ball_[ii]**2 + self.Force_z_contact_ball_[ii]**2) # magnitude of contact force
            
            T=np.array([[np.cos(t),-np.sin(t)],[np.sin(t),np.cos(t)]]) # transformation matrix   
            VYpp=T@np.array([[0],[1]]) # transform coordinates X frame
            VXpp=T@np.array([[1],[0]]) # transform coordinates Y frame
            VXpp=VXpp.flatten() # flatten the matrix 
            VYpp=VYpp.flatten() # flatten the matrix
        
            Vx[ii,:]=VXpp # Save the array X frame
            Vy[ii,:]=VYpp  # save the array Y frame      
            
            T1=np.array([[np.cos(theta1),-np.sin(theta1)],[np.sin(theta1),np.cos(theta1)]]) # transformation matrix   
            T2=np.array([[np.cos(-theta1),-np.sin(-theta1)],[np.sin(-theta1),np.cos(-theta1)]]) # transformation matrix       
            tem=Vy[ii,:].T
            C1[:,ii]=mag*T1@tem # positive friction cone
            C2[:,ii]=mag*T2@tem # negative friction cone
        
        
            mag2=np.sqrt(self.dir_xx_contact_ball_[ii]**2 +  self.dir_xz_contact_ball_[ii]**2) # magnitude of x contact frame vector
            temp_dirr=[self.dir_xx_contact_ball_[ii]/mag2,self.dir_xz_contact_ball_[ii]/mag2] # normalize the vector
        
       
            theta1=np.arctan2(.2,1) #+ frames[j,2]
            temp=np.round(np.nan_to_num(np.arccos(np.dot(VYpp ,temp_dirr))),2) # check if vector is in the cone
            
            fx=self.Force_x_contact_ball_[ii] # pull x contact force
            fy=self.Force_z_contact_ball_[ii] # pull y contact force
            
            mag_=np.sqrt(fx**2 +fy**2)
            f_=np.array([fx/mag_,fy/mag_])
            temp2=np.round(np.nan_to_num(np.arccos(np.dot(f_ ,Vy[ii,:]))),2)
            
            #### fix frame if the angle is out of order
            if temp<np.pi and temp > np.pi-theta1:
                fx=np.cos(-temp)*self.Force_x_contact_ball_[ii]-np.sin(-temp)*self.Force_z_contact_ball_[ii]
                fy=np.sin(-temp)*self.Force_x_contact_ball_[ii]+np.cos(-temp)*self.Force_z_contact_ball_[ii]
                dirxx=np.cos(-temp)*self.dir_xx_contact_ball_[ii]-np.sin(-temp)*self.dir_xz_contact_ball_[ii]
                dirxz=np.sin(-temp)*self.dir_xx_contact_ball_[ii]+np.cos(-temp)*self.dir_xz_contact_ball_[ii]
                temp=temp-np.pi
                temp2=np.round(np.nan_to_num(np.arccos(np.dot(f_ ,Vy[ii,:]))),2)
               # print("temp2:", temp2)
            else:
                fx=self.Force_x_contact_ball_[ii]
                fy=self.Force_z_contact_ball_[ii]
                temp=temp
                temp2=temp2
            #### check if force is within friction cone
            if temp<=theta1:
                if temp2<theta1:
                    # if within cone append variables 
                    
                    temp_offset_theta.append(theta1) # append offset frame
                    temp_frames.append(frames[ii,:]) # append frame
                    temp_theta.append(frames[ii,2]) # append theta
                    temp_id.append(ii) # append id
                    
                    temp_position_x.append(X[ii]) # append position X
                    temp_position_z.append(Y[ii]) # append position Z
                    
                    temp_force_x.append(fx) # append force x
                    temp_force_z.append(fy) # append force y
                    
                    temp_n.append(mag_*np.cos(temp2)) # normal force applied
                     
                    temp_vx.append(Vx[ii,:]) # contact frame x
                    temp_vy.append(Vy[ii,:]) # contact frame y 
                    
                    temp_c1.append(C1[:,ii]) # positive friction cone frame
                    temp_c2.append(C2[:,ii]) # negative friction cone frame                       
                      
                    # positive friction cone wrench
                    m_t=np.cross(np.array([X[ii],Y[ii]]),C1[:,ii])
                    temp=[C1[0,ii],C1[1,ii],m_t]
                    wrench1=temp
                    temp_wrenches.append(temp)

                    wrench_norm=np.zeros(3)
                    wrench_norm[0]=wrench1[0]
                    wrench_norm[1]=wrench1[1]
                    wrench_norm[2]=wrench1[2]/self.rho
                    temp_wrenches_norm.append(wrench_norm)

                    # negative friction cone wrench
                    m_t=np.cross(np.array([X[ii],Y[ii]]),C2[:,ii])
                    temp=[C2[0,ii],C2[1,ii],m_t]
                    wrench2=temp
                    temp_wrenches.append(temp)
                    
                    wrench_norm=np.zeros(3)
                    wrench_norm[0]=wrench2[0]
                    wrench_norm[1]=wrench2[1]
                    wrench_norm[2]=wrench2[2]/self.rho
                    temp_wrenches_norm.append(wrench_norm)   
        
        #### Calculate epsilon metric
        if len(temp_position_x)!=0:
            if len(temp_id)==0: # if there are no contacts
                self.calc_type.append(1)
                self.calcultation_time.append(0)
                self.EPSILON4.append(0)
                print("epsilon=0"," nc= ",len(temp_position_x)," try1") 
            elif len(temp_id)==1: # if there is one contact
                self.calc_type.append(2)
                self.calcultation_time.append(0)
                self.EPSILON4.append(0)
                print("epsilon=0"," nc= ",len(temp_position_x),"try2") 
             
            elif np.all((temp_wrenches_norm==0)): # if all the contact points have zero force 
                self.calc_type.append(3)
                self.calcultation_time.append(0)
                self.EPSILON4.append(0)
                print("epsilon=0"," nc= ",len(temp_position_x)," try3")     

            elif len(temp_id)>1: # if there are more than one contact point 
                wrench_norm=np.asarray(temp_wrenches_norm)
                try:
                    hull = ConvexHull(wrench_norm)
                    sim_start=timeit.default_timer() 
                    (epsilon,hullwrenchnorm,hullwrenchmags)=self.ferrari_canny_metric(wrench_norm)
                    #(hullwrenchnorm,epsilon,hullwrenchmags)=self.calculate_hull(hull,wrench_norm)
                    sim_end=timeit.default_timer() 
                    print("compute_time:",np.round(sim_end-sim_start,3))
                    epsilon=np.round(epsilon,3)
                    p=[0,0,0]
                    if self.in_hull(p,wrench_norm)==False: # if the hull is not centered
                        self.calc_type.append(4)
                        self.calcultation_time.append(np.round(sim_end-sim_start,3))
                        self.EPSILON4.append(0)
                        print("epsilon=0"," nc= ",len(temp_position_x)," try4") 
                    else: #if the hull is centered 
                        self.calc_type.append(5)
                        self.calcultation_time.append(np.round(sim_end-sim_start,3))
                        print("epsilon="+str(epsilon)," nc= ",len(self.position_x_contact_ball_)," try5") 
                        self.EPSILON4.append(epsilon)
            
                except: # if there is a error epsilon is zero
                    print("epsilon=0"," nc= ",len(self.position_x_contact_ball_)," try6")   
                    self.calcultation_time.append(0)
                    self.EPSILON4.append(0)
                    self.calc_type.append(6)
                    
        else: # if no contact eplsion is zero 
            print("epsilon=0","try7") 
            self.EPSILON4.append(0)
            self.calc_type.append(7)
            self.calcultation_time.append(0)          

    def save_contacts(self):
        ''' Save contact positions and forces '''
        if self.epoch%self.save_rate==0: # if enough time past then save variables
            self.my_system.GetContactContainer().ReportAllContacts(self.my_rep)
            crt_list = self.my_rep.GetList()

            if self.my_system.GetContactContainer().GetNcontacts()!=0: # if there are contact then append
                self.number_contacts.append(self.my_system.GetContactContainer().GetNcontacts())
                self.time_contact.append(self.my_system.GetChTime()) 
                
                # contact positions
                self.Contact_points_x.append(crt_list[0])
                self.Contact_points_y.append(crt_list[1])
                self.Contact_points_z.append(crt_list[2])
                
                # contact forces global frame 
                self.Contact_force_x.append(crt_list[3])
                self.Contact_force_y.append(crt_list[4])
                self.Contact_force_z.append(crt_list[5])

                # contact forces local frame                
                self.Contact_force_x2.append(crt_list[6])
                self.Contact_force_y2.append(crt_list[7])
                self.Contact_force_z2.append(crt_list[8]) 
                
                # contact bodies name
                self.bodiesA.append(crt_list[9])
                self.bodiesB.append(crt_list[10])
                
                # contact bodies id numbers
                self.bodiesA_ID.append(crt_list[11])
                self.bodiesB_ID.append(crt_list[12]) 

                # x vector of contact frame
                self.dir_xx.append(crt_list[14])
                self.dir_xy.append(crt_list[15])
                self.dir_xz.append(crt_list[16])
                
                # y vector of contact frame
                self.dir_yx.append(crt_list[17])
                self.dir_yy.append(crt_list[18])
                self.dir_yz.append(crt_list[19]) 
                
                # z vector of contact frame
                self.dir_zx.append(crt_list[20])
                self.dir_zy.append(crt_list[21])
                self.dir_zz.append(crt_list[22])     
                
                
                self.number_contacts_bot.append(self.my_system.GetContactContainer().GetNcontacts())
                self.time_contact_bot.append(self.my_system.GetChTime()) 
                
                self.Contact_points_x_bot.append(crt_list[0])
                self.Contact_points_y_bot.append(crt_list[1])
                self.Contact_points_z_bot.append(crt_list[2])
                
                self.Contact_force_x_bot.append(crt_list[3])
                self.Contact_force_y_bot.append(crt_list[4])
                self.Contact_force_z_bot.append(crt_list[5])
                
                self.Contact_force_x2_bot.append(crt_list[6])
                self.Contact_force_y2_bot.append(crt_list[7])
                self.Contact_force_z2_bot.append(crt_list[8]) 
                
                self.bodiesA_bot.append(crt_list[9])
                self.bodiesB_bot.append(crt_list[10])
                
                self.bodiesA_ID_bot.append(crt_list[11])
                self.bodiesB_ID_bot.append(crt_list[12]) 

                self.dir_xx_bot.append(crt_list[14])
                self.dir_xy_bot.append(crt_list[15])
                self.dir_xz_bot.append(crt_list[16])
                
                self.dir_yx_bot.append(crt_list[17])
                self.dir_yy_bot.append(crt_list[18])
                self.dir_yz_bot.append(crt_list[19]) 
                
                self.dir_zx_bot.append(crt_list[20])
                self.dir_zy_bot.append(crt_list[21])
                self.dir_zz_bot.append(crt_list[22])                  
                    
    def save_parameters(self):
        ''' Function that collects data of for the system '''
        if self.epoch%self.save_rate==0:
            self.time.append(np.round(self.my_system.GetChTime(),4)) # append time
            self.Bots.save_data_position() # save robot positions
            self.Bots.save_data_Forces() # save membrane forces
            self.Bots.save_data_velocity() # save robot velocites
            self.Bots.save_data_Forces_contact() # save robot contact forces
        
            self.Bots.save_data_forces() # save total forces

            self.particles.save_data_position() # save particle positions
            self.particles.save_data_velocity() # save particle velocities
            self.particles.save_data_Forces_contact() # save total contact forces
            self.particles.save_data_Forces()  # save total forces
            
            self.controller.save_field_value() # save field value of the distance function
            self.controller.save_controller_forces() # save the controller forces applied

            # if the system is grasping 
            if self.control_mode=="grasping" or self.control_mode=="grasping_explore" or self.control_mode=="grasping_u" or self.control_mode=="grasping_epsilon":
                self.find_contact_forces() # find epsilon metric
                self.ball.save_data_position() # save ball positions
                self.ball.save_data_velocity() # save ball velocity
                self.ball.save_contact_force() # save ball contact forces
                self.ball.save_total_force() # save ball total forces
                self.ball.save_angle_data() # save ball angle
                
                # specifices for pull test
                self.ball.Fb["Fb"].append(self.controller.fb) # pull force on ball
                self.ball.TIME["TIME"].append(np.round(self.my_system.GetChTime(),4)) # time 
                self.ball.PX["PX"].append(self.ball.balls[0].GetPos().x) # x position 
                self.ball.PY["PY"].append(self.ball.balls[0].GetPos().z) # z position 
                
                if self.control_mode=="grasping_explore":
                    self.THETA.append(self.controller.theta__)
                    self.Rr_.append(self.controller.Rr)
                
                if self.control_mode=="grasping_u": 
                    self.controller.TRIG1_.append(self.controller.trig1_) # variable for when the system transitions 
                    self.controller.smooth_epsilon(self.EPSILON4,np.round(self.my_system.GetChTime(),3)) # save smoothed epsilon value
    
    def angle(self,x, y):
        """ Angle function """
        rad = np.arctan2(y, x)
        return rad                
    
    def calculate_hull(self,hull,Wrench):
        """ Create a 3D hull of the wrench space and calculates the epsilon metric """
        origin = Point3D(0,0,0)
        nfacets = np.shape(hull.simplices)[0] #how many facets
        hullwrenchmags = np.zeros(nfacets)
        hullwrenchnorm = np.zeros((nfacets,3))
        i=0
        for s in hull.simplices:
            triangle = Wrench[s]   #convert from simplices to 3D points
            point1 = Point3D(triangle[0])
            point2 = Point3D(triangle[1])
            point3 = Point3D(triangle[2])
            theplane = Plane(point1,point2,point3)
            planedistance =  theplane.distance(origin)
            temp=theplane.normal_vector
            hullwrenchmags[i]=planedistance
            mag=sqrt(N(temp[0])**2 + N(temp[1])**2 + N(temp[2])**2)
            hullwrenchnorm[i,0]=N(temp[0])/mag
            hullwrenchnorm[i,1]=N(temp[1])/mag
            hullwrenchnorm[i,2]=N(temp[2])/mag
            i=i+1
        leastwrench = np.round(np.amin(hullwrenchmags),2)
       
        #print('least wrench (if enclosing), Union hull:',np.round(leastwrench,2))
        epsilon=leastwrench
        return(hullwrenchnorm,epsilon,hullwrenchmags)   
    
    def min_norm_vector_in_facet(self,facet, wrench_regularizer=1e-8):
          """ Finds the minimum norm point in the convex hull of a given facet (aka simplex) by solving a QP.
          Parameters
          ----------
          facet : 6xN :obj:`numpy.ndarray`
             vectors forming the facet
          wrench_regularizer : float
             small float to make quadratic program positive semidefinite
          Returns
          -------
          float
             minimum norm of any point in the convex hull of the facet
          Nx1 :obj:`numpy.ndarray`
             vector of coefficients that achieves the minimum
          """
          dim = facet.shape[1] # num vertices in facet
    
          # create alpha weights for vertices of facet
          G = facet.T.dot(facet)
          grasp_matrix = G + wrench_regularizer * np.eye(G.shape[0])
    
          # Solve QP to minimize .5 x'Px + q'x subject to Gx <= h, Ax = b
          P = cvx.matrix(2 * grasp_matrix)   # quadratic cost for Euclidean dist
          q = cvx.matrix(np.zeros((dim, 1)))
          G = cvx.matrix(-np.eye(dim))       # greater than zero constraint
          h = cvx.matrix(np.zeros((dim, 1)))
          A = cvx.matrix(np.ones((1, dim)))  # sum constraint to enforce convex
          b = cvx.matrix(np.ones(1))         # combinations of vertices
          
          sol = cvx.solvers.qp(P, q, G, h, A, b,options={'show_progress':False})
          v = np.array(sol['x'])
          
          min_norm = np.sqrt(sol['primal objective'])
          v=facet@v
          return(abs(min_norm),v)
      
    def in_hull(self,p, hull):
         """ Create hull """
         from scipy.spatial import Delaunay
         if not isinstance(hull,Delaunay):
             hull = Delaunay(hull)
         return (hull.find_simplex(p)>=0)  
     
    def ferrari_canny_metric(self,wrenches):
         """ Calculate ferrari canny metric """
         G=wrenches.T # transpose of wrench hull
         hull = cvh.ConvexHull(wrenches)
         min_dist = 100
         wrenchmags=[]
         for i in range(wrenches.shape[0]):
             wrenchmags.append(np.linalg.norm(wrenches[i]))    
         nfacets = np.shape(hull.simplices)[0] #how many facets
         hullwrenchmags = np.zeros(nfacets)
         hullwrenchnorm = np.zeros((nfacets,3))
         i=0
         for v in hull.vertices:
             if np.max(np.array(v)) < G.shape[1]: # because of some occasional odd behavior from pyhull
                 facet = G[:, v]
                 dist, v = self.min_norm_vector_in_facet(facet, wrench_regularizer=1e-10)
             #print(dist)
                 hullwrenchnorm[i,:]=v.T
                 hullwrenchmags[i]=np.linalg.norm(v)
                 if dist < min_dist:
                     min_dist = dist 
                 i=i+1   
         return(min_dist,hullwrenchnorm,hullwrenchmags)
     
        
    def normalize(self,F):
        (fy,fx)=np.gradient(F)
        return(F/np.sqrt(F**2 + fy**2 + fx**2))

    def line_(self,x,y,x1,x2,y1,y2):
        return(((x-x1)*(y2-y1)-(y-y1)*(x2-x1))/(np.sqrt((x2-x1)**2 + (y2-y1)**2)))

    def parabola(self,x,y,px,py):
        return((x-px)**2 - py - y)

    def equivalence(self,w1,w2,m):
        return(w1*w2/((w1**m +w2**m)**(1/m)))

    def Union(self,w1,w2):
        return(((w1+w2)/2) + ((np.sqrt((w1-w2)**2))/2))

    def intersection(self,w1,w2):
        return(((w1+w2)/2) - ((np.sqrt((w1-w2)**2))/2))          
        
    def F(self,x,y,x1,x2,y1,y2):
         L=np.sqrt((x2-x1)**2 + (y2-y1)**2)
         return(((x-x1)*(y2-y1)-(y-y1)*(x2-x1))/L)

    def delfx(self,x,y,x1,x2,y1,y2):
        L=np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return((-y1+y2)/L)

    def delfy(self,x,y,x1,x2,y1,y2):
        L=np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return((x1-x2)/L)

    def T(self,x,y,x1,x2,y1,y2):
        L=np.sqrt((x2-x1)**2 + (y2-y1)**2)
        xc=np.array([(x2+x1)/2,(y2+y1)/2])
        t = (1/L)*((L/2)**2 - ((x-xc[0])**2 + (y-xc[1])**2))    
        return(t)

    def deltx(self,x,y,x1,x2,y1,y2):
        xc = np.array([(x2+x1)/2,(y2+y1)/2]) 
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(-(x-xc[0])/ (L*np.sqrt((x-xc[0])**2 + (y-xc[1])**2)))

    def delty(self,x,y,x1,x2,y1,y2):
        xc = np.array([(x2+x1)/2,(y2+y1)/2]) 
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(-(y-xc[1])/ (L*np.sqrt((x-xc[0])**2 + (y-xc[1])**2)))


    def phi(self,x,y,x1,x2,y1,y2):
        t=self.T(x,y,x1,x2,y1,y2)
        f=self.F(x,y,x1,x2,y1,y2)
        rho=np.sqrt(t**2 +f**4)
        return(np.sqrt(f**2+((rho-t)/2)**2))

    def delphix(self,x,y,x1,x2,y1,y2):
        ff=self.F(x,y,x1,x2,y1,y2)
        dfx=self.delfx(x,y,x1,x2,y1,y2)
        tf=self.T(x,y,x1,x2,y1,y2)
        dtx=self.deltx(x,y,x1,x2,y1,y2)
        term1 = 0.5*((2 * ff**3 * dfx + tf*dtx)/(np.sqrt(ff**4 + tf**2)) - dtx)*((np.sqrt(ff**4 + tf**2)-tf)/2)
        term2 = ff*dfx
        term3 =  np.sqrt((((np.sqrt(ff**4 + tf**2)-tf)/2)**2) + ff**2)  
        return((term1+term2)/term3)

    def delphiy(self,x,y,x1,x2,y1,y2):
        ff=self.F(x,y,x1,x2,y1,y2)
        dfy=self.delfy(x,y,x1,x2,y1,y2)
        tf=self.T(x,y,x1,x2,y1,y2)
        dty=self.delty(x,y,x1,x2,y1,y2)
        term1 = 0.5*((2 * ff**3 * dfy + tf*dty)/(np.sqrt(ff**4 + tf**2)) - dty)*((np.sqrt(ff**4 + tf**2)-tf)/2)
        term2 = ff*dfy
        term3 =  np.sqrt((((np.sqrt(ff**4 + tf**2)-tf)/2)**2) + ff**2)    
        return((term1+term2)/term3)  

    def PHI(self,x,y,segments):
        #m=4
        R=0
        for i in range(len(segments[:,0])):
            R = R + 1/self.phi(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3]) ** self.m
        R = 1/R**(1/self.m)
        return(R)

    def PHIDX(self,x,y,segments):
        term1=0
        term2=0
        term3=0
        for i in range(len(segments[:,0])):
            term1=-self.m*(self.phi(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**-self.m) *self.delphix(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3]) + term1
            term2=(self.phi(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**-self.m)**(-1/self.m) + term2
            term3=self.m*(self.phi(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**-self.m) + term3      
        R=(-term1*term2/term3)
        return(R)
     
        
    def PHIDY(self,x,y,segments):
        term1=0
        term2=0
        term3=0
        for i in range(len(segments[:,0])):
            term1=-self.m*(self.phi(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**-self.m) * self.delphiy(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3]) + term1
            term2=(self.phi(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**-self.m)**(-1/self.m) + term2
            term3=self.m*(self.phi(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**-self.m) + term3      
        R=(-term1*term2/term3)
        return(R)
    
    

    def Trim(self,f,t):
        """ Trim function for two functions  """
        phi=np.sqrt(t**2 + f**4)
        return(np.sqrt(f**2 + ((phi-t)/2)**2))
    
    def Trimx(self,f,t,fx,tx):
        """Derivative Trim function for two functions wrt x  """
        term1 = (2*(f**3)*fx + tx*t)/(np.sqrt(f**4 + t**2)) - tx
        term2 = np.sqrt(f**4 + t**2)/2 - t/2
        term3 = f*fx
        term4 = np.sqrt((np.sqrt(f**4 + t**2)/2 - t/2)**2 + f**2)
        
        return((0.5*term1*term2 +term3)/term4)
        
    def Trimy(self,f,t,fy,ty):
        """Derivative Trim function for two functions wrt y """
        term1 = (2*(f**3)*fy + ty*t)/(np.sqrt(f**4 + t**2)) - ty
        term2 = np.sqrt(f**4 + t**2)/2 - t/2
        term3 = f*fy
        term4 = np.sqrt((np.sqrt(f**4 + t**2)/2 - t/2)**2 + f**2)
        
        return((0.5*term1*term2 +term3)/term4)    
      

    def phi_line_(self,x,y,x1,y1,x2,y2):
        """ Distance function of a line """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(((x-x1)*(y2-y1)-(y-y1)*(x2-x1))/L)


    def dphix_line_(self,x,y,x1,y1,x2,y2):
        """ Derivative distance function of a line wrt x """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return((-y1+y2)/L)


    def dphiy_line_(self,x,y,x1,y1,x2,y2):
        """ Derivative distance function of a line wrt y """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return((x1-x2)/L)


    def trim(self,x,y,x1,y1,x2,y2):
        """ Trim function """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        xc = np.array([(x2+x1)/2,(y2+y1)/2])
        t = (1/L)*((L/2)**2 - ((x-xc[0])**2 + (y-xc[1])**2))    
        return(t)


    def dtrimx(self,x,y,x1,y1,x2,y2):
        """ Derivative of Trim function wrt x """
        xc = np.array([(x2+x1)/2,(y2+y1)/2]) 
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(-(x-xc[0])/ (L*np.sqrt((x-xc[0])**2 + (y-xc[1])**2)))

    def dtrimy(self,x,y,x1,y1,x2,y2):
        """ Derivative of Trim function wrt y """
        xc = np.array([(x2+x1)/2,(y2+y1)/2]) 
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(-(y-xc[1])/ (L*np.sqrt((x-xc[0])**2 + (y-xc[1])**2)))

    def phi_line(self,x,y,x1,y1,x2,y2):
        """ Trimmed line segment"""
        t = self.trim(x,y,x1,y1,x2,y2)
        f = self.phi_line_(x,y,x1,y1,x2,y2)
        rho = np.sqrt(t**2 + f**4)
        return(np.sqrt(f**2 + ((rho-t)/2)**2))

    def dphix_line(self,x,y,x1,x2,y1,y2):
        """ Derivative of trimmed line segment wrt x"""
        ff = self.phi_line_(x,y,x1,x2,y1,y2)
        dfx = self.dphix_line_(x,y,x1,x2,y1,y2)
        tf = self.trim(x,y,x1,x2,y1,y2)
        dtx = self.dtrimx(x,y,x1,x2,y1,y2)
        term1 = 0.5*((2 * ff**3 * dfx + tf*dtx)/(np.sqrt(ff**4 + tf**2)) - dtx)*((np.sqrt(ff**4 + tf**2)-tf)/2)
        term2 = ff*dfx
        term3 = np.sqrt((((np.sqrt(ff**4 + tf**2)-tf)/2)**2) + ff**2)  
        return((term1+term2)/term3)

    def dphiy_line(self,x,y,x1,x2,y1,y2):
        """ Derivative of trimmed line segment wrt y"""
        ff = self.phi_line_(x,y,x1,x2,y1,y2)
        dfy = self.dphiy_line_(x,y,x1,x2,y1,y2)
        tf = self.trim(x,y,x1,x2,y1,y2)
        dty = self.dtrimy(x,y,x1,x2,y1,y2)
        term1 = 0.5*((2 * ff**3 * dfy + tf*dty)/(np.sqrt(ff**4 + tf**2)) - dty)*((np.sqrt(ff**4 + tf**2)-tf)/2)
        term2 = ff*dfy
        term3 = np.sqrt((((np.sqrt(ff**4 + tf**2)-tf)/2)**2) + ff**2)    
        return((term1+term2)/term3)   
    
    def circle(self,x,y,R,a,b):
        return(abs((R**2 - (x-a)**2 - (y-b)**2)))

    def dphix_circle(self,x,y,a,b,R):
        """ Derivative of the distance function of cirlce wrt x """
        return(R*(x-a)*np.sign((R/2)*(- R**2 +(a-x)**2 + (b-y)**2)))
    
    
    def dphiy_circle(self,x,y,a,b,R):
        """ Derivative of the distance function of cirlce wrt y """
        return(R*(y-b)*np.sign((R/2)*(- R**2 +(a-x)**2 + (b-y)**2)))    
    
    
    def phi_segments(self,x,y,segments):
        """ R equivelent of trimmed line segments"""
        R=0
        for i in range(len(segments[:,0])):
            R = R + 1/self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**self.m
        R = 1/R**(1/self.m)
        return(R)
    
    def dphix_segments(self,x,y,segments):
        """ Derivative of R equivelent of trimmed line segments wrt x"""

        term1=0
        term2=0
        term3=0
        for i in range(len(segments[:,0])):
            term1=-self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**-self.m-1) * self.dphix_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3]) + term1
            term2=(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**-self.m) + term2
            term3=self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**-self.m) + term3      
        R=(-term1*(term2**(-1/self.m))/term3)
        return(R)

    def dphiy_segments(self,x,y,segments):
        """ Derivative of R equivelent of trimmed line segments wrt y"""

        term1=0
        term2=0
        term3=0
        for i in range(len(segments[:,0])):
            term1=-self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**(-self.m-1)) * self.dphiy_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3]) + term1
            term2=(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**(-self.m)) + term2
            term3=self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**(-self.m)) + term3      
        R=(-term1*(term2**(-1/self.m))/term3)
        return(R)    

    def F_random_objdx(self,x,y):

        R=1.25
        theta1=0.41
        theta2=2*np.pi-theta1
        theta=np.linspace(theta1,theta2,100)

        theta3=0.41
        theta4=2*np.pi-theta1

        x1_=-R*np.cos(theta)
        y1_=R*np.sin(theta)

        x1a_=R*np.cos(theta3)
        y1a_=R*np.sin(theta3)

        x1b_=R*np.cos(theta4)
        y1b_=R*np.sin(theta4)

        a=.504
        b=1.65
        c=2.02

        w1=.5
        w2=.63


        x2_=[x1_[0],x1_[0]-a]
        y2_=[w1,w1]

        x3_=[-b,-b]
        y3_=[w1,w2]

        x4_=[-b,-c]
        y4_=[w2,w2]

        x5_=[-c,-c]
        y5_=[w2,-w2]

        x6_=[-c,-b]
        y6_=[-w2,-w2]

        x7_=[-b,-b]
        y7_=[-w1,-w2]

        x8_=[-b,x1_[-1]]
        y8_=[-w1,-w1]

        x_=[]
        y_=[]
        
        for i in range(len(x1_)):
            x_.append(x1_[i])
            
        x_.append(x2_[0])
        x_.append(x2_[1])
        x_.append(x3_[1])
        x_.append(x4_[1])
        x_.append(x5_[1])
        x_.append(x6_[1])
        x_.append(x7_[0])
        x_.append(x8_[1])
        
        for i in range(len(y1_)):
            y_.append(y1_[i])
            
        y_.append(y2_[0])
        y_.append(y2_[1])
        y_.append(y3_[1])
        y_.append(y4_[1])
        y_.append(y5_[1])
        y_.append(y6_[1])
        y_.append(y7_[0])
        y_.append(y8_[1])     
        (self.segments)=self.create_segment(x_,y_)
        #f1=self.circle(x,y,R,0,0)
        #f1x=self.dphix_circle(x,y,0,0,R)
        #2 = self.phi_segments(x,y,self.segments)
        Fx = self.dphix_segments(x,y,self.segments)   
        #Fx = (f1**self.m + f2**self.m)**(-1/self.m)*f1*f2x + (f1**self.m + f2**self.m)**(-1/self.m)*f2*f1x - ((self.m*(f2**self.m)*f2x)/f2 + (self.m*(f1**self.m)*f1x)/f1)*((f1**self.m) +f2**self.m)**(-1/self.m)*f1*f2/(self.m*(f1**self.m + f2**self.m))
            
        return(Fx)
    
    def F_random_objdy(self,x,y):
        R=1.25
        theta1=0.41
        theta2=2*np.pi-theta1
        theta=np.linspace(theta1,theta2,100)

        theta3=0.41
        theta4=2*np.pi-theta1

        x1_=-R*np.cos(theta)
        y1_=R*np.sin(theta)

        x1a_=R*np.cos(theta3)
        y1a_=R*np.sin(theta3)

        x1b_=R*np.cos(theta4)
        y1b_=R*np.sin(theta4)

        a=.504
        b=1.65
        c=2.02

        w1=.5
        w2=.63


        x2_=[x1_[0],x1_[0]-a]
        y2_=[w1,w1]

        x3_=[-b,-b]
        y3_=[w1,w2]

        x4_=[-b,-c]
        y4_=[w2,w2]

        x5_=[-c,-c]
        y5_=[w2,-w2]

        x6_=[-c,-b]
        y6_=[-w2,-w2]

        x7_=[-b,-b]
        y7_=[-w1,-w2]

        x8_=[-b,x1_[-1]]
        y8_=[-w1,-w1]
        
        x_=[]
        y_=[]
        
        for i in range(len(x1_)):
            x_.append(x1_[i])
            
        x_.append(x2_[0])
        x_.append(x2_[1])
        x_.append(x3_[1])
        x_.append(x4_[1])
        x_.append(x5_[1])
        x_.append(x6_[1])
        x_.append(x7_[0])
        x_.append(x8_[1])
        
        for i in range(len(y1_)):
            y_.append(y1_[i])
            
        y_.append(y2_[0])
        y_.append(y2_[1])
        y_.append(y3_[1])
        y_.append(y4_[1])
        y_.append(y5_[1])
        y_.append(y6_[1])
        y_.append(y7_[0])
        y_.append(y8_[1])     
        (self.segments)=self.create_segment(x_,y_)
        Fy = self.dphiy_segments(x,y,self.segments)   
        return(Fy)
    
    
    def F_random_obj(self,x,y):
        R=1.25
        theta1=0.41
        theta2=2*np.pi-theta1
        theta=np.linspace(theta1,theta2,100)

        theta3=0.41
        theta4=2*np.pi-theta1

        x1_=-R*np.cos(theta)
        y1_=R*np.sin(theta)

        x1a_=R*np.cos(theta3)
        y1a_=R*np.sin(theta3)

        x1b_=R*np.cos(theta4)
        y1b_=R*np.sin(theta4)

        a=.504
        b=1.65
        c=2.02

        w1=.5
        w2=.63


        x2_=[x1_[0],x1_[0]-a]
        y2_=[w1,w1]

        x3_=[-b,-b]
        y3_=[w1,w2]

        x4_=[-b,-c]
        y4_=[w2,w2]

        x5_=[-c,-c]
        y5_=[w2,-w2]

        x6_=[-c,-b]
        y6_=[-w2,-w2]

        x7_=[-b,-b]
        y7_=[-w1,-w2]

        x8_=[-b,x1_[-1]]
        y8_=[-w1,-w1]

        x_=[]
        y_=[]
        
        for i in range(len(x1_)):
            x_.append(x1_[i])
            
        x_.append(x2_[0])
        x_.append(x2_[1])
        x_.append(x3_[1])
        x_.append(x4_[1])
        x_.append(x5_[1])
        x_.append(x6_[1])
        x_.append(x7_[0])
        x_.append(x8_[1])
        
        for i in range(len(y1_)):
            y_.append(y1_[i])
            
        y_.append(y2_[0])
        y_.append(y2_[1])
        y_.append(y3_[1])
        y_.append(y4_[1])
        y_.append(y5_[1])
        y_.append(y6_[1])
        y_.append(y7_[0])
        y_.append(y8_[1])   
        
        (self.segments)=self.create_segment(x_,y_)
        phi1_=self.phi_segments(x,y,self.segments)
        return(phi1_)    
    

    def create_segment(self,x,y):
        seglen=len(x)
        segments=np.zeros((seglen-1,4))
        for i in range(seglen-1):
            #[x1,y1,x2,y2]
            #[x2,y2,x3,y3]
            segments[i,0]=x[i]
            segments[i,1]=y[i]
            segments[i,2]=x[i+1]
            segments[i,3]=y[i+1]
        return(segments)                    
        
    def shoelace(self,vertices):
        (m,n)=np.shape(vertices)

        sum1=vertices[0,0]*(vertices[1,1]-vertices[1,n-1])
        for i in range(1,n-1):
            sum1=sum1 + vertices[0,i]*(vertices[1,i+1]-vertices[1,i-1])
        i=n-1
        sum1=sum1 + vertices[0,i]*(vertices[1,0]-vertices[1,i-1])

        A=.5*abs(sum1)
        return (A) 
    
    
    
class controller():
    """ Class for storung all the controller information"""
    def __init__(self,name,my_system,bots,Psi,Ball,path):
        self.name=name # name of simulation
        self.my_system = my_system # the system object
        self.robots = bots # robot objets
        self.Psi=Psi # potential fields 
        self.Ball=Ball # ball object
        if self.Ball == None: # if there is no ball there is no force
            self.forceb=None
            
        ##### Extract variables from other imported objects #####
        self.forces=self.robots.force 
        self.nb=self.robots.nb 
        
        
        #### random variables used
        self.px=0
        self.pz=0
        self.t=0
        self.trig1=1
        self.trig2=1
        self.alpha_=0
        self.a=0.0001
        self.b=0.0001
        
        self.epsilon_tilda=[] # smoothed out epsilon array
        self.time_tilda=[] # corresponding time
        self.check_field=0 # checkfield 
        
        #### Imported Variables #########
        self.mainDirectory = path   # main directory 
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters=parameters.tolist()                 
        self.control_mode=self.parameters['control_mode']  # control mode 
        self.dt = self.parameters['dt']
        self.epoch=0
        self.FX=np.zeros(self.nb)
        self.FZ=np.zeros(self.nb)
        self.save_rate=self.parameters['save_rate']
        
        self.T = 0
        self.tn = 0
        self.w = 5       
        
        if self.control_mode=="target_chasing":
            self.xc = self.parameters["xc"]
            self.zc = self.parameters["yc"]
            self.alpha = self.parameters['alpha1']
            self.beta = self.parameters['beta']     
            self.pwm = self.parameters["pwm"]
            
        if self.control_mode=="grasping" or self.control_mode=="grasping_epsilon":
            self.alpha1 = self.parameters['alpha1']
            self.alpha2 = self.parameters['alpha2']
            self.beta = self.parameters['beta']
            self.tcut1 = self.parameters['tcut1']
            self.tcut2 = self.parameters['tcut2']
            
            self.xc1 = self.parameters["xc1"]
            self.zc1 = self.parameters["yc1"]
            
            self.xc2 = self.parameters["xc2"]
            self.zc2 = self.parameters["yc2"]    
            self.a1 = self.parameters['a1']
            self.b1 = self.parameters['b1']
            self.a2 = self.parameters['a2']
            self.b2 = self.parameters['b2']  

            self.ball_radius = self.parameters["ball_radius"]
            self.fb = 0
            self.ball_geometry=self.parameters['ball_geometry']
            if self.control_mode=="grasping_epsilon":
                self.nojam = self.parameters['nojam']
            
            if self.ball_geometry=="c_shape":
                self.w=self.parameters['w']
                self.l=self.parameters['l']
                self.t=self.parameters['t']
                w=self.w/2
                l=self.l/2
                t=self.t
                x__=[w,-w,-w,w,w,-w+t,-w+t,w,w]
                y__=[-l,-l,l,l,l-t,l-t,-l+t,-l+t,-l]
                (self.segments)=self.Psi.create_segment(x__,y__)
                
                
        elif self.control_mode=="grasping_explore":
            self.alpha1 = self.parameters['alpha1']
            self.alpha2 = self.parameters['alpha2']
            self.beta = self.parameters['beta']
            self.tcut1 = self.parameters['tcut1']
            self.tcut2 = self.parameters['tcut2']
            self.tcut3 = self.parameters['tcut3']
            self.ballx=self.parameters['ballx']
            self.ballz=self.parameters['ballz']
            self.xc1 = self.parameters["xc1"]
            self.zc1 = self.parameters["yc1"]
            
            self.xc2 = self.parameters["xc2"]
            self.zc2 = self.parameters["yc2"]    
            self.increment = self.parameters["increment"]
            self.ball_radius = self.parameters["ball_radius"]
            
            self.fb = 0 
            self.Rr1 = self.parameters["Rr1"]
            self.Rr2 = self.parameters["Rr2"]
            self.Rr=0
            self.theta__ = self.parameters["theta"]
            
        elif self.control_mode=="grasping_u":  
            self.trig1_=0
            self.check1=0
            self.TRIG1_=[]
            self.alpha1 = self.parameters['alpha1']
            self.alpha2 = self.parameters['alpha2']
            self.beta = self.parameters['beta']
            self.tcut1 = self.parameters['tcut1']
            self.tcut2 = self.parameters['tcut2']
            self.tcut3 = self.parameters['tcut3']
            self.ballx=self.parameters['ballx']
            self.ballz=self.parameters['ballz']
            self.p = self.parameters['p']
            self.width_grasp = self.parameters['width_grasp']
            self.length_grasp = self.parameters['length_grasp']
            self.xcenter_grasp = self.parameters['xcenter_grasp']
            self.ycenter_grasp = self.parameters['ycenter_grasp']
            self.xc1 = self.parameters['xc1']
            self.yc1 = self.parameters['yc1']
            self.xc2 = self.parameters['xc2']
            self.yc2 = self.parameters['yc2']
            self.fb = 0
            self.ball_radius = self.parameters['ball_radius']
            self.update_rate = self.parameters['update_rate']
            self.error = self.parameters['error']
        else:    
            self.alpha1 = self.parameters['alpha1']
            self.alpha2 = self.parameters['alpha2']
            self.beta = self.parameters['beta']
                    

            
        self.bot_position_x=0
        self.bot_position_z=0
        
        self.bot_velocitiy_x=0
        self.bot_velocitiy_z=0
        
        self.Field_value={}
        self.F_controller_x={}
        self.F_controller_z={}
        for i in range(self.nb):
            self.Field_value["bot{0}".format(i)]=[]           
            self.F_controller_x["bot{0}_control_force_x".format(i)]=[]
            self.F_controller_z["bot{0}_control_force_z".format(i)]=[]
        self.fxt=[]
        self.fyt=[]
        self.fzt=[]
        
        
    def smooth_epsilon(self,epsilon,time):
        """ Smooths the noisy epsilon metric """
        if len(epsilon)<10:
            self.epsilon_tilda.append(0)
            self.time_tilda.append(time)
        else:
            temp=self.moving_average(epsilon, n=10)
            self.epsilon_tilda.append(temp[-1])
            self.time_tilda.append(time)
   
    def run_controller(self):
        """Function to run the controllers"""
        self.epoch = self.epoch + 1
        if self.control_mode =="shape_formation":
                (self.bot_position_x,self.bot_position_z) = self.get_position()     # get position of bots
                (self.bot_velocitiy_x,self.bot_velocitiy_z) = self.get_velocity()   # get velocity of bots
                (self.FX,self.FZ) = self.shape_controller() # run shape controller
                self.apply_force2(self.FX,self.FZ)         # apply force
                
        if self.control_mode =="shape_morphing":
                (self.bot_position_x,self.bot_position_z) = self.get_position()     # get position of bots
                (self.bot_velocitiy_x,self.bot_velocitiy_z) = self.get_velocity()   # get velocity of bots
                (self.FX,self.FZ) = self.morph_controller() # run shape controller
                self.apply_force(self.FX,self.FZ)         # apply force            
   
        if self.control_mode =="target_chasing":
                (self.bot_position_x,self.bot_position_z) = self.get_position()     # get position of bots
                (self.bot_velocitiy_x,self.bot_velocitiy_z) = self.get_velocity()   # get velocity of bots
                (self.FX,self.FZ) = self.target_controller() # run shape controller
                self.apply_force3(self.FX,self.FZ)         # apply force 
   
    
        if self.control_mode =="grasping":
                (self.bot_position_x,self.bot_position_z) = self.get_position()     # get position of bots
                (self.bot_velocitiy_x,self.bot_velocitiy_z) = self.get_velocity()   # get velocity of bots
                (self.FX,self.FZ) = self.grasping_controller() # run shape controller
                self.apply_force(self.FX,self.FZ)         # apply force    
         
        if self.control_mode =="grasping_explore":
                (self.bot_position_x,self.bot_position_z) = self.get_position()     # get position of bots
                (self.bot_velocitiy_x,self.bot_velocitiy_z) = self.get_velocity()   # get velocity of bots
                (self.FX,self.FZ) = self.grasping_controller_explore()
                self.apply_force(self.FX,self.FZ)         # apply force 
                
        if self.control_mode =="grasping_u":
                (self.bot_position_x,self.bot_position_z) = self.get_position()     # get position of bots
                (self.bot_velocitiy_x,self.bot_velocitiy_z) = self.get_velocity()   # get velocity of bots
                if self.epoch%int(self.update_rate)==0:
                    (self.FX,self.FZ) = self.grasping_controller_u3()
                    self.apply_force(self.FX,self.FZ)  
                else:
                    self.FX=self.FX
                    self.FZ=self.FZ
                    self.apply_force(self.FX,self.FZ)         # apply force 
                    
        if self.control_mode =="grasping_epsilon":
                (self.bot_position_x,self.bot_position_z) = self.get_position()     # get position of bots
                (self.bot_velocitiy_x,self.bot_velocitiy_z) = self.get_velocity()   # get velocity of bots
                #self.smooth_epsilon(self,epsilon)
                #if self.epoch%100==0:
                    #print("update")
                (self.FX,self.FZ) = self.grasping_controller_epsilon()
                self.apply_force(self.FX,self.FZ)            
                
    def save_field_value(self):
        """ Function that saves the distance function field value"""
        if self.control_mode =="shape_formation":  
            for i in range(self.nb):
               self.Field_value["bot"+str(i)].append(self.Psi.F(self.bot_position_x[i],self.bot_position_z[i]))
        
        if self.control_mode =="shape_morphing":
            t=np.round(self.my_system.GetChTime(),3)
            for i in range(self.nb):
                self.Field_value["bot"+str(i)].append(self.Psi.F_morph(self.bot_position_x[i],self.bot_position_z[i],self.Psi.tanh(t)))    
            
        if self.control_mode=="grasping":
            t=np.round(self.my_system.GetChTime(),3)
            for i in range(self.nb):
                self.Field_value["bot"+str(i)].append(self.Psi.FGRASP(self.bot_position_x[i],self.bot_position_z[i],t))    
        
        if self.control_mode=="grasping_u":
            t=np.round(self.my_system.GetChTime(),3)
            for i in range(self.nb):
                self.Field_value["bot"+str(i)].append(self.Psi.FGraspU(self.bot_position_x[i],self.bot_position_z[i],t,self.trig1_))    
        
        if self.control_mode=="target_chasing":
            t=np.round(self.my_system.GetChTime(),3)
            for i in range(self.nb):
                self.Field_value["bot"+str(i)].append(self.Psi.dpoint_(self.bot_position_x[i],self.bot_position_z[i],self.xc,self.zc))

    def save_controller_forces(self):
        """ Save controller forces """
        for i in range(self.nb):
            self.F_controller_x["bot"+str(i)+"_control_force_x"].append(self.FX[i])
            self.F_controller_z["bot"+str(i)+"_control_force_z"].append(self.FZ[i])
                        
            
    def shape_controller(self):
        """ Shape controller """
        FX=[]
        FZ=[]
        for i in range(self.nb):
            Fx=self.Psi.FX(self.bot_position_x[i],self.bot_position_z[i])
            Fz=self.Psi.FY(self.bot_position_x[i],self.bot_position_z[i])
            mag=np.sqrt(Fx**2+Fz**2)
            if mag==0:
                FXX=0
                FZZ=0
            else:
                FXX=Fx/mag
                FZZ=Fz/mag
            fx=-self.alpha*FXX-self.beta*self.bot_velocitiy_x[i]
            fz=-self.alpha*FZZ-self.beta*self.bot_velocitiy_z[i]
            
            FX.append(fx)
            FZ.append(fz)
        return(np.asarray(FX),np.asarray(FZ))        


    def morph_controller(self):
        """ Morphing Controller for morphing from one shape to another """
        FX=[]
        FZ=[]
        t=np.round(self.my_system.GetChTime(),3)
        ft=self.Psi.tanh(t)
        #print("ft="+str(np.round(ft,2)),end='\r')
        for i in range(self.nb):
            Fx=self.Psi.FX_morph(self.bot_position_x[i],self.bot_position_z[i],ft)
            Fz=self.Psi.FY_morph(self.bot_position_x[i],self.bot_position_z[i],ft)
            
            mag=np.sqrt(Fx**2+Fz**2)
            
            if mag==0:
                FXX=0
                FZZ=0
            else:
                FXX=Fx/mag
                FZZ=Fz/mag
                
            fx=-self.alpha*FXX-self.beta*self.bot_velocitiy_x[i]
            fz=-self.alpha*FZZ-self.beta*self.bot_velocitiy_z[i]
        
            FX.append(fx)
            FZ.append(fz)
        return(np.asarray(FX),np.asarray(FZ))        


    def E_(self,x,xo,y,yo,a,b):
        """ E function used for the grasping u controller"""
        xbar=x-xo
        ybar=y-yo
        E=np.array([[2*xbar/(a**2),-2*ybar/(b**2)],
                   [2*ybar/(b**2),2*xbar/(a**2)]])
        return(E)
    
    
    def Gamma(self,x,y,xo,yo,a,b):
        """ Gamma function used for the grasping u controller"""
        xbar=x-xo
        ybar=y-yo
        return((xbar/a)**2 + (ybar/b)**2)
        
    def D_(self,G):
        """ D function used for the grasping u controller"""
        rho=5
        l1 = 1 - (1/abs(G)**(1/rho))
        l2 = 1 + (1/abs(G)**(1/rho))
        D=np.diag([l1,l2])
        return(D)
    
    
    def M_(self,x,y,xo,yo,a,b):
        """ M function used for the grasping u controller"""
        E=self.E_(x,xo,y,yo,a,b)
        Einv=np.linalg.inv(E)
        G=self.Gamma(x,y,xo,yo,a,b)
        D=self.D_(G)
        M=E@D@Einv
        return(M)


    def Uxy(self,x,y,a):
        """ Stream line function for flowing around a cylinder in x direction """
        phi = a**2 * ((y**2 -x**2) + (x**2 +y**2)**2) / (x**2 + y**2)**2
        return(phi)
    
    def Vxy(self,x,y,a):
        """ Stream line function for flowing around a cylinder in y direction """
        phi = (-2*a**2 * x * y) / (x**2 + y**2)**2
        return(phi)

    def field3(self,x,y,xo,yo,a,b,xs,ys):
        """ DS controller for grasping """
        M=self.M_(x,y,xo,yo,a,b)
        f=np.array([[-(x-xs)],[-(y-ys)]])
        temp=M@f
        temp=temp.flatten()
        zeta1=temp[0]
        zeta2=temp[1]
        return(zeta1,zeta2)    

    def target_controller(self):
        """ target Controller """
        FX=[]
        FZ=[]
        for i in range(self.nb):
            Fx=self.Psi.dxpoint_(self.bot_position_x[i],self.bot_position_z[i],self.xc,self.zc)
            Fz=self.Psi.dypoint_(self.bot_position_x[i],self.bot_position_z[i],self.xc,self.zc)
            mag=np.sqrt(Fx**2 + Fz**2)
            if mag==0:
                FXX=0
                FZZ=0
            else:
                FXX=Fx/mag
                FZZ=Fz/mag
            fx=-self.alpha*FXX-self.beta*self.bot_velocitiy_x[i]
            fz=-self.alpha*FZZ-self.beta*self.bot_velocitiy_z[i]
            FX.append(fx)
            FZ.append(fz)
        return(np.asarray(FX),np.asarray(FZ)) 


    



    def grasping_controller(self):
        """ Grasping Controller """
        FX=[]
        FZ=[]
        time=np.round(self.my_system.GetChTime(),3)

        
        if time>self.Psi.tcut2:
            #print("tcut2")
            self.Ball.balls[0].SetBodyFixed(False)             
            
        if time>self.Psi.tcut3:
            #print("tcut3")
            self.fb=self.fb+self.dt
            self.forceb[0].SetMforce(self.fb)
            self.forceb[0].SetDir(chrono.VECT_X)
            self.Ball.balls[0].SetBodyFixed(False)   
            
   
  
        #if 5<time:
            #print("spring_change")
            #self.change_springk(100)
            
        for i in range(self.nb):
            #Fx1=self.Psi.FXGRASP(self.bot_position_x[i],self.bot_position_z[i],time)
            #Fz1=self.Psi.FYGRASP(self.bot_position_x[i],self.bot_position_z[i],time)
            
            #Fx1 = self.Psi.Fx_point(self.bot_position_x[i]+random.uniform(-.01, 0.01),self.bot_position_z[i]+random.uniform(-.01, 0.01),self.xc2,self.zc2)
            #Fz1 = self.Psi.Fy_point(self.bot_position_x[i]+random.uniform(-.01, 0.01),self.bot_position_z[i]+random.uniform(-.01, 0.01),self.xc2,self.zc2)
            
            Fx1 = self.Psi.Fx_point(self.bot_position_x[i],self.bot_position_z[i],self.xc2,self.zc2)
            Fz1 = self.Psi.Fy_point(self.bot_position_x[i],self.bot_position_z[i],self.xc2,self.zc2)
            
            #fxb = self.Psi.dphix_circle(self.bot_position_x[i],self.bot_position_z[i],0,0,self.ball_radius-.2)
            #fyb = self.Psi.dphiy_circle(self.bot_position_x[i],self.bot_position_z[i],0,0,self.ball_radius-.2)

            
            #Fz1 = fxb-1*fyb
            #Fx1 = -fyb-1*fxb
            mag1=np.sqrt(Fx1**2 + Fz1**2)
            Fx1=Fx1/mag1
            Fz1=Fz1/mag1

            
            if mag1==0:
                FXX=0
                FZZ=0
            else:
                FXX=Fx1
                FZZ=Fz1
 
            
            if self.tcut1>time:
                alpha_=self.alpha1
                #alpha_=self.alpha1-random.uniform(0, 0.05)
                #print("alpha:",alpha_)
            else:
                alpha_=self.alpha2
                #alpha_=self.alpha2-random.uniform(0, 0.05)
                #print("alpha:",alpha_)
            fx=-(alpha_)*FXX#-self.beta*self.bot_velocitiy_x[i]
            fz=-(alpha_)*FZZ#-self.beta*self.bot_velocitiy_z[i]

            FX.append(fx)
            FZ.append(fz)
        return(np.asarray(FX),np.asarray(FZ))

    def grasping_controller_epsilon(self):
        """ Grasping Controller """
        FX=[]
        FZ=[]
        time=np.round(self.my_system.GetChTime(),3)
        #print("check_field=",self.check_field)
        if time>=self.Psi.tcut2-.2 and self.check_field==0:
            #print("initiated")
            self.check_field=1
            for ii in range(self.nb):
                self.Psi.xjam.append(self.bot_position_x[ii])
                self.Psi.yjam.append(self.bot_position_z[ii])    
            ii=0
            self.Psi.xjam.append(self.bot_position_x[ii])
            self.Psi.yjam.append(self.bot_position_z[ii])  
            (self.Psi.segments_jam)=self.Psi.create_segment(self.Psi.xjam,self.Psi.yjam)
            #print("completed")    
        # c=0
        # for i in range(self.nb):
        #     temp = np.sqrt((self.bot_position_x[i]-self.xc2)**2 + (self.bot_position_z[i]-self.zc2)**2)
        #     if temp<2.25:
        #         print(i)
        #         c=c+1
        # print("c=",str(c))
            #print(np.round(,2))
        if time>self.Psi.tcut2:
            #print("tcut2")
            self.Ball.balls[0].SetBodyFixed(False)             
            #self.Ball.balls[1].SetBodyFixed(False)             
            #self.Ball.balls[2].SetBodyFixed(False)             
        if self.tcut1>time:
            alpha_=self.alpha1
            #alpha_=self.alpha1-random.uniform(0, 0.05)
            #print("alpha:",alpha_)
        else:
            alpha_=self.alpha2    
        
        # if time>self.Psi.tcut3:
        #     #self.fb=self.fb+self.dt
        #     #self.fb=2*(signal.square(2 * np.pi * 0.1 * time) + 1)
        #     self.forceb[0].SetMforce(self.fb)
        #     self.forceb[0].SetDir(chrono.VECT_X)
  
        #if 5<time:
            #print("spring_change")
            #self.change_springk(100)
        for i in range(self.nb):
        #for i in entry:
            
            #Fx1 =  self.Psi.FxGraspEpsilon(self.bot_position_x[i],self.bot_position_z[i],time,self.nojam)
            #Fz1 =  self.Psi.FyGraspEpsilon(self.bot_position_x[i],self.bot_position_z[i],time,self.nojam)
            Fx1 = self.Psi.Fx_point(self.bot_position_x[i],self.bot_position_z[i],self.xc2,self.zc2)
            Fz1 = self.Psi.Fy_point(self.bot_position_x[i],self.bot_position_z[i],self.xc2,self.zc2)
            
            mag1=np.sqrt(Fx1**2 + Fz1**2)
            Fx1=Fx1/mag1
            Fz1=Fz1/mag1

            
            #moving_average(self,a, n=3)
            if mag1==0:
                FXX=0
                FZZ=0
            else:
                FXX=Fx1
                FZZ=Fz1
 
            
            fx=-(alpha_)*FXX#-self.beta*self.bot_velocitiy_x[i]
            fz=-(alpha_)*FZZ#-self.beta*self.bot_velocitiy_z[i]

            FX.append(fx)
            FZ.append(fz)

                
        return(np.asarray(FX),np.asarray(FZ)) 

    def grasping_controller_u(self):
        """ Grasping Controller_u """
        FX=[]
        FZ=[]
        time=np.round(self.my_system.GetChTime(),3)

        noise1=1
        noise2=1
        # if time<self.tcut1:
        #     rm1=np.random.choice(np.linspace(0,30,10))
        #     rm2=np.random.choice(np.linspace(0,30,10))
        #     noise1=np.sin(30*time)#+cos(rm2*time)
        #     noise2=1
        #     #noise1=1
        if time>=self.tcut1:
            #print("tcut2")
            self.Ball.balls[0].SetBodyFixed(False)             
            #self.Ball.balls[1].SetBodyFixed(False)             
            #self.Ball.balls[2].SetBodyFixed(False)             
            noise1=1
            noise2=1
        if time>self.tcut3:
            #print("tcut3")
            #self.fb=self.fb+self.dt
            self.forceb[0].SetMforce(self.fb)
            self.forceb[0].SetDir(chrono.VECT_X)
            self.Ball.balls[0].SetBodyFixed(False)   
            
            
        for i in range(self.nb):
            Fx1=self.Psi.FxGraspU(self.bot_position_x[i],self.bot_position_z[i],time)
            Fz1=self.Psi.FyGraspU(self.bot_position_x[i],self.bot_position_z[i],time)

            mag1=np.sqrt(Fx1**2 + Fz1**2)
            Fx1=Fx1/mag1
            Fz1=Fz1/mag1

            
            if mag1==0:
                FXX=0
                FZZ=0
            else:
                FXX=Fx1
                FZZ=Fz1
 
            
            if self.tcut1>time:
                alpha_=self.alpha1
            else:
                alpha_=self.alpha2



            fx=-abs(noise1)*(alpha_)*FXX#-self.beta*self.bot_velocitiy_x[i]
            fz=-abs(noise2)*(alpha_)*FZZ#-self.beta*self.bot_velocitiy_z[i]


            FX.append(fx)
            FZ.append(fz)
        return(np.asarray(FX),np.asarray(FZ)) 

    
    def grasping_controller_u3(self):
        """ Grasping Controller_u """
        FX=[]
        FZ=[]
        time=np.round(self.my_system.GetChTime(),3)
        
            
        if self.trig1_==0:
            alpha_=self.alpha1
        if self.trig1_!=0:
            alpha_=self.alpha2
        phi=0  
        #if self.trig1_==0:
            #print("phase1")
        for i in range(self.nb):
            Fx1=self.Psi.FxGraspU(self.bot_position_x[i],self.bot_position_z[i],time,self.trig1_)
            Fz1=self.Psi.FyGraspU(self.bot_position_x[i],self.bot_position_z[i],time,self.trig1_)    
            mag1=np.sqrt(Fx1**2 + Fz1**2)
            Fx1=Fx1/mag1
            Fz1=Fz1/mag1
            phi=self.Psi.FGraspU(self.bot_position_x[i],self.bot_position_z[i],time,self.trig1_)+phi
            
                
            if mag1==0:
                FXX=0
                FZZ=0
            else:
                FXX=Fx1
                FZZ=Fz1


            fx=-(alpha_)*FXX#-self.beta*self.bot_velocitiy_x[i]
            fz=-(alpha_)*FZZ#-self.beta*self.bot_velocitiy_z[i]
            FX.append(fx)
            FZ.append(fz)
        phibar=np.round(phi/self.nb,3)
        #print('alpha_= '+str(alpha_))
        print('update:  '+'  phi= '+str(np.round(phi,2))+'  '+'phibar= '+str(phibar)+'  '+'alpha_= '+str(alpha_))
        #print('phibar= '+str(np.round(phibar,3)))
        if phibar<=self.error and self.check1==0:
            self.trig1_=1
            self.check1=1
        #if self.check1==1:
            #print("rho="+str(self.Psi.rho_))
        return(np.asarray(FX),np.asarray(FZ)) 
    
   
    def grasping_controller_morph(self):
        """ Grasping Controller """
        FX=[]
        FZ=[]
        time = np.round(self.my_system.GetChTime(),3)

            
        if time<self.tcut1: # check on it every 10 times for efficiency
            self.alpha_=self.alpha1
            self.a=self.a1
            self.b=self.b1
            self.px=self.xc1
            self.pz=self.zc1
            
            
        if time>self.tcut1: # check on it every 10 times for efficiency
            self.alpha_=self.alpha2
            self.a=self.a2
            self.b=self.b2   
            self.px=self.xc2
            self.pz=self.zc2
            
        if time>self.tcut2:
            #print("tcut2")
            self.Ball.balls[0].SetBodyFixed(False)             
            #self.Ball.balls[1].SetBodyFixed(False) 
            #self.Ball.balls[2].SetBodyFixed(False) 
        
        entry=[31,32,33,34,35,36,37,38,39,0,1,2,3,4,5,6,7,8,9,10]
        #for i in range(self.nb):
        for i in entry:
            if time<self.tcut1: 
                fxb = self.Psi.dphix_oval(self.bot_position_x[i],self.bot_position_z[i],self.px,self.pz,self.a,self.b)
                fyb = self.Psi.dphiy_oval(self.bot_position_x[i],self.bot_position_z[i],self.px,self.pz,self.a,self.b)
            
            else:
                fxb = self.Psi.dphix_oval(self.bot_position_x[i],self.bot_position_z[i],self.px,self.pz,self.a,self.b)
                fyb = self.Psi.dphiy_oval(self.bot_position_x[i],self.bot_position_z[i],self.px,self.pz,self.a,self.b)
                #fxb = self.Psi.dphix_segments(self.bot_position_x[i],self.bot_position_z[i],self.segments)
                #fyb = self.Psi.dphiy_segments(self.bot_position_x[i],self.bot_position_z[i],self.segments)
            #fxb = self.Psi.Fx_point(self.bot_position_x[i],self.bot_position_z[i],self.px,self.pz)
            #fyb = self.Psi.Fy_point(self.bot_position_x[i],self.bot_position_z[i],self.px,self.pz)
            
            #fxb = self.Psi.dphix_circle(self.bot_position_x[i],self.bot_position_z[i],0,0,self.ball_radius-.2)
            #fyb = self.Psi.dphiy_circle(self.bot_position_x[i],self.bot_position_z[i],0,0,self.ball_radius-.2)
            
            
            
            Fz1 = fyb
            Fx1 = fxb
            mag1=np.sqrt(Fx1**2 + Fz1**2)
            Fx1=Fx1/mag1
            Fz1=Fz1/mag1

            
            if mag1==0:
                FXX=0
                FZZ=0
            else:
                FXX=Fx1
                FZZ=Fz1
 
            #if self.tcut1>time:
            #alpha_=self.alpha1
            #else:
                #alpha_=self.alpha2
                
            fx=-self.alpha_*FXX#-self.beta*self.bot_velocitiy_x[i]
            fz=-self.alpha_*FZZ#-self.beta*self.bot_velocitiy_z[i]

            FX.append(fx)
            FZ.append(fz)
        return(np.asarray(FX),np.asarray(FZ)) 

    def grasping_controller__(self):
        """ Grasping Controller """
        FX=[]
        FZ=[]
        time=np.round(self.my_system.GetChTime(),3)

        
        if time>self.Psi.tcut2:
            #print("tcut2")
            self.Ball.balls[0].SetBodyFixed(False)             
            
        if time>self.Psi.tcut3:
            #print("tcut3")
            self.fb=self.fb+self.dt
            self.forceb[0].SetMforce(self.fb)
            self.forceb[0].SetDir(chrono.VECT_X)
            self.Ball.balls[0].SetBodyFixed(False)   
            
            # self.Ball.Fb["Fb"].append(self.fb)
            # self.Ball.TIME["TIME"].append(time)
            # self.Ball.PX["PX"].append(self.Ball.balls[0].GetPos().x)
            # self.Ball.PY["PY"].append(self.Ball.balls[0].GetPos().z)     
  
        #if 5<time:
            #print("spring_change")
            #self.change_springk(100)
            
        for i in range(self.nb):
            #Fx1=self.Psi.FXGRASP(self.bot_position_x[i],self.bot_position_z[i],time)
            #Fz1=self.Psi.FYGRASP(self.bot_position_x[i],self.bot_position_z[i],time)
            
            #Fx1 = self.Psi.Fx_point(self.bot_position_x[i]+random.uniform(-.01, 0.01),self.bot_position_z[i]+random.uniform(-.01, 0.01),self.xc2,self.zc2)
            #Fz1 = self.Psi.Fy_point(self.bot_position_x[i]+random.uniform(-.01, 0.01),self.bot_position_z[i]+random.uniform(-.01, 0.01),self.xc2,self.zc2)
            
            Fx1 = self.Psi.Fx_point(self.bot_position_x[i],self.bot_position_z[i],self.xc2,self.zc2)
            Fz1 = self.Psi.Fy_point(self.bot_position_x[i],self.bot_position_z[i],self.xc2,self.zc2)
            
            #fxb = self.Psi.dphix_circle(self.bot_position_x[i],self.bot_position_z[i],0,0,self.ball_radius-.2)
            #fyb = self.Psi.dphiy_circle(self.bot_position_x[i],self.bot_position_z[i],0,0,self.ball_radius-.2)

            
            #Fz1 = fxb-1*fyb
            #Fx1 = -fyb-1*fxb
            mag1=np.sqrt(Fx1**2 + Fz1**2)
            Fx1=Fx1/mag1
            Fz1=Fz1/mag1

            
            if mag1==0:
                FXX=0
                FZZ=0
            else:
                FXX=Fx1
                FZZ=Fz1
 
            
            if self.tcut1>time:
                alpha_=self.alpha1
                #alpha_=self.alpha1-random.uniform(0, 0.05)
                #print("alpha:",alpha_)
            else:
                alpha_=self.alpha2
                #alpha_=self.alpha2-random.uniform(0, 0.05)
                #print("alpha:",alpha_)
            fx=-(alpha_)*FXX#-self.beta*self.bot_velocitiy_x[i]
            fz=-(alpha_)*FZZ#-self.beta*self.bot_velocitiy_z[i]

            FX.append(fx)
            FZ.append(fz)
        return(np.asarray(FX),np.asarray(FZ)) 

    def grasping_controller_explore(self):
        """ Grasping Controller explore"""
        FX=[]
        FZ=[]
        time = np.round(self.my_system.GetChTime(),3) 
        noise=random.uniform(-.01, 0.01)
        noise=0
        if self.t<self.tcut1:
            print("approach")
        
        elif self.t>=self.tcut1 and self.t<=self.tcut2:
            print("back up")
            
        # move to next position 
        else:
            print("move")

            
        if self.t<self.tcut1 and self.trig1!=0: # check on it every 10 times for efficiency

            self.theta__ = self.theta__
            #self.Rr = 0
            self.Rr = self.Rr1
            self.trig1 = 0
            self.alpha_=self.alpha1
            self.a=.001
            self.b=.001
        
        
        
          # back off of object    
        elif self.t>=self.tcut1 and self.t<=self.tcut2:
            #print("back up")
            self.theta__ = self.theta__
            #self.Rr = 2
            self.Rr = self.Rr2
            self.alpha_=self.alpha2
            self.a=self.parameters['R']#+.05
            self.b=self.parameters['R']#+.05
            
        # move to next position 
        elif self.t>self.tcut2 and self.t<self.tcut3 and self.trig2!=0:
            #print("move")
            self.trig2 = 0
            self.theta__ = self.theta__ + self.increment 
            #self.Rr = 2
            self.Rr = self.Rr2
            self.alpha_=self.alpha2
            self.a=self.parameters['R']#+.05
            self.b=self.parameters['R']#+.05
            
        elif self.t>=self.tcut3:
            self.t = 0
            self.trig1 = 1
            self.trig2 = 1
            print("reset")
        

        self.px = self.Rr*np.cos(self.theta__) #+ self.ballx
        self.pz = self.Rr*np.sin(self.theta__) #+ self.ballz
        
        for i in range(self.nb):
            fxb = self.Psi.dphix_oval(self.bot_position_x[i]+noise,self.bot_position_z[i]+noise,self.px,self.pz,self.a,self.b)
            fyb = self.Psi.dphiy_oval(self.bot_position_x[i]+noise,self.bot_position_z[i]+noise,self.px,self.pz,self.a,self.b)
            #fxb = self.Psi.Fx_point(self.bot_position_x[i],self.bot_position_z[i],self.px,self.pz)
            #fyb = self.Psi.Fy_point(self.bot_position_x[i],self.bot_position_z[i],self.px,self.pz)
            
            #fxb = self.Psi.dphix_circle(self.bot_position_x[i],self.bot_position_z[i],0,0,self.ball_radius-.2)
            #fyb = self.Psi.dphiy_circle(self.bot_position_x[i],self.bot_position_z[i],0,0,self.ball_radius-.2)
            
            
            
            Fz1 = fyb
            Fx1 = fxb
            mag1=np.sqrt(Fx1**2 + Fz1**2)
            Fx1=Fx1/mag1
            Fz1=Fz1/mag1

            
            if mag1==0:
                FXX=0
                FZZ=0
            else:
                FXX=Fx1
                FZZ=Fz1
 
            #if self.tcut1>time:
            #alpha_=self.alpha1
            #else:
                #alpha_=self.alpha2
                
            fx=-self.alpha_*FXX#-self.beta*self.bot_velocitiy_x[i]
            fz=-self.alpha_*FZZ#-self.beta*self.bot_velocitiy_z[i]

            FX.append(fx)
            FZ.append(fz)
        return(np.asarray(FX),np.asarray(FZ)) 
    
    def change_springk(self,k):
        """ Function to change the spring stiffness"""
        for i in range(len(self.robots.Springs)):
            self.robots.Springs[i].Set_SpringK(k)
            
    def get_position(self):
        """ get position of boundary robots """
        xb=[]        
        zb=[]
        for i in range(self.nb):
            xb.append(self.robots.bots[i].GetPos().x)
            zb.append(self.robots.bots[i].GetPos().z)
        return(xb,zb)
      
    def get_velocity(self):
        """ get velocity of boundary robots """
        xbv=[]
        zbv=[]
        for i in range(self.nb):
            xbv.append(self.robots.bots[i].GetPos_dt().x)
            zbv.append(self.robots.bots[i].GetPos_dt().z)
        return(xbv,zbv)

    def moving_average(self,a, n=3):
        ret = np.cumsum(a, dtype=float)
        ret[n:] = ret[n:] - ret[:-n]
        return ret[n - 1:] / n

    def apply_force(self,FX,FZ):  
        """ Appy forces to robots """
        entry=[0,2,5,8,11,14,17,20,23,26]
        #for i in range(self.nb):
        for i in entry:
            self.fxt.append(FX[i])
            self.fyt.append(0)
            self.fzt.append(self.FZ[i])
            self.forces[3*i].SetMforce(float(FX[i]))
            self.forces[3*i].SetDir(chrono.VECT_X)
            self.forces[3*i+2].SetMforce(float(FZ[i]))
            self.forces[3*i+2].SetDir(chrono.VECT_Z)
            
            
    def apply_force3(self,FX,FZ):  
        """ Appy forces to robots """
        self.tn=(self.pwm/255)/self.w 
        self.T=self.dt+self.T
        if self.T>0 and self.T<=self.tn:
            self.FX=FX
            self.FZ=FZ
        else:
            self.FX=np.zeros(len(FX))
            self.FZ=np.zeros(len(FX))
            
        if self.T>(1/self.w):
            self.T=0
            print('reset',np.round(self.my_system.GetChTime(),2))
        for i in range(self.nb):
            self.fxt.append(FX[i])
            self.fyt.append(0)
            self.fzt.append(self.FZ[i])
            self.forces[3*i].SetMforce(float(FX[i]))
            self.forces[3*i].SetDir(chrono.VECT_X)
            self.forces[3*i+2].SetMforce(float(FZ[i]))
            self.forces[3*i+2].SetDir(chrono.VECT_Z)
 
    def apply_force2(self,FX,FZ):  
        """ Appy forces to robots """
        t=np.round(self.my_system.GetChTime(),3)
        
        if t<10:
            
            for i in range(self.nb):
                self.fxt.append(FX[i])
                self.fyt.append(0)
                self.fzt.append(self.FZ[i])
                self.forces[3*i].SetMforce(float(FX[i]))
                self.forces[3*i].SetDir(chrono.VECT_X)
                self.forces[3*i+2].SetMforce(float(FZ[i]))
                self.forces[3*i+2].SetDir(chrono.VECT_Z)
        if t>=10 and t<15:
            for i in range(self.nb):
                self.fxt.append(FX[i])
                self.fyt.append(0)
                self.fzt.append(self.FZ[i])
                rm1=np.random.choice(np.linspace(0,30,10))
                rm2=np.random.choice(np.linspace(0,30,10))
                rm3=np.random.choice(np.linspace(0,30,10))
                rm4=np.random.choice(np.linspace(0,30,10))
                self.forces[3*i].SetMforce(float(FX[i])+3*self.alpha*np.sin(rm1*t)+3*self.alpha*np.sin(rm2*t))
                self.forces[3*i].SetDir(chrono.VECT_X)
                self.forces[3*i+2].SetMforce(float(FZ[i])+3*self.alpha*np.cos(rm3*t)+3*self.alpha*np.cos(rm4*t))
                self.forces[3*i+2].SetDir(chrono.VECT_Z)                
                
                
                # rm1=np.random.choice(np.linspace(-10,10,10))
                # rm2=np.random.choice(np.linspace(-10,10,10))
                
            
                # self.forces[3*i].SetMforce(float(FX[i])+rm1)
                # self.forces[3*i].SetDir(chrono.VECT_X)
                # self.forces[3*i+2].SetMforce(float(FZ[i])+rm2)
                # self.forces[3*i+2].SetDir(chrono.VECT_Z)                
                
                
        if t>=15:
            for i in range(self.nb):
                self.fxt.append(FX[i])
                self.fyt.append(0)
                self.fzt.append(self.FZ[i])
                self.forces[3*i].SetMforce(float(FX[i]))
                self.forces[3*i].SetDir(chrono.VECT_X)
                self.forces[3*i+2].SetMforce(float(FZ[i]))
                self.forces[3*i+2].SetDir(chrono.VECT_Z)      
                
    def clear_temp_forces(self):
        """ Clear Temp forces """
        self.fxt=[]
        self.fyt=[]
        self.fzt=[]
  



class export_data():
    def __init__(self,my_system,robots,controls,interior,ball,simulation,Psi,my_rep,path,name):
        self.name = name # name of simulation
        self.mainDirectory = path   # main directory 
        
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters = parameters.tolist()
        self.my_system = my_system
        self.robots = robots
        self.controls = controls
        self.interior = interior
        self.simulation = simulation
        self.ball = ball
        self.Psi=Psi
        self.results_dir=self.mainDirectory+self.name+'/results'
        self.control_mode=self.parameters['control_mode']
        if not os.path.exists(self.results_dir):
            os.mkdir(self.results_dir)
        
        #### Pull data to export
        
        # time
        self.time = {'time':self.simulation.time} 
        
        # pull robot data
        (self.bot_xposition,self.bot_yposition,self.bot_zposition) = self.robots.return_position_data()
        (self.skin_xposition,self.skin_yposition,self.skin_zposition) = self.robots.return_position_membrane_data()
        (self.bot_xForcetotal,self.bot_yForcetotal,self.bot_zForcetotal) = self.robots.return_total_force()
        (self.bot_xvelocity,self.bot_yvelocity,self.bot_zvelocity) = self.robots.return_velocity_data()
        (self.bot_xForcecontact,self.bot_yForcecontact,self.bot_zForcecontact) = self.robots.return_force_data_contact()            
        (self.skin_x_contact_forces,self.skin_y_contact_forces,self.skin_z_contact_forces,self.skin_x_total_forces,self.skin_y_total_forces,self.skin_z_total_forces)=self.robots.return_skin_forces()


        # interior data
        (self.particle_xposition,self.particle_yposition,self.particle_zposition) = self.interior.return_position_data()
        (self.particle_xvelocity,self.particle_yvelocity,self.particle_zvelocity) = self.interior.return_velocity_data()
        (self.particle_xForcetotal,self.particle_yForcetotal,self.particle_zForcetotal) = self.interior.return_force_data()
        (self.particle_xForcecontact,self.particle_yForcecontact,self.particle_zForcecontact) = self.interior.return_force_data()
        
        #### If grasping then certain other features need to be exported
        if self.control_mode=="grasping" or self.control_mode=="grasping_explore" or self.control_mode=="grasping_u" or self.control_mode=="grasping_epsilon":
            (self.ball_xposition,self.ball_zposition)=self.ball.return_position_data()
            (self.ball_xvelocity,self.ball_zvelocity)=self.ball.return_velocity_data()        
            self.Fb=self.ball.Fb
            self.TIME=self.ball.TIME
            self.PX=self.ball.PX
            self.PY=self.ball.PY
            (self.bFx,self.bFy,self.bFz)=self.ball.return_ball_contact_forces()
            (self.bFtx,self.bFty,self.bFtz)=self.ball.return_ball_total_forces()
            (self.bq0,self.bq1,self.bq2,self.bq3)=self.ball.return_angle_data()
            self.THETA=self.simulation.THETA
            self.Rr_=self.simulation.Rr_
            self.calc_type=self.simulation.calc_type
            self.epsilon_tilda=self.controls.epsilon_tilda
            self.time_tilda=self.controls.time_tilda
            if self.control_mode=="grasping_u":
                self.TRIG1_=self.controls.TRIG1_
                
            if self.control_mode=="grasping_epsilon":
                self.xjam=self.Psi.xjam
                self.yjam=self.Psi.yjam
         
        # potential field or distance function values of boundary robots
        self.Field_value=self.controls.Field_value
        
        # control forces
        self.F_controller_x=self.controls.F_controller_x
        self.F_controller_z=self.controls.F_controller_z
        
        #### Contact information
        self.time_contact = self.simulation.time_contact
        self.number_contacts = self.simulation.number_contacts
        self.Contact_points_x = self.simulation.Contact_points_x
        self.Contact_points_y = self.simulation.Contact_points_y
        self.Contact_points_z = self.simulation.Contact_points_z
        
        self.Contact_force_x = self.simulation.Contact_force_x
        self.Contact_force_y = self.simulation.Contact_force_y
        self.Contact_force_z = self.simulation.Contact_force_z
        
        self.Contact_force_x2 = self.simulation.Contact_force_x2
        self.Contact_force_y2 = self.simulation.Contact_force_y2
        self.Contact_force_z2 = self.simulation.Contact_force_z2
        
        self.bodiesA = self.simulation.bodiesA
        self.bodiesB = self.simulation.bodiesB
        
        self.bodiesA_ID = self.simulation.bodiesA_ID
        self.bodiesB_ID = self.simulation.bodiesB_ID
        

        
        self.Dir_xx = self.simulation.dir_xx
        self.Dir_xy = self.simulation.dir_xy
        self.Dir_xz = self.simulation.dir_xz
        
        self.Dir_yx = self.simulation.dir_yx
        self.Dir_yy = self.simulation.dir_yy
        self.Dir_yz = self.simulation.dir_yz
        
        self.Dir_zx = self.simulation.dir_zx
        self.Dir_zy = self.simulation.dir_zy
        self.Dir_zz = self.simulation.dir_zz

        self.EPSILON = self.simulation.EPSILON4
        self.calcultation_time  = self.simulation.calcultation_time
        self.Qcm = self.simulation.Qcm 
        self.centroidx = self.simulation.centroidx
        self.centroidz = self.simulation.centroidz
        self.number_contacts=np.asarray(self.number_contacts) # number of contacts
        self.lengthm=np.amax(self.number_contacts) # length of nc
        self.count=len(self.time_contact) 
        
        # #empty array to fill contact locations in each time step
        self.xc=np.zeros((self.lengthm,self.count)) # x points
        self.yc=np.zeros((self.lengthm,self.count)) # y points 
        self.zc=np.zeros((self.lengthm,self.count)) # z points
        
        
        # contact forces
        self.Fcx=np.zeros((self.lengthm,self.count))
        self.Fcy=np.zeros((self.lengthm,self.count))
        self.Fcz=np.zeros((self.lengthm,self.count))
        
        # contact forces 2
        self.Fcx2=np.zeros((self.lengthm,self.count))
        self.Fcy2=np.zeros((self.lengthm,self.count))
        self.Fcz2=np.zeros((self.lengthm,self.count))
        
        
        # contact ID
        self.AID=np.zeros((self.lengthm,self.count))
        self.BID=np.zeros((self.lengthm,self.count))       
        
        
        # contact frames
        self.Dirxx_=np.zeros((self.lengthm,self.count))
        self.Dirxy_=np.zeros((self.lengthm,self.count))
        self.Dirxz_=np.zeros((self.lengthm,self.count))

        self.Diryx_=np.zeros((self.lengthm,self.count))
        self.Diryy_=np.zeros((self.lengthm,self.count))
        self.Diryz_=np.zeros((self.lengthm,self.count))
 
        
        self.Dirzx_=np.zeros((self.lengthm,self.count))
        self.Dirzy_=np.zeros((self.lengthm,self.count))
        self.Dirzz_=np.zeros((self.lengthm,self.count))
        
        self.AN={} # empty array of names of contact bodies
        self.BN={} # empty array of names of contact bodes
        #print(len(self.number_contacts))
        for i in range(len(self.number_contacts)-1):    
             #print(i)
             self.AN["AN{0}".format(i)]=self.bodiesA[i]  # A names
             self.BN["BN{0}".format(i)]=self.bodiesB[i]  # B names
            
        # fill the arrays with contact information 
        for i in range(self.count):
            ind=self.number_contacts[i]
            temp1=self.Contact_points_x[i]
            temp2=self.Contact_points_y[i]
            temp3=self.Contact_points_z[i]
            temp4=self.Contact_force_x[i]
            temp5=self.Contact_force_y[i]
            temp6=self.Contact_force_z[i]
            
            temp7=self.bodiesA_ID[i]
            temp8=self.bodiesB_ID[i]
            
            temp9=self.Contact_force_x2[i]
            temp10=self.Contact_force_y2[i]
            temp11=self.Contact_force_z2[i]   
            
            temp12=self.Dir_xx[i] 
            temp13=self.Dir_xy[i] 
            temp14=self.Dir_xz[i] 
            
            temp15=self.Dir_yx[i] 
            temp16=self.Dir_yy[i] 
            temp17=self.Dir_yz[i] 
        
            temp18=self.Dir_zx[i] 
            temp19=self.Dir_zy[i] 
            temp20=self.Dir_zz[i]             
            
        
            # convert to array
            temp1=np.asarray(temp1)
            temp2=np.asarray(temp2)
            temp3=np.asarray(temp3)
            temp4=np.asarray(temp4)
            temp5=np.asarray(temp5)
            temp6=np.asarray(temp6)
            temp7=np.asarray(temp7)
            temp8=np.asarray(temp8)
            
            
            temp9=np.asarray(temp9)
            temp10=np.asarray(temp10)
            temp11=np.asarray(temp11)
            
            temp12=np.asarray(temp12)
            temp13=np.asarray(temp13)
            temp14=np.asarray(temp14)
            
            temp15=np.asarray(temp15)
            temp16=np.asarray(temp16)
            temp17=np.asarray(temp17)  
            
            temp18=np.asarray(temp18)
            temp19=np.asarray(temp19)
            temp20=np.asarray(temp20)             
            
            #fill array position
            #print(np.shape(temp1))
            #print(np.shape(self.xc[0:ind,i]))
            self.xc[0:ind,i]=np.transpose(temp1)
            self.yc[0:ind,i]=np.transpose(temp2)
            self.zc[0:ind,i]=np.transpose(temp3)
    
            # Fill array forces
            self.Fcx[0:ind,i]=np.transpose(temp4)
            self.Fcy[0:ind,i]=np.transpose(temp5)
            self.Fcz[0:ind,i]=np.transpose(temp6)  
            self.AID[0:ind,i]=np.transpose(temp7)
            self.BID[0:ind,i]=np.transpose(temp8)
            
            self.Fcx2[0:ind,i]=np.transpose(temp9)
            self.Fcy2[0:ind,i]=np.transpose(temp10)
            self.Fcz2[0:ind,i]=np.transpose(temp11) 
            
            
            self.Dirxx_[0:ind,i]=np.transpose(temp12)
            self.Dirxy_[0:ind,i]=np.transpose(temp13)
            self.Dirxz_[0:ind,i]=np.transpose(temp14)
            
            self.Diryx_[0:ind,i]=np.transpose(temp15)
            self.Diryy_[0:ind,i]=np.transpose(temp16)
            self.Diryz_[0:ind,i]=np.transpose(temp17)
            
            self.Dirzx_[0:ind,i]=np.transpose(temp18)
            self.Dirzy_[0:ind,i]=np.transpose(temp19) 
            self.Dirzz_[0:ind,i]=np.transpose(temp20) 

            
    def export_data(self):  
        '''Export Bot positions '''
        
        #### Control forces
        file_name=self.results_dir+'/control_forces.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.F_controller_x.items():
                w.writerow([key, *val])
            
            # write y position to csv file    
            for key, val in self.F_controller_z.items():
                w.writerow([key, *val]) 
                    
        #### Bot positions
        file_name=self.results_dir+'/bot_position.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.bot_xposition.items():
                w.writerow([key, *val])
            
        # write y position to csv file    
            for key, val in self.bot_yposition.items():
                w.writerow([key, *val]) 
            
        # write z position to csv file     
            for key, val in self.bot_zposition.items():
                w.writerow([key, *val])         
        
        
        
        #### Bot total forces
        file_name=self.results_dir+'/bot_total_forces.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.bot_xForcetotal.items():
                w.writerow([key, *val])
            
            # write y position to csv file    
            for key, val in self.bot_yForcetotal.items():
                w.writerow([key, *val]) 
            
            # write z position to csv file     
            for key, val in self.bot_zForcetotal.items():
                w.writerow([key, *val])            
        
        #### Bot Contact forces
        file_name=self.results_dir+'/bot_contact_forces.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.bot_xForcecontact.items():
                w.writerow([key, *val])
            
            # write y position to csv file    
            for key, val in self.bot_yForcecontact.items():
                w.writerow([key, *val]) 
            
            # write z position to csv file     
            for key, val in self.bot_zForcecontact.items():
                w.writerow([key, *val])   
                



        #### Particle total forces
        file_name=self.results_dir+'/particle_total_forces.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.particle_xForcetotal.items():
                w.writerow([key, *val])
            
            # write y position to csv file    
            for key, val in self.particle_yForcetotal.items():
                w.writerow([key, *val]) 
            
            # write z position to csv file     
            for key, val in self.particle_zForcetotal.items():
                w.writerow([key, *val])            
        
        #### Particle contact forces
        file_name=self.results_dir+'/particle_contact_forces.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.particle_xForcecontact.items():
                w.writerow([key, *val])
            
            # write y position to csv file    
            for key, val in self.particle_yForcecontact.items():
                w.writerow([key, *val]) 
            
            # write z position to csv file     
            for key, val in self.particle_zForcecontact.items():
                w.writerow([key, *val])   





                
        #### Field values
        file_name=self.results_dir+'/field_values.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
                
            # write x position to csv file
            for key, val in self.Field_value.items():
                w.writerow([key, *val])            
                
                
                
        ####Export membrane positions 
        file_name=self.results_dir+'/membrane_position.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.skin_xposition.items():
                w.writerow([key, *val])
            
            # write y position to csv file    
            for key, val in self.skin_yposition.items():
                w.writerow([key, *val]) 
            
            # write z position to csv file     
            for key, val in self.skin_zposition.items():
                w.writerow([key, *val])    





        ####Export membrane positions
        file_name=self.results_dir+'/membrane_total_forces.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.skin_x_total_forces.items():
                w.writerow([key, *val])
            
            # write y position to csv file    
            for key, val in self.skin_y_total_forces.items():
                w.writerow([key, *val]) 
            
            # write z position to csv file     
            for key, val in self.skin_z_total_forces.items():
                w.writerow([key, *val])    



        ####Export membrane positions
        file_name=self.results_dir+'/membrane_contact_forces.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.skin_x_contact_forces.items():
                w.writerow([key, *val])
            
            # write y position to csv file    
            for key, val in self.skin_y_contact_forces.items():
                w.writerow([key, *val]) 
            
            # write z position to csv file     
            for key, val in self.skin_z_contact_forces.items():
                w.writerow([key, *val])    

       
             
        ####Export Bot velocities    
        file_name=self.results_dir+'/bot_velocity.csv'
        # export bot position
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.bot_xvelocity.items():
                w.writerow([key, *val])
            
            # write y position to csv file    
            for key, val in self.bot_yvelocity.items():
                w.writerow([key, *val]) 
            
            # write z position to csv file     
            for key, val in self.bot_zvelocity.items():
                w.writerow([key, *val])                   
        
        
        ####Export particle positions    
        file_name=self.results_dir+'/particle_position.csv'

        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
            
            # write x position to csv file
            for key, val in self.particle_xposition.items():
                w.writerow([key, *val])
            
            # write y position to csv file    
            for key, val in self.particle_yposition.items():
                w.writerow([key, *val]) 
            
            # write z position to csv file     
            for key, val in self.particle_zposition.items():
                w.writerow([key, *val])                 
  
        #### AN        
        file_name=self.results_dir+'/AN.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.AN.items():
                w.writerow([key,*val])
        
        # names of objects in contact        
        file_name=self.results_dir+'/BN.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.BN.items():
                w.writerow([key,*val])            

 
                
        #### contact points x
        file_name=self.results_dir+'/x_contact_points.csv' 
        savetxt(file_name,self.xc, delimiter=',')
    
        #### contact points y
        file_name=self.results_dir+'/y_contact_points.csv' 
        savetxt(file_name,self.yc, delimiter=',')
    
        #### contact points z
        file_name=self.results_dir+'/z_contact_points.csv' 
        savetxt(file_name,self.zc, delimiter=',')
    
        #### contact force x
        file_name=self.results_dir+'/x_contact_force.csv' 
        savetxt(file_name,self.Fcx, delimiter=',')
    
        #### contact force y
        file_name=self.results_dir+'/y_contact_force.csv' 
        savetxt(file_name,self.Fcy, delimiter=',')
    
        #### contact force z
        file_name=self.results_dir+'/z_contact_force.csv' 
        savetxt(file_name,self.Fcz, delimiter=',')
    
    
    
        #### contact force x
        file_name=self.results_dir+'/x_contact_force2.csv' 
        savetxt(file_name,self.Fcx2, delimiter=',')
    
        #### contact force y
        file_name=self.results_dir+'/y_contact_force2.csv' 
        savetxt(file_name,self.Fcy2, delimiter=',')
    
        #### contact force z
        file_name=self.results_dir+'/z_contact_force2.csv' 
        savetxt(file_name,self.Fcz2, delimiter=',')    
    
        #### contact_dirxx
        file_name=self.results_dir+'/contact_dirxx.csv' 
        savetxt(file_name,self.Dirxx_, delimiter=',')     
    
    
        #### contact_dirxy
        file_name=self.results_dir+'/contact_dirxy.csv' 
        savetxt(file_name,self.Dirxy_, delimiter=',')       
    
        #### contact_dirxz
        file_name=self.results_dir+'/contact_dirxz.csv' 
        savetxt(file_name,self.Dirxz_, delimiter=',')  
        
        #### contact_diryx
        file_name=self.results_dir+'/contact_diryx.csv' 
        savetxt(file_name,self.Diryx_, delimiter=',')     
    
        #### contact_diryy
        file_name=self.results_dir+'/contact_diryy.csv' 
        savetxt(file_name,self.Diryy_, delimiter=',')       
    
        #### contact_diryz
        file_name=self.results_dir+'/contact_diryz.csv' 
        savetxt(file_name,self.Diryz_, delimiter=',')         
        
        #### contact_dirzx
        file_name=self.results_dir+'/contact_dirzx.csv' 
        savetxt(file_name,self.Dirzx_, delimiter=',')     
    
        #### contact_dirzy
        file_name=self.results_dir+'/contact_dirzy.csv' 
        savetxt(file_name,self.Dirzy_, delimiter=',')       
    
        #### contact_dirzz
        file_name=self.results_dir+'/contact_dirzz.csv' 
        savetxt(file_name,self.Dirzz_, delimiter=',')  
         
        #### number of contacts
        file_name=self.results_dir+'/number_contacts.csv' 
        savetxt(file_name,self.number_contacts, delimiter=',')
        
        #### contact A id
        file_name=self.results_dir+'/AID.csv' 
        savetxt(file_name,self.AID, delimiter=',')
                    
        #### contact B id
        file_name=self.results_dir+'/BID.csv' 
        savetxt(file_name,self.BID, delimiter=',') 
        
        
        #### Time contact
        file_name=self.results_dir+'/time_contact.csv' 
        savetxt(file_name,self.time_contact, delimiter=',')        
        
        if self.control_mode=="grasping" or self.control_mode=="grasping_explore" or self.control_mode=="grasping_u" or self.control_mode=="grasping_epsilon" :
            
            #### Export ball positions        
            file_name=self.results_dir+'/ball_position.csv'
    
            with open(file_name, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.ball_xposition.items():
                    w.writerow([key, *val])
                
                # write z position to csv file     
                for key, val in self.ball_zposition.items():
                    w.writerow([key, *val])              
                    
            #### Export ball velocity        
            file_name=self.results_dir+'/ball_velocity.csv'
    
            with open(file_name, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.ball_xvelocity.items():
                    w.writerow([key, *val])

                # write z position to csv file     
                for key, val in self.ball_zvelocity.items():
                    w.writerow([key, *val])    
                    
            #### Export ball angles      
            file_name=self.results_dir+'/ball_angle.csv'

            with open(file_name, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.bq0.items():
                    w.writerow([key, *val])
                
                # write z position to csv file     
                for key, val in self.bq1.items():
                    w.writerow([key, *val])     
                    
                # write z position to csv file     
                for key, val in self.bq2.items():
                    w.writerow([key, *val])                
           
                # write z position to csv file     
                for key, val in self.bq3.items():
                    w.writerow([key, *val])                      
           
            #### Export ball contact force      
            file_name=self.results_dir+'/ball_contact_forces.csv'
    
            with open(file_name, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.bFx.items():
                    w.writerow([key, *val])
                
                      
                    
                # write z position to csv file     
                for key, val in self.bFz .items():
                    w.writerow([key, *val])                       
                    
                    
            #### ball total force       
            file_name=self.results_dir+'/ball_total_forces.csv'
    
            with open(file_name, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.bFtx.items():
                    w.writerow([key, *val])
                
                # write y position to csv file     
                for key, val in self.bFty.items():
                    w.writerow([key, *val])                        
                    
                # write z position to csv file     
                for key, val in self.bFtz.items():
                    w.writerow([key, *val])                       
                    
                    
                    
            #### Pull Force        
            file_name=self.results_dir+'/pull_force.csv'
    
            with open(file_name, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.TIME.items():
                    w.writerow([key,*val])
                    
                for key, val in self.PX.items():
                    w.writerow([key,*val])  
                    
                for key, val in self.PY.items():
                    w.writerow([key,*val])  
                    
                # write x position to csv file
                for key, val in self.Fb.items():
                    w.writerow([key, *val])


            #### EPSILON
            file_name=self.results_dir+'/EPSILON.csv' 
            savetxt(file_name,self.EPSILON, delimiter=',') 

            #### calcultation_time
            file_name=self.results_dir+'/calcultation_time.csv' 
            savetxt(file_name,self.calcultation_time , delimiter=',') 


            #### THETA
            file_name=self.results_dir+'/THETA.csv' 
            savetxt(file_name,self.THETA, delimiter=',') 
            
            #### Rr
            file_name=self.results_dir+'/Rr_.csv' 
            savetxt(file_name,self.Rr_, delimiter=',')  
            
            #### centroidx
            file_name=self.results_dir+'/centroidx.csv' 
            savetxt(file_name,self.centroidx, delimiter=',')  
            
            #### centroidz
            file_name=self.results_dir+'/centroidz.csv' 
            savetxt(file_name,self.centroidz, delimiter=',')   
            
            #### Qcm
            file_name=self.results_dir+'/Qcm.csv' 
            savetxt(file_name,self.Qcm, delimiter=',')    

            #### calc_type
            file_name=self.results_dir+'/calc_type.csv' 
            savetxt(file_name,self.calc_type, delimiter=',') 
            
            #### epsilon tilda
            file_name=self.results_dir+'/epsilon_tilda.csv' 
            savetxt(file_name,self.epsilon_tilda, delimiter=',') 
            
            #### time tilda
            file_name=self.results_dir+'/time_tilda.csv' 
            savetxt(file_name,self.time_tilda, delimiter=',')
            
            #### TRIG1_
            if self.control_mode=="grasping_u":
                file_name=self.results_dir+'/TRIG1_.csv' 
                savetxt(file_name,self.TRIG1_, delimiter=',')
                
            if self.control_mode=="grasping_epsilon":
                file_name=self.results_dir+'/xjam.csv' 
                savetxt(file_name,self.xjam, delimiter=',')   
 
                file_name=self.results_dir+'/yjam.csv' 
                savetxt(file_name,self.yjam, delimiter=',')   
                
class MyReportContactCallback(chrono.ReportContactCallback):
    """ Class for reporting and storing the the contact forces and postions  """
    def __init__(self):

        chrono.ReportContactCallback.__init__(self)
        self.Contact_force_x=[]
        self.Contact_force_y=[]
        self.Contact_force_z=[]
        
        self.Contact_force_x2=[]
        self.Contact_force_y2=[]
        self.Contact_force_z2=[]
        
        self.Contact_points_x= []
        self.Contact_points_y = []
        self.Contact_points_z = []
        
        self.bodiesA = []
        self.bodiesB = []
        
        self.bodiesA_ID=[]
        self.bodiesB_ID=[]



        self.dir_xx=[]
        self.dir_xy=[]
        self.dir_xz=[]
        
        
        self.dir_yx=[]
        self.dir_yy=[]
        self.dir_yz=[]      
        
        self.dir_zx=[]
        self.dir_zy=[]
        self.dir_zz=[]



        self.CA=[]
        
    def OnReportContact(self,vA,vB,cA,dist,rad,force,torque,modA,modB):
        bodyUpA = chrono.CastContactableToChBody(modA)
        nameA = bodyUpA.GetName()
        bodyUpB = chrono.CastContactableToChBody(modB)
        nameB = bodyUpB.GetName()
        
        IDA = bodyUpA.GetId()
        IDB = bodyUpB.GetId()
        cA=np.asarray(cA.GetMatr())
        self.CA.append(cA)
        self.Contact_points_x.append(vA.x)
        self.Contact_points_y.append(vA.y)
        self.Contact_points_z.append(vA.z)
        

        
        self.Contact_force_x.append(force.x)
        self.Contact_force_y.append(force.y)
        self.Contact_force_z.append(force.z)
        
        
        force=np.array([force.x,force.y,force.z])
        #print("force",force)
        #print("cA",cA)
        force=np.matmul(cA,force)
        #print("force",force)
        self.Contact_force_x2.append(force[0])
        self.Contact_force_y2.append(force[1])
        self.Contact_force_z2.append(force[2])
        
        dirx=np.matmul(cA,np.array([1,0,0]))
        diry=np.matmul(cA,np.array([0,1,0]))        
        dirz=np.matmul(cA,np.array([0,0,1]))
        
        
        #print(dirx,diry,dirz)
        
        #print(dirx.x)
        self.dir_xx.append(dirx[0])
        self.dir_xy.append(dirx[1])
        self.dir_xz.append(dirx[2])
        
        
        self.dir_yx.append(diry[0])
        self.dir_yy.append(diry[1])
        self.dir_yz.append(diry[2])       
        
        self.dir_zx.append(dirz[0])
        self.dir_zy.append(dirz[1])
        self.dir_zz.append(dirz[2])        
        
        self.bodiesA.append(nameA)
        self.bodiesB.append(nameB)
        
        self.bodiesA_ID.append(IDA)
        self.bodiesB_ID.append(IDB)
        
        
        return True        # return False to stop reporting contacts

    # reset after every run 
    def ResetList(self):
        self.Contact_force_x = []
        self.Contact_force_y = []
        self.Contact_force_z = []
        
        self.Contact_force_x2 = []
        self.Contact_force_y2 = []
        self.Contact_force_z2 = [] 
        
        self.Contact_points_x = []
        self.Contact_points_y = []
        self.Contact_points_z = []
        
        self.bodiesA = []
        self.bodiesB = []
        
        self.bodiesA_ID=[]
        self.bodiesB_ID=[]
        self.CA=[]
        
        self.dir_xx=[]
        self.dir_xy=[]
        self.dir_xz=[]
        
        
        self.dir_yx=[]
        self.dir_yy=[]
        self.dir_yz=[]      
        
        self.dir_zx=[]
        self.dir_zy=[]
        self.dir_zz=[]
        
    # Get the points
    def GetList(self):
        return (self.Contact_points_x, # 0
                self.Contact_points_y, # 1
                self.Contact_points_z, # 2
                self.Contact_force_x,  # 3
                self.Contact_force_y,  # 4
                self.Contact_force_z,  # 5
                self.Contact_force_x2, # 6
                self.Contact_force_y2, # 7
                self.Contact_force_z2, # 8               
                self.bodiesA,          # 9
                self.bodiesB,          #10     
                self.bodiesA_ID,       #11
                self.bodiesB_ID,       #12
                self.CA,               #13
                self.dir_xx,           #14
                self.dir_xy,           #15
                self.dir_xz,           #16
                self.dir_yx,           #17
                self.dir_yy,           #18
                self.dir_yz,           #19
                self.dir_zx,           #20
                self.dir_zy,           #21
                self.dir_zz)           #22




class R_functions():  
    """ R-function Class """
    def __init__(self,name,path):
        self.direct = os.path.dirname(__file__)
        self.name = name
        self.path=path
        self.mainDirectory = self.path
        parameters = np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters = parameters.tolist()
        self.control_mode=self.parameters['control_mode']  # control mode
        self.m = 8
        ######### SHAPE FORMATION #########
        if self.control_mode=="shape_formation":
            self.geometry = self.parameters['geometry'] 

            if self.geometry=='circle':
                self.segments = 0
                self.a = self.parameters['a']
                self.b = self.parameters['b']
                self.R = self.parameters['a']
            
            if self.geometry=='pacman':
                self.a = self.parameters['xcenter']
                self.b = self.parameters['zcenter']
                self.R = self.parameters['a']
                self.segments = np.array([[self.a,self.b,self.R*np.cos(np.pi/4),self.R*np.sin(np.pi/4)],[self.a,self.b,self.R*np.cos(-np.pi/4),self.R*np.sin(-np.pi/4)]])
                theta=np.linspace(np.pi/4,7*np.pi/4,100)
                self.xp=[]
                self.yp=[]
                for i in range(len(theta)):
                    
                    self.xp.append(self.R*np.cos(theta[i])+self.a)
                    self.yp.append(self.R*np.sin(theta[i])+self.b)
                
                self.xp.append(self.R*np.cos(-np.pi/4))
                self.xp.append(self.a)
                self.xp.append(self.R*np.cos(np.pi/4))
                
            
                self.yp.append(self.R*np.sin(-np.pi/4))
                self.yp.append(self.a)
                self.yp.append(self.R*np.sin(np.pi/4))                
                
                
                np.savez(self.mainDirectory+'/'+self.name+'/'+'outline'+self.geometry+'.npz',xp=self.xp,yp=self.yp)          
            else:
                data = np.load(self.direct+'/shapes/'+self.geometry+'.npz')
                self.segments = data['segments']
                self.scale = self.parameters['scale']
                self.segments=self.segments


            
        ######### SHAPE MORPHING #########
        if self.control_mode=="shape_morphing":  
            self.p = self.parameters['p']
            self.geometry1 = self.parameters['geometry1'] 
            self.geometry2 = self.parameters['geometry2'] 
            self.scale1 = self.parameters['scale1']
            self.scale2 = self.parameters['scale2']
            
        
            
            if self.geometry1=='circle':
                self.segments = 0
                self.a = self.parameters['a']
                self.b = self.parameters['b']
                self.R = self.parameters['R']
                
                
            if self.geometry2=='circle':
                self.segments = 0
                self.a = self.parameters['a']
                self.b = self.parameters['b']
                self.R = self.parameters['R']


            if self.geometry1!='circle':                 
                data = np.load(self.direct+'/shapes/'+self.geometry1+'.npz')
                self.segments = data['segments']
                self.segments = self.scale1*self.segments                   
                
            
            if self.geometry2!='circle':                 
                data = np.load(self.direct+'/shapes/'+self.geometry2+'.npz')
                self.segments = data['segments']
                np.savez(self.mainDirectory+'/'+self.name+'/'+'shapes'+self.geometry2+'.npz',segments=self.segments,xp=data['xp'],yp=data['yp'])          
        
        
        ######### GRASPING_U #########
        if self.control_mode=="grasping_u":
            self.p = self.parameters['p']
            self.width_grasp = self.parameters['width_grasp']
            self.length_grasp = self.parameters['length_grasp']
            self.lengtho_grasp = self.parameters['lengtho_grasp']
            
            self.xcenter_grasp = self.parameters['xcenter_grasp']
            self.ycenter_grasp = self.parameters['ycenter_grasp']
            
            self.xcenter_grasp2 = self.parameters['xcenter_grasp2']
            self.ycenter_grasp2 = self.parameters['ycenter_grasp2']
          
            self.xcenter = self.parameters['xcenter']
            self.ycenter = self.parameters['zcenter']            
            self.R = self.parameters['R']
            self.Atilda=self.parameters['Area'] 
            #self.length_grasp = self.Atilda/(2*self.width_grasp) - self.lengtho_grasp/2
           # self.parameters['length_grasp']=self.length_grasp
            
            self.tcut1=self.parameters['tcut1']
            self.tcut2=self.parameters['tcut2']
            self.tcut3=self.parameters['tcut3']
            self.lengtho2_grasp = 0
            self.length2_grasp = 0
            self.xc1 = self.parameters['xc1']     
            self.yc1 = self.parameters['yc1']
            self.xc2 = self.parameters['xc2']     
            self.yc2 = self.parameters['yc2']   
            self.ball_radius = self.parameters["ball_radius"]
            self.rho_ = self.parameters['rho_']
            self.rtilda = self.parameters['rtilda']
        ######### GRASPING & GRASPING_EXPLORE #########    
        if self.control_mode=="grasping" or self.control_mode=="grasping_explore" or self.control_mode=="grasping_epsilon":

            self.segments = 0
            self.a1 = self.parameters['a1']
            self.b1 = self.parameters['b1']
            self.xc1 = self.parameters['xc1']     
            self.yc1 = self.parameters['yc1']
            
            self.a2 = self.parameters['a2']
            self.b2 = self.parameters['b2']
            self.xc2 = self.parameters['xc2']     
            self.yc2 = self.parameters['yc2']   
            
            self.xcenter = self.parameters['xcenter']
            self.ycenter = self.parameters['zcenter']
            self.R = self.parameters['R']
            self.tcut1 = self.parameters['tcut1'] 
            self.tcut2 = self.parameters['tcut2'] 
            self.tcut3 = self.parameters['tcut3'] 
            self.segments_jam=0
            self.xjam=[]
            self.yjam=[]
##############################################################################            
            

    
    def phi_oval(self,x,y,xc,yc,a,b):
        phi=((x-xc)/a)**2 + ((y-yc)/b)**2 - 1 
        return(abs(phi))     
        
    def dphix_oval(self,x,y,xc,yc,a,b):
        phi=((x-xc)/a)**2 + ((y-yc)/b)**2 - 1 
        return(np.sign(phi) * (2*x - 2*xc) / a**2)

    def dphiy_oval(self,x,y,xc,yc,a,b):
        phi=((x-xc)/a)**2 + ((y-yc)/b)**2 - 1 
        return(np.sign(phi) * (2*y - 2*yc) / b**2)
    

    def phi_circle(self,x,y,a,b,R):
        """ Normalized distance function of a circle """
        phi = (R**2 - (x-a)**2 - (y-b)**2)
        return(abs(phi))
        
    def dphix_circle(self,x,y,a,b,R):
        """ Derivative of the distance function of cirlce wrt x """
        return(R*(x-a)*np.sign((R/2)*(- R**2 +(a-x)**2 + (b-y)**2)))
    
    
    def dphiy_circle(self,x,y,a,b,R):
        """ Derivative of the distance function of cirlce wrt y """
        return(R*(y-b)*np.sign((R/2)*(- R**2 +(a-x)**2 + (b-y)**2)))
    
    
    def phi_quarter_circle(self,x,y,a,b,R):
        """ Distance function for partial circles """
        theta1=np.pi/4
        theta2=-np.pi/4
        x1=R*np.cos(theta1)
        y1=R*np.sin(theta1)

        x2=R*np.cos(theta2)
        y2=R*np.sin(theta2)

        T=self.phi_line_(x,y,x1,y1,x2,y2)
        f=self.phi_circle(x,y,self.a,self.b,self.R)
        
        phi1=self.Trim(f,T)
        
        return(phi1)
    
    
    def dphix_quarter_circle(self,x,y,a,b,R):
        """ Distance function for partial circles wrt x  """
        theta1=np.pi/4
        theta2=-np.pi/4
        x1=R*np.cos(theta1)
        y1=R*np.sin(theta1)

        x2=R*np.cos(theta2)
        y2=R*np.sin(theta2)
        
        T=self.phi_line_(x,y,x1,y1,x2,y2)
        Tx=self.dphix_line_(x,y,x1,y1,x2,y2)
        
        
        f=self.phi_circle(x,y,self.a,self.b,self.R)
        fx=self.dphix_circle(x,y,self.a,self.b,self.R)

        
        phi1=self.Trimx(f,T,fx,Tx)
        
        return(phi1)    
    

    def dphiy_quarter_circle(self,x,y,a,b,R):
        """ Distance function for partial circles wrt y  """
        theta1=np.pi/4
        theta2=-np.pi/4
        x1=R*np.cos(theta1)
        y1=R*np.sin(theta1)

        x2=R*np.cos(theta2)
        y2=R*np.sin(theta2)
        
        T=self.phi_line_(x,y,x1,y1,x2,y2)
        Ty=self.dphiy_line_(x,y,x1,y1,x2,y2)
        
        
        f=self.phi_circle(x,y,self.a,self.b,self.R)
        fy=self.dphiy_circle(x,y,self.a,self.b,self.R)

        
        phi1=self.Trimy(f,T,fy,Ty)
        
        return(phi1)


    
    
    def Trim(self,f,t):
        """ Trim function for two functions  """
        phi=np.sqrt(t**2 + f**4)
        return(np.sqrt(f**2 + ((phi-t)/2)**2))

    def Trimx(self,f,t,fx,tx):
        """Derivative Trim function for two functions wrt x  """
        term1 = (2*(f**3)*fx + tx*t)/(np.sqrt(f**4 + t**2)) - tx
        term2 = np.sqrt(f**4 + t**2)/2 - t/2
        term3 = f*fx
        term4 = np.sqrt((np.sqrt(f**4 + t**2)/2 - t/2)**2 + f**2)
        
        return((0.5*term1*term2 +term3)/term4)
        
    def Trimy(self,f,t,fy,ty):
        """Derivative Trim function for two functions wrt y """
        term1 = (2*(f**3)*fy + ty*t)/(np.sqrt(f**4 + t**2)) - ty
        term2 = np.sqrt(f**4 + t**2)/2 - t/2
        term3 = f*fy
        term4 = np.sqrt((np.sqrt(f**4 + t**2)/2 - t/2)**2 + f**2)
        
        return((0.5*term1*term2 +term3)/term4)    
    
##############################################################################   

    def phi_line_(self,x,y,x1,y1,x2,y2):
        """ Distance function of a line """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(((x-x1)*(y2-y1)-(y-y1)*(x2-x1))/L)


    def dphix_line_(self,x,y,x1,y1,x2,y2):
        """ Derivative distance function of a line wrt x """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return((-y1+y2)/L)


    def dphiy_line_(self,x,y,x1,y1,x2,y2):
        """ Derivative distance function of a line wrt y """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return((x1-x2)/L)


    def trim(self,x,y,x1,y1,x2,y2):
        """ Trim function """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        xc = np.array([(x2+x1)/2,(y2+y1)/2])
        t = (1/L)*((L/2)**2 - ((x-xc[0])**2 + (y-xc[1])**2))    
        return(t)


    def dtrimx(self,x,y,x1,y1,x2,y2):
        """ Derivative of Trim function wrt x """
        xc = np.array([(x2+x1)/2,(y2+y1)/2]) 
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(-(x-xc[0])/ (L*np.sqrt((x-xc[0])**2 + (y-xc[1])**2)))

    def dtrimy(self,x,y,x1,y1,x2,y2):
        """ Derivative of Trim function wrt y """
        xc = np.array([(x2+x1)/2,(y2+y1)/2]) 
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(-(y-xc[1])/ (L*np.sqrt((x-xc[0])**2 + (y-xc[1])**2)))

    def phi_line(self,x,y,x1,y1,x2,y2):
        """ Trimmed line segment"""
        t = self.trim(x,y,x1,y1,x2,y2)
        f = self.phi_line_(x,y,x1,y1,x2,y2)
        rho = np.sqrt(t**2 + f**4)
        return(np.sqrt(f**2 + ((rho-t)/2)**2))

    def dphix_line(self,x,y,x1,x2,y1,y2):
        """ Derivative of trimmed line segment wrt x"""
        ff = self.phi_line_(x,y,x1,x2,y1,y2)
        dfx = self.dphix_line_(x,y,x1,x2,y1,y2)
        tf = self.trim(x,y,x1,x2,y1,y2)
        dtx = self.dtrimx(x,y,x1,x2,y1,y2)
        term1 = 0.5*((2 * ff**3 * dfx + tf*dtx)/(np.sqrt(ff**4 + tf**2)) - dtx)*((np.sqrt(ff**4 + tf**2)-tf)/2)
        term2 = ff*dfx
        term3 = np.sqrt((((np.sqrt(ff**4 + tf**2)-tf)/2)**2) + ff**2)  
        return((term1+term2)/term3)

    def dphiy_line(self,x,y,x1,x2,y1,y2):
        """ Derivative of trimmed line segment wrt y"""
        ff = self.phi_line_(x,y,x1,x2,y1,y2)
        dfy = self.dphiy_line_(x,y,x1,x2,y1,y2)
        tf = self.trim(x,y,x1,x2,y1,y2)
        dty = self.dtrimy(x,y,x1,x2,y1,y2)
        term1 = 0.5*((2 * ff**3 * dfy + tf*dty)/(np.sqrt(ff**4 + tf**2)) - dty)*((np.sqrt(ff**4 + tf**2)-tf)/2)
        term2 = ff*dfy
        term3 = np.sqrt((((np.sqrt(ff**4 + tf**2)-tf)/2)**2) + ff**2)    
        return((term1+term2)/term3)   


    def phi_segments(self,x,y,segments):
        """ R equivelent of trimmed line segments"""
        R=0
        for i in range(len(segments[:,0])):
            R = R + 1/self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**self.m
        R = 1/R**(1/self.m)
        return(R)

    def dphix_segments(self,x,y,segments):
        """ Derivative of R equivelent of trimmed line segments wrt x"""

        term1=0
        term2=0
        term3=0
        for i in range(len(segments[:,0])):
            term1=-self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**-self.m) * self.dphix_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3]) + term1
            term2=(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**-self.m)**(-1/self.m) + term2
            term3=self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**-self.m) + term3      
        R=(-term1*term2/term3)
        return(R)

    def dphiy_segments(self,x,y,segments):
        """ Derivative of R equivelent of trimmed line segments wrt y"""

        term1=0
        term2=0
        term3=0
        for i in range(len(segments[:,0])):
            term1=-self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**(-self.m)) * self.dphiy_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3]) + term1
            term2=(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**(-self.m))**(-1/self.m) + term2
            term3=self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**(-self.m)) + term3      
        R=(-term1*term2/term3)
        return(R)
    
    def E_(self,x,xo,y,yo,r):
        xbar=x-xo
        ybar=y-yo
        E=np.array([[2*xbar/(r**2),-2*ybar/(r**2)],
                   [2*ybar/(r**2),2*xbar/(r**2)]])
        return(E)
    
    
    def Gamma(self,x,y,xo,yo,r):
        xbar=x-xo
        ybar=y-yo
        return((xbar/r)**2 + (ybar/r)**2)
    
    
    def D_(self,G,rho):
        lambda1 = 1 - (1/(abs(G)))**(1/rho)
        lambda2 = 1 + (1/(abs(G)))**(1/rho)
        D=np.diag([lambda1,lambda2])
        return(D)
    
    
    def M_(self,x,y,xo,yo,r,rho):
        E=self.E_(x,xo,y,yo,r)
        Einv=np.linalg.inv(E)
        G=self.Gamma(x,y,xo,yo,r)
        D=self.D_(G,rho)
        M=E@D@Einv
        return(M)


    def field3(self,x,y,xo,yo,xg,yg,r,rho_):
        M=self.M_(x,y,xo,yo,r,rho_)
        f=np.array([[-(x-xg)],[-(y-yg)]])
        #r_=np.sqrt(((x-xo)**2 + (y-yo)**2))
        # if r_<r:
        #     f=np.array([[0],[0]])
        #     temp=M@f
        #     temp=temp.flatten()
        #     zeta1=temp[0]
        #     zeta2=temp[1]
        # else:
        temp=M@f
        temp=temp.flatten()
        zeta1=temp[0]
        zeta2=temp[1]
        return(zeta1,zeta2)  
    
    def field4(self,x,y,xo,yo,xg,yg,r,rho):
        zeta1=np.zeros((len(y),len(x)))
        zeta2=np.zeros((len(y),len(x)))
        for i in range(len(y)):
            for j in range(len(x)):
                r_=np.sqrt(((x[j]-xo)**2 + (y[i]-yo)**2))
                M=self.M_(x[j],y[i],xo,yo,r,rho)
                f=np.array([[-(x[j]-xg)],[-(y[i]-yg)]])
                if r_<r:
                    f=np.array([[0],[0]])
                    temp=M@f
                    temp=temp.flatten()
                    zeta1[i,j]=temp[0]
                    zeta2[i,j]=temp[1]
                else:
                    temp=M@f
                    temp=temp.flatten()
                    zeta1[i,j]=temp[0]
                    zeta2[i,j]=temp[1]
        return(zeta1,zeta2)  
 
    
    def FxGraspEpsilon(self,x,y,t,nojam):
        if nojam==False:
            if t<=self.tcut2:
                Fx1 = self.Fx_point(x,y,self.xc2,self.yc2)
            elif t>self.tcut2:
            
                Fx1 = self.dphix_segments(x,y,self.segments_jam)        
        if nojam==True:
            Fx1 = self.Fx_point(x,y,self.xc2,self.yc2)    
        return(Fx1)
    
    def FyGraspEpsilon(self,x,y,t,nojam):
        if nojam==False:
            if t<=self.tcut2:
                Fy1 = self.Fy_point(x,y,self.xc2,self.yc2)
            elif t>self.tcut2:
            
                Fy1 = self.dphiy_segments(x,y,self.segments_jam)        
        if nojam==True:
            Fy1 = self.Fy_point(x,y,self.xc2,self.yc2)    
              
        return(Fy1)

    def FxGraspU(self,x,y,t,trig):
        
        if trig==0:
            #ft_=self.ft2(t,self.tcut1,self.p) 
            #theta=(1-ft_)*np.pi/2 + ft_*0
            theta=np.pi/2
            x_=[]
            y_=[]
            self.lengtho2_grasp = self.lengtho_grasp + 2*self.width_grasp*np.tan((np.pi/2 - theta)/2)
            self.length2_grasp = self.length_grasp +self.width_grasp*np.tan((np.pi/2 - theta)/2)
    
            x_.append(self.length_grasp*np.cos(theta) + self.xcenter_grasp)
            y_.append(self.length_grasp*np.sin(theta) + self.lengtho_grasp/2 + self.ycenter_grasp)
    
            x_.append(0 + self.xcenter_grasp)
            y_.append(self.lengtho_grasp/2 + self.ycenter_grasp)
    
            x_.append(0 + self.xcenter_grasp)
            y_.append(-self.lengtho_grasp/2 + self.ycenter_grasp)
    
            x_.append(self.length_grasp*np.cos(-theta) + self.xcenter_grasp)
            y_.append(self.length_grasp*np.sin(-theta) - self.lengtho_grasp/2 + self.ycenter_grasp)
     
            x_.append(self.length2_grasp*np.cos(-theta) + self.xcenter_grasp - self.width_grasp)
            y_.append(self.length2_grasp*np.sin(-theta) - self.lengtho2_grasp/2 + self.ycenter_grasp)
            
            x_.append(0 + self.xcenter_grasp - self.width_grasp)
            y_.append(-self.lengtho2_grasp/2 + self.ycenter_grasp)
       
            x_.append(0 + self.xcenter_grasp - self.width_grasp)
            y_.append(self.lengtho2_grasp/2 + self.ycenter_grasp)
       
            x_.append(self.length2_grasp*np.cos(theta) - self.width_grasp + self.xcenter_grasp)
            y_.append(self.length2_grasp*np.sin(theta) + self.lengtho2_grasp/2 + self.ycenter_grasp)
       
            x_.append(self.length_grasp*np.cos(theta) + self.xcenter_grasp)
            y_.append(self.length_grasp*np.sin(theta) + self.lengtho_grasp/2 + self.ycenter_grasp)
            
            self.segements=self.create_segment(x_,y_)       
            # if t<self.tcut1:
            #     ft2_=self.ft2(t,0,self.p) 
            #     phi2=self.phi_segments(x,y,self.segements)
            #     phi1=self.phi_circle(x,y,self.xcenter,self.ycenter,self.R)
            #     dphi2x=self.dphix_segments(x,y,self.segements)
            #     dphi1x=self.dphix_circle(x,y,self.xcenter,self.ycenter,self.R)               
            #     phi_=self.C_morphx(phi1,phi2,dphi1x,dphi2x,ft2_)
            # elif t>=self.tcut1:
            phi_=self.dphix_segments(x,y,self.segements) 
        if trig==1:
            fx,fy=self.field3(x,y,self.xc1,self.yc1,self.xc2,self.yc2,self.rtilda,self.rho_)
            phi_=-fx
        #if t>self.tcut2 or trig==1:
            #phi_=self.dphix_oval(x,y,self.xcenter_grasp2,self.ycenter_grasp2,0.1,0.1)

        return(phi_)  
    
    
    def FyGraspU(self,x,y,t,trig):
        
        if trig==0:
            #ft_=self.ft2(t,self.tcut1,self.p) 
            #theta=(1-ft_)*np.pi/2 + ft_*0
            theta=np.pi/2
            x_=[]
            y_=[]
            self.lengtho2_grasp = self.lengtho_grasp + 2*self.width_grasp*np.tan((np.pi/2 - theta)/2)
            self.length2_grasp = self.length_grasp +self.width_grasp*np.tan((np.pi/2 - theta)/2)
    
            x_.append(self.length_grasp*np.cos(theta) + self.xcenter_grasp)
            y_.append(self.length_grasp*np.sin(theta) + self.lengtho_grasp/2 + self.ycenter_grasp)
    
            x_.append(0 + self.xcenter_grasp)
            y_.append(self.lengtho_grasp/2 + self.ycenter_grasp)
    
            x_.append(0 + self.xcenter_grasp)
            y_.append(-self.lengtho_grasp/2 + self.ycenter_grasp)
    
            x_.append(self.length_grasp*np.cos(-theta) + self.xcenter_grasp)
            y_.append(self.length_grasp*np.sin(-theta) - self.lengtho_grasp/2 + self.ycenter_grasp)
     
            x_.append(self.length2_grasp*np.cos(-theta) + self.xcenter_grasp - self.width_grasp)
            y_.append(self.length2_grasp*np.sin(-theta) - self.lengtho2_grasp/2 + self.ycenter_grasp)
            
            x_.append(0 + self.xcenter_grasp - self.width_grasp)
            y_.append(-self.lengtho2_grasp/2 + self.ycenter_grasp)
       
            x_.append(0 + self.xcenter_grasp - self.width_grasp)
            y_.append(self.lengtho2_grasp/2 + self.ycenter_grasp)
       
            x_.append(self.length2_grasp*np.cos(theta) - self.width_grasp + self.xcenter_grasp)
            y_.append(self.length2_grasp*np.sin(theta) + self.lengtho2_grasp/2 + self.ycenter_grasp)
       
            x_.append(self.length_grasp*np.cos(theta) + self.xcenter_grasp)
            y_.append(self.length_grasp*np.sin(theta) + self.lengtho_grasp/2 + self.ycenter_grasp)
            
            self.segements=self.create_segment(x_,y_)       
            # if t<self.tcut1:
            #     ft2_=self.ft2(t,0,self.p*3) 
            #     phi2=self.phi_segments(x,y,self.segements)
            #     phi1=self.phi_circle(x,y,self.xcenter,self.ycenter,self.R)
            #     dphi2y=self.dphiy_segments(x,y,self.segements)
            #     dphi1y=self.dphiy_circle(x,y,self.xcenter,self.ycenter,self.R)               
            #     phi_=self.C_morphy(phi1,phi2,dphi1y,dphi2y,ft2_)
            # elif t>=self.tcut1:
            phi_=self.dphiy_segments(x,y,self.segements) 
            
        if trig==1:
            fx,fy=self.field3(x,y,self.xc1,self.yc1,self.xc2,self.yc2,self.rtilda,self.rho_)
            phi_=-fy
        return(phi_)  
      
    def FGraspU(self,x,y,t,trig):
        
        if trig==0:
            #ft_=self.ft2(t,self.tcut1,self.p*3) 
            #theta=(1-ft_)*np.pi/2 + ft_*0
            theta=np.pi/2
            x_=[]
            y_=[]
            self.lengtho2_grasp = self.lengtho_grasp + 2*self.width_grasp*np.tan((np.pi/2 - theta)/2)
            self.length2_grasp = self.length_grasp +self.width_grasp*np.tan((np.pi/2 - theta)/2)
    
            x_.append(self.length_grasp*np.cos(theta) + self.xcenter_grasp)
            y_.append(self.length_grasp*np.sin(theta) + self.lengtho_grasp/2 + self.ycenter_grasp)
    
            x_.append(0 + self.xcenter_grasp)
            y_.append(self.lengtho_grasp/2 + self.ycenter_grasp)
    
            x_.append(0 + self.xcenter_grasp)
            y_.append(-self.lengtho_grasp/2 + self.ycenter_grasp)
    
            x_.append(self.length_grasp*np.cos(-theta) + self.xcenter_grasp)
            y_.append(self.length_grasp*np.sin(-theta) - self.lengtho_grasp/2 + self.ycenter_grasp)
     
            x_.append(self.length2_grasp*np.cos(-theta) + self.xcenter_grasp - self.width_grasp)
            y_.append(self.length2_grasp*np.sin(-theta) - self.lengtho2_grasp/2 + self.ycenter_grasp)
            
            x_.append(0 + self.xcenter_grasp - self.width_grasp)
            y_.append(-self.lengtho2_grasp/2 + self.ycenter_grasp)
       
            x_.append(0 + self.xcenter_grasp - self.width_grasp)
            y_.append(self.lengtho2_grasp/2 + self.ycenter_grasp)
       
            x_.append(self.length2_grasp*np.cos(theta) - self.width_grasp + self.xcenter_grasp)
            y_.append(self.length2_grasp*np.sin(theta) + self.lengtho2_grasp/2 + self.ycenter_grasp)
       
            x_.append(self.length_grasp*np.cos(theta) + self.xcenter_grasp)
            y_.append(self.length_grasp*np.sin(theta) + self.lengtho_grasp/2 + self.ycenter_grasp)
            
            self.segements=self.create_segment(x_,y_)       
            # if t<self.tcut1:
            #     ft2_=self.ft2(t,0,self.p*3) 
            #     phi2=self.phi_segments(x,y,self.segements)
            #     phi1=self.phi_circle(x,y,self.xcenter,self.ycenter,self.R)            
            #     phi_=self.C_morph(phi1,phi2,ft2_)
            # elif t>=self.tcut1:
            phi_=self.phi_segments(x,y,self.segements) 
            
        if trig==1:
            phi_=np.sqrt((x-self.xc2)**2 + (y-self.yc2)**2)
            #phi_=np.sqrt((x-self.xcenter)**2 + (y-self.ycenter)**2)
        return(phi_)  
    
    
    def FGRASP(self,x,y,t):
        if self.tcut1>t:
            phi=self.phi_oval(x,y,self.xc1,self.yc1,self.a1,self.b1)
        if self.tcut1<t:
            phi=self.phi_oval(x,y,self.xc2,self.yc2,self.a2,self.b2)
        return(phi)
    
    def FXGRASP(self,x,y,t):
        if self.tcut1>=t:
            phix=self.dphix_oval(x,y,self.xc1,self.yc1,self.a1,self.b1)
        if self.tcut1<t:
            phix=self.dphix_oval(x,y,self.xc2,self.yc2,self.a2,self.b2)
        return(phix)

    def FYGRASP(self,x,y,t):
        if self.tcut1>=t:
            phiy=self.dphiy_oval(x,y,self.xc1,self.yc1,self.a1,self.b1)
        if self.tcut1<t:
            phiy=self.dphiy_oval(x,y,self.xc2,self.yc2,self.a2,self.b2)
        return(phiy)    
    
    
    def F_object(self,x,y):
        R=3
        theta1=0.13718011
        theta2=2*np.pi-0.13718011
        theta=np.linspace(theta1,theta2,100)
        x1_=-R*np.cos(theta)
        y1_=R*np.sin(theta)
        
        
    
    def FX(self,x,y):
        """ Single function to call on derivative wrt x"""
        if self.geometry=='circle':
            Fx = self.dphix_circle(x,y,self.a,self.b,self.R)
            
            
        if self.geometry=='pacman':
            f1 = self.phi_quarter_circle(x,y,self.a,self.b,self.R)
            f1x = self.dphix_quarter_circle(x,y,self.a,self.b,self.R)
            
            f2 = self.phi_segments(x,y,self.segments)
            f2x = self.dphix_segments(x,y,self.segments)
            
            Fx = (f1**self.m + f2**self.m)**(-1/self.m)*f1*f2x + (f1**self.m + f2**self.m)**(-1/self.m)*f2*f1x - ((self.m*(f2**self.m)*f2x)/f2 + (self.m*(f1**self.m)*f1x)/f1)*((f1**self.m) +f2**self.m)**(-1/self.m)*f1*f2/(self.m*(f1**self.m + f2**self.m))
            
        else:
            Fx = self.dphix_segments(x,y,self.segments)
    
        return(Fx)


    def FY(self,x,y):
        """ Single function to call on derivative wrt y"""
        if self.geometry=='circle':
            Fy = self.dphiy_circle(x,y,self.a,self.b,self.R)
            
        if self.geometry=='pacman':
            f1 = self.phi_quarter_circle(x,y,self.a,self.b,self.R)
            f1y = self.dphiy_quarter_circle(x,y,self.a,self.b,self.R)
        
            f2 = self.phi_segments(x,y,self.segments)
            f2y = self.dphiy_segments(x,y,self.segments)
            
            Fy = (f1**self.m + f2**self.m)**(-1/self.m)*f1*f2y + (f1**self.m + f2**self.m)**(-1/self.m)*f2*f1y - ((self.m*(f2**self.m)*f2y)/f2 + (self.m*(f1**self.m)*f1y)/f1)*((f1**self.m) +f2**self.m)**(-1/self.m)*f1*f2/(self.m*(f1**self.m +f2**self.m))
            
        else:
            Fy = self.dphiy_segments(x,y,self.segments)
    
        return(Fy)    
    
    
    def F(self,x,y):
       """ Single function to call field"""
       if self.geometry=='circle':
           F = self.phi_circle(x,y,self.a,self.b,self.R)
            
       if self.geometry=='pacman':
           f = self.phi_quarter_circle(x,y,self.a,self.b,self.R)
           f2=self.phi_segments(x,y,self.segments)
           F = (f*f2)/((f**self.m +f2**self.m)**(1/self.m))
           
         
       else:
           F = self.phi_segments(x,y,self.segments)
           
       return(F)

       
##################################################################################    
    
#### MORPHING FUNCTIONS
   
    def F_morph(self,x,y,ft):
        phi1 = self.F1(x,y)
        phi2 = self.F2(x,y)
        #print(phi1,phi2,ft)     
        F = self.C_morph(phi1,phi2,ft)
        return(F)
    

    def FX_morph(self,x,y,ft):
        """ Control for x component for morphing """
        phi1 = self.F1(x,y)
        phi2 = self.F2(x,y)
        
        dphi1x = self.F1X(x,y)
        dphi2x = self.F2X(x,y)
        
        Fx = self.C_morphx(phi1,phi2,dphi1x,dphi2x,ft)
        
        
        return(Fx)

    def FY_morph(self,x,y,ft):
        """ Control for y component for morphing """
        phi1 = self.F1(x,y)
        phi2 = self.F2(x,y)
        
        dphi1y = self.F1Y(x,y)
        dphi2y = self.F2Y(x,y)
        
        Fy = self.C_morphy(phi1,phi2,dphi1y,dphi2y,ft)
        
    
        return(Fy)
    
    

    def F1(self,x,y):
        """ Field of the initial starting field """
        if self.geometry1=='circle':
            F1 = self.phi_circle(x,y,self.a,self.b,self.R)
        else:
            F1 = self.dphix_segments(x,y,self.segments1)
    
        return(F1)        
        
    def F2(self,x,y):
        """ Field of the desired  field """
        if self.geometry2=='circle':
            F2 = self.phi_circle(x,y,self.a,self.b,self.R)
        else:
            F2 = self.phi_segments(x,y,self.segments)
    
        return(F2)      


    def F1X(self,x,y):
        """ Single function to call on derivative wrt x"""
        if self.geometry1=='circle':
            Fx = self.dphix_circle(x,y,self.a,self.b,self.R)
        else:
            Fx = self.dphix_segments(x,y,self.segments)
    
        return(Fx)


    def F1Y(self,x,y):
        """ Single function to call on derivative wrt y"""
        if self.geometry1=='circle':
            Fy = self.dphiy_circle(x,y,self.a,self.b,self.R)
        else:
            Fy = self.dphiy_segments(x,y,self.segments)
    
        return(Fy)    


    def F2X(self,x,y):
        """ Single function to call on derivative wrt x"""
        if self.geometry2=='circle':
            Fx = self.dphix_circle(x,y,self.a,self.b,self.R)
        else:
            Fx = self.dphix_segments(x,y,self.segments)
    
        return(Fx)


    def F2Y(self,x,y):
        """ Single function to call on derivative wrt y"""
        if self.geometry2=='circle':
            Fy = self.dphiy_circle(x,y,self.a,self.b,self.R)
        else:
            Fy = self.dphiy_segments(x,y,self.segments)
    
        return(Fy)  


    def tanh(self,t):
        """ tanh function """
        tanh=(np.exp(self.p*(t))-1)/(np.exp(self.p*(t))+1)
        #print('tanh=',tanh)
        return(tanh)   
    
    def ft2(self,t,tact,p):
        y=p*(t-tact)
        yp=np.maximum(0,y)
        ypp=np.minimum(1,yp)
        return(ypp)
    
    
    def g1(self,phi1,t):
        """ intersection of initial field and -f(t) """
        return(phi1 - t - np.sqrt(phi1**2 + t**2))
        
    def g2(self,phi2,t):
        """ intersection of final field and f(t)-1 """
        return(phi2 + (t-1) - np.sqrt(phi2**2+(t-1)**2))
    
    
    def dgx(self,phi,dphix,s):
        """ derivative of g1 or g2 wrt x """
        # s is either (t) or t-1
        return(dphix-(phi*dphix)/(np.sqrt(s**2 +phi**2)))
    
    
    def dgy(self,phi,dphiy,s):
        """ derivative of g1 or g2 wrt y """
        return(dphiy-(phi*dphiy)/(np.sqrt(s**2 +phi**2)))    
    
    
    def w1(self,g1,g2): 
        """ Weighted function 1 """
        return(g2/(g1+g2))
    
    
    def w2(self,g1,g2):
        """ Weighted function 2 """
        return(g1/(g1+g2))    
    
  
    def dw1x(self,dg1x,dg2x,g1,g2):
        """ derivative of weighted function 1 wrt x """
        return((dg2x) / (g1+g2) - (dg1x+dg2x)*g2 / (g1+g2)**2)
    
    
    def dw2x(self,dg1x,dg2x,g1,g2):
        """ derivative of weighted function 2 wrt x """
        return((dg1x) / (g1+g2) - (dg1x+dg2x)*g1 / (g1+g2)**2)
    
    
    def dw1y(self,dg1y,dg2y,g1,g2):
        """ derivative of weighted function 1 wrt y """
        return((dg2y) / (g1+g2) - (dg1y+dg2y)*g2 / (g1+g2)**2)
    
    
    def dw2y(self,dg1y,dg2y,g1,g2):
        """ derivative of weighted function 2 wrt y """
        return((dg1y) / (g1+g2) - (dg1y+dg2y)*g1 / (g1+g2)**2)  
    
  

    def C_morph(self,phi1,phi2,ft):
        """ Morphing function from phi1 to phi2 """
        #ft=self.tanh(t)
        G1 = self.g1(phi1,ft)
        G2 = self.g2(phi2,ft)
        W1 = self.w1(G1,G2)
        W2 = self.w2(G1,G2)
        
        return(W1*phi1+W2*phi2)
    
    

    def C_morphx(self,phi1,phi2,dphi1x,dphi2x,ft):
        """ Derivative Morphing function from phi1 to phi2 wrt x """
        #ft=self.tanh(t)
        G1 = self.g1(phi1,ft)
        G2 = self.g2(phi2,ft)
        
        dg1x = self.dgx(phi1,dphi1x,ft)
        dg2x = self.dgx(phi2,dphi2x,ft-1)
        W1 = self.w1(G1,G2)
        W2 = self.w2(G1,G2)
        DW1X = self.dw1x(dg1x,dg2x,G1,G2)
        DW2X = self.dw2x(dg1x,dg2x,G1,G2)
        
        return(phi1*DW1X + phi2*DW2X + W1*dphi1x + W2*dphi2x)
    
    def C_morphy(self,phi1,phi2,dphi1y,dphi2y,ft):
        """ Derivative Morphing function from phi1 to phi2 wrt y """
        #ft=self.tanh(t)
        G1 = self.g1(phi1,ft)
        G2 = self.g2(phi2,ft)
        
        dg1y = self.dgy(phi1,dphi1y,ft)
        dg2y = self.dgy(phi2,dphi2y,ft-1)
    
        W1 = self.w1(G1,G2)
        W2 = self.w2(G1,G2)
        
        DW1Y = self.dw1y(dg1y,dg2y,G1,G2)
        DW2Y = self.dw2y(dg1y,dg2y,G1,G2)
        
        return(phi1*DW1Y + phi2*DW2Y + W1*dphi1y + W2*dphi2y)


    def dpoint_(self,x,y,xc,yc):
        return((np.sqrt((x-xc)**2 + (y-yc)**2)))

    def dxpoint_(self,x,y,xc,yc):
        return((x-xc)/(np.sqrt((x-xc)**2 + (y-yc)**2)))

    def dypoint_(self,x,y,xc,yc):
        return((y-yc)/(np.sqrt((x-xc)**2 + (y-yc)**2)))

    def F_point(self,x,y,xc,yc):
        d=self.dpoint_(x,y,xc,yc)
        return(0.5*d**2)

    def Fx_point(self,x,y,xc,yc):
        dx=self.dxpoint_(x,y,xc,yc)
        d=self.dpoint_(x,y,xc,yc)
        return(dx*d)

    def Fy_point(self,x,y,xc,yc):
        dy=self.dypoint_(x,y,xc,yc)
        d=self.dpoint_(x,y,xc,yc)
        return(dy*d)

    def create_segment(self,x,y):
        """ Create segment matrix for points of R-function """
        seglen=len(x)
        segments=np.zeros((seglen-1,4))
        for i in range(seglen-1):
            #[x1,y1,x2,y2]
            #[x2,y2,x3,y3]
            segments[i,0]=x[i]
            segments[i,1]=y[i]
            segments[i,2]=x[i+1]
            segments[i,3]=y[i+1]
        return(segments)        
 
    
    def plot_zero_contours(self,xp1,yp1,xp2,yp2,d):
        xticks = np.linspace(-d, d,5,endpoint=True)
        fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3,3))
        fig.subplots_adjust(left=0.05,bottom=0.1,right=.95,top=0.9,wspace=.3,hspace=1)         
        axs.plot(xp1,yp1,color='k',linewidth=3)
        axs.plot(xp2,yp2,color='tab:red',linewidth=3)
        
        
    def plot_R_function(self,X,Y,R,Rx,Ry,d):
        """ Plot R-function and its derivatives """
        xticks = np.linspace(-d, d,5,endpoint=True)
        fig, axs = plt.subplots(nrows=1, ncols=3,figsize=(10,3))
        fig.subplots_adjust(left=0.05,bottom=0.1,right=.95,top=0.9,wspace=.3,hspace=1)
        
        # Plot Phi
        im1=axs[0].contourf(X, Y,R,cmap = 'jet',levels=50,alpha=1,linestyles='solid') 
        axs[0].contour(X,Y,R,levels = 50,colors=('k',),linestyles=('-',),linewidths=(.1,))
        #axs[0].plot(xp,yp,color='k',linewidth=3)
        axs[0].set_title('$\phi(x)$')
        axs[0].set_xticks(xticks)
        axs[0].set_yticks(xticks)
        fig.colorbar(im1, ax=axs[0])
        
        # Plot Phix
        im2=axs[1].contourf(X, Y,Rx,cmap = 'jet',levels=50,alpha=1,linestyles='solid') 
        #axs[1].plot(xp,yp,color='k',linewidth=3)
        axs[1].contour(X,Y,Rx,levels = 50,colors=('k',),linestyles=('-',),linewidths=(.1,))
        axs[1].set_xticks(xticks)
        axs[1].set_yticks(xticks)
        axs[1].set_title(r"$\frac{\partial \phi}{\partial x}$")
        fig.colorbar(im2, ax=axs[1])
        
        # Plot Phiy
        im3=axs[2].contourf(X,Y,Ry,cmap = 'jet',levels=50,alpha=1,linestyles='solid') 
        #axs[2].plot(xp,yp,color='k',linewidth=3)
        axs[2].contour(X,Y,Ry,levels = 50,colors=('k',),linestyles=('-',),linewidths=(.1,))
        axs[2].set_xticks(xticks)
        axs[2].set_yticks(xticks)
        axs[2].set_title(r"$\frac{\partial \phi}{\partial y}$")
        fig.colorbar(im3, ax=axs[2])
         
        
        
    def plot_R_function_morph(self,X,Y,d,phi1,phi2,dphi1x,dphi2x,dphi1y,dphi2y,nr,nc,t):        
        
        C=[]
        CX=[]
        CY=[]
        for i in range(len(t)):
            C.append(self.C_morph(phi1,phi2,t[i]))
            CX.append(self.C_morphx(phi1,phi2,dphi1x,dphi2x,t[i]))
            CY.append(self.C_morphy(phi1,phi2,dphi1y,dphi2y,t[i]))
            
        fig, axs = plt.subplots(nrows=nr, ncols=nc,figsize=(6,6),dpi=150)
        count=0
        xticks=np.linspace(-d,d,5)
        yticks=xticks
        for i in range(nr):
            for j in range(nc):
                #print(np.min(C[count]))
                CS1 = axs[i,j].contour(X,Y,CX[count],levels = [0],colors=('tab:red'),linestyles=('-',),linewidths=(3,))
                CS2 = axs[i,j].contour(X,Y,CY[count],levels = [0],colors=('tab:blue'),linestyles=('--',),linewidths=(3,))
                #(xi,yi)=find_intersection(CS1,CS2)
                #axs[i,j].plot(xi,yi,'ko', ms=3)
                #CS2 = axs[i,j].contour(X,Y,CX[count],5,colors='black',zorder=0)
                #axs[i,j].set_xlim([-d,d])
                #axs[i,j].set_ylim([-d,d])
                axs[i,j].set_title(str(np.round(t[count],3)),fontsize=10,fontname="Arial")
                axs[i,j].set_xticks(xticks)
                axs[i,j].set_yticks(yticks)
                count=count+1
        fig.tight_layout()
        
        
        
    def plot_R_function_morph_color(self,X,Y,d,phi1,phi2,dphi1x,dphi2x,dphi1y,dphi2y,nr,nc,t):        
        
        C=[]
        CX=[]
        CY=[]
        for i in range(len(t)):
            C.append(self.C_morph(phi1,phi2,t[i]))
            CX.append(self.C_morphx(phi1,phi2,dphi1x,dphi2x,t[i]))
            CY.append(self.C_morphy(phi1,phi2,dphi1x,dphi2x,t[i]))
            
        fig, axs = plt.subplots(nrows=nr, ncols=nc,figsize=(6,6),dpi=150)
        count=0
        xticks=np.linspace(-d,d,5)
        yticks=xticks        
        for i in range(nr):
            for j in range(nc):
                print(np.min(CX[count]))
                #axs[i,j].axis('equal')
                #CS1 = axs[i,j].contour(X,Y,C[count],linewidths=(1,))
                im1=axs[i,j].contourf(X, Y,C[count],cmap = 'jet',levels=50,alpha=1,linestyles='solid')  
                axs[i,j].set_title(str(np.round(t[count],3)),fontsize=10,fontname="Arial")
                axs[i,j].set_xticks(xticks)
                axs[i,j].set_yticks(yticks)
                count=count+1
                fig.colorbar(im1, ax=axs[i,j])
        plt.tight_layout()        



def plot_line(xp,yp):  
    fm._rebuild()
    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['mathtext.fontset'] = 'dejavuserif'
    plt.rcParams['font.size'] = 6
    plt.rcParams['axes.linewidth'] = .1
    #xticks = np.linspace(-d, d,5,endpoint=True)
    fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(10,3))
    fig.subplots_adjust(left=0.05,bottom=0.1,right=.95,top=0.9,wspace=.3,hspace=1)
    axs.plot(xp,yp,linewidth=1,color='k')
        
        
def shoelace(vertices):
    """ Find the cross sectional area"""
    (m,n)=np.shape(vertices)

    sum1=vertices[0,0]*(vertices[1,1]-vertices[1,n-1])
    for i in range(1,n-1):
        sum1=sum1 + vertices[0,i]*(vertices[1,i+1]-vertices[1,i-1])
    i=n-1
    sum1=sum1 + vertices[0,i]*(vertices[1,0]-vertices[1,i-1])

    A=.5*abs(sum1)
    return (A)            


def create_segment(x,y):
    """ Create segment matrix for points of R-function """
    seglen=len(x)
    segments=np.zeros((seglen-1,4))
    for i in range(seglen-1):
        #[x1,y1,x2,y2]
        #[x2,y2,x3,y3]
        segments[i,0]=x[i]
        segments[i,1]=y[i]
        segments[i,2]=x[i+1]
        segments[i,3]=y[i+1]
    return(segments)     

         
class import_data:
     def __init__(self,name,path,wxmin,wxmax,wymin,wymax,Psi=None):
         self.name=name
         self.path=path
         self.mainDirectory = path   # main directory 
         parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
         #print(parameters)
         data=np.load(self.mainDirectory+self.name+'/Radii'+self.name+'.npz',allow_pickle=True) 
         self.Rm=data['Rm'] 
         self.wxmin = wxmin 
         self.wxmax = wxmax
         self.wymin = wymin
         self.wymax = wymax
         self.delta = .1
         self.xx = np.arange(self.wxmin,self.wxmax,self.delta)
         self.yy = np.arange(self.wymin,self.wymax,self.delta)
         self.X,self.Y = np.meshgrid(self.xx,self.yy)
         d=1
         self.m=8
         self.wxmin2 = -d
         self.wxmax2 = d
         self.wymin2 = -d
         self.wymax2 = d
         self.epsilon_tilda_ = []
         self.Psi=Psi
         self.parameters=parameters.tolist()    # loads saved parameters      
         #print(self.parameters)
         self.nb=self.parameters['nb'] # number of bots
         self.ni=self.parameters['total_particles']
         self.ns=self.parameters['ns']
         self.nm=self.nb*self.ns # total membrane particles 
         self.bot_width=self.parameters['bot_width']
         #self.geom = self.parameters['ball_geometry']
         self.particle_width=self.parameters['particle_width']
         self.particle_width2=self.parameters['particle_width2']
         self.radius2=self.particle_width/2 # radius of particles
         self.radius3=self.particle_width2/2 # radius of particles
         self.control_mode=self.parameters['control_mode']
         self.skin_width=self.parameters['skin_width']
         self.height=self.parameters['bot_height']
         if self.control_mode=="target_chasing":
             self.tunnel_geom = self.parameters['tunnel_geom']
             self.xtarget = self.parameters['xc']
             self.ytarget = self.parameters['yc']
         #self.geom = self.parameters['ball_geometry']
         
         
         
         # if control mode is shape formation
         if self.control_mode=="shape_formation":
             self.geometry = self.parameters['geometry']
             data2=np.load(self.mainDirectory+self.name+'/outline'+self.geometry+'.npz',allow_pickle=True)
             self.xp=data2['xp']
             self.yp=data2['yp']
         
             
         # if control mode is shape morphing
         if self.control_mode=="shape_morphing":
             self.geometry1=self.parameters['geometry1']
             self.geometry2=self.parameters['geometry2']
             data2=np.load(self.mainDirectory+self.name+'/shapes'+self.geometry2+'.npz',allow_pickle=True)
             self.xp=data2['xp']
             self.yp=data2['yp']
         
         # if control mode is grasping  
         if self.control_mode=="grasping" or self.control_mode=="grasping_explore" :
             self.a2 = self.parameters['a2']
             self.b2 = self.parameters['b2']
             self.xc2 = self.parameters['xc2']     
             self.yc2 = self.parameters['yc2'] 
             self.ballx_ = self.parameters['ballx']
             self.ballz_ = self.parameters['ballz']
         self.path=self.path+self.name+"/results/"
         os.chdir(self.path)
         self.files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)
         
         
         #### Robot Position
         self.bot_position=np.genfromtxt(self.files[self.files.index('bot_position.csv') ] ,delimiter=',')
         (self.m1,self.n1)=np.shape(self.bot_position)
         self.bot_position=self.bot_position[:,1:self.n1]
         self.time=self.bot_position[0,:]
         self.bot_position_x=self.bot_position[1:self.nb+1,:]
         self.bot_position_y=self.bot_position[self.nb+1:2*self.nb+1,:]
         self.bot_position_z=self.bot_position[(2*self.nb)+1:3*self.nb+1,:] 

         #### robot contact forces
         self.bot_contact_forces=np.genfromtxt(self.files[self.files.index('bot_contact_forces.csv') ] ,delimiter=',')
         (self.m1,self.n1)=np.shape(self.bot_contact_forces)
         self.bot_contact_forces=self.bot_contact_forces[:,1:self.n1]
         self.bot_contact_forces_x=self.bot_contact_forces[1:self.nb+1,:]
         self.bot_contact_forces_y=self.bot_contact_forces[self.nb+1:2*self.nb+1,:]
         self.bot_contact_forces_z=self.bot_contact_forces[(2*self.nb)+1:3*self.nb+1,:]          


         #### robot total forces
         self.bot_total_forces=np.genfromtxt(self.files[self.files.index('bot_total_forces.csv') ] ,delimiter=',')
         (self.m1,self.n1)=np.shape(self.bot_total_forces)
         self.bot_total_forces=self.bot_total_forces[:,1:self.n1]
         self.bot_total_forces_x=self.bot_total_forces[1:self.nb+1,:]
         self.bot_total_forces_y=self.bot_total_forces[self.nb+1:2*self.nb+1,:]
         self.bot_total_forces_z=self.bot_total_forces[(2*self.nb)+1:3*self.nb+1,:]          


         #### membrane_positions
         self.membrane_position=np.genfromtxt(self.files[self.files.index('membrane_position.csv') ] ,delimiter=',')
         (m,n)=np.shape(self.membrane_position)
         self.membrane_position=self.membrane_position[:,1:n]
         self.membrane_position_x=self.membrane_position[1:self.nm+1,:]
         self.membrane_position_y=self.membrane_position[self.nm+1:2*self.nm+1,:]
         self.membrane_position_z=self.membrane_position[(2*self.nm)+1:3*self.nm+1,:]          
         
         
         
         #### control forces
         self.control_forces=np.genfromtxt(self.files[self.files.index('control_forces.csv') ] ,delimiter=',')
         (m,n)=np.shape(self.control_forces)
         self.control_forces=self.control_forces[:,1:n]
         self.control_forces_x=self.control_forces[1:self.nb+1,:]
         self.control_forces_z=self.control_forces[self.nb+1:2*self.nb+1,:]
       
        
         if self.control_mode=="grasping" or self.control_mode=="grasping_u":
             #### Ball contact forces
             # These were saved directly from chrono.; It calculuaedc the sum of contact forces on the ball
             self.ball_contact_forces=np.genfromtxt(self.files[self.files.index('ball_contact_forces.csv') ] ,delimiter=',')
             (m,n)=np.shape(self.ball_contact_forces)
             self.ball_contact_forces=self.ball_contact_forces[:,1:n]
             self.bFx=self.ball_contact_forces[1,:]
             self.bFy=self.ball_contact_forces[2,:]
             self.ballx_ = self.parameters['ballx']
             self.ballz_ = self.parameters['ballz']
             #self.bFz=self.ball_contact_forces[3,:]
         
         
         
         
         #### Potential Field values    
         #self.Field_value=np.genfromtxt(self.files[self.files.index('field_values.csv') ] ,delimiter=',')
         # (m,n)=np.shape(self.Field_value)
         # self.Field_value=self.Field_value[:,1:n]
         # self.Field_value_sum=[]
         # for i in range(len(self.time)):
         #     self.Field_value_sum.append(np.sum(abs(self.Field_value[:,i])))
         
         #### Particle Position    
         if self.ni==0:
             pass
         else:
             self.particle_position=np.genfromtxt(self.files[self.files.index('particle_position.csv') ] ,delimiter=',')
             (self.m4a,self.n4a)=np.shape(self.particle_position)
             self.particle_position=self.particle_position[:,1:self.n4a]
             self.particle_position_x=self.particle_position[1:self.ni+1,:]
             self.particle_position_y=self.particle_position[self.ni+1:2*self.ni+1,:]
             self.particle_position_z=self.particle_position[(2*self.ni)+1:3*self.ni+1,:]
         
             #### particle contact forces
             #these were calulculated by chrono
             self.particle_contact_forces=np.genfromtxt(self.files[self.files.index('particle_contact_forces.csv') ] ,delimiter=',')
             (self.m1,self.n1)=np.shape(self.particle_contact_forces)
             self.particle_contact_forces=self.particle_contact_forces[:,1:self.n1]
             self.particle_contact_forces_x=self.particle_contact_forces[1:self.ni+1,:]
             self.particle_contact_forces_y=self.particle_contact_forces[self.ni+1:2*self.ni+1,:]
             self.particle_contact_forces_z=self.particle_contact_forces[(2*self.ni)+1:3*self.ni+1,:]          


             #### particle total forces 
             self.particle_total_forces=np.genfromtxt(self.files[self.files.index('particle_total_forces.csv') ] ,delimiter=',')
             (self.m1,self.n1)=np.shape(self.particle_total_forces)
             self.particle_total_forces=self.particle_total_forces[:,1:self.n1]
             self.particle_total_forces_x=self.particle_total_forces[1:self.ni+1,:]
             self.particle_total_forces_y=self.particle_total_forces[self.ni+1:2*self.ni+1,:]
             self.particle_total_forces_z=self.particle_total_forces[(2*self.ni)+1:3*self.ni+1,:] 
         
         
         
         #### Contact Points and forces 
         # These were collected from chrono.
         self.time_contact = np.genfromtxt(self.files[self.files.index('time_contact.csv') ] ,delimiter=',')
         self.number_contacts = np.genfromtxt(self.files[self.files.index('number_contacts.csv') ] ,delimiter=',')
        
         #### Contact points 
         self.Contact_points_x = np.genfromtxt(self.files[self.files.index('x_contact_points.csv') ] ,delimiter=',')
         self.Contact_points_y = np.genfromtxt(self.files[self.files.index('y_contact_points.csv') ] ,delimiter=',')
         self.Contact_points_z = np.genfromtxt(self.files[self.files.index('z_contact_points.csv') ] ,delimiter=',')
        
         #### Contact forces
         # These were the ones in refenece to the local frame of the contact
         self.Contact_force_x = np.genfromtxt(self.files[self.files.index('x_contact_force.csv') ] ,delimiter=',')
         self.Contact_force_y = np.genfromtxt(self.files[self.files.index('y_contact_force.csv') ] ,delimiter=',')
         self.Contact_force_z = np.genfromtxt(self.files[self.files.index('z_contact_force.csv') ] ,delimiter=',') 
         
         #### Contact forces 2
         # These were the ones in refenece to the global frame 
         self.x_contact_force2=np.genfromtxt(self.files[self.files.index('x_contact_force2.csv') ] ,delimiter=',') 
         self.y_contact_force2=np.genfromtxt(self.files[self.files.index('y_contact_force2.csv') ] ,delimiter=',') 
         self.z_contact_force2=np.genfromtxt(self.files[self.files.index('z_contact_force2.csv') ] ,delimiter=',') 
        
         #### Contact body ID's
         # These were the ID's of the bodys that were in contact 
         self.AID = np.genfromtxt(self.files[self.files.index('AID.csv') ] ,delimiter=',')  # Body A
         self.BID = np.genfromtxt(self.files[self.files.index('BID.csv') ] ,delimiter=',')  # Body B
         
         # We are now sorting them into lists. Its makes it easier to access. 
         self.AN=[] # empty array of contact ID A
         self.BN=[] # empty array of contact ID B
         infile = open(self.files[self.files.index('AN.csv') ], 'r') # now we fill the array AN
         for row in csv.reader(infile):
             self.AN.append(row[1:])
         infile = open(self.files[self.files.index('BN.csv') ], 'r') # now we fill the array BN
         for row in csv.reader(infile):
             self.BN.append(row[1:])         
         
         #### Contact frame dir x
         # These are the global vecotrs of the x direction of the contact frame 
         self.Dirxx_=np.genfromtxt(self.files[self.files.index('contact_dirxx.csv') ] ,delimiter=',') 
         self.Dirxy_=np.genfromtxt(self.files[self.files.index('contact_dirxy.csv') ] ,delimiter=',') 
         self.Dirxz_=np.genfromtxt(self.files[self.files.index('contact_dirxz.csv') ] ,delimiter=',') 
         
         #### Contact frame dir y
         # These are the global vecotrs of the y direction of the contact frame 
         self.Diryx_=np.genfromtxt(self.files[self.files.index('contact_diryx.csv') ] ,delimiter=',') 
         self.Diryy_=np.genfromtxt(self.files[self.files.index('contact_diryy.csv') ] ,delimiter=',') 
         self.Diryz_=np.genfromtxt(self.files[self.files.index('contact_diryz.csv') ] ,delimiter=',') 

         #### Contact frame dir z
         # These are the global vecotrs of the z direction of the contact frame 
         self.Dirzx_=np.genfromtxt(self.files[self.files.index('contact_dirzx.csv') ] ,delimiter=',') 
         self.Dirzy_=np.genfromtxt(self.files[self.files.index('contact_dirzy.csv') ] ,delimiter=',') 
         self.Dirzz_=np.genfromtxt(self.files[self.files.index('contact_dirzz.csv') ] ,delimiter=',')       
        
         self.MAG_pressure=np.zeros((self.nb+self.ni,len(self.time)-1))
         self.MAG_pressure_no_boundary=np.zeros((self.ni,len(self.time)-1))
         
         self.Mag_avg_pressure=[]
         self.Mag_avg_pressure_no_boundary=[]
         #### If the control modeis set for grasping. 
         if self.control_mode=="grasping" or self.control_mode=="grasping_explore" or self.control_mode=="grasping_u" or self.control_mode=="grasping_epsilon" :
             self.geom = self.parameters['ball_geometry'] 
             self.ball_radius = self.parameters['ball_radius']
             self.tcut1=self.parameters['tcut1']
             self.tcut2=self.parameters['tcut2']
             self.tcut3=self.parameters['tcut3']
             if self.geom=="c_shape":
                 self.w=self.parameters['w']
                 self.l=self.parameters['l']
                 self.t=self.parameters['t']
            
             if self.control_mode=="grasping_epsilon" or self.control_mode=="grasping_u":
                 self.epsilon_tilda=np.genfromtxt(self.files[self.files.index('epsilon_tilda.csv') ] ,delimiter=',')
                 self.time_tilda=np.genfromtxt(self.files[self.files.index('time_tilda.csv')] ,delimiter=',')
                 self.xc2=self.parameters['xc2']
                 self.yc2=self.parameters['yc2']
                 self.tcut1 = self.parameters['tcut1']
                 self.tcut2 = self.parameters['tcut2']
                 self.tcut3 = self.parameters['tcut3']
                 
             if self.control_mode=="grasping_u":
                self.lengtho_grasp=self.parameters["lengtho_grasp"]
                self.length_grasp=self.parameters["length_grasp"]
                self.width_grasp=self.parameters["width_grasp"]
                self.xcenter_grasp=self.parameters["xcenter_grasp"]
                self.ycenter_grasp=self.parameters["ycenter_grasp"]
                self.xcenter_grasp2=self.parameters["xcenter_grasp2"]
                self.ycenter_grasp2=self.parameters["ycenter_grasp2"]
                self.p=self.parameters["p"]
                self.xc1 = self.parameters['xc1']     
                self.yc1 = self.parameters['yc1']
                self.xc2 = self.parameters['xc2']     
                self.yc2 = self.parameters['yc2']   
                self.rho_ = self.parameters['rho_']
                self.rtilda = self.parameters['rtilda']
                
                
                self.Field_value=np.genfromtxt(self.files[self.files.index('field_values.csv') ] ,delimiter=',')
                (m,n)=np.shape(self.Field_value)
                self.Field_value=self.Field_value[1:m,1:n]
                self.TRIG1_=np.genfromtxt(self.files[self.files.index('TRIG1_.csv')] ,delimiter=',')
                self.Field_value_sum=[]
                for i in range(len(self.time)):
                    self.Field_value_sum.append(np.sum(abs(self.Field_value[:,i])))
         
             if self.control_mode=="grasping_epsilon":
                 self.xjam=np.genfromtxt(self.files[self.files.index('xjam.csv')] ,delimiter=',')  
                 self.yjam=np.genfromtxt(self.files[self.files.index('yjam.csv')] ,delimiter=',') 
                 self.nojam=self.parameters['nojam']
                 
             self.mu = self.parameters['lateralFriction']
             self.ballx = self.parameters['ballx'] 
             self.ballz = self.parameters['ballz'] 
             #self.xc2 = self.parameters['xc2']
             #self.yc2 = self.parameters['yc2']
             const=self.ball_radius*2*np.pi/4
             rx=const
             ry=const
             w=rx
             h=ry
             xcenter=self.ballx
             ycenter=self.ballz
             x=[w,-w,-w,w,w]
             y=[h,h,-h,-h,h]
             self.m=8
             x=x + xcenter*np.ones(len(x))
             y=y + ycenter*np.ones(len(x))
             (self.segments)=self.create_segment(x,y)             
             if self.geom=="square":
                 self.k=(self.ball_radius*2)/(2*np.sqrt(3))
             
             if self.geom=="circle":
                 self.k = self.ball_radius/np.sqrt(2)   
                            
             if self.geom=="triangle":
                 const=self.ball_radius*2*np.pi/3
                 #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                 r=const*np.sqrt(3)/3
                 x1=r
                 y1=0
                 x2=r*np.cos(2*np.pi/3)
                 y2=r*np.sin(2*np.pi/3)
                 d=np.sqrt((x1-x2)**2 + (y1-y2)**2)
                 self.k = d*.2041 
             
             if self.geom=="import":
                 self.k=2.18
            
            
             #### EPSILON CALCULATED 
             self.EPSILON_ = np.genfromtxt(self.files[self.files.index('EPSILON.csv') ] ,delimiter=',')  
             #### CALCULATION TIME
             self.calcultation_time  = np.genfromtxt(self.files[self.files.index('calcultation_time.csv') ] ,delimiter=',')
             
             
             #### RHO
             self.rho = self.parameters['rho'] 
             
             #### Qcm
             self.Qcm = np.genfromtxt(self.files[self.files.index('Qcm.csv') ] ,delimiter=',')
             
             #### xcentroid
             self.centroidx = np.genfromtxt(self.files[self.files.index('centroidx.csv') ] ,delimiter=',')
             self.centroidz = np.genfromtxt(self.files[self.files.index('centroidz.csv') ] ,delimiter=',')
             
             #### Ball position 
             self.ball_position=np.genfromtxt(self.files[self.files.index('ball_position.csv') ] ,delimiter=',')            
             m,n=np.shape(self.ball_position)
             self.ball_position=self.ball_position[:,1:n]
             self.ball_position=self.ball_position[1:m,:]
             self.ballx_position=self.ball_position[0,:]
             self.ballz_position=self.ball_position[1,:]
             
             #### ball velocity
             self.ball_velocity=np.genfromtxt(self.files[self.files.index('ball_velocity.csv') ] ,delimiter=',')
             (self.m1,self.n1)=np.shape(self.ball_velocity)
             self.ball_velocity_x=self.ball_velocity[1,:]
             self.ball_velocity_z=self.ball_velocity[2,:]                      
             
             self.F_control = []
             self.Forces_ball_x = []
             self.Forces_ball_z = []
             
             self.contact_points_ball_x = []
             self.contact_points_ball_z = []
             
             self.magnitude_forces_on_ball = []  
             self.torque_ball = []

             self.THETA=np.genfromtxt(self.files[self.files.index('THETA.csv') ] ,delimiter=',')
             self.Rr_=np.genfromtxt(self.files[self.files.index('Rr_.csv') ] ,delimiter=',')
             self.angle_entries=[]
             self.epsilon_section={}
             self.epsilon_theta_section={}
             
             self.average_epsilon=[]
             self.max_epsilon=[]
             
             self.calc_type=[]
             # PRESSURE empty arrays
             self.Pressure_x_bots = np.zeros((self.nb,len(self.time_contact))) # empty array for the pressure of each bot in the x direction 
             self.Pressure_z_bots = np.zeros((self.nb,len(self.time_contact))) # empty array for the pressure of each bot in the z direction
             
             self.Pressure_x_particles = np.zeros((self.ni,len(self.time_contact))) # empty array for the pressure of each particle in the x direction 
             self.Pressure_z_particles = np.zeros((self.ni,len(self.time_contact))) # empty array for the pressure of each particle in the z direction 


             
             self.Forces_x_contact_bots = np.zeros((self.nb,len(self.time_contact)))
             self.Forces_z_contact_bots = np.zeros((self.nb,len(self.time_contact)))


             #### EMPTY ARRAYs FOR CALCULATING ALL THE CONTACT FORCES ####
             
             # empty list for contact forces on particle
             self.Forces_x_contact_particles={}
             self.Forces_z_contact_particles={}
             
             # empty list for contact forces on boundary robots
             self.Forces_x_contact_bots={}
             self.Forces_z_contact_bots={}    
        
             # empty list for contact forces on ball
             self.Force_x_contact_ball={} 
             self.Force_z_contact_ball={}
        
             # empty list for contact positions on ball
             self.position_x_contact_ball={}
             self.position_z_contact_ball={}
        
             # empty list for contact directions on ball x direction
             self.dir_xx_contact_ball={}
             self.dir_xz_contact_ball={}
             
             # empty list for contact directions on ball z direction
             self.dir_zx_contact_ball={}
             self.dir_zz_contact_ball={}        
        
             # empty list for contact positions on bot
             self.position_x_contact_bot={}
             self.position_z_contact_bot={}
        
             # empty list of forces that the robot is applying on the ball 
             self.Forces_x_ball_bot={}
             self.Forces_z_ball_bot={}
        
             # empty list of positions that the robot is applying on the ball 
             self.position_x_ball_bot={}
             self.position_z_ball_bot={}
        
        
             # FILL THE EMPTY ARRAYS
             for i in range(len(self.time_contact)):
                 # positions
                 self.Forces_x_contact_particles["time_contact{0}".format(i)]=[]  #x position
                 self.Forces_z_contact_particles["time_contact{0}".format(i)]=[]  #x position
                     
                 self.Forces_x_contact_bots["time_contact{0}".format(i)]=[]  #x position
                 self.Forces_z_contact_bots["time_contact{0}".format(i)]=[]  #x position
    
                 self.Force_x_contact_ball["time_contact{0}".format(i)]=[]
                 self.Force_z_contact_ball["time_contact{0}".format(i)]=[]
                
                 self.position_x_contact_ball["time_contact{0}".format(i)]=[]
                 self.position_z_contact_ball["time_contact{0}".format(i)]=[]
                
                 self.Forces_x_ball_bot["time_contact{0}".format(i)]=[]
                 self.Forces_z_ball_bot["time_contact{0}".format(i)]=[]
                
                 self.position_x_ball_bot["time_contact{0}".format(i)]=[]
                 self.position_z_ball_bot["time_contact{0}".format(i)]=[]
                
                 self.position_x_contact_bot["time_contact{0}".format(i)]=[]
                 self.position_z_contact_bot["time_contact{0}".format(i)]=[]
    
                 self.dir_xx_contact_ball["time_contact{0}".format(i)]=[]
                 self.dir_xz_contact_ball["time_contact{0}".format(i)]=[]
            
                 self.dir_zx_contact_ball["time_contact{0}".format(i)]=[]
                 self.dir_zz_contact_ball["time_contact{0}".format(i)]=[]          
            

             #### Version 1
             # THERESE CORRESPOND TO THE FORCES THE ROBOT IS APPLY ONLY
             self.grasp_id = [] #  id of the robots in contact applying forces  
             self.grasp_position_x = [] # positon x
             self.grasp_position_z = [] # position z
             
             self.grasp_force_x = [] # force x in contact
             self.grasp_force_z = [] # forces z in contact   
             self.grasp_torque = []   # toruqes being applied  
             
             self.WRENCHES = 0    
             self.WRENCH_NORM = 0
             
             self.G = []
             
             self.FRAMES=[] # empty array of the contact frames
              
             self.EPSILON=[] # empty array of the epsilon metric
             self.HULLWRENCHNORM=[] # empty array of the wrench norm
             self.HULLWRENCHMAGS=[] # empty array of the magnitude of the wrench magnitutudes
             
             self.framex = []
             self.framez = []

             self.FT = []
             
             self.Cplus = []
             self.Cminus = [] 
             
             self.FCplus = []
             self.FCinus = []
             
             self.F_mag = []
             self.HULL=[]
             
             self.WRENCHXY=[]
             self.HULLXY=[]
                 
             self.WRENCHXT=[]
             self.HULLXT=[]

             self.WRENCHYT=[]
             self.HULLYT=[]            

             self.ANGLE_CHECK=[]


             self.pull_data=np.genfromtxt(self.files[self.files.index('pull_force.csv') ] ,delimiter=',')            
             (self.m6a,self.n6a)=np.shape(self.pull_data)
             self.pull_data=self.pull_data[:,1:self.n6a]
             self.TIME=self.pull_data[0,:]
             self.PX=self.pull_data[1,:]
             self.PZ=self.pull_data[2,:]
             self.FB=self.pull_data[3,:]            




     def angle(self,x, y):
    
        rad = np.arctan2(y, x)
        #degrees = np.int(rad*180/np.pi)
        #if rad < 0:
            #rad = 2*np.pi - rad
        return rad



      
         
         
  
 
     def smooth_epsilon(self,epsilon):
        if len(epsilon)<10:
            entry=0
            self.epsilon_tilda_.append(0)
            #self.time_tilda.append(time)
        else:
            temp=self.moving_average(epsilon, n=10)
            self.epsilon_tilda_.append(temp[-1])
            #self.time_tilda.append(time)
        return(self.epsilon_tilda_)
    
     def smooth_epsilon2(self,epsilon,nn):
         epsilon_=[]
         for i in range(len(epsilon)):
             if i<nn:
                 epsilon_.append(0)
   
             else:
                 temp=self.moving_average(epsilon, n=nn)
                 self.epsilon_tilda_.append(temp[-1])
             #self.time_tilda.append(time)
         return(self.epsilon_tilda_) 
    
     def moving_average(self,a, n=3):
        ret = np.cumsum(a, dtype=float)
        ret[n:] = ret[n:] - ret[:-n]
        return ret[n - 1:] / n
    
    

     def plot_epsilon4(self): 
         """ Epsilon regaridng the contact foprces without the smoothed out signal"""
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,3),dpi=300)
         axs.plot(self.time[0:-2],self.EPSILON_[0:-2],color='r',linewidth=1,label='during grasping')
         axs.set_title(r'$\epsilon$'+" vs time" )
         axs.set_ylabel('$\epsilon$',labelpad=1)
         axs.set_xlabel('time [s]',labelpad=-2)
         axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.grid(True)
         axs.legend()
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon4_value.jpg')
        # plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon4_value.svg')    
         #plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon4_value.pdf')          
         plt.close('all')
    
        
     def plot_epsilon5(self): 
         """ Epsilon regaridng the contact foprces and has the smoothed out one """
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,3),dpi=300)
         axs.plot(self.time[0:-2],self.EPSILON_[0:-2],color='r',linewidth=1,label=r"$\epsilon$")
         axs.plot(self.time_tilda,self.epsilon_tilda,color='b',linewidth=1,label=r"$\bar{\epsilon}$")
         axs.set_title(r'$\epsilon_{5}$'+" vs time" )
         axs.set_ylabel('$\epsilon_{5}$',labelpad=1)
         axs.set_xlabel('time [s]',labelpad=-2)
         axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.grid(True)
         axs.legend()
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon5_value.jpg')
        # plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon5_value.svg')    
         #plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon5_value.pdf')          
         plt.close('all')
     
        
     def plot_epsilon5dx(self): 
         """ Epsilon regaridng the contact foprces """
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,3),dpi=300)
         nn=20
         epsilon_tilda=self.moving_average(self.EPSILON_, n=nn)
         epsilon_tilda=np.concatenate((np.zeros(nn-1),epsilon_tilda))
         epsilon_tildadx=np.gradient(epsilon_tilda,self.time)
         dt=self.time_tilda[0]-self.time_tilda[1]
         axs.plot(self.time,epsilon_tildadx,color='g',linewidth=1,label=r"$\bar{\epsilon}$")
         axs.set_title(r'D$\epsilon_{5}$'+" vs time" )
         axs.set_ylabel('$D\epsilon_{5}$',labelpad=1)
         axs.set_xlabel('time [s]',labelpad=-2)
         axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.grid(True)
         axs.legend()
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon5_valuedx.jpg')
        # plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon5_valuedx.svg')    
         #plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon5_valuedx.pdf')          
         plt.close('all')
        
     def plot_Qcm(self): 
         """ Qcm vs time """
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,3),dpi=300)
         axs.plot(self.time,self.Qcm,color='g',linewidth=1)
         axs.set_title('Qcm'+" vs time" )
         axs.set_ylabel('Qcm',labelpad=1)
         axs.set_xlabel('time [s]',labelpad=-2)
         axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.grid(True)
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'Qcm_value.jpg')
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'Qcm_value.svg')    
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'Qcm_value.pdf')          
         plt.close('all')
     
     
     def sort_epsilon_and_theta(self):
         """ function used to sort epsilon measurement vs the angle of approach """
        theta=self.THETA#[0:-2] 
        epsilon=self.EPSILON_
        for k, g in itertools.groupby(theta):
            self.angle_entries.append(k) 
            self.epsilon_section["theta:"+str(k)]=[]
            
        for i in range(12):
            self.epsilon_theta_section[str(i)]=[]
            
            
        for i in range(len(self.angle_entries)):
            res=np.where(theta==self.angle_entries[i])
            #print(res[0])
            self.epsilon_section["theta:"+str(self.angle_entries[i])].append(epsilon[res[0]])
            epsilon2=epsilon[res[0]]
            res2=np.nonzero(epsilon[res[0]])
            res2=res2[0]
            self.average_epsilon.append(np.mean(epsilon2[res2]))
            self.max_epsilon.append(max(epsilon[res[0]]))
            #print(np.mean(epsilon2[res2]))
        
        count=0
        for i in range(len(self.max_epsilon)):
            
            if count==12:
                count=0
            else:
                count=count
            
            self.epsilon_theta_section[str(count)].append(self.max_epsilon[i])
            count=count+1
        
     def plot_epsilon_vs_theta(self): 
         """ Epsilon vs the angle the system is approaching from """
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,3),dpi=300)
         #epsilon=np.asarray(self.EPSILON4)
         epsilon=self.EPSILON_
         axs.scatter(self.THETA,epsilon,color='b',marker='s')
         
         for i in range(len(self.angle_entries)):
             axs.scatter(self.angle_entries[i],self.average_epsilon[i],color='r',marker='s')
         
         x_ticks = np.linspace(self.THETA[0], self.THETA[-2],5,endpoint=True)
         y_ticks = np.linspace(np.min(epsilon), np.max(epsilon),10,endpoint=True)
         axs.set_xticks(np.round(x_ticks,2))
         axs.set_yticks(np.round(y_ticks,2))
         axs.set_title(r'$\epsilon_{4}$'+"vs"+r'$\theta$' )
         axs.set_ylabel('$\epsilon_{4}$',labelpad=1)
         axs.set_xlabel(r'$\theta$' ,labelpad=-2)
         axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.grid(True)
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon_v_theta.jpg')
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon_v_theta.svg')    
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon_v_theta.pdf')      
         #plt.close('all')
         
            
         
         color=iter(cm.rainbow(np.linspace(0,1,len(self.angle_entries))))
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,3),dpi=300)
         for i in range(len(self.angle_entries)):
             c=next(color)
             epsilon=self.epsilon_section['theta:'+str(self.angle_entries[i])][0]
             axs.plot(epsilon,color=c,label=str(np.round(self.angle_entries[i],2)))    
         
         axs.grid(True)
         axs.legend()
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon_section.jpg')
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon_section.svg')    
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon_section.pdf')   
         plt.close('all')
         
         
     def plot_epsilon_vs_theta_section(self):         
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,3),dpi=300)
         
         #axs.scatter(self.THETA[0:-2],epsilon,color='b',marker='s')
         y=[]
         
         for i in range(len(self.epsilon_theta_section)):
             entry=int(str(i))
             y_=self.epsilon_theta_section[str(i)]
             y.append(y_)
             
             x=entry*np.ones(len(y_))
             axs.scatter(x,y_,color='b',marker='s')
             axs.scatter(entry,np.mean(y_),color='r',marker='s')
         
         #bp = axs.boxplot(y)    
         #positions=[0,1,2,3,4,5,6,7,8,9,10,11]
         #labels=['0','$\pi$/6','$\pi$/3','$\pi$/2','$2\pi$/3','$5\pi$/6','$\pi$','$7\pi$/6','$8\pi$/6','$3\pi$/2','$10\pi$/6','$11\pi$/6']
         #axs.set_xticks(positions)
         #axs.set_xticklabels(labels,color='k')
         
         axs.set_title(r'$\epsilon$'+" vs "+r'$\theta$' )
         axs.set_ylabel('$\epsilon$',labelpad=1)
         axs.set_xlabel(r'$\theta$' ,labelpad=-2)
         axs.grid(True)
         #axs.legend()
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon_theta_section.jpg')
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon_theta_section.svg')    
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'epsilon_theta_section.pdf')   
         plt.close('all')
         
     
     
     def plot_contact_number(self):
         """ contact numbers"""
         Num_contact1=[]
         Num_contact2=[]
         for i in range(len(self.temp_id2)):
             Num_contact1.append(len(self.temp_id2[i]))
                      
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,3),dpi=300)
         axs.plot(self.time[0:-2],Num_contact1,color='g',linewidth=1)
         x_ticks = np.linspace(self.time[0], self.time[-2],5,endpoint=True)
         y_ticks = np.linspace(0, np.max(Num_contact1),np.max(Num_contact1)+1,endpoint=True)
         axs.set_xticks(np.round(x_ticks,2))
         axs.set_yticks(np.round(y_ticks,2))
         axs.set_title(r'number_contact'+" vs time" )
         axs.set_ylabel('number of contact',labelpad=1)
         axs.set_xlabel('time [s]',labelpad=-2)
         axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.grid(True)
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'number of contact.jpg')
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'number of contact.svg')    
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'number of contact.pdf')           
         

         
     def plot_ball_position(self):
         ''' plot the control forces'''
         direct = os.path.join(self.mainDirectory+'/'+self.name+'/'+'_force_analysis')    
         if not os.path.isdir(direct):
             os.makedirs(direct)         
         fig, axs = plt.subplots(nrows=2, ncols=1,figsize=(5,4),dpi=300)

               
         axs[0].plot(self.time,self.ballx_position,color='tab:red',linewidth=1,label='x')
         x_ticks = np.linspace(self.time[0], self.time[-1],5,endpoint=True)
         y_ticks = np.linspace(np.min(self.ballx_position), np.max(self.ballx_position),5,endpoint=True)
         axs[0].xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs[0].yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs[0].set_xticks(np.round(x_ticks,2))
         axs[0].set_yticks(np.round(y_ticks,2))
         axs[0].set_title('ball_x_position [x]'+" vs time" )
         axs[0].set_ylabel('ball_x_position [x]',labelpad=1)         
         axs[0].grid(True)        
     # axs[0].set_xlim([2.5,10])  
        
         axs[1].plot(self.time,self.ballz_position,color='tab:blue',linewidth=1,label='y')
         x_ticks = np.linspace(self.time[0], self.time[-1],5,endpoint=True)
         y_ticks = np.linspace(np.min(self.ballz_position), np.max(self.ballz_position),5,endpoint=True)
         axs[1].xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs[1].yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs[1].set_xticks(np.round(x_ticks,2))
         #axs[1].set_yticks(np.round(y_ticks,2))
         axs[1].set_title('ball_z_position'+" vs time" )
         axs[1].set_ylabel('ball_z_position [z]',labelpad=1)
         axs[1].set_xticks(np.round(x_ticks,2))
         axs[1].grid(True)   
     # axs[1].set_xlim([2.5,10])  
        

         
     # axs[2].set_xlim([2.5,10]) 
         #fig.suptitle('Control forces', fontsize=16)        
         plt.tight_layout()
         plt.savefig(direct+"/ball_positions.jpg")
         plt.savefig(direct+"/ball_positions.pdf")
         plt.savefig(direct+"/ball_positions.svg")
         plt.savefig(direct+"/ball_positions.eps")
         plt.close('all') 

         
     def plot_ball_position_vs_estimate(self):
         ''' plot the ball position vs the center of contact hull '''
         direct = os.path.join(self.mainDirectory+'/'+self.name)    
         if not os.path.isdir(direct):
             os.makedirs(direct)         
         fig, axs = plt.subplots(nrows=2, ncols=1,figsize=(5,4),dpi=300)

               
         axs[0].plot(self.time,self.ballx_position,color='tab:red',linewidth=1,label='x')
         axs[0].plot(self.time[0:-2],self.centroidx,color='tab:red',linewidth=1,linestyle='--',label='xe')
         #x_ticks = np.linspace(self.time[0], self.time[-1],5,endpoint=True)
         #y_ticks = np.linspace(np.min(self.ballx_position), np.max(self.ballx_position),5,endpoint=True)
         axs[0].xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs[0].yaxis.set_tick_params(width=.25,length=2,pad=1)
         #axs[0].set_xticks(np.round(x_ticks,2))
         #axs[0].set_yticks(np.round(y_ticks,2))
         axs[0].set_title('ball_x_position [x]'+" vs time" )
         axs[0].set_ylabel('ball_x_position [x]',labelpad=1)         
         axs[0].grid(True)        
     # axs[0].set_xlim([2.5,10])  
        
         axs[1].plot(self.time,self.ballz_position,color='tab:blue',linewidth=1,label='y')
         axs[1].plot(self.time[0:-2],self.centroidz,color='tab:blue',linewidth=1,linestyle='--',label='ye')
         #x_ticks = np.linspace(self.time[0], self.time[-1],5,endpoint=True)
         #y_ticks = np.linspace(np.min(self.ballz_position), np.max(self.ballz_position),5,endpoint=True)
         axs[1].xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs[1].yaxis.set_tick_params(width=.25,length=2,pad=1)
         #axs[1].set_xticks(np.round(x_ticks,2))
         #axs[1].set_yticks(np.round(y_ticks,2))
         axs[1].set_title('ball_z_position'+" vs time" )
         axs[1].set_ylabel('ball_z_position [z]',labelpad=1)
         #axs[1].set_xticks(np.round(x_ticks,2))
         axs[1].grid(True)   
     # axs[1].set_xlim([2.5,10])  
        

         
     # axs[2].set_xlim([2.5,10]) 
         #fig.suptitle('Control forces', fontsize=16)        
         plt.tight_layout()
         plt.savefig(direct+"/ball_positions_vs_estimate.jpg")
         plt.savefig(direct+"/ball_positions_vs_estimate.pdf")
         plt.savefig(direct+"/ball_positions_vs_estimate.svg")
         plt.savefig(direct+"/ball_positions_vs_estimate.eps")
         plt.close('all') 


     def plot_ball_velocity(self):
         direct = os.path.join(self.mainDirectory+'/'+self.name+'/'+'_force_analysis')    
         if not os.path.isdir(direct):
             os.makedirs(direct)      
         y1_velx =self.smooth(self.ball_velocity_x[0:-1], 20)
         y2_velz =self.smooth(self.ball_velocity_z[0:-1], 20)
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,4),dpi=300)

         axs.plot(self.time,self.ball_velocity_x[0:-1],color="tab:blue",linewidth=1,label='vx')
         axs.plot(self.time,y1_velx,color="k",linewidth=1,linestyle='-')
         axs.plot(self.time,self.ball_velocity_z[0:-1],color="tab:red",linewidth=1,label='vz')
         axs.plot(self.time,y2_velz,color="k",linewidth=1,linestyle='--')
         axs.set_xlabel('time (s)',fontsize=8,labelpad=1)
         axs.set_ylabel('velocity (m/s)',labelpad=1)
         axs.set_title('Ball velocity')
         axs.xaxis.set_tick_params(width=.25,length=2)
         axs.yaxis.set_tick_params(width=.25,length=2)
         axs.grid(True,linewidth=0.25)
         #axs.set_ylim([-0.05,0.05])
         fig.legend( loc='upper right', borderaxespad=0,frameon=False)
         #plt.tight_layout()
         #plt.tight_layout()
         plt.savefig(direct+"/ball_velocity.jpg")
         plt.savefig(direct+"/ball_velocity.pdf")
         plt.savefig(direct+"/ball_velocity.svg")
         plt.savefig(direct+"/ball_velocity.eps")
         plt.close('all') 

     def plot_ball_pull_forces_trial(self):
         ''' plot the control forces'''
         direct = os.path.join(self.mainDirectory+self.name,'_pull_trial')    
 
         if not os.path.isdir(direct):
             os.makedirs(direct)  
         count=0    
         for i in range(len(self.time)):
             fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,4),dpi=300)
  
             #print("FB",np.round(self.FB[i],2))
             axs.plot(self.FB[:i],self.PX[:i]-self.PX[54],color='green',linewidth=5)
             axs.scatter(self.FB[i],self.PX[i]-self.PX[54],color='red',s=20)
             #y_ticks = np.linspace(self.PX[0]-self.PX[0], self.PX[-1]-self.PX[0],5,endpoint=True)
             #x_ticks = np.linspace(np.min(self.FB), np.max(self.FB),5,endpoint=True)
             #axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
             #axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
             #axs.set_xticks(np.round(x_ticks,2))
             #axs.set_yticks(np.round(y_ticks,2))
             axs.set_ylim([0,2])
             #axs.set_title('ball Force ext [x]'+" vs ball position" )
             axs.set_xlabel('Pull Force [N]',labelpad=1)  
             axs.set_ylabel('Ball Position [m]',labelpad=1) 
             axs.grid(True)            
   
             #plt.tight_layout()
             fig.suptitle('Time= ' + str(np.round(self.time[i],0)))
             plt.savefig(direct+"/frame%04d.jpg" % count)
                 
             count=count+1 
            
             plt.close('all')  
         self.create_video('_pull_trial','_pull_trial')    

     def plot_ball_pull_forces(self):
         ''' plot the control forces'''
         direct = os.path.join(self.mainDirectory+'/'+self.name+'/'+'_force_analysis')    
         if not os.path.isdir(direct):
             os.makedirs(direct)         
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,4),dpi=300)
  
         
         axs.plot(self.FB,self.PX-self.PX[0],color='tab:green',linewidth=1,label='x')
         y_ticks = np.linspace(self.PX[0]-self.PX[0], self.PX[-1]-self.PX[0],5,endpoint=True)
         x_ticks = np.linspace(np.min(self.FB), np.max(self.FB),5,endpoint=True)
         axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.set_xticks(np.round(x_ticks,2))
         axs.set_yticks(np.round(y_ticks,2))
         axs.set_title('ball Force ext [x]'+" vs ball position" )
         axs.set_xlabel('Pull Force [N]',labelpad=1)  
         axs.set_ylabel('Ball Position [m]',labelpad=1) 
         axs.grid(True)            
   
         plt.tight_layout()
         
         
         plt.savefig(direct+"/ball_pull_force.jpg")
         plt.savefig(direct+"/ball_pull_force.pdf")
         plt.savefig(direct+"/ball_pull_force.svg")
         plt.savefig(direct+"/ball_pull_force.eps")
         plt.close('all') 

     def plot_ball_contact_forces(self):
         ''' plot the ball contact forces '''
         direct = os.path.join(self.mainDirectory+'/'+self.name+'/'+'_force_analysis')    
         if not os.path.isdir(direct):
             os.makedirs(direct)         
         fig, axs = plt.subplots(nrows=3, ncols=1,figsize=(5,4),dpi=300)
         from scipy.ndimage import gaussian_filter1d
         self.bFx= gaussian_filter1d(self.bFx[80:-1], 50)      
         self.bFz= gaussian_filter1d(self.bFz[80:-1], 50) 
         axs[0].plot(self.time[80:-1],self.bFx,color='tab:red',linewidth=1,label='x')
         #x_ticks = np.linspace(self.time[0], self.time[-1],5,endpoint=True)
         #y_ticks = np.linspace(np.min(self.bFx), np.max(self.bFx),5,endpoint=True)
         axs[0].xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs[0].yaxis.set_tick_params(width=.25,length=2,pad=1)
         #axs[0].set_xticks(np.round(x_ticks,2))
         #axs[0].set_yticks(np.round(y_ticks,2))
         axs[0].set_title('ball Force [x]'+" vs time" )
         axs[0].set_ylabel('ball Force  [x]',labelpad=1)         
         axs[0].grid(True)        
     # axs[0].set_xlim([2.5,10])  
        
         axs[1].plot(self.time[80:-1],self.bFz,color='tab:blue',linewidth=1,label='x')
         #x_ticks = np.linspace(self.time[0], self.time[-1],5,endpoint=True)
         #y_ticks = np.linspace(np.min(self.bFz), np.max(self.bFz),5,endpoint=True)
         axs[1].xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs[1].yaxis.set_tick_params(width=.25,length=2,pad=1)
         #axs[1].set_xticks(np.round(x_ticks,2))
         #axs[1].set_yticks(np.round(y_ticks,2))
         axs[1].set_title('ball Force [z]'+" vs time" )
         axs[1].set_ylabel('ball Force  [z]',labelpad=1)         
         axs[1].grid(True)   
         
         axs[2].plot(self.TIME,self.FB,color='tab:green',linewidth=1,label='x')
         x_ticks = np.linspace(self.TIME[0], self.TIME[-1],5,endpoint=True)
         y_ticks = np.linspace(np.min(self.FB), np.max(self.FB),5,endpoint=True)
         axs[2].xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs[2].yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs[2].set_xticks(np.round(x_ticks,2))
         axs[2].set_yticks(np.round(y_ticks,2))
         axs[2].set_title('ball Force ext [x]'+" vs time" )
         axs[2].set_ylabel('ball Force ext  [x]',labelpad=1)         
         axs[2].grid(True)            
         
     # axs[1].set_xlim([2.5,10])  
        

         
     # axs[2].set_xlim([2.5,10]) 
         #fig.suptitle('Control forces', fontsize=16)        
         plt.tight_layout()
         plt.savefig(direct+"/ball_contact_force.jpg")
         plt.savefig(direct+"/ball_contact_force.pdf")
         plt.savefig(direct+"/ball_contact_force.svg")
         plt.savefig(direct+"/ball_contact_force.eps")
         plt.close('all')  
        
     def plot_potential_Field_value(self):
         ''' plot the sum of the potential field sum'''
        
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,4),dpi=300)
         axs.plot(self.time,self.Field_value_sum,color='tab:red',linewidth=1)
         #x_ticks = np.linspace(self.time[0], self.time[-1],5,endpoint=True)
         #y_ticks = np.linspace(np.min(self.bFx), np.max(self.bFx),5,endpoint=True)
         axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.set_ylim([0,100])
         
         #axs[0].set_xticks(np.round(x_ticks,2))
         #axs[0].set_yticks(np.round(y_ticks,2))
         axs.set_title('$\Sigma \phi$'+" vs time" )
         #axs[0].set_ylabel('ball Force  [x]',labelpad=1)         
         axs.grid(True)        
         #fig.suptitle('Control forces', fontsize=16)        
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'potential_field_sum.jpg')
         #plt.savefig(self.mainDirectory+'/'+self.name+'/'+'potential_field_sum.svg')    
         #plt.savefig(self.mainDirectory+'/'+self.name+'/'+'potential_field_sum.pdf')   
         plt.close('all') 
         
         dx=np.gradient(self.Field_value_sum,self.time)
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,4),dpi=300)
         axs.plot(self.time,dx,color='tab:red',linewidth=1)
         #x_ticks = np.linspace(self.time[0], self.time[-1],5,endpoint=True)
         #y_ticks = np.linspace(np.min(self.bFx), np.max(self.bFx),5,endpoint=True)
         axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.set_ylim([-1,1])
         
         #axs[0].set_xticks(np.round(x_ticks,2))
         #axs[0].set_yticks(np.round(y_ticks,2))
         axs.set_title('$\Sigma d \phi$'+" vs time" )
         #axs[0].set_ylabel('ball Force  [x]',labelpad=1)         
         axs.grid(True)        
         #fig.suptitle('Control forces', fontsize=16)        
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'potential_field_sum_dx.jpg')
         #plt.savefig(self.mainDirectory+'/'+self.name+'/'+'potential_field_sum_dx.svg')    
         #plt.savefig(self.mainDirectory+'/'+self.name+'/'+'potential_field_sum_dx.pdf')   
         plt.close('all') 
         
         
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,4),dpi=300)
         axs.plot(self.time,np.dot(1/self.nb,self.Field_value_sum),color='tab:red',linewidth=1)
         axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.set_title('$\Sigma\phi$'+" vs time" )
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'potential_field_sumbar.jpg')
         #plt.savefig(self.mainDirectory+'/'+self.name+'/'+'potential_field_sumbar.svg')    
         #plt.savefig(self.mainDirectory+'/'+self.name+'/'+'potential_field_sumbar.pdf')   
         plt.close('all') 
         
    
     def plot_potential_Field_value_individual(self):
         ''' plot the potnetial field individial values'''
 
         NB_=np.linspace(0,self.nb-1,self.nb,endpoint=True)
         T,NB__ = np.meshgrid(self.time,NB_)
         fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(5,5),dpi=300)
         im1=axs.pcolormesh(T,NB__,self.Field_value, cmap='jet',shading='auto')
         #axs.set_xticklabels(x, minor=True)
         #axs.set_yticks(NB_, minor=False)
         #axs.set_yticklabels(nb_label, minor=False)
         #axs.set_xticklabels(self.time, minor=True)
         #axs.set_yticks(NB_, minor=False)
         axs.xaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.yaxis.set_tick_params(width=.25,length=2,pad=1)
         axs.set_title('$\phi$'+" vs time" )
         #axs[0].set_ylabel('ball Force  [x]',labelpad=1)         
         axs.grid(True)  
         fig.colorbar(im1, ax=axs)
         #fig.suptitle('Control forces', fontsize=16)        
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'potential_field.jpg')
         #plt.savefig(self.mainDirectory+'/'+self.name+'/'+'potential_field.svg')    
         #plt.savefig(self.mainDirectory+'/'+self.name+'/'+'potential_field.pdf')   
         plt.close('all') 
         
         
     def create_frames_contact_forces_(self,membrane,directory,entry,wxmin,wxmax,wymin,wymax,fxs,fys,name):
         plt.rcParams['font.family'] = 'Times New Roman'
         plt.rcParams['mathtext.fontset'] = 'dejavuserif'
         plt.rcParams['font.size'] = 8
         plt.rcParams['axes.linewidth'] = .1
        
         i=entry


         F_contact_ballx_entry=self.Force_x_contact_ball["time_contact"+str(i)] # EXTRACT THE ENTRY
         F_contact_ballz_entry=self.Force_z_contact_ball["time_contact"+str(i)] # EXTRACT THE ENTRY

         #print(F_contact_ballx_entry)
         F_contact_ballx_entry=F_contact_ballx_entry[0]['ballx'] #  FURTHER EXTRACTION 
         F_contact_ballz_entry=F_contact_ballz_entry[0]['ballz'] #  FURTHER EXTRACTION 
        
    
         Position_x_contact_entry=self.position_x_contact_ball["time_contact"+str(i)] # EXTRACT THE ENTRY
         Position_z_contact_entry=self.position_z_contact_ball["time_contact"+str(i)] # EXTRACT THE ENTRY

         #print(Position_x_contact_entry)
         Position_x_contact_entry=Position_x_contact_entry[0]['ballx'] #  FURTHER EXTRACTION 
         Position_z_contact_entry=Position_z_contact_entry[0]['ballz'] #  FURTHER EXTRACTION 

         dir_xx_contact_ball_entry=self.dir_xx_contact_ball["time_contact"+str(i)]
         dir_xz_contact_ball_entry=self.dir_xz_contact_ball["time_contact"+str(i)]
         dir_zx_contact_ball_entry=self.dir_zx_contact_ball["time_contact"+str(i)]
         dir_zz_contact_ball_entry=self.dir_zz_contact_ball["time_contact"+str(i)]

         dir_xx_contact_ball_entry=dir_xx_contact_ball_entry[0]['ballx']
         dir_xz_contact_ball_entry=dir_xz_contact_ball_entry[0]['ballz']
         dir_zx_contact_ball_entry=dir_zx_contact_ball_entry[0]['ballx']
         dir_zz_contact_ball_entry=dir_zz_contact_ball_entry[0]['ballz']
        
        
         x0b,y0b=self.ballx_position[i],self.ballz_position[i]
            
            
         if self.geom=="square":
             const=self.ball_radius*2-.01
             rx=const
             ry=const
             w=rx/2
             h=ry/2                    

             x__=[w,-w,-w,w,w]
             y__=[h,h,-h,-h,h]
             (segments)=self.create_segment(x__,y__) 

         if self.geom=="triangle":
             const=self.ball_radius*2*np.pi/3
             r=const*np.sqrt(3)/3 -.01
             x1=r
             y1=0
           
             x2=r*np.cos(2*np.pi/3)
             y2=r*np.sin(2*np.pi/3)
           
             x3=r*np.cos(4*np.pi/3)
             y3=r*np.sin(4*np.pi/3)
           
             x__ = [x1,x2,x3,x1]
             y__ = [y1,y2,y3,y1]
             (segments)=self.create_segment(x__,y__)

         fig, ax = plt.subplots(figsize=(fxs,fys),dpi=300)
         ax = plt.axes(xlim=(wxmin,wxmax), ylim=(wymin, wymax))
         plt.axis('off')
         
                 #ax.plot(x,y)
         for j in range(0,self.nb):
             x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]
             if self.geom=="square":
                 if self.PHI(x0,y0,self.segments)<.15:
                     patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:orange')
                     ax.add_patch(patch)
               
                 else:
                     patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                     ax.add_patch(patch) 
                    
             elif self.geom=="triangle":
                 if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.1:
                     patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                     ax.add_patch(patch)
               
                 else:
                     patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                     ax.add_patch(patch)                         
                    
             elif self.geom=="circle":
                
                 q=np.sqrt((x0-self.ballx_position[i])**2 + (y0-self.ballz_position[i])**2)
                 if q<=(2 * self.bot_width/2 + self.ball_radius):
                     patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:orange')
                     ax.add_patch(patch)
               
                 else:
                     patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                     ax.add_patch(patch)  
                    
            
                
         if membrane==True:
             for j in range(0,self.nm):
                
                 x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                 patch = plt.Circle((x0, y0),self.skin_width/2, fc='tab:red')
                 ax.add_patch(patch)
            
            

         for j in range(self.ni):
             x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

             if np.round(self.Rm[j],4)==0.0508:
                 c='tab:blue'
             else:
                 c='tab:green'
                
             patch = plt.Circle((x0, y0),self.Rm[j], fc=c)
             ax.add_patch(patch)         
 
         
         
         
         
         
         
         if self.geom=="square":
             const_=self.ball_radius*2
             xb,yb=0-const_/2,0 - const_/2
             x0_=xb
             y0_=yb
             patch = matplotlib.patches.Rectangle((x0b, y0b),const_, const_,fc='none',edgecolor='k')     
             ax.add_patch(patch)
         if self.geom=="circle":
             x0_=0
             y0_=0              
             patch = plt.Circle((x0b, y0b),self.ball_radius,fc='none',edgecolor='k')
             ax.add_patch(patch)
            
         if self.geom=="triangle":               
             const=self.ball_radius*2*np.pi/3
             r=const*np.sqrt(3)/3 +.01
             patch = matplotlib.patches.RegularPolygon((0,0),int(3),r,orientation=-np.pi/2,fc='none',edgecolor='k')
             ax.add_patch(patch) 
            

         X=[]
         Y=[]
         theta=[]
        

         frames = np.zeros((len(Position_x_contact_entry),3))
         Vx=np.zeros((len(Position_x_contact_entry),2))
         Vy=np.zeros((len(Position_x_contact_entry),2))
         C1=np.zeros((2,len(Position_x_contact_entry))) # positive cone
         C2=np.zeros((2,len(Position_x_contact_entry))) # negative cone 
         for j in range(len(Position_x_contact_entry)):
             x0,y0=Position_x_contact_entry[j],Position_z_contact_entry[j]
             #ax.text(x0-x0b,y0-y0b,str(j),size=8)
             theta1=np.arctan2(.2,1) #+ frames[j,2]             
             if self.geom=="square" or self.geom=="triangle":
                Fx1=self.PHIDX(x0-x0b,y0-y0b,segments)
                Fy1=self.PHIDY(x0-x0b,y0-y0b,segments)
                mag=np.sqrt(Fx1**2 + Fy1**2)
                Fx1=-Fx1/mag
                Fy1=-Fy1/mag
                F_t=np.array([Fx1,Fy1])
                X.append(x0-x0b)
                Y.append(y0-y0b)
                t=self.angle(Fx1, Fy1)-np.pi/2
                theta.append(t)
            
                frames[j,0]=x0-x0b
                frames[j,1]=y0-y0b
                frames[j,2]=t
                #mag=np.sqrt(F_contact_ballx_entry[j]**2 + F_contact_ballz_entry[j]**2)
                mag=1
            

                
                
             if self.geom=="circle":
                 Fx1,Fy1=(self.ballx_position[i]-x0),(self.ballz_position[i]-y0)
                 mag=np.sqrt(Fx1**2 + Fy1**2)
                 Fx1=Fx1/mag
                 Fy1=Fy1/mag
                 F_t=np.array([Fx1,Fy1])
                 X.append(x0-x0b)
                 Y.append(y0-y0b)
                 t=self.angle(Fx1, Fy1)-np.pi/2
                 theta.append(t)
            
                 frames[j,0]=x0-x0b
                 frames[j,1]=y0-y0b
                 frames[j,2]=t
                
                
                
                    
            
                
             T=np.array([[np.cos(t),-np.sin(t)],[np.sin(t),np.cos(t)]]) # transformation matrix   
             VYpp=T@np.array([[0],[1]]) # transform coordinates X
             VXpp=T@np.array([[1],[0]]) # transform coordinates Y
             VXpp=VXpp.flatten() # flatten the matrix
             VYpp=VYpp.flatten() # flatten the matrix
        
             Vx[j,:]=VXpp # Save the array X
             Vy[j,:]=VYpp  # save the array Y     
            
             T1=np.array([[np.cos(theta1),-np.sin(theta1)],[np.sin(theta1),np.cos(theta1)]]) # transformation matrix   
             T2=np.array([[np.cos(-theta1),-np.sin(-theta1)],[np.sin(-theta1),np.cos(-theta1)]])    
             tem=Vy[j,:].T
             C1[:,j]=T1@tem
             C2[:,j]=T2@tem
        
        
            
             mag1 = np.sqrt(F_contact_ballx_entry[j]**2 + F_contact_ballz_entry[j]**2)
             mag2=np.sqrt(dir_xx_contact_ball_entry[j]**2 + dir_xz_contact_ball_entry[j]**2)
             mag3=np.sqrt(dir_zx_contact_ball_entry[j]**2 + dir_zz_contact_ball_entry[j]**2) 
             temp_dirr=[dir_xx_contact_ball_entry[j]/mag2,dir_xz_contact_ball_entry[j]/mag2]
        
            
             # ax.quiver(x0-x0b,y0-y0b, dir_xx_contact_ball_entry[j]/mag2,dir_xz_contact_ball_entry[j]/mag2,scale=20,color="k",width=.007,zorder=3)    
             # ax.quiver(x0-x0b,y0-y0b, F_contact_ballx_entry[j]/mag1,F_contact_ballz_entry[j]/mag1,scale=10, color="tab:orange",width=.007,zorder=5)      
             # ax.quiver(x0-x0b,y0-y0b, Fx1,Fy1,scale=10, color="green",width=.007,zorder=1)
             # ax.quiver(x0-x0b,y0-y0b,C1[0,j],C1[1,j] ,color="magenta", scale=10,label='Positive cone')     
             # ax.quiver(x0-x0b,y0-y0b,C2[0,j],C2[1,j] ,color="magenta", scale=10,label='negative cone')
             # ax.quiver(x0-x0b,y0-y0b, VXpp[0],VXpp[1],scale=15,color="b",width=.007,zorder=2)  
             # ax.quiver(x0-x0b,y0-y0b, VYpp[0],VYpp[1],scale=15,color="r",width=.007,zorder=2)  
        

             theta1=np.arctan2(.2,1) 
             temp=np.round(np.nan_to_num(np.arccos(np.dot(VYpp ,temp_dirr))),2)
             if temp<np.pi and temp > np.pi-theta1:
                 #print("triggered")
                 fx=np.cos(-temp)*F_contact_ballx_entry[j]-np.sin(-temp)*F_contact_ballz_entry[j]
                 fy=np.sin(-temp)*F_contact_ballx_entry[j]+np.cos(-temp)*F_contact_ballz_entry[j]
                 dirxx=np.cos(-temp)*dir_xx_contact_ball_entry[j]-np.sin(-temp)*dir_xz_contact_ball_entry[j]
                 dirxz=np.sin(-temp)*dir_xx_contact_ball_entry[j]+np.cos(-temp)*dir_xz_contact_ball_entry[j]
                 temp=temp-np.pi
                
                 #ax.quiver(x0-x0b,y0-y0b, dirxx/mag2,dirxz/mag2,scale=20,color="k",width=.007,zorder=3)    
                 ax.quiver(x0,y0, fx/mag1,fy/mag1,scale=10, color="k",width=.007,zorder=10)      
                 #ax.quiver(x0-x0b,y0-y0b, Fx1,Fy1,scale=10, color="green",width=.007,zorder=1)
                 ax.quiver(x0,y0,C1[0,j],C1[1,j] ,color="magenta", scale=10,label='Positive cone')     
                 ax.quiver(x0,y0,C2[0,j],C2[1,j] ,color="magenta", scale=10,label='negative cone')
                 #ax.quiver(x0-x0b,y0-y0b, VXpp[0],VXpp[1],scale=15,color="b",width=.007,zorder=2)  
                 #ax.quiver(x0-x0b,y0-y0b, VYpp[0],VYpp[1],scale=15,color="r",width=.007,zorder=2)  
        
             else:
                 temp=temp
                 fx=F_contact_ballx_entry[j]
                 fy=F_contact_ballz_entry[j]
                 #ax.quiver(x0-x0b,y0-y0b, dir_xx_contact_ball_entry[j]/mag2,dir_xz_contact_ball_entry[j]/mag2,scale=20,color="k",width=.007,zorder=3)    
                 ax.quiver(x0,y0, F_contact_ballx_entry[j]/mag1,F_contact_ballz_entry[j]/mag1,scale=10, color="k",width=.007,zorder=5,label='Contact Force')      
                 #.quiver(x0-x0b,y0-y0b, Fx1,Fy1,scale=10, color="green",width=.007,zorder=1)
                 ax.quiver(x0,y0,C1[0,j],C1[1,j] ,color="magenta", scale=10,label='Positive cone')     
                 ax.quiver(x0,y0,C2[0,j],C2[1,j] ,color="magenta", scale=10,label='negative cone')
                 #ax.quiver(x0-x0b,y0-y0b, VXpp[0],VXpp[1],scale=15,color="b",width=.007,zorder=2)  
                 #ax.quiver(x0-x0b,y0-y0b, VYpp[0],VYpp[1],scale=15,color="r",width=.007,zorder=2)  
             
             ax.legend() 
             #ax.set_xlim([-2,-1])
             mag_=np.sqrt(fx**2 +fy**2)
             f_=np.array([fx/mag_,fy/mag_])
             temp2=np.round(np.nan_to_num(np.arccos(np.dot(f_ ,Vy[j,:]))),2)
 



         plt.savefig(directory+'/'+self.name+'_'+'time'+str(np.round(self.time[i],2))+name+'.jpg')
         plt.savefig(directory+'/'+self.name+'_'+'time'+str(np.round(self.time[i],2))+name+'.svg')    
         plt.savefig(directory+'/'+self.name+'_'+'time'+str(np.round(self.time[i],2))+name+'.pdf')
    
        
         plt.close('all')    

     #### Video create functions
     def create_frames_search(self,membrane,d,entry,title):
        ''' Create frames to show the robot and the pull force and epsilon metric in one plot'''
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        count=0
        i=entry    
        #for i in range(len(self.time)-2):    
  
        fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(1.5,1.5),dpi=300)
        axs.set_xlim([-d,d])
        axs.set_ylim([-d,d])
        plt.axis('off')
        xcenter=self.ballx_position[i]
        ycenter=self.ballz_position[i]
        
        x0b=self.ballx_position[2]
        y0b=self.ballz_position[2]
        wxmax=x0b+d
        wxmin=x0b-d
        wymax=y0b+d
        wymin=y0b-d
        
 

        #### Square
        if self.geom=="square":
            const=self.ball_radius*2
            rx=const
            ry=const
            w=rx/2
            h=ry/2                    
            x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
            y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]
            (self.segments)=self.create_segment(x,y) 
        
        #### Triangle
        if self.geom=="triangle":
            const=self.ball_radius*2*np.pi/3
            r=const*np.sqrt(3)/3
            x1=-r
            y1=0
            x2=-r*np.cos(2*np.pi/3)
            y2=r*np.sin(2*np.pi/3)
            x3=-r*np.cos(4*np.pi/3)
            y3=r*np.sin(4*np.pi/3)
           
            x__ = [x1,x2,x3,x1]
            y__ = [y1,y2,y3,y1]
           
            (self.segments)=self.create_segment(x__,y__)   
        
        
        
        #ax.plot(x,y)
        for j in range(0,self.nb):
            x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]
            if self.geom=="square":
                if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.1:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                    axs.add_patch(patch)
               
                else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                    axs.add_patch(patch) 
                    
            if self.geom=="triangle":
                if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.1:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                    axs.add_patch(patch)
               
                else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                    axs.add_patch(patch)                         
                    
            if self.geom=="circle":
    
                q=np.sqrt((x0-self.ballx_position[i])**2 + (y0-self.ballz_position[i])**2)
                if q<=(2 * self.bot_width/2 + self.ball_radius):
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                    axs.add_patch(patch)
               
                else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                    axs.add_patch(patch)  
                    
            else:
                if self.F_random_obj(x0-self.ballx_position[i],y0-self.ballz_position[i])<.2:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                    axs.add_patch(patch)
               
                else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                    axs.add_patch(patch)          
            
        if membrane==True:
            for j in range(0,self.nm):
                
                x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                patch = plt.Circle((x0, y0),self.skin_width/2, fc='tab:red')
                axs.add_patch(patch)
            
            

        for j in range(self.ni):
            x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

            if np.round(self.Rm[j],4)==0.0508:
                c='tab:blue'
            else:
                c='tab:green'
            patch = plt.Circle((x0, y0),self.Rm[j], fc=c)
            axs.add_patch(patch)         
 
        if self.control_mode=="grasping" or self.control_mode=="grasping_explore":
            if self.control_mode=="grasping_explore":
                x_=self.Rr_[i]*np.cos(self.THETA[i])
                y_=self.Rr_[i]*np.sin(self.THETA[i])
      
                #patch = plt.Circle((x_,y_),.05,fc='m',edgecolor='m',linewidth=1,zorder=2)
                #axs.add_patch(patch)
            
            #patch = plt.Circle((self.centroidx[i],self.centroidz[i]),.05,fc='tab:orange',edgecolor='tab:orange',linewidth=1,zorder=2)
            #axs.add_patch(patch)
 
        
            if self.geom=="square":
                const_=self.ball_radius*2
                x0,y0=self.ballx_position[i] - const_/2,self.ballz_position[i] - const_/2
                patch = matplotlib.patches.Rectangle((x0, y0),const_, const_,fc='none',edgecolor='black',linewidth=1)     
                axs.add_patch(patch)   
                
   
        
            #axs.plot([-d,d],[0,0],color='k',linestyle='--',linewidth=0.5,zorder=-3)
            #axs.plot([-d,d],[-d,d],color='k',linestyle='--',linewidth=0.5,zorder=-3)    
            #axs.plot([-d,d],[d,-d],color='k',linestyle='--',linewidth=0.5,zorder=-3)   
            #axs.plot([0,0],[-d,d],color='k',linestyle='--',linewidth=0.5,zorder=-3)
        axs.set_title(title,fontsize=9)
        plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+str(np.round(self.time[i],0))+"_ball_search.jpg")
        plt.savefig("C:/soro/python/Pychrono/Strings/String_grasping/paper_plots/"+str(np.round(self.time[i],0))+"_ball_search.svg")

        plt.close('all')  
        count=count+1 


     def create_frames_rfunction(self,entry):
        ''' Create frames for a video this was used to see the instantaneous R-function used for grasping_epsilon '''
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 9
        plt.rcParams['axes.linewidth'] = .1

        i=entry
        fig, axs = plt.subplots(figsize=(3,3),dpi=300)
        #ax = plt.axes(xlim=(wxmin,wxmax), ylim=(wymin, wymax))
        #plt.axis('off')
        x_=[]
        y_=[]
        #ax.plot(x,y)
        for j in range(0,self.nb):
            x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]
            x_.append(x0)
            y_.append(y0)
            
        j=0
        x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]
        x_.append(x0)
        y_.append(y0)
        #print(x_)
        delta=0.1
        wxmin=np.min(x_)-delta
        wxmax=np.max(x_)+delta
        wymin=np.min(y_)-delta
        wymax=np.max(y_)+delta

        ratio=(wymax-wymin)/(wxmax-wxmin)
        xt = np.arange(wxmin, wxmax, delta)
        yt = np.arange(wymin, wymax, delta)
        #print(xt)
        #print(yt)
        (X,Y) = np.meshgrid(xt, yt)
        
        (segments)=self.create_segment(x_,y_)
       # print(segments)
        #print(X)
        #print(Y)
        #phi=np.zeros((len(yt),len(xt)))
        #for ii in range(len(xt)):
            #for jj in range(len(yt)):
        phi=self.PHI(X,Y,segments)
        print(phi)
        xp=np.hstack([segments[:,0],segments[0,0]])
        yp=np.hstack([segments[:,1],segments[0,1]])
        fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3,3),dpi=300)
        #len(xp)
        #print(np.shape(segments))
        im1=axs.contourf(X, Y,phi,cmap = 'jet',levels=50,alpha=1,linestyles='solid') 
        axs.contour(X,Y,phi,levels = 50,colors=('k',),linestyles=('-',),linewidths=(.1))
        axs.contour(X,Y,phi,levels = [0],colors=('white',),linestyles=('-',),linewidths=(1))
        axs.plot(xp,yp,color="white")
        
        #axs.set_xticks(x_ticks)
        #axs.set_yticks(x_ticks)
        axs.set_title('$\phi(x)$')
        fig.colorbar(im1, ax=axs)
       

     def create_frames_snapshot(self,name,save,membrane,Directory,entry,wxmin,wxmax,wymin,wymax,fxs,fys,title_on):
        ''' Create frames for for single snapshot  '''
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 9
        plt.rcParams['axes.linewidth'] = .1

        i=entry
        fig, ax = plt.subplots(figsize=(fxs,fys),dpi=300)
        ax = plt.axes(xlim=(wxmin,wxmax), ylim=(wymin, wymax))
        plt.axis('off')

        xcenter=self.ballx_position[i]
        ycenter=self.ballz_position[i]
        
        
        if self.geom=="square":
            const=self.ball_radius*2
            rx=const
            ry=const
            w=rx/2
            h=ry/2                    
            x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
            y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]
            (self.segments)=self.create_segment(x,y) 
            
        if self.geom=="triangle":
            const=self.ball_radius*2*np.pi/3
            #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
            r=const*np.sqrt(3)/3
            x1=-r
            y1=0
           
            x2=-r*np.cos(2*np.pi/3)
            y2=r*np.sin(2*np.pi/3)
           
            x3=-r*np.cos(4*np.pi/3)
            y3=r*np.sin(4*np.pi/3)
           
            x__ = [x1,x2,x3,x1]
            y__ = [y1,y2,y3,y1]
           
            (self.segments)=self.create_segment(x__,y__)   
        
        
        if self.geom=="import": 
            R=1.25
            nsides=50
            theta=np.linspace(3*np.pi/2 + .415,(2*np.pi + 3*np.pi/2) - .415,nsides,endpoint=True)
            x__=[]
            y__=[]
            for iii in range(len(theta)):
                x__.append(R*np.cos(theta[iii]))
                y__.append(R*np.sin(theta[iii]))
                
            box1_width=1.007974825964066
            box1_length=box1_width/2
            box2_width=1.5
            Rr=np.sqrt(R**2 - (box1_width/2)**2)
              
            box1_x=-(Rr + box1_length/2)
            #box1_z=0 
            
            box2_width=1.5
            box2_length=.25
            
            #box2_x=box1_x - box1_length/2 - box2_length/2
            #box2_z=0 
            
            x__.append(-box1_width/2)
            y__.append(box1_x - box1_length/2)
            
            x__.append(-box2_width/2)
            y__.append(box1_x - box1_length/2)
            
            
            x__.append(-box2_width/2)
            y__.append(box1_x - box1_length/2 - box2_length)
            
            
            x__.append(box2_width/2)
            y__.append(box1_x - box1_length/2 - box2_length)
            
            
            x__.append(box2_width/2)
            y__.append(box1_x - box1_length/2)
            
            x__.append(box1_width/2)
            y__.append(box1_x - box1_length/2)
            
            x__.append(R*np.cos(theta[0]))
            y__.append(R*np.sin(theta[0]))
            #x__=np.dot(-1,x__)
            (self.segments)=self.create_segment(y__,x__)
        
        
        for j in range(0,self.nb):
            x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]
            if self.geom=="square":
                if self.PHI(x0,y0,self.segments)<.15:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:orange')
                    ax.add_patch(patch)
               
                else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                    ax.add_patch(patch) 
                    
            elif self.geom=="triangle":
                if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.1:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                    ax.add_patch(patch)
               
                else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                    ax.add_patch(patch)                         
                    
            elif self.geom=="circle":
                
                q=np.sqrt((x0-self.ballx_position[i])**2 + (y0-self.ballz_position[i])**2)
                if q<=(2 * self.bot_width/2 + self.ball_radius):
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:orange')
                    ax.add_patch(patch)
               
                else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                    ax.add_patch(patch)  
                    
                    
            elif self.geom=="import":
                if self.F_random_obj(x0-self.ballx_position[i],y0-self.ballz_position[i])<.2:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                    ax.add_patch(patch)
               
                else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                    ax.add_patch(patch)  
                
        if membrane==True:
            for j in range(0,self.nm):
                
                x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                patch = plt.Circle((x0, y0),self.skin_width/2, fc='tab:red')
                ax.add_patch(patch)
            
            

        for j in range(self.ni):
            x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

            if np.round(self.Rm[j],4)==0.0508:
                c='tab:blue'
            else:
                c='tab:green'
                
            patch = plt.Circle((x0, y0),self.Rm[j], fc=c)
            ax.add_patch(patch)         
 
        if self.control_mode=="grasping" or self.control_mode=="grasping_explore":
            if self.geom=="circle":
                x0,y0=self.ballx_position[i],self.ballz_position[i]
                patch = plt.Circle((x0, y0),self.ball_radius,fc='none',edgecolor='black',linewidth=1)
                ax.add_patch(patch)
        
            elif self.geom=="square":
                const_=self.ball_radius*2
                x0,y0=self.ballx_position[i] - const_/2,self.ballz_position[i] - const_/2
                patch = matplotlib.patches.Rectangle((x0, y0),const_, const_,fc='none',edgecolor='black',linewidth=1)     
                ax.add_patch(patch)   
                
            elif self.geom=="triangle":
                x0,y0=self.ballx_position[i],self.ballz_position[i] 
                const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                r=const*np.sqrt(3)/3
                #print(r)
                patch = matplotlib.patches.RegularPolygon((x0,y0),int(3),r,orientation=np.pi/2,fc='none',edgecolor='black',linewidth=1)
                ax.add_patch(patch) 
                #xp=np.hstack([self.segments[:,0],self.segments[0,0]])
                #yp=np.hstack([self.segments[:,1],self.segments[0,1]])
                #ax.plot(xp+x0,yp+y0,color='tab:red',linestyle='dashed',linewidth=2,zorder=0)
           
            elif self.geom=="import":
                ax.plot(y__,x__,color='k',linewidth=1)
        
        #ax.streamplot(self.X,self.Y,Fx1,Fz1,color='b',density = 2,linewidth=0.1,arrowsize=0.25)
        if title_on==True:
            plt.title('Time= ' + str(np.round(self.time[i],0))+' (s)',fontsize=9)
        plt.gca().set_aspect('equal', adjustable='box')
        
        if save==True:
            plt.savefig(Directory+'/'+name+'_'+'time'+str(np.round(self.time[i],2))+name+'.jpg')
            plt.savefig(Directory+'/'+name+'_'+'time'+str(np.round(self.time[i],2))+name+'.svg')    
            #plt.savefig(Directory+'/'+name+'_'+'time'+str(np.round(self.time[i],2))+name+'.pdf')
        
            
            plt.close('all')          

         

     def Forcechains(self):
        """ create plot force chains"""
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        direct = os.path.join(self.mainDirectory+self.name,'_forcechains')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        for i in range(len(self.time)-2):
            x0,y0=self.ballx_position[i],self.ballz_position[i]
            

            fig = plt.figure(dpi=300)
            fig.set_size_inches(4, 4)
            
            ax = plt.axes(xlim=(self.wxmin,self.wxmax), ylim=(self.wymin, self.wymax))
            cmap = plt.cm.get_cmap('seismic')
            boundaries=np.arange(1,100,1)
                           
            norm = colors.BoundaryNorm(boundaries, cmap.N, [boundaries[0], 100])
            #count=0
            #for i in range(1,len(self.time)-1):
            Fx=self.Contact_force_x[0:int(self.number_contacts[i]),i]
            Fz=self.Contact_force_z[0:int(self.number_contacts[i]),i]

            abs_force=np.power(np.add(np.power(Fx,2),np.power(Fz,2)),.5)
    

            x=self.Contact_points_x[0:int(self.number_contacts[i]),i]
            y=self.Contact_points_z[0:int(self.number_contacts[i]),i]
            x2=[]
            y2=[]
            F2=[]
            for j in range(len(abs_force)):
                x2.append(x[j])
                y2.append(y[j])
                F2.append(abs_force[j])
    
            plt.scatter(x2,y2,s=2*np.power(F2,.65),c=F2,cmap=cmap,norm=norm)
            plt.grid(True)
            plt.colorbar()         
            fig.suptitle('Time= ' + str(np.round(self.time[i],0)))
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')
        
        self.create_video('_forcechains','_forcechains')
        
        
     def Forcechains_arrows(self,d):
        """ create plot force chains"""
        membrane=True
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        direct = os.path.join(self.mainDirectory+self.name,'_contact_arrow_frames')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        for i in range(len(self.time)-1): 
            x0,y0=self.ballx_position[i],self.ballz_position[i]
            
            wxmax=x0+d
            wxmin=x0-d
            wymax=y0+d
            wymin=y0-d
            
            wxmin=-2.1
            wxmax=-2.1+1
            wymin=-0.7
            wymax=0.7
            
            const=(wxmax-wxmin)/(wymax-wymin)
            fig = plt.figure(dpi=300)
            fig.set_size_inches(const*4, 4)
            ax = plt.axes(xlim=(wxmin,wxmax), ylim=(wymin, wymax))
            const=self.ball_radius*2
            rx=const
            ry=const
            w=rx/2
            h=ry/2          
            xcenter=self.ballx_position[i]
            ycenter=self.ballz_position[i]
            x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
            y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]
            (self.segments)=self.create_segment(x,y) 
            x0,y0=self.ballx_position[i],self.ballz_position[i]
        
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]  
                if self.PHI(x0,y0,self.segments)<.1:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='none',edgecolor='tab:orange')
                    ax.add_patch(patch)
                else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='none',edgecolor="k")
                    ax.add_patch(patch) 
    
            if membrane==True:
                for j in range(0,self.nm):
    
                    x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                    patch = plt.Circle((x0, y0),self.skin_width/2, fc='none',edgecolor='tab:red')
                    ax.add_patch(patch)
                                
            
            if self.geom=="circle":
                x0,y0=self.ballx_position[i],self.ballz_position[i]
                patch = plt.Circle((x0, y0),self.ball_radius, fc='tab:grey')
                ax.add_patch(patch)
                
            if self.geom=="square":
                const_=self.ball_radius*2
                x0,y0=self.ballx_position[i]-const_/2,self.ballz_position[i] - const_/2
                
                patch = matplotlib.patches.Rectangle((x0, y0),const_, const_,fc='none',edgecolor='tab:grey')     
                ax.add_patch(patch)
                
            if self.geom=="triangle":
                const_=self.ball_radius*2*np.pi/3
                r=const_*np.sqrt(3)/3
                x0,y0=self.ballx_position[i],self.ballz_position[i]
                patch = RegularPolygon((x0,y0),3,r,orientation=-np.pi/2,fc='none',edgecolor='tab:grey')
                ax.add_patch(patch)             
    
             
            if self.geom=="import":
                R=1.25
                theta1=0.41
                theta2=2*np.pi-theta1
                theta=np.linspace(theta1,theta2,100)

                x1_=-R*np.cos(theta)
                y1_=R*np.sin(theta)


                a=.5
                b=1.65
                c=2.02

                w1=.5
                w2=.63
                
                x2_=[x1_[0],x1_[0]-a]
                y2_=[w1,w1]

                x3_=[-b,-b]
                y3_=[w1,w2]

                x4_=[-b,-c]
                y4_=[w2,w2]

                x5_=[-c,-c]
                y5_=[w2,-w2]

                x6_=[-c,-b]
                y6_=[-w2,-w2]

                x7_=[-b,-b]
                y7_=[-w1,-w2]

                x8_=[-b,x1_[-1]]
                y8_=[-w1,-w1]
                ax.plot(x1_,y1_,color='k')
                ax.plot(x2_,y2_,color='k')
                ax.plot(x3_,y3_,color='k')
                ax.plot(x4_,y4_,color='k')
                ax.plot(x5_,y5_,color='k')
                ax.plot(x6_,y6_,color='k')
                ax.plot(x7_,y7_,color='k')
                ax.plot(x8_,y8_,color='k') 
                   
            for j in range(self.ni):
                x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]
    
                if np.round(self.Rm[j],4)==0.0508:
                    c='tab:blue'
                else:
                    c='tab:green'
                patch = plt.Circle((x0, y0),self.Rm[j], fc="none",edgecolor=c)
                ax.add_patch(patch)          

            Dirxx=self.Dirxx_[0:int(self.number_contacts[i]),i]
            Dirxz=self.Dirxz_[0:int(self.number_contacts[i]),i]
            Dirzx=self.Dirzx_[0:int(self.number_contacts[i]),i]
            Dirzz=self.Dirzz_[0:int(self.number_contacts[i]),i]         
    
            Fx=self.x_contact_force2[0:int(self.number_contacts[i]),i]
            Fz=self.z_contact_force2[0:int(self.number_contacts[i]),i]
    
            abs_force=np.power(np.add(np.power(Fx,2),np.power(Fz,2)),.5)
    
    
            x=self.Contact_points_x[0:int(self.number_contacts[i]),i]
            y=self.Contact_points_z[0:int(self.number_contacts[i]),i]
            x0,y0=self.ballx_position[i],self.ballz_position[i]
    
            for j in range(len(abs_force)):
                mag=np.sqrt(Fx[j]**2 + Fz[j]**2)
                ax.quiver(x[j],y[j],Fx[j]/mag,Fz[j]/mag,color="purple",scale=10,width=0.005,zorder=1)
                mag2=np.sqrt(Dirxx[j]**2+Dirxz[j]**2)
                mag3=np.sqrt(Dirzx[j]**2+Dirzz[j]**2)
                
                ax.quiver(x[j],y[j],Dirxx[j]/mag2,Dirxz[j]/mag2,color="red",scale=20,width=0.005,zorder=2)
                ax.quiver(x[j],y[j],Dirzx[j]/mag3,Dirzz[j]/mag3,color="blue",scale=20,width=0.005,zorder=2)
    
            fig.suptitle('Time= ' + str(np.round(self.time[i],0)))
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')          
        self.create_video('_contact_arrow_frames','_contact_arrow_frames')        
        
        
     def create_frames(self,membrane,wxmin,wxmax,wymin,wymax):
        ''' Create frames of sim for robot'''
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        
        direct = os.path.join(self.mainDirectory+self.name,'_frames_combined')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        #Num_contact1=[]
       # Num_contact2=[]
        #for i in range(len(self.temp_id2)):
        #    Num_contact1.append(len(self.temp_id2[i]))
            
        for i in range(len(self.time)-2):    
  
            fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3,3),dpi=300)
            # epsilon 1
            
            #epsilon=np.asarray(self.EPSILON)
            xcenter=self.ballx_position[i]
            ycenter=self.ballz_position[i]
            
            x0b=self.ballx_position[2]
            y0b=self.ballz_position[2]
            #wxmax=x0b+d
            #wxmin=x0b-d
            #wymax=y0b+d
            #wymin=y0b-d
            
     
            axs.set_xlim([wxmin,wxmax])
            axs.set_ylim([wymin,wymax])
            #### Square
            if self.geom=="square":
                const=self.ball_radius*2
                rx=const
                ry=const
                w=rx/2
                h=ry/2                    
                x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
                y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]
                (self.segments)=self.create_segment(x,y) 
            
            #### Triangle
            if self.geom=="triangle":
                #const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                
                const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                r=const*np.sqrt(3)/3
                x1=-r
                y1=0
               
                x2=-r*np.cos(2*np.pi/3)
                y2=r*np.sin(2*np.pi/3)
               
                x3=-r*np.cos(4*np.pi/3)
                y3=r*np.sin(4*np.pi/3)
               
                x__ = [x1,x2,x3,x1]
                y__ = [y1,y2,y3,y1]
               
                (self.segments)=self.create_segment(x__,y__)   
            #### c shape
            if self.geom=="c_shape":
                #x0b,y0b=self.ballx_position[i],self.ballz_position[i]
                w=self.w/2
                l=self.l/2
                t=self.t
                x__=[w,-w,-w,w,w,-w+t,-w+t,w,w]
                y__=[-l,-l,l,l,l-t,l-t,-l+t,-l+t,-l]
                (self.segments)=self.create_segment(x__,y__)
                #x0b=x0b+np.sign(-w/2+(t/2))*(-w/2+(t/2))
                
                #z0b=z0b+np.sign(z0b)*(z0b)
                
            if self.geom=="import":
                
                R=1.25
                nsides=50
                theta=np.linspace(3*np.pi/2 + .415,(2*np.pi + 3*np.pi/2) - .415,nsides,endpoint=True)
                x__=[]
                y__=[]
                for iii in range(len(theta)):
                    x__.append(R*np.cos(theta[iii]))
                    y__.append(R*np.sin(theta[iii]))
                    
                
                box1_width=1.007974825964066
                box1_length=box1_width/2
                box2_width=1.5
                Rr=np.sqrt(R**2 - (box1_width/2)**2)
                  
                box1_x=-(Rr + box1_length/2)
                #box1_z=0 
                
                box2_width=1.5
                box2_length=.25
                
                #box2_x=box1_x - box1_length/2 - box2_length/2
                #box2_z=0 
                
                x__.append(-box1_width/2)
                y__.append(box1_x - box1_length/2)
                
                x__.append(-box2_width/2)
                y__.append(box1_x - box1_length/2)
                
                
                x__.append(-box2_width/2)
                y__.append(box1_x - box1_length/2 - box2_length)
                
                
                x__.append(box2_width/2)
                y__.append(box1_x - box1_length/2 - box2_length)
                
                
                x__.append(box2_width/2)
                y__.append(box1_x - box1_length/2)
                
                x__.append(box1_width/2)
                y__.append(box1_x - box1_length/2)
                
                x__.append(R*np.cos(theta[0]))
                y__.append(R*np.sin(theta[0]))
                #x__=np.dot(-1,x__)
                (self.segments)=self.create_segment(y__,x__)
            #ax.plot(x,y)
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]
                if self.geom=="square":
                    #if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.1:
                        #patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        #axs[0].add_patch(patch)
                   
                    #else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                    axs.add_patch(patch) 
                        
                if self.geom=="triangle":
                    if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.1:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs.add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs.add_patch(patch)                         
                
                        
                
                if self.geom=="c_shape":
                    difx=self.ballx_position[0]-self.ballx_
                   
                    dify=self.ballz_position[0]-self.ballz_
                    
                    if self.PHI(x0+(x0b-difx),y0+(x0b-difx),self.segments)<.1:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs.add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs.add_patch(patch)    
                        
                if self.geom=="circle":
                    
                    q=np.sqrt((x0-self.ballx_position[i])**2 + (y0-self.ballz_position[i])**2)
                    #if q<=(2 * self.bot_width/2 + self.ball_radius):
                        #patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        #axs[0].add_patch(patch)
                   
                    #else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                    axs.add_patch(patch)  
                        
                if self.geom=="import":
                    #if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.5:
                        #patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        #axs[0].add_patch(patch)
                   
                    #else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                    axs.add_patch(patch)          
                
            if membrane==True:
                for j in range(0,self.nm):
                    
                    x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                    patch = plt.Circle((x0, y0),self.skin_width/2, fc='tab:red')
                    axs.add_patch(patch)
                
                

            for j in range(self.ni):
                x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

                if np.round(self.Rm[j],4)==0.0508:
                    c='tab:blue'
                else:
                    c='tab:green'
                patch = plt.Circle((x0, y0),self.Rm[j], fc=c)
                axs.add_patch(patch)         
     
            if self.control_mode=="grasping" or self.control_mode=="grasping_explore" or self.control_mode=="grasping_u":

                if self.geom=="circle":
                    x0,y0=self.ballx_position[i]+self.ballx,self.ballz_position[i]+self.ballz
                    patch = plt.Circle((x0, y0),self.ball_radius,fc='none',edgecolor='black',linewidth=1)
                    axs.add_patch(patch)
            
                if self.geom=="square":
                    const_=self.ball_radius*2
                    x0,y0=self.ballx_position[i] - const_/2,self.ballz_position[i] - const_/2
                    patch = matplotlib.patches.Rectangle((x0, y0),const_, const_,fc='none',edgecolor='black',linewidth=1)     
                    axs.add_patch(patch)   
                    
                if self.geom=="triangle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i] 
                    const=self.ball_radius*2*np.pi/3
                    #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                    r=const*np.sqrt(3)/3
                    #print(r)
                    patch = matplotlib.patches.RegularPolygon((x0,y0),int(3),r,orientation=np.pi/2,fc='none',edgecolor='black',linewidth=1)
                    axs.add_patch(patch) 
                    #xp=np.hstack([self.segments[:,0],self.segments[0,0]])
                    #yp=np.hstack([self.segments[:,1],self.segments[0,1]])
                    #axs[0].plot(xp+x0,yp+y0,color='tab:red',linestyle='dashed',linewidth=2,zorder=0)
            
            
                if self.geom=="c_shape":
                   x0b,y0b=self.ballx_position[i],self.ballz_position[i] 
                   difx=self.ballx_position[0]-self.ballx_
                   
                   dify=self.ballz_position[0]-self.ballz_
                   
                   x__=x__+np.ones(len(x__))*(x0b-difx)
                   y__=y__+np.ones(len(x__))*(y0b-dify)
                   axs.plot(x__,y__)
                   
                if self.geom=="import":  
                   x0b,y0b=self.ballx_position[i],self.ballz_position[i] 
                   difx=self.ballx_position[0]-self.ballx_
                   
                   dify=self.ballz_position[0]-self.ballz_
                   
                   x__=x__+np.ones(len(x__))*(y0b-dify)
                   y__=y__+np.ones(len(x__))*(x0b-difx)
                   axs.plot(y__,x__,color="k")
                    
            axs.set_title('Time= ' + str(np.round(self.time[i],0)),fontsize=12)

            print(str(i)+ "of"+ str(len(self.time-1)))
            #plt.gca().set_aspect('equal', adjustable='box')
            #fig.delaxes(axs[1,1])
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')          
        self.create_video('_frames_combined','_frames_combined') 


    
     def create_frames_u3(self,membrane,wxmin,wxmax,wymin,wymax):
        ''' Create frames to show the robot and the pull force and epsilon metric in one plot'''
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        
        direct = os.path.join(self.mainDirectory+self.name,'_frames_combinedu3')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
 
        a = self.rtilda
        b = self.rtilda
        delta = .01
        #x = np.arange(wxmin,wxmax, delta)
        #y = np.arange(wymin,wymax, delta)
        #X,Y=np.meshgrid(x,y)
        #zeta1,zeta2=self.Psi.field4(x,y,self.xc1,self.yc1,self.xc2,self.yc2,self.rtilda,self.Psi.rho_)

        for i in range(len(self.time)-2):
            #theta=(1-self.tanh(t))*np.pi/2 + self.tanh(t)*0
            x_=[]
            y_=[]

            theta=np.pi/2
            #theta=(1-self.Psi.ft2(self.time[i],self.tcut1,self.p))*np.pi/2 + self.Psi.ft2(self.time[i],self.tcut1,self.p)*0
            
            self.lengtho2_grasp = self.lengtho_grasp + 2*self.width_grasp*np.tan((np.pi/2 - theta)/2)
            self.length2_grasp = self.length_grasp +self.width_grasp*np.tan((np.pi/2 - theta)/2)

            x_.append(self.length_grasp*np.cos(theta) + self.xcenter_grasp)
            y_.append(self.length_grasp*np.sin(theta) + self.lengtho_grasp/2 + self.ycenter_grasp)

            x_.append(0 + self.xcenter_grasp)
            y_.append(self.lengtho_grasp/2 + self.ycenter_grasp)

            x_.append(0 + self.xcenter_grasp)
            y_.append(-self.lengtho_grasp/2 + self.ycenter_grasp)

            x_.append(self.length_grasp*np.cos(-theta) + self.xcenter_grasp)
            y_.append(self.length_grasp*np.sin(-theta) - self.lengtho_grasp/2 + self.ycenter_grasp)


            x_.append(self.length2_grasp*np.cos(-theta) + self.xcenter_grasp - self.width_grasp)
            y_.append(self.length2_grasp*np.sin(-theta) - self.lengtho2_grasp/2 + self.ycenter_grasp)
        
            x_.append(0 + self.xcenter_grasp - self.width_grasp)
            y_.append(-self.lengtho2_grasp/2 + self.ycenter_grasp)
   
            x_.append(0 + self.xcenter_grasp - self.width_grasp)
            y_.append(self.lengtho2_grasp/2 + self.ycenter_grasp)
   
            x_.append(self.length2_grasp*np.cos(theta) - self.width_grasp + self.xcenter_grasp)
            y_.append(self.length2_grasp*np.sin(theta) + self.lengtho2_grasp/2 + self.ycenter_grasp)
        
            x_.append(self.length_grasp*np.cos(theta) + self.xcenter_grasp)
            y_.append(self.length_grasp*np.sin(theta) + self.lengtho_grasp/2 + self.ycenter_grasp)
        
  
            fig, axs = plt.subplots(nrows=2, ncols=2,figsize=(6.5,6.5),dpi=300)
            bx=self.ballx_position
         #x_ticks = np.linspace(self.time[0], self.time[-2],5,endpoint=True)
            #epsilon=self.EPSILON_
            axs[0,1].plot(self.time[:i],bx[:i],color='tab:blue',linewidth=1)
            
            #axs[0,1].plot(self.time_tilda[:i],self.epsilon_tilda[:i],color='tab:green',linewidth=2,label=r"$\bar{\epsilon}$")
            axs[0,1].scatter(self.time[i-1],bx[i-1],color='k',s=30)
            #axs[0,1].scatter(self.time_tilda[i-1],self.epsilon_tilda[i-1],color='k',s=30)
            axs[0,1].set_title(r'ballx='+str(np.round(bx[i-1],2)))
            axs[0,1].set_ylabel('ballx',labelpad=1)
            axs[0,1].set_xlabel('time [s]',labelpad=-2)
            axs[0,1].xaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[0,1].yaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[0,1].grid(True) 
            
            
            
            # dx=np.gradient(self.Field_value_sum,self.time)
            # axs[1,1].plot(self.time[:i],dx[:i],color='tab:orange',linewidth=1)
            # axs[1,1].scatter(self.time[i-1],dx[i-1],color='k',s=30)
            # #x_ticks = np.linspace(self.time[0], self.time[-1],5,endpoint=True)
            # #y_ticks = np.linspace(np.min(self.bFx), np.max(self.bFx),5,endpoint=True)
            # axs[1,1].xaxis.set_tick_params(width=.25,length=2,pad=1)
            # axs[1,1].yaxis.set_tick_params(width=.25,length=2,pad=1)
            # #axs[0].set_ylim([0,100])
         
            # #axs[0].set_xticks(np.round(x_ticks,2))
            # #axs[0].set_yticks(np.round(y_ticks,2))
            # axs[1,1].set_title('D$\Sigma \phi$'+str(np.round(dx[i],2)))
            # #axs[0].set_ylabel('ball Force  [x]',labelpad=1)         
            # axs[1,1].grid(True) 
            
            axs[1,1].plot(self.time[:i],self.Field_value_sum[:i],color='tab:orange',linewidth=1)
            axs[1,1].scatter(self.time[i-1],self.Field_value_sum[i-1],color='k',s=30)
            axs[1,1].xaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[1,1].yaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[1,1].set_title('$\Sigma \phi$'+str(np.round(self.Field_value_sum[i],2)))
            #axs[0].set_ylabel('ball Force  [x]',labelpad=1)         
            axs[1,1].grid(True) 
                      
            
            
            axs[1,0].plot(self.time[:i],np.dot(self.Field_value_sum[:i],1/self.nb),color='tab:red',linewidth=1)
            axs[1,0].scatter(self.time[i-1],np.dot(self.Field_value_sum[i-1],1/self.nb),color='k',s=30)

            axs[1,0].xaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[1,0].yaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[1,0].set_title('$\phi_{avg}$'+str(np.round(self.Field_value_sum[i]/self.nb,2)))
            #axs[0].set_ylabel('ball Force  [x]',labelpad=1)         
            axs[1,0].grid(True) 
            
            
            if self.TRIG1_[i]==0:
                patch = plt.Circle((self.xc2,self.yc2),.1, fc='tab:red')
                axs[0,0].add_patch(patch)
                axs[0,0].plot(x_,y_,color='tab:red',linestyle='dashed',linewidth=1,zorder=2)
                #axs[0,0].plot(x_,y_,color='tab:red',linestyle='dashed',linewidth=1,zorder=2)
            else:
                #axs[0,0].streamplot(X, Y, zeta1, zeta2, color = 'tab:red', density = 1, linewidth = 0.25, arrowsize = 0.5, arrowstyle ='->')
                patch = plt.Circle((self.xc2,self.yc2),.05, fc='m')
                axs[0,0].add_patch(patch)
            xcenter=self.ballx_position[i]
            ycenter=self.ballz_position[i]
            
            x0b=self.ballx_position[2]
            y0b=self.ballz_position[2]
            #wxmax=x0b+d
            #wxmin=x0b-d
            #wymax=y0b+d
            #wymin=y0b-d
            
     
            axs[0,0].set_xlim([wxmin,wxmax])
            axs[0,0].set_ylim([wymin,wymax])
            #### Square
            if self.geom=="square":
                const=self.ball_radius*2
                rx=const
                ry=const
                w=rx/2
                h=ry/2                    
                x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
                y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]
                (self.segments)=self.create_segment(x,y) 
            
            #### Triangle
            if self.geom=="triangle":
                #const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                
                const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                r=const*np.sqrt(3)/3
                x1=-r
                y1=0
               
                x2=-r*np.cos(2*np.pi/3)
                y2=r*np.sin(2*np.pi/3)
               
                x3=-r*np.cos(4*np.pi/3)
                y3=r*np.sin(4*np.pi/3)
               
                x__ = [x1,x2,x3,x1]
                y__ = [y1,y2,y3,y1]
               
                (self.segments)=self.create_segment(x__,y__)   
            #### c shape
            if self.geom=="c_shape":
                #x0b,y0b=self.ballx_position[i],self.ballz_position[i]
                w=self.w/2
                l=self.l/2
                t=self.t
                x__=[w,-w,-w,w,w,-w+t,-w+t,w,w]
                y__=[-l,-l,l,l,l-t,l-t,-l+t,-l+t,-l]
                (self.segments)=self.create_segment(x__,y__)

            
            #ax.plot(x,y)
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]
                if self.geom=="square":
                    #if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.1:
                        #patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        #axs[0].add_patch(patch)
                   
                    #else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                    axs[0,0].add_patch(patch) 
                        
                if self.geom=="triangle":
                    if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.1:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[0,0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[0,0].add_patch(patch)                         
                
                        
                
                if self.geom=="c_shape":
                    #difx=self.ballx_position[0]-self.ballx_
                   
                    #dify=self.ballz_position[0]-self.ballz_
                    
                    #if self.PHI(x0+(x0b-difx),y0+(x0b-difx),self.segments)<.1:
                       # patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        #axs[0,0].add_patch(patch)
                   
                    #else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                    axs[0,0].add_patch(patch)    
                        
                if self.geom=="circle":
                    
                    q=np.sqrt((x0-self.ballx_position[i])**2 + (y0-self.ballz_position[i])**2)
                    #if q<=(2 * self.bot_width/2 + self.ball_radius):
                        #patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        #axs[0].add_patch(patch)
                   
                    #else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                    axs[0,0].add_patch(patch)  
                               
                
            if membrane==True:
                for j in range(0,self.nm):
                    
                    x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                    patch = plt.Circle((x0, y0),self.skin_width/2, fc='tab:red')
                    axs[0,0].add_patch(patch)
                
                

            for j in range(self.ni):
                x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

                if np.round(self.Rm[j],4)==0.0508:
                    c='tab:blue'
                else:
                    c='tab:green'
                patch = plt.Circle((x0, y0),self.Rm[j], fc=c)
                axs[0,0].add_patch(patch)         
     
            if self.control_mode=="grasping" or self.control_mode=="grasping_explore" or self.control_mode=="grasping_u":

                if self.geom=="circle":
                    x0,y0=self.ballx_position[i]+self.ballx,self.ballz_position[i]+self.ballz
                    patch = plt.Circle((x0, y0),self.ball_radius,fc='none',edgecolor='black',linewidth=1)
                    axs[0,0].add_patch(patch)
            
                if self.geom=="square":
                    const_=self.ball_radius*2
                    x0,y0=self.ballx_position[i] - const_/2,self.ballz_position[i] - const_/2
                    patch = matplotlib.patches.Rectangle((x0, y0),const_, const_,fc='none',edgecolor='black',linewidth=1)     
                    axs[0,0].add_patch(patch)   
                    
                if self.geom=="triangle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i] 
                    const=self.ball_radius*2*np.pi/3
                    #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                    r=const*np.sqrt(3)/3
                    #print(r)
                    patch = matplotlib.patches.RegularPolygon((x0,y0),int(3),r,orientation=np.pi/2,fc='none',edgecolor='black',linewidth=1)
                    axs[0,0].add_patch(patch) 
                    #xp=np.hstack([self.segments[:,0],self.segments[0,0]])
                    #yp=np.hstack([self.segments[:,1],self.segments[0,1]])
                    #axs[0].plot(xp+x0,yp+y0,color='tab:red',linestyle='dashed',linewidth=2,zorder=0)
            
            
                if self.geom=="c_shape":
                   x0b,y0b=self.ballx_position[i],self.ballz_position[i] 
                   difx=self.ballx_position[0]-self.ballx_
                   
                   dify=self.ballz_position[0]-self.ballz_
                   
                   x__=x__+np.ones(len(x__))*(x0b-difx)
                   y__=y__+np.ones(len(x__))*(y0b-dify)
                   axs[0,0].plot(x__,y__,color='k')
                   
                if self.geom=="import":  
                   x0b,y0b=self.ballx_position[i],self.ballz_position[i] 
                   difx=self.ballx_position[0]-self.ballx_
                   
                   dify=self.ballz_position[0]-self.ballz_
                   
                   x__=x__+np.ones(len(x__))*(y0b-dify)
                   y__=y__+np.ones(len(x__))*(x0b-difx)
                   axs[0,0].plot(y__,x__,color="k")
                    
            axs[0,0].set_title('Time= ' + str(np.round(self.time[i],0)),fontsize=12)
            #print(str(i)+ "of"+ str(len(self.time-1))+" "+"theta="+str(np.round(theta,2)))
            print(str(i)+ "of"+ str(len(self.time-1)))
            #plt.gca().set_aspect('equal', adjustable='box')
            #fig.delaxes(axs[1,1])
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')          
        self.create_video('_frames_combinedu3','_frames_combinedu3') 

     def create_frames_target(self,membrane,wxmin,wxmax,wymin,wymax):
        ''' Create frames to show the robot and the pull force and epsilon metric in one plot'''
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        
        direct = os.path.join(self.mainDirectory+self.name,'_frames_combined')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        for i in range(len(self.time)):
            ratio  = (wymax-wymin)/(wxmax-wxmin)
            fsx=5
            fsy=5*ratio
            fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(fsx,fsy),dpi=300)
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]
                patch = plt.Circle((-x0, y0),self.bot_width/2, fc='k')
                axs.add_patch(patch) 
                        
                
            if membrane==True:
                for j in range(0,self.nm):
                    
                    x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                    patch = plt.Circle((-x0, y0),self.skin_width/2, fc='tab:red')
                    axs.add_patch(patch)
                
                

            for j in range(self.ni):
                x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

                if np.round(self.Rm[j],4)==0.0508:
                    c='tab:blue'
                else:
                    c='tab:green'
                patch = plt.Circle((-x0, y0),self.Rm[j], fc=c)
                axs.add_patch(patch)  
                
            if self.tunnel_geom == "tanh":
                c1=0
                c2=10            
                x2=np.linspace(c1,c2,100)
                x1=np.linspace(c1,c2,100)                
                line1=np.tanh(x1-5)-3.75-.25
                line2=np.tanh(x2-5)-2.25+.25 
                axs.fill_between(np.flipud(x1),np.flipud(line2),color='tab:grey',alpha=1)
                axs.fill_between(np.flipud(x1),np.flipud(line1),-6.25,color='tab:grey',alpha=1)            
                #axs.plot(np.flipud(x1),np.flipud(line2),color='k',linewidth=1)
                #axs.plot(np.flipud(x1),np.flipud(line1),color='k',linewidth=1) 
            if self.tunnel_geom == "sin":
                c1=0
                c2=10 
                x2=np.linspace(c1,c2,1000)
                x1=np.linspace(c1,c2,1000)                
                line1=np.sin(1.745*x1)-4
                line2=np.sin(1.745*x1)-1.5
                axs.fill_between(np.flipud(x1),np.flipud(line2),color='tab:grey',alpha=1)
                axs.fill_between(np.flipud(x1),np.flipud(line1),-6.25,color='tab:grey',alpha=1) 
                
            
            
            axs.set_xlim([wxmin,wxmax])
            axs.set_ylim([wymin,wymax])
            axs.set_title('Time= ' + str(np.round(self.time[i],0)),fontsize=12)
            print(str(i)+ "of"+ str(len(self.time-1)))
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')          
        self.create_video('_frames_combined','_frames_combined') 

     def create_frames_target_snapshot(self,entry,membrane,wxmin,wxmax,wymin,wymax,fsx,fsy):
        ''' Create frames to show the robot and the pull force and epsilon metric in one plot'''
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        
        #direct = os.path.join(self.mainDirectory+self.name,'_frames_combined')    
       # if not os.path.isdir(direct):
            #os.makedirs(direct)
        #count=0
        fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(fsx,fsy),dpi=300)
        axs.scatter(-self.xtarget,self.ytarget,color='tab:cyan',s=80,marker='*') 
        
        if self.tunnel_geom == "tanh":
            c1=0
            c2=10            
            x2=np.linspace(c1,c2,100)
            x1=np.linspace(c1,c2,100)                
            line1=np.tanh(x1-5)-3.75-.25
            line2=np.tanh(x2-5)-2.25+.25 
            axs.fill_between(np.flipud(x1),np.flipud(line2),color='tab:grey',alpha=1)
            axs.fill_between(np.flipud(x1),np.flipud(line1),-6.25,color='tab:grey',alpha=1)            
            #axs.plot(np.flipud(x1),np.flipud(line2),color='k',linewidth=1)
            #axs.plot(np.flipud(x1),np.flipud(line1),color='k',linewidth=1) 
            axs.set_xlim([wxmin,wxmax])
            axs.set_ylim([wymin,wymax])
        if self.tunnel_geom == "sin":
            c1=0
            c2=10 
            x2=np.linspace(c1,c2,1000)
            x1=np.linspace(c1,c2,1000)                
            line1=np.sin(1.745*x1)-4
            line2=np.sin(1.745*x1)-1.5
            axs.fill_between(np.flipud(x1),np.flipud(line2),color='tab:grey',alpha=1)
            axs.fill_between(np.flipud(x1),np.flipud(line1),-6.25,color='tab:grey',alpha=1) 


        for i in entry:
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]
                patch = plt.Circle((-x0, y0),self.bot_width/2, fc='k')
                axs.add_patch(patch) 
                        
                
            if membrane==True:
                for j in range(0,self.nm):
                    x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                    patch = plt.Circle((-x0, y0),self.skin_width/2, fc='tab:red')
                    axs.add_patch(patch)
                
                

            for j in range(self.ni):
                x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

                if np.round(self.Rm[j],4)==0.0508:
                    c='tab:blue'
                else:
                    c='tab:green'
                patch = plt.Circle((-x0, y0),self.Rm[j], fc=c)
                axs.add_patch(patch) 
            axs.text(-self.bot_position_x[0,i],self.bot_position_z[0,i], 'T = '+str(np.round(self.time[i],0))+' s', color='k',fontsize=8,horizontalalignment='center',verticalalignment='center')
        
        
        axs.text(-self.bot_position_x[0,entry[0]],self.bot_position_z[0,entry[0]], 'Start', color='k',fontsize=8,horizontalalignment='center',verticalalignment='center')
        axs.text(-self.bot_position_x[0,entry[-1]],self.bot_position_z[0,entry[-1]], 'End', color='k',fontsize=8,horizontalalignment='center',verticalalignment='center')
        axs.text(11, -5.5, '1 m', color='k',fontsize=8,horizontalalignment='center',verticalalignment='center')

        
        axs.annotate('', xy=(11,-5), xytext=(12,-5),arrowprops=dict(arrowstyle='|-|', color='k',linewidth=0.5))
        axs.set_xlim([wxmin,wxmax])
        axs.set_ylim([wymin,wymax])
        plt.axis('off')
        axs.set_aspect('equal')
        plt.savefig(self.mainDirectory+self.name+"/snap_shots_"+".svg")
        plt.savefig(self.mainDirectory+self.name+"/snap_shots_"+".png")
        plt.close('all')   
        #plt.savefig(direct+"/frame%04d.jpg" % count)
                 


     def create_frames_u3_snapshots(self,membrane,wxmin,wxmax,wymin,wymax,fsx,fsy,xticks,yticks,entry):
        ''' Create frames to show the robot and the pull force and epsilon metric in one plot'''
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 9
        plt.rcParams['axes.linewidth'] = .1  
        count=0
        ratio  = (wymax-wymin)/(wxmax-wxmin)
        
        for i in entry:
            #theta=(1-self.tanh(t))*np.pi/2 + self.tanh(t)*0
            x_=[]
            y_=[]

            theta=np.pi/2
            self.lengtho2_grasp = self.lengtho_grasp + 2*self.width_grasp*np.tan((np.pi/2 - theta)/2)
            self.length2_grasp = self.length_grasp +self.width_grasp*np.tan((np.pi/2 - theta)/2)

            x_.append(self.length_grasp*np.cos(theta) + self.xcenter_grasp)
            y_.append(self.length_grasp*np.sin(theta) + self.lengtho_grasp/2 + self.ycenter_grasp)

            x_.append(0 + self.xcenter_grasp)
            y_.append(self.lengtho_grasp/2 + self.ycenter_grasp)

            x_.append(0 + self.xcenter_grasp)
            y_.append(-self.lengtho_grasp/2 + self.ycenter_grasp)

            x_.append(self.length_grasp*np.cos(-theta) + self.xcenter_grasp)
            y_.append(self.length_grasp*np.sin(-theta) - self.lengtho_grasp/2 + self.ycenter_grasp)


            x_.append(self.length2_grasp*np.cos(-theta) + self.xcenter_grasp - self.width_grasp)
            y_.append(self.length2_grasp*np.sin(-theta) - self.lengtho2_grasp/2 + self.ycenter_grasp)
        
            x_.append(0 + self.xcenter_grasp - self.width_grasp)
            y_.append(-self.lengtho2_grasp/2 + self.ycenter_grasp)
   
            x_.append(0 + self.xcenter_grasp - self.width_grasp)
            y_.append(self.lengtho2_grasp/2 + self.ycenter_grasp)
   
            x_.append(self.length2_grasp*np.cos(theta) - self.width_grasp + self.xcenter_grasp)
            y_.append(self.length2_grasp*np.sin(theta) + self.lengtho2_grasp/2 + self.ycenter_grasp)
        
            x_.append(self.length_grasp*np.cos(theta) + self.xcenter_grasp)
            y_.append(self.length_grasp*np.sin(theta) + self.lengtho_grasp/2 + self.ycenter_grasp)
        
  
            fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(fsx,fsy),dpi=300)
            
            if self.TRIG1_[i]==0:
                axs.plot(x_,y_,color='m',linestyle='dashed',linewidth=0.5,zorder=-5)
            else:    
                patch = plt.Circle((self.xc2,self.yc2),.1, fc='m')
                axs.add_patch(patch)
            
            xcenter=self.ballx_position[i]
            ycenter=self.ballz_position[i]
            
            x0b=self.ballx_position[2]
            y0b=self.ballz_position[2]

            axs.set_xlim([wxmin,wxmax])
            axs.set_ylim([wymin,wymax])
            #### Square
            if self.geom=="square":
                const=self.ball_radius*2
                rx=const
                ry=const
                w=rx/2
                h=ry/2                    
                x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
                y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]
                (self.segments)=self.create_segment(x,y) 
            
      
  
            
            #ax.plot(x,y)
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]
                patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                axs.add_patch(patch) 

            if membrane==True:
                for j in range(0,self.nm):
                    x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                    patch = plt.Circle((x0, y0),self.skin_width/2, fc='tab:red')
                    axs.add_patch(patch)

            for j in range(self.ni):
                x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

                if np.round(self.Rm[j],4)==0.0508:
                    c='tab:blue'
                else:
                    c='tab:green'
                patch = plt.Circle((x0, y0),self.Rm[j], fc=c)
                axs.add_patch(patch)         
     
            if self.control_mode=="grasping" or self.control_mode=="grasping_explore" or self.control_mode=="grasping_u":

                if self.geom=="circle":
                    x0,y0=self.ballx_position[i]+self.ballx,self.ballz_position[i]+self.ballz
                    patch = plt.Circle((x0, y0),self.ball_radius,fc='none',edgecolor='black',linewidth=1)
                    axs.add_patch(patch)
            
                if self.geom=="square":
                    const_=self.ball_radius*2
                    x0,y0=self.ballx_position[i] - const_/2,self.ballz_position[i] - const_/2
                    patch = matplotlib.patches.Rectangle((x0, y0),const_, const_,fc='none',edgecolor='black',linewidth=1)     
                    axs.add_patch(patch) 
                    
            axs.set_yticks(yticks)
            axs.set_xticks(xticks)
            axs.set_ylabel("$y$ (m)",labelpad=-1)
            axs.set_xlabel("$x$(m)",labelpad=-1)
            axs.xaxis.set_tick_params(width=.25,length=2)
            axs.yaxis.set_tick_params(width=.25,length=2)
            plt.axis('off')
            axs.set_title('T= ' + str(np.floor(self.time[i]))+" s",fontsize=8)
            print(str(count)+ "of"+ str(len(entry)))
            count=count+1
            plt.gca().set_aspect('equal', adjustable='box')
            plt.savefig(self.mainDirectory+self.name+"/snap_shot_"+str(self.time[i])+".svg")
            plt.savefig(self.mainDirectory+self.name+"/snap_shot_"+str(self.time[i])+".png")
            plt.close('all')
     def create_frames_pull_potfield(self,membrane,wxmin,wxmax,wymin,wymax):
        ''' Create frames to show the robot and the pull force and epsilon metric in one plot'''
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        
        direct = os.path.join(self.mainDirectory+self.name,'_frames_combined_potfield')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0

        for i in range(len(self.time)-2):    
  
            fig, axs = plt.subplots(nrows=1, ncols=3,figsize=(9,3),dpi=300)
            # epsilon 1
            
            
            axs[1].plot(self.time[:i],self.Field_value_sum[:i],color='tab:red',linewidth=1)
            axs[1].scatter(self.time[i-1],self.Field_value_sum[i-1],color='tab:blue',s=30)
            #x_ticks = np.linspace(self.time[0], self.time[-1],5,endpoint=True)
            #y_ticks = np.linspace(np.min(self.bFx), np.max(self.bFx),5,endpoint=True)
            axs[1].xaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[1].yaxis.set_tick_params(width=.25,length=2,pad=1)
            #axs[0].set_ylim([0,100])
         
            #axs[0].set_xticks(np.round(x_ticks,2))
            #axs[0].set_yticks(np.round(y_ticks,2))
            axs[1].set_title('$\Sigma \phi$'+str(np.round(self.Field_value_sum[-1],2)))
            #axs[0].set_ylabel('ball Force  [x]',labelpad=1)         
            axs[1].grid(True) 
            
            
            
            #epsilon=np.asarray(self.EPSILON)
            epsilon=self.EPSILON_
            axs[2].plot(self.time[:i],epsilon[:i],color='tab:blue',linewidth=1)
            axs[2].scatter(self.time[i-1],epsilon[i-1],color='tab:red',s=30)
            axs[2].set_title(r'$\epsilon_{1}$='+str(np.round(epsilon[i-1],2)))
            axs[2].set_ylabel('$\epsilon_{1}$',labelpad=1)
            axs[2].set_xlabel('time [s]',labelpad=-2)
            axs[2].xaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[2].yaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[2].grid(True) 

            xcenter=self.ballx_position[i]
            ycenter=self.ballz_position[i]
            
            x0b=self.ballx_position[2]
            y0b=self.ballz_position[2]
            #wxmax=x0b+d
            #wxmin=x0b-d
            #wymax=y0b+d
            #wymin=y0b-d
            
     
            axs[0].set_xlim([wxmin,wxmax])
            axs[0].set_ylim([wymin,wymax])
            #### Square
            if self.geom=="square":
                const=self.ball_radius*2
                rx=const
                ry=const
                w=rx/2
                h=ry/2                    
                x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
                y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]
                (self.segments)=self.create_segment(x,y) 
            
            #### Triangle
            if self.geom=="triangle":
                #const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                
                const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                r=const*np.sqrt(3)/3
                x1=-r
                y1=0
               
                x2=-r*np.cos(2*np.pi/3)
                y2=r*np.sin(2*np.pi/3)
               
                x3=-r*np.cos(4*np.pi/3)
                y3=r*np.sin(4*np.pi/3)
               
                x__ = [x1,x2,x3,x1]
                y__ = [y1,y2,y3,y1]
               
                (self.segments)=self.create_segment(x__,y__)   
            #### c shape
            if self.geom=="c_shape":
                #x0b,y0b=self.ballx_position[i],self.ballz_position[i]
                w=self.w/2
                l=self.l/2
                t=self.t
                x__=[w,-w,-w,w,w,-w+t,-w+t,w,w]
                y__=[-l,-l,l,l,l-t,l-t,-l+t,-l+t,-l]
                (self.segments)=self.create_segment(x__,y__)
                #x0b=x0b+np.sign(-w/2+(t/2))*(-w/2+(t/2))
                
                #z0b=z0b+np.sign(z0b)*(z0b)
                
            if self.geom=="import":
                
                R=1.25
                nsides=50
                theta=np.linspace(3*np.pi/2 + .415,(2*np.pi + 3*np.pi/2) - .415,nsides,endpoint=True)
                x__=[]
                y__=[]
                for iii in range(len(theta)):
                    x__.append(R*np.cos(theta[iii]))
                    y__.append(R*np.sin(theta[iii]))
                    
                
                box1_width=1.007974825964066
                box1_length=box1_width/2
                box2_width=1.5
                Rr=np.sqrt(R**2 - (box1_width/2)**2)
                  
                box1_x=-(Rr + box1_length/2)
                #box1_z=0 
                
                box2_width=1.5
                box2_length=.25
                
                #box2_x=box1_x - box1_length/2 - box2_length/2
                #box2_z=0 
                
                x__.append(-box1_width/2)
                y__.append(box1_x - box1_length/2)
                
                x__.append(-box2_width/2)
                y__.append(box1_x - box1_length/2)
                
                
                x__.append(-box2_width/2)
                y__.append(box1_x - box1_length/2 - box2_length)
                
                
                x__.append(box2_width/2)
                y__.append(box1_x - box1_length/2 - box2_length)
                
                
                x__.append(box2_width/2)
                y__.append(box1_x - box1_length/2)
                
                x__.append(box1_width/2)
                y__.append(box1_x - box1_length/2)
                
                x__.append(R*np.cos(theta[0]))
                y__.append(R*np.sin(theta[0]))
                #x__=np.dot(-1,x__)
                (self.segments)=self.create_segment(y__,x__)
            #ax.plot(x,y)
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]
                if self.geom=="square":
                    #if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.1:
                        #patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        #axs[0].add_patch(patch)
                   
                    #else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                    axs[0].add_patch(patch) 
                        
                if self.geom=="triangle":
                    if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.1:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[0].add_patch(patch)                         
                
                        
                
                if self.geom=="c_shape":
                    difx=self.ballx_position[0]-self.ballx_
                   
                    dify=self.ballz_position[0]-self.ballz_
                    
                    if self.PHI(x0+(x0b-difx),y0+(x0b-difx),self.segments)<.1:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[0].add_patch(patch)    
                        
                if self.geom=="circle":
                    
                    q=np.sqrt((x0-self.ballx_position[i])**2 + (y0-self.ballz_position[i])**2)
                    #if q<=(2 * self.bot_width/2 + self.ball_radius):
                        #patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        #axs[0].add_patch(patch)
                   
                    #else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                    axs[0].add_patch(patch)  
                        
                if self.geom=="import":
                    #if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.5:
                        #patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        #axs[0].add_patch(patch)
                   
                    #else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                    axs[0].add_patch(patch)          
                
            if membrane==True:
                for j in range(0,self.nm):
                    
                    x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                    patch = plt.Circle((x0, y0),self.skin_width/2, fc='tab:red')
                    axs[0].add_patch(patch)
                
                

            for j in range(self.ni):
                x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

                if np.round(self.Rm[j],4)==0.0508:
                    c='tab:blue'
                else:
                    c='tab:green'
                patch = plt.Circle((x0, y0),self.Rm[j], fc=c)
                axs[0].add_patch(patch)         
     
            if self.control_mode=="grasping" or self.control_mode=="grasping_explore" or self.control_mode=="grasping_epsilon":
                if self.control_mode=="grasping_explore":
                    x_=self.Rr_[i]*np.cos(self.THETA[i]) + self.ballx
                    y_=self.Rr_[i]*np.sin(self.THETA[i]) + self.ballz
          
                    #patch = plt.Circle((x_,y_),.1,fc='m',edgecolor='m',linewidth=1,zorder=2)
                    #axs[0].add_patch(patch)
                
                    #patch = plt.Circle((self.centroidx[i],self.centroidz[i]),.1,fc='tab:orange',edgecolor='tab:orange',linewidth=1,zorder=2)
                    #axs[0].add_patch(patch)
                if self.geom=="circle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i]
                    patch = plt.Circle((x0, y0),self.ball_radius,fc='none',edgecolor='black',linewidth=1)
                    axs[0].add_patch(patch)
            
                if self.geom=="square":
                    const_=self.ball_radius*2
                    x0,y0=self.ballx_position[i] - const_/2,self.ballz_position[i] - const_/2
                    patch = matplotlib.patches.Rectangle((x0, y0),const_, const_,fc='none',edgecolor='black',linewidth=1)     
                    axs[0].add_patch(patch)   
                    
                if self.geom=="triangle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i] 
                    const=self.ball_radius*2*np.pi/3
                    #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                    r=const*np.sqrt(3)/3
                    #print(r)
                    patch = matplotlib.patches.RegularPolygon((x0,y0),int(3),r,orientation=np.pi/2,fc='none',edgecolor='black',linewidth=1)
                    axs[0].add_patch(patch) 
                    #xp=np.hstack([self.segments[:,0],self.segments[0,0]])
                    #yp=np.hstack([self.segments[:,1],self.segments[0,1]])
                    #axs[0].plot(xp+x0,yp+y0,color='tab:red',linestyle='dashed',linewidth=2,zorder=0)
            
            
                if self.geom=="c_shape":
                   x0b,y0b=self.ballx_position[i],self.ballz_position[i] 
                   difx=self.ballx_position[0]-self.ballx_
                   
                   dify=self.ballz_position[0]-self.ballz_
                   
                   x__=x__+np.ones(len(x__))*(x0b-difx)
                   y__=y__+np.ones(len(x__))*(y0b-dify)
                   axs[0].plot(x__,y__)
                   
                if self.geom=="import":  
                   x0b,y0b=self.ballx_position[i],self.ballz_position[i] 
                   difx=self.ballx_position[0]-self.ballx_
                   
                   dify=self.ballz_position[0]-self.ballz_
                   
                   x__=x__+np.ones(len(x__))*(y0b-dify)
                   y__=y__+np.ones(len(x__))*(x0b-difx)
                   axs[0].plot(y__,x__,color="k")
                    
            axs[0].set_title('Time= ' + str(np.round(self.time[i],0)),fontsize=12)

            print(str(i)+ "of"+ str(len(self.time-1)))
            #plt.gca().set_aspect('equal', adjustable='box')
            #fig.delaxes(axs[1,1])
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')          
        self.create_video('_frames_combined_potfield','_frames_combined_potfield')   


     def create_frames_epsilon(self,membrane,d):
        ''' Create frames to show the robot and the pull force and epsilon metric in one plot'''
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        
        direct = os.path.join(self.mainDirectory+self.name,'_frames_combined')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        Num_contact1=[]
        Num_contact2=[]
        for i in range(len(self.temp_id2)):
            Num_contact1.append(len(self.temp_id2[i]))
            
        for i in range(len(self.time)-2):    
  
            fig, axs = plt.subplots(nrows=2, ncols=3,figsize=(12,8),dpi=300)

            # epsilon 1
            epsilon=np.asarray(self.EPSILON4)
            axs[0,1].plot(self.time[:i],epsilon[:i],color='tab:green',linewidth=1)
            axs[0,1].scatter(self.time[i-1],epsilon[i-1],color='tab:red',s=30)
            axs[0,1].set_title(r'$\epsilon_{1}$='+str(np.round(epsilon[i-1],2)))
            axs[0,1].set_ylabel('$\epsilon_{1}$',labelpad=1)
            axs[0,1].set_xlabel('time [s]',labelpad=-2)
            axs[0,1].xaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[0,1].yaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[0,1].grid(True) 

            
            # # pressure
            # axs[1,0].plot(self.time[:i],self.Mag_avg_pressure_no_boundary[:i],color='k',linewidth=1)
            # axs[1,0].scatter(self.time[i-1],self.Mag_avg_pressure_no_boundary[i-1],color='r',s=30)
            # axs[1,0].set_title('Pressue_no boundary: '+str(np.round(self.Mag_avg_pressure_no_boundary[i-1],2)))
            # axs[1,0].set_ylabel('Pressure (N/m^2)',labelpad=1)
            # axs[1,0].set_xlabel('time [s]',labelpad=-2)
            # axs[1,0].xaxis.set_tick_params(width=.25,length=2,pad=1)
            # axs[1,0].yaxis.set_tick_params(width=.25,length=2,pad=1)
            # axs[1,0].grid(True)
            
            
            # axs[0,2].scatter(np.round(self.THETA[:i],2),epsilon[:i],color='b',marker='s')
            # axs[0,2].scatter(np.round(self.THETA[i-1],2),epsilon[i-1],color='r',s=30)
            # axs[0,2].set_title(r'$\epsilon_{1}$='+str(np.round(epsilon[i-1],2)))
            # axs[0,2].set_ylabel('$\epsilon_{1}$',labelpad=1)
            # axs[0,2].set_xlabel(r'$\theta$' ,labelpad=-2)
            # axs[0,2].xaxis.set_tick_params(width=.25,length=2,pad=1)
            # axs[0,2].yaxis.set_tick_params(width=.25,length=2,pad=1)
            # axs[0,2].grid(True)

            
            
            # axs[1,1].plot(self.time[:i],Num_contact1[:i],color='tab:blue',linewidth=1)
            # axs[1,1].scatter(self.time[i-1],Num_contact1[i-1],color='r',s=30)
            # axs[1,1].set_title(r'number_contact: '+str(Num_contact1[i-1]))
            # axs[1,1].set_ylabel('number of contact',labelpad=1)
            # axs[1,1].set_xlabel('time [s]',labelpad=-2)
            # axs[1,1].xaxis.set_tick_params(width=.25,length=2,pad=1)
            # axs[1,1].yaxis.set_tick_params(width=.25,length=2,pad=1)
            # axs[1,1].grid(True)
    
      
            #### Robot simulation
            xcenter=self.ballx_position[i]
            ycenter=self.ballz_position[i]
            
            x0b=self.ballx_position[2]
            y0b=self.ballz_position[2]
            wxmax=x0b+d
            wxmin=x0b-d
            wymax=y0b+d
            wymin=y0b-d
            
     
            axs[0,0].set_xlim([wxmin,wxmax])
            axs[0,0].set_ylim([wymin,wymax])
            if self.geom=="square":
                const=self.ball_radius*2
                rx=const
                ry=const
                w=rx/2
                h=ry/2                    
                x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
                y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]
                (self.segments)=self.create_segment(x,y) 
                
            if self.geom=="triangle":
                #const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                
                const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                r=const*np.sqrt(3)/3
                x1=r
                y1=0
               
                x2=r*np.cos(2*np.pi/3)
                y2=r*np.sin(2*np.pi/3)
               
                x3=r*np.cos(4*np.pi/3)
                y3=r*np.sin(4*np.pi/3)
               
                x__ = [x1,x2,x3,x1]
                y__ = [y1,y2,y3,y1]
               
                (self.segments)=self.create_segment(x__,y__)   
            
            #ax.plot(x,y)
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]
                if self.geom=="square":
                    if self.PHI(x0,y0,self.segments)<.15:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[0,0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[0,0].add_patch(patch) 
                        
                if self.geom=="triangle":
                    if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.1:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[0,0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[0,0].add_patch(patch)                         
                        
                if self.geom=="circle":
                    
                    q=np.sqrt((x0-self.ballx_position[i])**2 + (y0-self.ballz_position[i])**2)
                    if q<=(2 * self.bot_width/2 + self.ball_radius):
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[0,0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[0,0].add_patch(patch)  
                        
                        
                
            if membrane==True:
                for j in range(0,self.nm):
                    
                    x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                    patch = plt.Circle((x0, y0),self.skin_width/2, fc='tab:red')
                    axs[0,0].add_patch(patch)
                
                

            for j in range(self.ni):
                x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

                if self.Rm[j]==self.particle_width/2:
                    c='tab:blue'
                if self.Rm[j]==self.particle_width*np.sqrt(2)/2:
                    c='tab:green'
                patch = plt.Circle((x0, y0),self.Rm[j], fc=c)
                axs[0,0].add_patch(patch)         
     
            if self.control_mode=="grasping":
                patch = plt.Circle((self.xc2, self.yc2),self.a2,fc='none',edgecolor='tab:blue',linewidth=1,zorder=2)
                axs[0,0].add_patch(patch)
                if self.geom=="circle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i]
                    patch = plt.Circle((x0, y0),self.ball_radius,fc='none',edgecolor='black',linewidth=1)
                    axs[0,0].add_patch(patch)
            
                if self.geom=="square":
                    const_=self.ball_radius*2
                    x0,y0=self.ballx_position[i] - const_/2,self.ballz_position[i] - const_/2
                    patch = matplotlib.patches.Rectangle((x0, y0),const_, const_,fc='none',edgecolor='black',linewidth=1)     
                    axs[0,0].add_patch(patch)   
                    
                if self.geom=="triangle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i] 
                    const=self.ball_radius*2*np.pi/3
                    #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                    r=const*np.sqrt(3)/3
                    #print(r)
                    patch = matplotlib.patches.RegularPolygon((x0,y0),int(3),r,orientation=-np.pi/2,fc='none',edgecolor='black',linewidth=1)
                    axs[0,0].add_patch(patch) 
                    xp=np.hstack([self.segments[:,0],self.segments[0,0]])
                    yp=np.hstack([self.segments[:,1],self.segments[0,1]])
                    axs[0,0].plot(xp+x0,yp+y0,color='tab:red',linestyle='dashed',linewidth=2,zorder=0)
            
            
            axs[0,0].set_title('Time= ' + str(np.round(self.time[i],0)),fontsize=12)
            
          
            print(str(i)+ "of"+ str(len(self.time-1)))
            #plt.gca().set_aspect('equal', adjustable='box')
            #fig.delaxes(axs[1,1])
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')          
        self.create_video('_frames_combined','_frames_combined')  

     def create_frames_pull_epsilon6(self,membrane,wxmin,wxmax,wymin,wymax):
        ''' Create frames to show the robot and the pull force and epsilon metric in one plot'''
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        
        direct = os.path.join(self.mainDirectory+self.name,'_frames_combined6')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        Num_contact1=[]
        Num_contact2=[]
        for i in range(len(self.temp_id2)):
            Num_contact1.append(len(self.temp_id2[i]))
            
        for i in range(len(self.time)-2):    
  
            fig, axs = plt.subplots(nrows=1, ncols=1,figsize=(3,3),dpi=300)
    
            #wxmax=x0b+d
            #wxmin=x0b-d
            #wymax=y0b+d
            #wymin=y0b-d
            
            xcenter=self.ballx_position[i]
            ycenter=self.ballz_position[i]
            axs.set_xlim([wxmin,wxmax])
            axs.set_ylim([wymin,wymax])
            #### Square
            if self.geom=="square":
                const=self.ball_radius*2
                rx=const
                ry=const
                w=rx/2
                h=ry/2                    
                x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
                y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]
                (self.segments)=self.create_segment(x,y) 
            
            #### Triangle
            if self.geom=="triangle":
                #const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                
                const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                r=const*np.sqrt(3)/3
                x1=-r
                y1=0
               
                x2=-r*np.cos(2*np.pi/3)
                y2=r*np.sin(2*np.pi/3)
               
                x3=-r*np.cos(4*np.pi/3)
                y3=r*np.sin(4*np.pi/3)
               
                x__ = [x1,x2,x3,x1]
                y__ = [y1,y2,y3,y1]
               
                (self.segments)=self.create_segment(x__,y__)   
            #### c shape
            if self.geom=="c_shape":
                #x0b,y0b=self.ballx_position[i],self.ballz_position[i]
                w=self.w/2
                l=self.l/2
                t=self.t
                x__=[w,-w,-w,w,w,-w+t,-w+t,w,w]
                y__=[-l,-l,l,l,l-t,l-t,-l+t,-l+t,-l]
                (self.segments)=self.create_segment(x__,y__)
                #x0b=x0b+np.sign(-w/2+(t/2))*(-w/2+(t/2))
                
                #z0b=z0b+np.sign(z0b)*(z0b)
                
            if self.geom=="import":
                
                R=1.25
                nsides=50
                theta=np.linspace(3*np.pi/2 + .415,(2*np.pi + 3*np.pi/2) - .415,nsides,endpoint=True)
                x__=[]
                y__=[]
                for iii in range(len(theta)):
                    x__.append(R*np.cos(theta[iii]))
                    y__.append(R*np.sin(theta[iii]))
                    
                
                box1_width=1.007974825964066
                box1_length=box1_width/2
                box2_width=1.5
                Rr=np.sqrt(R**2 - (box1_width/2)**2)
                  
                box1_x=-(Rr + box1_length/2)
                #box1_z=0 
                
                box2_width=1.5
                box2_length=.25
                
                #box2_x=box1_x - box1_length/2 - box2_length/2
                #box2_z=0 
                
                x__.append(-box1_width/2)
                y__.append(box1_x - box1_length/2)
                
                x__.append(-box2_width/2)
                y__.append(box1_x - box1_length/2)
                
                
                x__.append(-box2_width/2)
                y__.append(box1_x - box1_length/2 - box2_length)
                
                
                x__.append(box2_width/2)
                y__.append(box1_x - box1_length/2 - box2_length)
                
                
                x__.append(box2_width/2)
                y__.append(box1_x - box1_length/2)
                
                x__.append(box1_width/2)
                y__.append(box1_x - box1_length/2)
                
                x__.append(R*np.cos(theta[0]))
                y__.append(R*np.sin(theta[0]))
                #x__=np.dot(-1,x__)
                (self.segments)=self.create_segment(y__,x__)
            #ax.plot(x,y)
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]
                if self.geom=="square":
                    #if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.1:
                        #patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        #axs[0].add_patch(patch)
                   
                    #else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                    axs.add_patch(patch) 
                        
                if self.geom=="triangle":
                    if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.1:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs.add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs.add_patch(patch)                         
                
                        
                
                if self.geom=="c_shape":
                    difx=self.ballx_position[0]-self.ballx_
                   
                    dify=self.ballz_position[0]-self.ballz_
                    
                    if self.PHI(x0+(x0b-difx),y0+(x0b-difx),self.segments)<.1:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs.add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs.add_patch(patch)    
                        
                if self.geom=="circle":
                    
                    q=np.sqrt((x0-self.ballx_position[i])**2 + (y0-self.ballz_position[i])**2)
                    #if q<=(2 * self.bot_width/2 + self.ball_radius):
                        #patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        #axs[0].add_patch(patch)
                   
                    #else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                    axs.add_patch(patch)  
                        
                if self.geom=="import":
                    #if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.5:
                        #patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        #axs[0].add_patch(patch)
                   
                    #else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                    axs.add_patch(patch)          
                
            if membrane==True:
                for j in range(0,self.nm):
                    
                    x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                    patch = plt.Circle((x0, y0),self.skin_width/2, fc='tab:red')
                    axs.add_patch(patch)
                
                

            for j in range(self.ni):
                x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

                if np.round(self.Rm[j],4)==0.0508:
                    c='tab:blue'
                else:
                    c='tab:green'
                patch = plt.Circle((x0, y0),self.Rm[j], fc=c)
                axs.add_patch(patch)         
     
            if self.control_mode=="grasping" or self.control_mode=="grasping_explore":
                if self.control_mode=="grasping_explore":
                    x_=self.Rr_[i]*np.cos(self.THETA[i]) + self.ballx
                    y_=self.Rr_[i]*np.sin(self.THETA[i]) + self.ballz
          
                    #patch = plt.Circle((x_,y_),.1,fc='m',edgecolor='m',linewidth=1,zorder=2)
                    #axs[0].add_patch(patch)
                
                    #patch = plt.Circle((self.centroidx[i],self.centroidz[i]),.1,fc='tab:orange',edgecolor='tab:orange',linewidth=1,zorder=2)
                    #axs[0].add_patch(patch)
                if self.geom=="circle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i]
                    patch = plt.Circle((x0, y0),self.ball_radius,fc='none',edgecolor='black',linewidth=1)
                    axs.add_patch(patch)
            
                if self.geom=="square":
                    const_=self.ball_radius*2
                    x0,y0=self.ballx_position[i] - const_/2,self.ballz_position[i] - const_/2
                    patch = matplotlib.patches.Rectangle((x0, y0),const_, const_,fc='none',edgecolor='black',linewidth=1)     
                    axs.add_patch(patch)   
                    
                if self.geom=="triangle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i] 
                    const=self.ball_radius*2*np.pi/3
                    #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                    r=const*np.sqrt(3)/3
                    #print(r)
                    patch = matplotlib.patches.RegularPolygon((x0,y0),int(3),r,orientation=np.pi/2,fc='none',edgecolor='black',linewidth=1)
                    axs.add_patch(patch) 
                    #xp=np.hstack([self.segments[:,0],self.segments[0,0]])
                    #yp=np.hstack([self.segments[:,1],self.segments[0,1]])
                    #axs[0].plot(xp+x0,yp+y0,color='tab:red',linestyle='dashed',linewidth=2,zorder=0)
            
            
                if self.geom=="c_shape":
                   x0b,y0b=self.ballx_position[i],self.ballz_position[i] 
                   difx=self.ballx_position[0]-self.ballx_
                   
                   dify=self.ballz_position[0]-self.ballz_
                   
                   x__=x__+np.ones(len(x__))*(x0b-difx)
                   y__=y__+np.ones(len(x__))*(y0b-dify)
                   axs.plot(x__,y__)
                   
                if self.geom=="import":  
                    axs.plot(y__,x__,color="k")
                    
            axs.set_title('Time= ' + str(np.round(self.time[i],0)),fontsize=12)

            print(str(i)+ "of"+ str(len(self.time-1)))
            #plt.gca().set_aspect('equal', adjustable='box')
            #fig.delaxes(axs[1,1])
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')          
        self.create_video('_frames_combined6','_frames_combined6')    


        
     def create_frames_pull_epsilon4(self,membrane,d):
        ''' Create frames to show the robot and the pull force
            and epsilon metric in one plot'''
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        
        direct = os.path.join(self.mainDirectory+self.name,'_frames_combined4')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        Num_contact1=[]
        Num_contact2=[]
        for i in range(len(self.temp_id2)):
            Num_contact1.append(len(self.temp_id2[i]))
            
        for i in range(len(self.time)-2):    
  
            fig, axs = plt.subplots(nrows=2, ncols=2,figsize=(6.5,6.5),dpi=300)
            
            #### PULL FORCE
            axs[0,1].plot(self.FB[:i],self.PX[:i]-self.PX[54],color='tab:green',linewidth=1.5)
            axs[0,1].scatter(self.FB[i-1],self.PX[i-1]-self.PX[54],color='tab:red',s=30)
            axs[0,1].set_ylim([-0.25,1])
            axs[0,1].set_title('FB='+str(np.round(self.FB[i-1],2))+" Px= "+str(np.round(self.PX[i-1]-self.PX[54],2))  )
            axs[0,1].set_xlabel('Pull Force [N]',labelpad=-1)  
            axs[0,1].set_ylabel('Ball Position [m]',labelpad=1) 
            axs[0,1].grid(True) 
            
            
            
            #### EPSILON
            epsilon=np.asarray(self.EPSILON_)
            axs[1,0].plot(self.time[:i],epsilon[:i],color='tab:blue',linewidth=1.5)
            axs[1,0].scatter(self.time[i-1],epsilon[i-1],color='tab:red',s=30)
            axs[1,0].set_title(r'$\epsilon_{1}$='+str(np.round(epsilon[i-1],2)))
            axs[1,0].set_ylabel('$\epsilon_{1}$',labelpad=1)
            axs[1,0].set_xlabel('time [s]',labelpad=-2)
            axs[1,0].xaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[1,0].yaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[1,0].grid(True) 

            #### Qcm
            #epsilon=np.asarray(self.EPSILON_)
            axs[1,1].plot(self.time[:i],self.Qcm[:i],color='tab:red',linewidth=1.5)
            axs[1,1].scatter(self.time[i-1],self.Qcm[i-1],color='k',s=30)
            axs[1,1].set_title('Qcm= '+str(np.round(self.Qcm[i-1],2)))
            axs[1,1].set_ylabel('Qcm',labelpad=1)
            axs[1,1].set_xlabel('time [s]',labelpad=-2)
            axs[1,1].xaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[1,1].yaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[1,1].grid(True) 

            #### ROBOT 
            xcenter=self.ballx_position[i]
            ycenter=self.ballz_position[i]
            
            x0b=self.ballx_position[2]
            y0b=self.ballz_position[2]
            wxmax=x0b+d
            wxmin=x0b-d
            wymax=y0b+d
            wymin=y0b-d
            
     
            axs[0,0].set_xlim([wxmin,wxmax])
            axs[0,0].set_ylim([wymin,wymax])
            if self.geom=="square":
                const=self.ball_radius*2
                rx=const
                ry=const
                w=rx/2
                h=ry/2                    
                x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
                y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]
                (self.segments)=self.create_segment(x,y) 
                
            if self.geom=="triangle":
                #const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                
                const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                r=const*np.sqrt(3)/3
                x1=-r
                y1=0
               
                x2=-r*np.cos(2*np.pi/3)
                y2=r*np.sin(2*np.pi/3)
               
                x3=-r*np.cos(4*np.pi/3)
                y3=r*np.sin(4*np.pi/3)
               
                x__ = [x1,x2,x3,x1]
                y__ = [y1,y2,y3,y1]
               
                (self.segments)=self.create_segment(x__,y__)   
            
            #ax.plot(x,y)
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]
                if self.geom=="square":
                    if self.PHI(x0,y0,self.segments)<.15:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[0,0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[0,0].add_patch(patch) 
                        
                if self.geom=="triangle":
                    if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.1:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[0,0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[0,0].add_patch(patch)                         
                        
                if self.geom=="circle":
                    
                    q=np.sqrt((x0-self.ballx_position[i])**2 + (y0-self.ballz_position[i])**2)
                    if q<=(2 * self.bot_width/2 + self.ball_radius):
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[0,0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[0,0].add_patch(patch)  
                        
                        
                
            if membrane==True:
                for j in range(0,self.nm):
                    
                    x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                    patch = plt.Circle((x0, y0),self.skin_width/2, fc='tab:red')
                    axs[0,0].add_patch(patch)
                
                

            for j in range(self.ni):
                x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

                if np.round(self.Rm[j],4)==0.0508:
                    c='tab:blue'
                else:
                    c='tab:green'
                patch = plt.Circle((x0, y0),self.Rm[j], fc=c)
                axs[0,0].add_patch(patch)         
     
            if self.control_mode=="grasping":
                patch = plt.Circle((self.xc2, self.yc2),self.a2,fc='none',edgecolor='tab:blue',linewidth=1,zorder=2)
                axs[0,0].add_patch(patch)
                if self.geom=="circle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i]
                    patch = plt.Circle((x0, y0),self.ball_radius,fc='none',edgecolor='black',linewidth=1)
                    axs[0,0].add_patch(patch)
            
                if self.geom=="square":
                    const_=self.ball_radius*2
                    x0,y0=self.ballx_position[i] - const_/2,self.ballz_position[i] - const_/2
                    patch = matplotlib.patches.Rectangle((x0, y0),const_, const_,fc='none',edgecolor='black',linewidth=1)     
                    axs[0,0].add_patch(patch)   
                    
                if self.geom=="triangle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i] 
                    const=self.ball_radius*2*np.pi/3
                    #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                    r=const*np.sqrt(3)/3
                    #print(r)
                    patch = matplotlib.patches.RegularPolygon((x0,y0),int(3),r,orientation=np.pi/2,fc='none',edgecolor='black',linewidth=1)
                    axs[0,0].add_patch(patch) 
                    xp=np.hstack([self.segments[:,0],self.segments[0,0]])
                    yp=np.hstack([self.segments[:,1],self.segments[0,1]])
                    axs[0,0].plot(xp+x0,yp+y0,color='tab:red',linestyle='dashed',linewidth=2,zorder=0)
            
            axs[0,0].scatter(self.centroidx[i],self.centroidz[i],marker='o',color='tab:orange')
            axs[0,0].set_title('Time= ' + str(np.round(self.time[i],0)),fontsize=12)
    
            print(str(i)+ "of"+ str(len(self.time-1)))
            #plt.gca().set_aspect('equal', adjustable='box')
            #fig.delaxes(axs[1,1])
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')          
        self.create_video('_frames_combined4','_frames_combined4')   


     def create_frames_pull_epsilon3(self,membrane,wxmin,wxmax,wymin,wymax):
        ''' Create frames to show the robot and the pull force and epsilon metric in one plot'''
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        
        direct = os.path.join(self.mainDirectory+self.name,'_frames_combined3')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        Num_contact1=[]
        Num_contact2=[]
        for i in range(len(self.temp_id2)):
            Num_contact1.append(len(self.temp_id2[i]))
            
        for i in range(len(self.time)-2):    
  
            fig, axs = plt.subplots(nrows=1, ncols=2,figsize=(6,3),dpi=300)
            # epsilon 1
            #epsilon=np.asarray(self.EPSILON)
            epsilon=self.EPSILON_
            
            axs[1].plot(self.time[:i],epsilon[:i],color='tab:blue',linewidth=1)
            axs[1].scatter(self.time[i-1],epsilon[i-1],color='tab:blue',s=30)
            
            axs[1].plot(self.time_tilda[:i],self.epsilon_tilda[:i],color='tab:green',linewidth=2,label=r"$\bar{\epsilon}$")
            axs[1].scatter(self.time_tilda[i-1],self.epsilon_tilda[i-1],color='tab:green',s=30)
            
            axs[1].set_title(r'$\epsilon_{1}$='+str(np.round(epsilon[i-1],2)))
            axs[1].set_ylabel('$\epsilon_{1}$',labelpad=1)
            axs[1].set_xlabel('time [s]',labelpad=-2)
            axs[1].xaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[1].yaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[1].grid(True) 

            xcenter=self.ballx_position[i]
            ycenter=self.ballz_position[i]
            
            x0b=self.ballx_position[2]
            y0b=self.ballz_position[2]
            #wxmax=x0b+d
            #wxmin=x0b-d
            #wymax=y0b+d
            #wymin=y0b-d
            
     
            axs[0].set_xlim([wxmin,wxmax])
            axs[0].set_ylim([wymin,wymax])
            #### Square
            if self.geom=="square":
                const=self.ball_radius*2
                rx=const
                ry=const
                w=rx/2
                h=ry/2                    
                x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
                y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]
                (self.segments)=self.create_segment(x,y) 
            
            #### Triangle
            if self.geom=="triangle":
                #const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                
                const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                r=const*np.sqrt(3)/3
                x1=-r
                y1=0
               
                x2=-r*np.cos(2*np.pi/3)
                y2=r*np.sin(2*np.pi/3)
               
                x3=-r*np.cos(4*np.pi/3)
                y3=r*np.sin(4*np.pi/3)
               
                x__ = [x1,x2,x3,x1]
                y__ = [y1,y2,y3,y1]
               
                (self.segments)=self.create_segment(x__,y__)   
            #### c shape
            if self.geom=="c_shape":
                #x0b,y0b=self.ballx_position[i],self.ballz_position[i]
                w=self.w/2
                l=self.l/2
                t=self.t
                x__=[w,-w,-w,w,w,-w+t,-w+t,w,w]
                y__=[-l,-l,l,l,l-t,l-t,-l+t,-l+t,-l]
                (self.segments)=self.create_segment(x__,y__)
                #x0b=x0b+np.sign(-w/2+(t/2))*(-w/2+(t/2))
                
                #z0b=z0b+np.sign(z0b)*(z0b)
                
            if self.geom=="import":
                
                R=1.25
                nsides=50
                theta=np.linspace(3*np.pi/2 + .415,(2*np.pi + 3*np.pi/2) - .415,nsides,endpoint=True)
                x__=[]
                y__=[]
                for iii in range(len(theta)):
                    x__.append(R*np.cos(theta[iii]))
                    y__.append(R*np.sin(theta[iii]))
                    
                
                box1_width=1.007974825964066
                box1_length=box1_width/2
                box2_width=1.5
                Rr=np.sqrt(R**2 - (box1_width/2)**2)
                  
                box1_x=-(Rr + box1_length/2)
                #box1_z=0 
                
                box2_width=1.5
                box2_length=.25
                
                #box2_x=box1_x - box1_length/2 - box2_length/2
                #box2_z=0 
                
                x__.append(-box1_width/2)
                y__.append(box1_x - box1_length/2)
                
                x__.append(-box2_width/2)
                y__.append(box1_x - box1_length/2)
                
                
                x__.append(-box2_width/2)
                y__.append(box1_x - box1_length/2 - box2_length)
                
                
                x__.append(box2_width/2)
                y__.append(box1_x - box1_length/2 - box2_length)
                
                
                x__.append(box2_width/2)
                y__.append(box1_x - box1_length/2)
                
                x__.append(box1_width/2)
                y__.append(box1_x - box1_length/2)
                
                x__.append(R*np.cos(theta[0]))
                y__.append(R*np.sin(theta[0]))
                #x__=np.dot(-1,x__)
                (self.segments)=self.create_segment(y__,x__)
            #ax.plot(x,y)
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]
                if self.geom=="square":
                    #if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.1:
                        #patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        #axs[0].add_patch(patch)
                   
                    #else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                    axs[0].add_patch(patch) 
                        
                if self.geom=="triangle":
                    if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.1:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[0].add_patch(patch)                         
                
                        
                
                if self.geom=="c_shape":
                    difx=self.ballx_position[0]-self.ballx_
                   
                    dify=self.ballz_position[0]-self.ballz_
                    
                    if self.PHI(x0+(x0b-difx),y0+(x0b-difx),self.segments)<.1:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[0].add_patch(patch)    
                        
                if self.geom=="circle":
                    
                    q=np.sqrt((x0-self.ballx_position[i])**2 + (y0-self.ballz_position[i])**2)
                    #if q<=(2 * self.bot_width/2 + self.ball_radius):
                        #patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        #axs[0].add_patch(patch)
                   
                    #else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                    axs[0].add_patch(patch)  
                        
                if self.geom=="import":
                    #if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.5:
                        #patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        #axs[0].add_patch(patch)
                   
                    #else:
                    patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                    axs[0].add_patch(patch)          
                
            if membrane==True:
                for j in range(0,self.nm):
                    
                    x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                    patch = plt.Circle((x0, y0),self.skin_width/2, fc='tab:red')
                    axs[0].add_patch(patch)
                
                

            for j in range(self.ni):
                x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

                if np.round(self.Rm[j],4)==0.0508:
                    c='tab:blue'
                else:
                    c='tab:green'
                patch = plt.Circle((x0, y0),self.Rm[j], fc=c)
                axs[0].add_patch(patch)         
     
            if self.control_mode=="grasping" or self.control_mode=="grasping_explore" or self.control_mode=="grasping_epsilon":
                if self.control_mode=="grasping_explore":
                    x_=self.Rr_[i]*np.cos(self.THETA[i]) + self.ballx
                    y_=self.Rr_[i]*np.sin(self.THETA[i]) + self.ballz
          
                    #patch = plt.Circle((x_,y_),.1,fc='m',edgecolor='m',linewidth=1,zorder=2)
                    #axs[0].add_patch(patch)
                
                    #patch = plt.Circle((self.centroidx[i],self.centroidz[i]),.1,fc='tab:orange',edgecolor='tab:orange',linewidth=1,zorder=2)
                    #axs[0].add_patch(patch)
                if self.geom=="circle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i]
                    patch = plt.Circle((x0, y0),self.ball_radius,fc='none',edgecolor='black',linewidth=1)
                    axs[0].add_patch(patch)
            
                if self.geom=="square":
                    const_=self.ball_radius*2
                    x0,y0=self.ballx_position[i] - const_/2,self.ballz_position[i] - const_/2
                    patch = matplotlib.patches.Rectangle((x0, y0),const_, const_,fc='none',edgecolor='black',linewidth=1)     
                    axs[0].add_patch(patch)   
                    
                if self.geom=="triangle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i] 
                    const=self.ball_radius*2*np.pi/3
                    #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                    r=const*np.sqrt(3)/3
                    #print(r)
                    patch = matplotlib.patches.RegularPolygon((x0,y0),int(3),r,orientation=np.pi/2,fc='none',edgecolor='black',linewidth=1)
                    axs[0].add_patch(patch) 
                    #xp=np.hstack([self.segments[:,0],self.segments[0,0]])
                    #yp=np.hstack([self.segments[:,1],self.segments[0,1]])
                    #axs[0].plot(xp+x0,yp+y0,color='tab:red',linestyle='dashed',linewidth=2,zorder=0)
            
            
                if self.geom=="c_shape":
                   x0b,y0b=self.ballx_position[i],self.ballz_position[i] 
                   difx=self.ballx_position[0]-self.ballx_
                   
                   dify=self.ballz_position[0]-self.ballz_
                   
                   x__=x__+np.ones(len(x__))*(x0b-difx)
                   y__=y__+np.ones(len(x__))*(y0b-dify)
                   axs[0].plot(x__,y__)
                   
                if self.geom=="import":  
                   x0b,y0b=self.ballx_position[i],self.ballz_position[i] 
                   difx=self.ballx_position[0]-self.ballx_
                   
                   dify=self.ballz_position[0]-self.ballz_
                   
                   x__=x__+np.ones(len(x__))*(y0b-dify)
                   y__=y__+np.ones(len(x__))*(x0b-difx)
                   axs[0].plot(y__,x__,color="k")
                    
            axs[0].set_title('Time= ' + str(np.round(self.time[i],0)),fontsize=12)

            print(str(i)+ "of"+ str(len(self.time-1)))
            #plt.gca().set_aspect('equal', adjustable='box')
            #fig.delaxes(axs[1,1])
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')          
        self.create_video('_frames_combined3','_frames_combined3')    
        
        

     def create_frames_pull_epsilon2(self,membrane,d):
        ''' Create frames to show the robot and the pull force
            and epsilon metric in one plot'''
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        
        direct = os.path.join(self.mainDirectory+self.name,'_frames_combined2')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        Num_contact1=[]
        Num_contact2=[]
        for i in range(len(self.temp_id2)):
            Num_contact1.append(len(self.temp_id2[i]))

        for i in range(len(self.time)-2):    
  
            fig, axs = plt.subplots(nrows=2, ncols=2,figsize=(8,8),dpi=300)
            if self.time[i]>=self.tcut2-1 and self.nojam==False:
                axs[0,0].plot(self.xjam,self.yjam,color='m',linestyle='dashed',linewidth=2,zorder=-5)
            else:
                patch = plt.Circle((self.xc2,self.yc2),.05, fc='m')
                axs[0,0].add_patch(patch)
                
            #axs[0,0].scatter(self.xjam,self.yjam,color='m',s=3)
            #### PULL FORCE
            axs[1,0].plot(self.FB[:i],self.PX[:i],color='tab:green',linewidth=1)
            axs[1,0].scatter(self.FB[i-1],self.PX[i-1],color='tab:red',s=30)
            axs[1,0].set_xlabel('Pull Force [N]',labelpad=1)  
            axs[1,0].set_ylabel('Ball Position [m]',labelpad=1)  
            axs[1,0].grid(True)   
            axs[1,0].set_ylim([-0.25,1])
            # axs[1,0].plot(self.time[:i],self.FB[:i],color='tab:green',linewidth=1)
            # axs[1,0].scatter(self.time[i-1],self.FB[i-1],color='tab:green',s=30)
            # axs[1,0].set_title('FB='+str(np.round(self.FB[i-1],2)))
            # axs[1,0].set_ylabel('Pull Force [N]',labelpad=1)  
            # axs[1,0].set_xlabel('time [s]',labelpad=1) 
            # axs[1,0].grid(True) 
            
            axs[1,1].plot(self.time[:i],self.PX[:i],color='tab:red',linewidth=1)
            axs[1,1].scatter(self.time[i-1],self.PX[i-1],color='tab:red',s=30)
            axs[1,1].set_title('PX='+str(np.round(self.PX[i-1],2)))
            axs[1,1].set_ylabel('Ball Position [m]',labelpad=1)  
            axs[1,1].set_xlabel('time [s]',labelpad=1) 
            axs[1,1].grid(True) 
            axs[1,1].set_ylim([-0.25,3])
            
            #### EPSILON
            epsilon=np.asarray(self.EPSILON_)
            epsilon_tilda=self.smooth_epsilon(epsilon[:i])
            axs[0,1].plot(self.time[:i],epsilon[:i],color='tab:blue',linewidth=1)
            axs[0,1].plot(self.time[:i],epsilon_tilda[:i],color='r',linewidth=3)
            axs[0,1].scatter(self.time[i-1],epsilon[i-1],color='r',s=30)
            axs[0,1].set_title(r'$\epsilon_{1}$='+str(np.round(epsilon[i-1],2))+'etilda='+str(np.round(epsilon_tilda[i-1],2)))
            axs[0,1].set_ylabel('$\epsilon_{1}$',labelpad=1)
            axs[0,1].set_xlabel('time [s]',labelpad=-2)
            axs[0,1].xaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[0,1].yaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[0,1].grid(True) 

            #### ROBOT 
            xcenter=self.ballx_position[i]
            ycenter=self.ballz_position[i]
            
            x0b=self.ballx_position[2]
            y0b=self.ballz_position[2]
            wxmax=x0b+d
            wxmin=x0b-d
            wymax=y0b+d
            wymin=y0b-d
            
     
            axs[0,0].set_xlim([wxmin,wxmax])
            axs[0,0].set_ylim([wymin,wymax])
            if self.geom=="square":
                const=self.ball_radius*2
                rx=const
                ry=const
                w=rx/2
                h=ry/2                    
                x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
                y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]
                (self.segments)=self.create_segment(x,y) 
                
            if self.geom=="triangle":
                #const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                
                const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                r=const*np.sqrt(3)/3
                x1=r
                y1=0
               
                x2=r*np.cos(2*np.pi/3)
                y2=r*np.sin(2*np.pi/3)
               
                x3=r*np.cos(4*np.pi/3)
                y3=r*np.sin(4*np.pi/3)
               
                x__ = [x1,x2,x3,x1]
                y__ = [y1,y2,y3,y1]
               
                (self.segments)=self.create_segment(x__,y__)   
            
            #ax.plot(x,y)
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]
                if self.geom=="square":
                    if self.PHI(x0,y0,self.segments)<.15:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[0,0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[0,0].add_patch(patch) 
                        
                if self.geom=="triangle":
                    if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.1:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[0,0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[0,0].add_patch(patch)                         
                        
                if self.geom=="circle":
                    patch = plt.Circle((x0, y0),self.bot_width/2,fc='none',edgecolor='black')
                    axs[0,0].add_patch(patch)  
                        
                        
                
            if membrane==True:
                for j in range(0,self.nm):
                    
                    x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                    patch = plt.Circle((x0, y0),self.skin_width/2, fc='tab:red')
                    axs[0,0].add_patch(patch)
                
                

            for j in range(self.ni):
                x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

                if np.round(self.Rm[j],4)==0.0508:
                    c='tab:blue'
                else:
                    c='tab:green'
                patch = plt.Circle((x0, y0),self.Rm[j], fc=c)
                axs[0,0].add_patch(patch)         
     
            if self.control_mode=="grasping" or self.control_mode=="grasping_epsilon":
                #patch = plt.Circle((self.xc2, self.yc2),self.a2,fc='none',edgecolor='tab:blue',linewidth=1,zorder=2)
                #axs[0].add_patch(patch)
                if self.geom=="circle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i]
                    patch = plt.Circle((x0, y0),self.ball_radius,fc='none',edgecolor='black',linewidth=1)
                    axs[0,0].add_patch(patch)
            
                if self.geom=="square":
                    const_=self.ball_radius*2
                    x0,y0=self.ballx_position[i] - const_/2,self.ballz_position[i] - const_/2
                    patch = matplotlib.patches.Rectangle((x0, y0),const_, const_,fc='none',edgecolor='black',linewidth=1)     
                    axs[0,0].add_patch(patch)   
                    
                if self.geom=="triangle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i] 
                    const=self.ball_radius*2*np.pi/3
                    #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                    r=const*np.sqrt(3)/3
                    #print(r)
                    patch = matplotlib.patches.RegularPolygon((x0,y0),int(3),r,orientation=-np.pi/2,fc='none',edgecolor='black',linewidth=1)
                    axs[0,0].add_patch(patch) 
                    xp=np.hstack([self.segments[:,0],self.segments[0,0]])
                    yp=np.hstack([self.segments[:,1],self.segments[0,1]])
                    axs[0,0].plot(xp+x0,yp+y0,color='tab:red',linestyle='dashed',linewidth=2,zorder=0)
            
            
            axs[0,0].set_title('Time= ' + str(np.round(self.time[i],0)),fontsize=12)
    
            print(str(i)+ "of"+ str(len(self.time-1)))
            #plt.gca().set_aspect('equal', adjustable='box')
            #fig.delaxes(axs[1,1])
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')          
        self.create_video('_frames_combined2','_frames_combined2')       





     def create_frames_pull_epsilon(self,membrane,d):
        ''' Create frames to show the robot and the pull force and epsilon metric in one plot'''
        fm._rebuild()
        plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'dejavuserif'
        plt.rcParams['font.size'] = 8
        plt.rcParams['axes.linewidth'] = .1
        
        direct = os.path.join(self.mainDirectory+self.name,'_frames_combined')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        Num_contact1=[]
        Num_contact2=[]
        for i in range(len(self.temp_id2)):
            Num_contact1.append(len(self.temp_id2[i]))
            
        for i in range(len(self.time)-2):    
  
            fig, axs = plt.subplots(nrows=1, ncols=3,figsize=(12,4),dpi=300)
            
            axs[0,0].plot(self.FB[:i],self.PX[:i]-self.PX[54],color='tab:blue',linewidth=1)
            axs[0,0].scatter(self.FB[i-1],self.PX[i-1]-self.PX[54],color='tab:red',s=30)
            axs[0,0].set_ylim([-0.25,1])
            axs[0,0].set_title('FB='+str(np.round(self.FB[i-1],2))+" Px= "+str(np.round(self.PX[i-1]-self.PX[54],2))  )
            axs[0,0].set_xlabel('Pull Force [N]',labelpad=1)  
            axs[0,0].set_ylabel('Ball Position [m]',labelpad=1) 
            axs[0,0].grid(True) 
            
            
            
            # epsilon 1
            epsilon=np.asarray(self.EPSILON4)
            axs[0,1].plot(self.time[:i],epsilon[:i],color='tab:green',linewidth=1)
            axs[0,1].scatter(self.time[i-1],epsilon[i-1],color='tab:red',s=30)
            #x_ticks = np.linspace(self.time[0], self.time[-2],5,endpoint=True)
            #y_ticks = np.linspace(np.min(epsilon), np.max(epsilon),10,endpoint=True)
            #axs[0,1].set_xticks(np.round(x_ticks,2))
            #axs[0,1].set_yticks(np.round(y_ticks,2))
            axs[0,1].set_title(r'$\epsilon_{1}$='+str(np.round(epsilon[i-1],2)))
            axs[0,1].set_ylabel('$\epsilon_{1}$',labelpad=1)
            axs[0,1].set_xlabel('time [s]',labelpad=-2)
            axs[0,1].xaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[0,1].yaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[0,1].grid(True) 

            # # epsilon 2
            # epsilon=np.asarray(self.EPSILON2)
            # axs[0,2].plot(self.time[:i],epsilon[:i],color='tab:orange',linewidth=1)
            # axs[0,2].scatter(self.time[i-1],epsilon[i-1],color='k',s=30)
            # #x_ticks = np.linspace(self.time[0], self.time[-2],5,endpoint=True)
            # #y_ticks = np.linspace(np.min(epsilon), np.max(epsilon),10,endpoint=True)
            # #axs[0,2].set_xticks(np.round(x_ticks,2))
            # #axs[0,2].set_yticks(np.round(y_ticks,2))
            # axs[0,2].set_title(r'$\epsilon_{2}$='+str(np.round(epsilon[i-1],2)))
            # axs[0,2].set_ylabel('$\epsilon_{2}$',labelpad=1)
            # axs[0,2].set_xlabel('time [s]',labelpad=-2)
            # axs[0,2].xaxis.set_tick_params(width=.25,length=2,pad=1)
            # axs[0,2].yaxis.set_tick_params(width=.25,length=2,pad=1)    
            # axs[0,2].grid(True) 
            
            
            
            # pressure
            axs[0,2].plot(self.time[:i],self.Mag_avg_pressure_no_boundary[:i],color='cyan',linewidth=1)
            axs[0,2].scatter(self.time[i-1],self.Mag_avg_pressure_no_boundary[i-1],color='r',s=30)
            #x_ticks = np.linspace(self.time[0], self.time[-2],5,endpoint=True)
            #y_ticks = np.linspace(np.min(self.Mag_avg_pressure_no_boundary), np.max(self.Mag_avg_pressure_no_boundary),10,endpoint=True)
            #axs[1,0].set_xticks(np.round(x_ticks,2))
            #axs1,0].set_yticks(np.round(y_ticks,2))
            axs[0,2].set_title('Pressue_no boundary: '+str(np.round(self.Mag_avg_pressure_no_boundary[i-1],2)))
            axs[0,2].set_ylabel('Pressure (N/m^2)',labelpad=1)
            axs[0,2].set_xlabel('time [s]',labelpad=-2)
            axs[0,2].xaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[0,2].yaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[0,2].grid(True)
            
            
            
            # # epsilon 3
            # epsilon=np.asarray(self.EPSILON3)
            # axs[1,1].plot(self.time[:i],epsilon[:i],color='m',linewidth=1)
            # axs[1,1].scatter(self.time[i-1],epsilon[i-1],color='k',s=30)
            # #x_ticks = np.linspace(self.time[0], self.time[-2],5,endpoint=True)
            # #y_ticks = np.linspace(np.min(epsilon), np.max(epsilon),10,endpoint=True)
            # #axs[1,1].set_xticks(np.round(x_ticks,2))
            # #axs[1,1].set_yticks(np.round(y_ticks,2))
            # axs[1,1].set_title(r'$\epsilon_{3}$='+str(np.round(epsilon[i-1],2)))
            # axs[1,1].set_ylabel('$\epsilon_{3}$',labelpad=1)
            # axs[1,1].set_xlabel('time [s]',labelpad=-2)
            # axs[1,1].xaxis.set_tick_params(width=.25,length=2,pad=1)
            # axs[1,1].yaxis.set_tick_params(width=.25,length=2,pad=1)    
            # axs[1,1].grid(True) 
            
            
            # # epsilon 4
            # epsilon=np.asarray(self.EPSILON4)
            # axs[1,2].plot(self.time[:i],epsilon[:i],color='tab:purple',linewidth=1)
            # axs[1,2].scatter(self.time[i-1],epsilon[i-1],color='k',s=30)
            # #x_ticks = np.linspace(self.time[0], self.time[-2],5,endpoint=True)
            # #y_ticks = np.linspace(np.min(epsilon), np.max(epsilon),10,endpoint=True)
            # #axs[1,2].set_xticks(np.round(x_ticks,2))
            # #axs[1,2].set_yticks(np.round(y_ticks,2))
            # axs[1,2].set_title(r'$\epsilon_{4}$='+str(np.round(epsilon[i-1],2)))
            # axs[1,2].set_ylabel('$\epsilon_{4}$',labelpad=1)
            # axs[1,2].set_xlabel('time [s]',labelpad=-2)
            # axs[1,2].xaxis.set_tick_params(width=.25,length=2,pad=1)
            # axs[1,2].yaxis.set_tick_params(width=.25,length=2,pad=1)    
            # axs[1,2].grid(True)            
    
    
    
            axs[1,1].plot(self.time[:i],Num_contact1[:i],color='tab:brown',linewidth=1)
            axs[1,1].scatter(self.time[i-1],Num_contact1[i-1],color='r',s=30)
            #x_ticks = np.linspace(self.time[0], self.time[-2],5,endpoint=True)
            #y_ticks = np.linspace(0, np.max(Num_contact1),np.max(Num_contact1)+1,endpoint=True)
            #axs[2,1].set_xticks(np.round(x_ticks,2))
            #axs[2,1].set_yticks(np.round(y_ticks,2))
            axs[1,1].set_title(r'number_contact: '+str(Num_contact1[i-1]))
            axs[1,1].set_ylabel('number of contact',labelpad=1)
            axs[1,1].set_xlabel('time [s]',labelpad=-2)
            axs[1,1].xaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[1,1].yaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[1,1].grid(True)
    
            axs[1,2].plot(self.time[:i],self.ball_velocity_x[:i],color="tab:grey",linewidth=1,label='vx')
            axs[1,2].scatter(self.time[i-1],self.ball_velocity_x[i-1],color='r',s=30)
            axs[1,2].set_title(r'ball velocityx: '+str(np.round(self.ball_velocity_x[i-1],2)))
            axs[1,2].set_ylabel('m/s',labelpad=1)
            axs[1,2].set_xlabel('time [s]',labelpad=-2)
            axs[1,2].xaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[1,2].yaxis.set_tick_params(width=.25,length=2,pad=1)
            axs[1,2].grid(True)
    
    
            xcenter=self.ballx_position[i]
            ycenter=self.ballz_position[i]
            
            x0b=self.ballx_position[2]
            y0b=self.ballz_position[2]
            wxmax=x0b+d
            wxmin=x0b-d
            wymax=y0b+d
            wymin=y0b-d
            
     
            axs[1,0].set_xlim([wxmin,wxmax])
            axs[1,0].set_ylim([wymin,wymax])
            if self.geom=="square":
                const=self.ball_radius*2
                rx=const
                ry=const
                w=rx/2
                h=ry/2                    
                x=[w+xcenter,-w+xcenter,-w+xcenter,w+xcenter,w+xcenter]
                y=[h+ycenter,h+ycenter,-h+ycenter,-h+ycenter,h+ycenter]
                (self.segments)=self.create_segment(x,y) 
                
            if self.geom=="triangle":
                #const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                
                const=self.ball_radius*2*np.pi/3
                #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                r=const*np.sqrt(3)/3
                x1=r
                y1=0
               
                x2=r*np.cos(2*np.pi/3)
                y2=r*np.sin(2*np.pi/3)
               
                x3=r*np.cos(4*np.pi/3)
                y3=r*np.sin(4*np.pi/3)
               
                x__ = [x1,x2,x3,x1]
                y__ = [y1,y2,y3,y1]
               
                (self.segments)=self.create_segment(x__,y__)   
            
            #ax.plot(x,y)
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]
                if self.geom=="square":
                    if self.PHI(x0,y0,self.segments)<.15:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[1,0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[1,0].add_patch(patch) 
                        
                if self.geom=="triangle":
                    if self.PHI(x0-xcenter,y0-ycenter,self.segments)<.1:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[1,0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[1,0].add_patch(patch)                         
                        
                if self.geom=="circle":
                    
                    q=np.sqrt((x0-self.ballx_position[i])**2 + (y0-self.ballz_position[i])**2)
                    if q<=(2 * self.bot_width/2 + self.ball_radius):
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='tab:red')
                        axs[1,0].add_patch(patch)
                   
                    else:
                        patch = plt.Circle((x0, y0),self.bot_width/2, fc='k')
                        axs[1,0].add_patch(patch)  
                        
                        
                
            if membrane==True:
                for j in range(0,self.nm):
                    
                    x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                    patch = plt.Circle((x0, y0),self.skin_width/2, fc='tab:red')
                    axs[1,0].add_patch(patch)
                
                

            for j in range(self.ni):
                x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

                if self.Rm[j]==self.particle_width/2:
                    c='tab:blue'
                if self.Rm[j]==self.particle_width*np.sqrt(2)/2:
                    c='tab:green'
                patch = plt.Circle((x0, y0),self.Rm[j], fc=c)
                axs[1,0].add_patch(patch)         
     
            if self.control_mode=="grasping":
                patch = plt.Circle((self.xc2, self.yc2),self.a2,fc='none',edgecolor='tab:blue',linewidth=1,zorder=2)
                axs[1,0].add_patch(patch)
                if self.geom=="circle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i]
                    patch = plt.Circle((x0, y0),self.ball_radius,fc='none',edgecolor='black',linewidth=1)
                    axs[1,0].add_patch(patch)
            
                if self.geom=="square":
                    const_=self.ball_radius*2
                    x0,y0=self.ballx_position[i] - const_/2,self.ballz_position[i] - const_/2
                    patch = matplotlib.patches.Rectangle((x0, y0),const_, const_,fc='none',edgecolor='black',linewidth=1)     
                    axs[1,0].add_patch(patch)   
                    
                if self.geom=="triangle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i] 
                    const=self.ball_radius*2*np.pi/3
                    #r=np.sqrt((const**2)/(2-2*np.cos(7*np.pi/6)))
                    r=const*np.sqrt(3)/3
                    #print(r)
                    patch = matplotlib.patches.RegularPolygon((x0,y0),int(3),r,orientation=-np.pi/2,fc='none',edgecolor='black',linewidth=1)
                    axs[1,0].add_patch(patch) 
                    xp=np.hstack([self.segments[:,0],self.segments[0,0]])
                    yp=np.hstack([self.segments[:,1],self.segments[0,1]])
                    axs[1,0].plot(xp+x0,yp+y0,color='tab:red',linestyle='dashed',linewidth=2,zorder=0)
            
            
            axs[1,0].set_title('Time= ' + str(np.round(self.time[i],0)),fontsize=12)
            
            
            
            
            
            
            
            
            
            
            
            
            
            print(str(i)+ "of"+ str(len(self.time-1)))
            #plt.gca().set_aspect('equal', adjustable='box')
            #fig.delaxes(axs[1,1])
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')          
        self.create_video('_frames_combined','_frames_combined')        
        
     
        
        
     def create_video(self,framename,videoname):
         #import pdb    
         img_array = []
         for index, filename in enumerate(glob.glob(self.mainDirectory+'/'+self.name+'/'+framename+'/'+'/*.jpg')):
             #pdb.set_trace()
             img = cv2.imread(filename)
             height, width, layers = img.shape
             size = (width,height)
             img_array.append(img)
         out = cv2.VideoWriter(self.mainDirectory+'/'+self.name+'/'+videoname+'.mp4',cv2.VideoWriter_fourcc(*'DIVX'), 15, (width,height))    

         for i in range(len(img_array)):
            out.write(img_array[i])
         out.release()  
        
        
        
         removing_files = glob.glob(self.mainDirectory+'/'+self.name+'/'+framename+'/'+'/*.jpg')
         for i in removing_files:
             os.remove(i)
        
     ####R-function functions

     def normalize(self,F):
        (fy,fx)=np.gradient(F)
        return(F/np.sqrt(F**2 + fy**2 + fx**2))

     def line_(self,x,y,x1,x2,y1,y2):
        return(((x-x1)*(y2-y1)-(y-y1)*(x2-x1))/(np.sqrt((x2-x1)**2 + (y2-y1)**2)))

     def parabola(self,x,y,px,py):
        return((x-px)**2 - py - y)



     def equivalence(self,w1,w2,m):
        return(w1*w2/((w1**m +w2**m)**(1/m)))

     def Union(self,w1,w2):
        return(((w1+w2)/2) + ((np.sqrt((w1-w2)**2))/2))

     def intersection(self,w1,w2):
        return(((w1+w2)/2) - ((np.sqrt((w1-w2)**2))/2))          
        
        
     def F(self,x,y,x1,x2,y1,y2):
         L=np.sqrt((x2-x1)**2 + (y2-y1)**2)
         return(((x-x1)*(y2-y1)-(y-y1)*(x2-x1))/L)

     def delfx(self,x,y,x1,x2,y1,y2):
        L=np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return((-y1+y2)/L)

     def delfy(self,x,y,x1,x2,y1,y2):
        L=np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return((x1-x2)/L)

     def T(self,x,y,x1,x2,y1,y2):
        L=np.sqrt((x2-x1)**2 + (y2-y1)**2)
        xc=np.array([(x2+x1)/2,(y2+y1)/2])
        t = (1/L)*((L/2)**2 - ((x-xc[0])**2 + (y-xc[1])**2))    
        return(t)

     def deltx(self,x,y,x1,x2,y1,y2):
        xc = np.array([(x2+x1)/2,(y2+y1)/2]) 
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(-(x-xc[0])/ (L*np.sqrt((x-xc[0])**2 + (y-xc[1])**2)))

     def delty(self,x,y,x1,x2,y1,y2):
        xc = np.array([(x2+x1)/2,(y2+y1)/2]) 
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(-(y-xc[1])/ (L*np.sqrt((x-xc[0])**2 + (y-xc[1])**2)))


     def phi(self,x,y,x1,x2,y1,y2):
        t=self.T(x,y,x1,x2,y1,y2)
        f=self.F(x,y,x1,x2,y1,y2)
        rho=np.sqrt(t**2 +f**4)
        return(np.sqrt(f**2+((rho-t)/2)**2))

     def delphix(self,x,y,x1,x2,y1,y2):
        ff=self.F(x,y,x1,x2,y1,y2)
        dfx=self.delfx(x,y,x1,x2,y1,y2)
        tf=self.T(x,y,x1,x2,y1,y2)
        dtx=self.deltx(x,y,x1,x2,y1,y2)
        term1 = 0.5*((2 * ff**3 * dfx + tf*dtx)/(np.sqrt(ff**4 + tf**2)) - dtx)*((np.sqrt(ff**4 + tf**2)-tf)/2)
        term2 = ff*dfx
        term3 =  np.sqrt((((np.sqrt(ff**4 + tf**2)-tf)/2)**2) + ff**2)  
        return((term1+term2)/term3)

     def delphiy(self,x,y,x1,x2,y1,y2):
        ff=self.F(x,y,x1,x2,y1,y2)
        dfy=self.delfy(x,y,x1,x2,y1,y2)
        tf=self.T(x,y,x1,x2,y1,y2)
        dty=self.delty(x,y,x1,x2,y1,y2)
        term1 = 0.5*((2 * ff**3 * dfy + tf*dty)/(np.sqrt(ff**4 + tf**2)) - dty)*((np.sqrt(ff**4 + tf**2)-tf)/2)
        term2 = ff*dfy
        term3 =  np.sqrt((((np.sqrt(ff**4 + tf**2)-tf)/2)**2) + ff**2)    
        return((term1+term2)/term3)  

     def PHI(self,x,y,segments):
        #m=4
        R=0
        for i in range(len(segments[:,0])):
            R = R + 1/self.phi(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3]) ** self.m
        R = 1/R**(1/self.m)
        return(R)

     def PHIDX(self,x,y,segments):
        #m=4
        term1=0
        term2=0
        term3=0
        for i in range(len(segments[:,0])):
            term1=-self.m*(self.phi(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**-self.m) *self.delphix(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3]) + term1
            term2=(self.phi(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**-self.m)**(-1/self.m) + term2
            term3=self.m*(self.phi(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**-self.m) + term3      
        R=(-term1*term2/term3)
        return(R)
     
        
     
        
     
        
     
     def PHIDY(self,x,y,segments):
        #m=4
        term1=0
        term2=0
        term3=0
        for i in range(len(segments[:,0])):
            term1=-self.m*(self.phi(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**-self.m) * self.delphiy(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3]) + term1
            term2=(self.phi(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**-self.m)**(-1/self.m) + term2
            term3=self.m*(self.phi(x,y,segments[i,0],segments[i,2],segments[i,1],segments[i,3])**-self.m) + term3      
        R=(-term1*term2/term3)
        return(R)
    
    
    
    
    
    
    
    
    
     def Trim(self,f,t):
        """ Trim function for two functions  """
        phi=np.sqrt(t**2 + f**4)
        return(np.sqrt(f**2 + ((phi-t)/2)**2))

     def Trimx(self,f,t,fx,tx):
        """Derivative Trim function for two functions wrt x  """
        term1 = (2*(f**3)*fx + tx*t)/(np.sqrt(f**4 + t**2)) - tx
        term2 = np.sqrt(f**4 + t**2)/2 - t/2
        term3 = f*fx
        term4 = np.sqrt((np.sqrt(f**4 + t**2)/2 - t/2)**2 + f**2)
        
        return((0.5*term1*term2 +term3)/term4)
        
     def Trimy(self,f,t,fy,ty):
        """Derivative Trim function for two functions wrt y """
        term1 = (2*(f**3)*fy + ty*t)/(np.sqrt(f**4 + t**2)) - ty
        term2 = np.sqrt(f**4 + t**2)/2 - t/2
        term3 = f*fy
        term4 = np.sqrt((np.sqrt(f**4 + t**2)/2 - t/2)**2 + f**2)
        
        return((0.5*term1*term2 +term3)/term4)    
      

     def phi_line_(self,x,y,x1,y1,x2,y2):
        """ Distance function of a line """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(((x-x1)*(y2-y1)-(y-y1)*(x2-x1))/L)


     def dphix_line_(self,x,y,x1,y1,x2,y2):
        """ Derivative distance function of a line wrt x """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return((-y1+y2)/L)


     def dphiy_line_(self,x,y,x1,y1,x2,y2):
        """ Derivative distance function of a line wrt y """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return((x1-x2)/L)


     def trim(self,x,y,x1,y1,x2,y2):
        """ Trim function """
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        xc = np.array([(x2+x1)/2,(y2+y1)/2])
        t = (1/L)*((L/2)**2 - ((x-xc[0])**2 + (y-xc[1])**2))    
        return(t)


     def dtrimx(self,x,y,x1,y1,x2,y2):
        """ Derivative of Trim function wrt x """
        xc = np.array([(x2+x1)/2,(y2+y1)/2]) 
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(-(x-xc[0])/ (L*np.sqrt((x-xc[0])**2 + (y-xc[1])**2)))

     def dtrimy(self,x,y,x1,y1,x2,y2):
        """ Derivative of Trim function wrt y """
        xc = np.array([(x2+x1)/2,(y2+y1)/2]) 
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        return(-(y-xc[1])/ (L*np.sqrt((x-xc[0])**2 + (y-xc[1])**2)))

     def phi_line(self,x,y,x1,y1,x2,y2):
        """ Trimmed line segment"""
        t = self.trim(x,y,x1,y1,x2,y2)
        f = self.phi_line_(x,y,x1,y1,x2,y2)
        rho = np.sqrt(t**2 + f**4)
        return(np.sqrt(f**2 + ((rho-t)/2)**2))

     def dphix_line(self,x,y,x1,x2,y1,y2):
        """ Derivative of trimmed line segment wrt x"""
        ff = self.phi_line_(x,y,x1,x2,y1,y2)
        dfx = self.dphix_line_(x,y,x1,x2,y1,y2)
        tf = self.trim(x,y,x1,x2,y1,y2)
        dtx = self.dtrimx(x,y,x1,x2,y1,y2)
        term1 = 0.5*((2 * ff**3 * dfx + tf*dtx)/(np.sqrt(ff**4 + tf**2)) - dtx)*((np.sqrt(ff**4 + tf**2)-tf)/2)
        term2 = ff*dfx
        term3 = np.sqrt((((np.sqrt(ff**4 + tf**2)-tf)/2)**2) + ff**2)  
        return((term1+term2)/term3)

     def dphiy_line(self,x,y,x1,x2,y1,y2):
        """ Derivative of trimmed line segment wrt y"""
        ff = self.phi_line_(x,y,x1,x2,y1,y2)
        dfy = self.dphiy_line_(x,y,x1,x2,y1,y2)
        tf = self.trim(x,y,x1,x2,y1,y2)
        dty = self.dtrimy(x,y,x1,x2,y1,y2)
        term1 = 0.5*((2 * ff**3 * dfy + tf*dty)/(np.sqrt(ff**4 + tf**2)) - dty)*((np.sqrt(ff**4 + tf**2)-tf)/2)
        term2 = ff*dfy
        term3 = np.sqrt((((np.sqrt(ff**4 + tf**2)-tf)/2)**2) + ff**2)    
        return((term1+term2)/term3)   
    
    
     def circle(self,x,y,R,a,b):
        return(abs((R**2 - (x-a)**2 - (y-b)**2)))

    
    
     def dphix_circle(self,x,y,a,b,R):
        """ Derivative of the distance function of cirlce wrt x """
        return(R*(x-a)*np.sign((R/2)*(- R**2 +(a-x)**2 + (b-y)**2)))
    
    
     def dphiy_circle(self,x,y,a,b,R):
        """ Derivative of the distance function of cirlce wrt y """
        return(R*(y-b)*np.sign((R/2)*(- R**2 +(a-x)**2 + (b-y)**2)))    
    
    
     def phi_segments(self,x,y,segments):
        """ R equivelent of trimmed line segments"""
        R=0
        self.m=4
        for i in range(len(segments[:,0])):
            R = R + 1/self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**self.m
        R = 1/R**(1/self.m)
        return(R)

     def dphix_segments(self,x,y,segments):
        """ Derivative of R equivelent of trimmed line segments wrt x"""

        term1=0
        term2=0
        term3=0
        for i in range(len(segments[:,0])):
            term1=-self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**-self.m-1) * self.dphix_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3]) + term1
            term2=(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**-self.m) + term2
            term3=self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**-self.m) + term3      
        R=(-term1*(term2**(-1/self.m))/term3)
        return(R)

     def dphiy_segments(self,x,y,segments):
        """ Derivative of R equivelent of trimmed line segments wrt y"""

        term1=0
        term2=0
        term3=0
        for i in range(len(segments[:,0])):
            term1=-self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**(-self.m-1)) * self.dphiy_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3]) + term1
            term2=(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**(-self.m)) + term2
            term3=self.m*(self.phi_line(x,y,segments[i,0],segments[i,1],segments[i,2],segments[i,3])**(-self.m)) + term3      
        R=(-term1*(term2**(-1/self.m))/term3)
        return(R)   
    
                  
    
    
     def F_random_objdx(self,x,y):
        # R=1.25
        # theta1=0.41
        # theta2=2*np.pi-theta1
        # theta=np.linspace(theta1,theta2,100)

        # x1_=-R*np.cos(theta)
        # y1_=R*np.sin(theta)


        # a=.5
        # b=1.65
        # c=2.02

        # w1=.5
        # w2=.63


        # x2_=[x1_[0],x1_[0]-a]
        # y2_=[w1,w1]

        # x3_=[-b,-b]
        # y3_=[w1,w2]

        # x4_=[-b,-c]
        # y4_=[w2,w2]

        # x5_=[-c,-c]
        # y5_=[w2,-w2]

        # x6_=[-c,-b]
        # y6_=[-w2,-w2]

        # x7_=[-b,-b]
        # y7_=[-w1,-w2]

        # x8_=[-b,x1_[-1]]
        # y8_=[-w1,-w1]


        # x_=[x2_[0],x2_[1],x3_[1],x4_[1],x5_[1],x6_[1],x7_[0],x8_[1]]
        # y_=[y2_[0],y2_[1],y3_[1],y4_[1],y5_[1],y6_[1],y7_[0],y8_[1]]
        
        R=1.25
        theta1=0.41
        theta2=2*np.pi-theta1
        theta=np.linspace(theta1,theta2,100)

        theta3=0.41
        theta4=2*np.pi-theta1

        x1_=-R*np.cos(theta)
        y1_=R*np.sin(theta)

        x1a_=R*np.cos(theta3)
        y1a_=R*np.sin(theta3)

        x1b_=R*np.cos(theta4)
        y1b_=R*np.sin(theta4)

        a=.504
        b=1.65
        c=2.02

        w1=.5
        w2=.63


        x2_=[x1_[0],x1_[0]-a]
        y2_=[w1,w1]

        x3_=[-b,-b]
        y3_=[w1,w2]

        x4_=[-b,-c]
        y4_=[w2,w2]

        x5_=[-c,-c]
        y5_=[w2,-w2]

        x6_=[-c,-b]
        y6_=[-w2,-w2]

        x7_=[-b,-b]
        y7_=[-w1,-w2]

        x8_=[-b,x1_[-1]]
        y8_=[-w1,-w1]

        x_=[]
        y_=[]
        
        for i in range(len(x1_)):
            x_.append(x1_[i])
            
        x_.append(x2_[0])
        x_.append(x2_[1])
        x_.append(x3_[1])
        x_.append(x4_[1])
        x_.append(x5_[1])
        x_.append(x6_[1])
        x_.append(x7_[0])
        x_.append(x8_[1])
        
        for i in range(len(y1_)):
            y_.append(y1_[i])
            
        y_.append(y2_[0])
        y_.append(y2_[1])
        y_.append(y3_[1])
        y_.append(y4_[1])
        y_.append(y5_[1])
        y_.append(y6_[1])
        y_.append(y7_[0])
        y_.append(y8_[1])     

        # x9_=[x1_[-1],x1a_]
        # y9_=[w1,w1]

        # x10_=[x1a_,x1a_+a]
        # y10_=[w1,w1]

        # x11_=[b,b]
        # y11_=[w1,w2]

        # x12_=[b,c]
        # y12_=[w2,w2]

        # x13_=[c,c]
        # y13_=[w2,-w2]

        # x14_=[c,b]
        # y14_=[-w2,-w2]

        # x15_=[b,b]
        # y15_=[-w1,-w2]

        # x16_=[b,x1b_]
        # y16_=[-w1,-w1]

        # x17_=[x1b_,x1_[-1]]
        # y17_=[-w1,-w1]

        # x_=[x2_[0],x2_[1],x3_[1],x4_[1],x5_[1],x6_[1],x7_[0],x8_[1],x9_[1],x10_[1],x11_[1],x12_[1],x13_[1],x14_[1],x15_[0],x16_[1]]
        # y_=[y2_[0],y2_[1],y3_[1],y4_[1],y5_[1],y6_[1],y7_[0],y8_[1],y9_[1],y10_[1],y11_[1],y12_[1],y13_[1],y14_[1],y15_[0],y16_[1]]
        
        (self.segments)=self.create_segment(x_,y_)
        #f1=self.circle(x,y,R,0,0)
        #f1x=self.dphix_circle(x,y,0,0,R)
        #2 = self.phi_segments(x,y,self.segments)
        Fx = self.dphix_segments(x,y,self.segments)   
        #Fx = (f1**self.m + f2**self.m)**(-1/self.m)*f1*f2x + (f1**self.m + f2**self.m)**(-1/self.m)*f2*f1x - ((self.m*(f2**self.m)*f2x)/f2 + (self.m*(f1**self.m)*f1x)/f1)*((f1**self.m) +f2**self.m)**(-1/self.m)*f1*f2/(self.m*(f1**self.m + f2**self.m))
            
        return(Fx)
    
     def F_random_objdy(self,x,y):
        # R=1.25
        # theta1=0.41
        # theta2=2*np.pi-theta1
        # theta=np.linspace(theta1,theta2,100)

        # x1_=-R*np.cos(theta)
        # y1_=R*np.sin(theta)


        # a=.5
        # b=1.65
        # c=2.02

        # w1=.5
        # w2=.63


        # x2_=[x1_[0],x1_[0]-a]
        # y2_=[w1,w1]

        # x3_=[-b,-b]
        # y3_=[w1,w2]

        # x4_=[-b,-c]
        # y4_=[w2,w2]

        # x5_=[-c,-c]
        # y5_=[w2,-w2]

        # x6_=[-c,-b]
        # y6_=[-w2,-w2]

        # x7_=[-b,-b]
        # y7_=[-w1,-w2]

        # x8_=[-b,x1_[-1]]
        # y8_=[-w1,-w1]

        # x_=[x2_[0],x2_[1],x3_[1],x4_[1],x5_[1],x6_[1],x7_[0],x8_[1]]
        # y_=[y2_[0],y2_[1],y3_[1],y4_[1],y5_[1],y6_[1],y7_[0],y8_[1]]
        
        
        R=1.25
        theta1=0.41
        theta2=2*np.pi-theta1
        theta=np.linspace(theta1,theta2,100)

        theta3=0.41
        theta4=2*np.pi-theta1

        x1_=-R*np.cos(theta)
        y1_=R*np.sin(theta)

        x1a_=R*np.cos(theta3)
        y1a_=R*np.sin(theta3)

        x1b_=R*np.cos(theta4)
        y1b_=R*np.sin(theta4)

        a=.504
        b=1.65
        c=2.02

        w1=.5
        w2=.63


        x2_=[x1_[0],x1_[0]-a]
        y2_=[w1,w1]

        x3_=[-b,-b]
        y3_=[w1,w2]

        x4_=[-b,-c]
        y4_=[w2,w2]

        x5_=[-c,-c]
        y5_=[w2,-w2]

        x6_=[-c,-b]
        y6_=[-w2,-w2]

        x7_=[-b,-b]
        y7_=[-w1,-w2]

        x8_=[-b,x1_[-1]]
        y8_=[-w1,-w1]
        
        x_=[]
        y_=[]
        
        for i in range(len(x1_)):
            x_.append(x1_[i])
            
        x_.append(x2_[0])
        x_.append(x2_[1])
        x_.append(x3_[1])
        x_.append(x4_[1])
        x_.append(x5_[1])
        x_.append(x6_[1])
        x_.append(x7_[0])
        x_.append(x8_[1])
        
        for i in range(len(y1_)):
            y_.append(y1_[i])
            
        y_.append(y2_[0])
        y_.append(y2_[1])
        y_.append(y3_[1])
        y_.append(y4_[1])
        y_.append(y5_[1])
        y_.append(y6_[1])
        y_.append(y7_[0])
        y_.append(y8_[1])     

        # x9_=[x1_[-1],x1a_]
        # y9_=[w1,w1]

        # x10_=[x1a_,x1a_+a]
        # y10_=[w1,w1]

        # x11_=[b,b]
        # y11_=[w1,w2]

        # x12_=[b,c]
        # y12_=[w2,w2]

        # x13_=[c,c]
        # y13_=[w2,-w2]

        # x14_=[c,b]
        # y14_=[-w2,-w2]

        # x15_=[b,b]
        # y15_=[-w1,-w2]

        # x16_=[b,x1b_]
        # y16_=[-w1,-w1]

        # x17_=[x1b_,x1_[-1]]
        # y17_=[-w1,-w1]

        #x_=[x2_[0],x2_[1],x3_[1],x4_[1],x5_[1],x6_[1],x7_[0],x8_[1],x9_[1],x10_[1],x11_[1],x12_[1],x13_[1],x14_[1],x15_[0],x16_[1]]
        #y_=[y2_[0],y2_[1],y3_[1],y4_[1],y5_[1],y6_[1],y7_[0],y8_[1],y9_[1],y10_[1],y11_[1],y12_[1],y13_[1],y14_[1],y15_[0],y16_[1]]
        (self.segments)=self.create_segment(x_,y_)
        #f1=self.circle(x,y,R,0,0)
        #f1y=self.dphiy_circle(x,y,0,0,R)
        #f2 = self.phi_segments(x,y,self.segments)
        Fy = self.dphiy_segments(x,y,self.segments)   
        #Fy = (f1**self.m + f2**self.m)**(-1/self.m)*f1*f2y + (f1**self.m + f2**self.m)**(-1/self.m)*f2*f1y - ((self.m*(f2**self.m)*f2y)/f2 + (self.m*(f1**self.m)*f1y)/f1)*((f1**self.m) +f2**self.m)**(-1/self.m)*f1*f2/(self.m*(f1**self.m + f2**self.m))
            
        return(Fy)
    
    
     def F_random_obj(self,x,y):
        # R=1.25
        # theta1=0.41
        # theta2=2*np.pi-theta1
        # theta=np.linspace(theta1,theta2,100)

        # x1_=-R*np.cos(theta)
        # y1_=R*np.sin(theta)


        # a=.5
        # b=1.65
        # c=2.02

        # w1=.5
        # w2=.63


        # x2_=[x1_[0],x1_[0]-a]
        # y2_=[w1,w1]

        # x3_=[-b,-b]
        # y3_=[w1,w2]

        # x4_=[-b,-c]
        # y4_=[w2,w2]

        # x5_=[-c,-c]
        # y5_=[w2,-w2]

        # x6_=[-c,-b]
        # y6_=[-w2,-w2]

        # x7_=[-b,-b]
        # y7_=[-w1,-w2]

        # x8_=[-b,x1_[-1]]
        # y8_=[-w1,-w1]

        # x_=[x2_[0],x2_[1],x3_[1],x4_[1],x5_[1],x6_[1],x7_[0],x8_[1]]
        # y_=[y2_[0],y2_[1],y3_[1],y4_[1],y5_[1],y6_[1],y7_[0],y8_[1]]

        R=1.25
        theta1=0.41
        theta2=2*np.pi-theta1
        theta=np.linspace(theta1,theta2,100)

        theta3=0.41
        theta4=2*np.pi-theta1

        x1_=-R*np.cos(theta)
        y1_=R*np.sin(theta)

        x1a_=R*np.cos(theta3)
        y1a_=R*np.sin(theta3)

        x1b_=R*np.cos(theta4)
        y1b_=R*np.sin(theta4)

        a=.504
        b=1.65
        c=2.02

        w1=.5
        w2=.63


        x2_=[x1_[0],x1_[0]-a]
        y2_=[w1,w1]

        x3_=[-b,-b]
        y3_=[w1,w2]

        x4_=[-b,-c]
        y4_=[w2,w2]

        x5_=[-c,-c]
        y5_=[w2,-w2]

        x6_=[-c,-b]
        y6_=[-w2,-w2]

        x7_=[-b,-b]
        y7_=[-w1,-w2]

        x8_=[-b,x1_[-1]]
        y8_=[-w1,-w1]

        x_=[]
        y_=[]
        
        for i in range(len(x1_)):
            x_.append(x1_[i])
            
        x_.append(x2_[0])
        x_.append(x2_[1])
        x_.append(x3_[1])
        x_.append(x4_[1])
        x_.append(x5_[1])
        x_.append(x6_[1])
        x_.append(x7_[0])
        x_.append(x8_[1])
        
        for i in range(len(y1_)):
            y_.append(y1_[i])
            
        y_.append(y2_[0])
        y_.append(y2_[1])
        y_.append(y3_[1])
        y_.append(y4_[1])
        y_.append(y5_[1])
        y_.append(y6_[1])
        y_.append(y7_[0])
        y_.append(y8_[1])        
        # x9_=[x1_[-1],x1a_]
        # y9_=[w1,w1]

        # x10_=[x1a_,x1a_+a]
        # y10_=[w1,w1]

        # x11_=[b,b]
        # y11_=[w1,w2]

        # x12_=[b,c]
        # y12_=[w2,w2]

        # x13_=[c,c]
        # y13_=[w2,-w2]

        # x14_=[c,b]
        # y14_=[-w2,-w2]

        # x15_=[b,b]
        # y15_=[-w1,-w2]

        # x16_=[b,x1b_]
        # y16_=[-w1,-w1]

        # x17_=[x1b_,x1_[-1]]
        # y17_=[-w1,-w1]

        #x_=[x2_[0],x2_[1],x3_[1],x4_[1],x5_[1],x6_[1],x7_[0],x8_[1],x9_[1],x10_[1],x11_[1],x12_[1],x13_[1],x14_[1],x15_[0],x16_[1]]
        #y_=[y2_[0],y2_[1],y3_[1],y4_[1],y5_[1],y6_[1],y7_[0],y8_[1],y9_[1],y10_[1],y11_[1],y12_[1],y13_[1],y14_[1],y15_[0],y16_[1]]
        (self.segments)=self.create_segment(x_,y_)

        phi1_=self.phi_segments(x,y,self.segments)


        #C=self.circle(x,y,R,0,0)
        #phi2_=C
        #phi3_=self.equivalence(phi1_,phi2_,self.m)
        return(phi1_)    
    
    
    
    
    
    
    
    
     def create_segment(self,x,y):
        seglen=len(x)
        segments=np.zeros((seglen-1,4))
        for i in range(seglen-1):
            #[x1,y1,x2,y2]
            #[x2,y2,x3,y3]
            segments[i,0]=x[i]
            segments[i,1]=y[i]
            segments[i,2]=x[i+1]
            segments[i,3]=y[i+1]
        return(segments)                    
        
     def shoelace(self,vertices):
        (m,n)=np.shape(vertices)

        sum1=vertices[0,0]*(vertices[1,1]-vertices[1,n-1])
        for i in range(1,n-1):
            sum1=sum1 + vertices[0,i]*(vertices[1,i+1]-vertices[1,i-1])
        i=n-1
        sum1=sum1 + vertices[0,i]*(vertices[1,0]-vertices[1,i-1])

        A=.5*abs(sum1)
        return (A)     
    
    
class select_import_data:
     def __init__(self,name,path,wxmin,wxmax,wymin,wymax,Psi=None):  
         self.name=name
         self.path=path
         self.mainDirectory = path   # main directory 
         parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
         #print(parameters)
         data=np.load(self.mainDirectory+self.name+'/Radii'+self.name+'.npz',allow_pickle=True)
         self.Rm=data['Rm'] 
         self.wxmin = wxmin 
         self.wxmax = wxmax
         self.wymin = wymin
         self.wymax = wymax
         d=1
         self.m=8
         self.wxmin2 = -d
         self.wxmax2 = d
         self.wymin2 = -d
         self.wymax2 = d

         self.Psi=Psi
         self.parameters=parameters.tolist()    # loads saved parameters      
         #print(self.parameters)
         self.nb=self.parameters['nb'] # number of bots
         self.ni=self.parameters['total_particles']
         self.ns=self.parameters['ns']
         self.nm=self.nb*self.ns # total membrane particles 
         self.bot_width=self.parameters['bot_width']
         self.geom = self.parameters['ball_geometry']
         self.particle_width=self.parameters['particle_width']
         self.particle_width2=self.parameters['particle_width2']
         self.radius2=self.particle_width/2 # radius of particles
         self.radius3=self.particle_width2/2 # radius of particles
         self.control_mode=self.parameters['control_mode']
         self.skin_width=self.parameters['skin_width']
         self.height=self.parameters['bot_height']
         #self.increment= self.parameters['increment']
         
         
         self.path=self.path+self.name+"/results/"
         os.chdir(self.path)
         self.files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)
         
         #### EPSILON_
         self.EPSILON_ = np.genfromtxt(self.files[self.files.index('EPSILON.csv') ] ,delimiter=',')  
         
         #### CALCULATION TIME
         self.calcultation_time  = np.genfromtxt(self.files[self.files.index('calcultation_time.csv') ] ,delimiter=',')
             
             
         #### RHO
         self.rho = self.parameters['rho'] 
             
         #### Qcm
         self.Qcm = np.genfromtxt(self.files[self.files.index('Qcm.csv') ] ,delimiter=',')
         
         #### THETA
         self.THETA=np.genfromtxt(self.files[self.files.index('THETA.csv') ] ,delimiter=',')
         
         #if np.sign(self.increment)==-1:
         # for i in range(len(self.THETA)):
         #    if self.THETA[i]==0:
         #        pass
         #    else:
         #        temp=abs(2*np.pi-self.THETA[i])
         #        while temp>2*np.pi:
         #            #temp=abs(2*np.pi-temp)
         #            temp=abs(temp-2*np.pi)
         #        if temp==2*np.pi:
         #            temp=0
         #        else:
         #            temp=temp
                    
         #        self.THETA[i]=temp

         #### Rr_
         self.Rr_=np.genfromtxt(self.files[self.files.index('Rr_.csv') ] ,delimiter=',')
         
        
         self.angle_entries=[]
         self.epsilon_section={}
         self.epsilon_theta_section_max={}
         self.epsilon_theta_section_mean={}
         self.epsilon_theta_section_median={}
         
         
         
         self.average_epsilon=[]
         self.max_epsilon=[]
         self.median_epsilon=[]
         
         
         
     def sort_epsilon_and_theta(self,count_range):
        theta=self.THETA#[0:-2] 
        epsilon=self.EPSILON_
        
        res0=np.where(epsilon<50)
        epsilon=epsilon[res0[0]]
        theta=theta[res0[0]]
        for k, g in itertools.groupby(theta):
            self.angle_entries.append(k) 
            self.epsilon_section["theta:"+str(k)]=[]
            
        for i in range(count_range):
            self.epsilon_theta_section_max[str(i)]=[]
            self.epsilon_theta_section_mean[str(i)]=[]
            self.epsilon_theta_section_median[str(i)]=[]
            
            
        for i in range(len(self.angle_entries)):
            res=np.where(theta==self.angle_entries[i])
            self.epsilon_section["theta:"+str(self.angle_entries[i])].append(epsilon[res[0]])
            epsilon2=epsilon[res[0]]
            res2=np.nonzero(epsilon[res[0]])
            res2=res2[0]

            self.average_epsilon.append(np.mean(epsilon2[res2]))
            self.max_epsilon.append(max(epsilon[res[0]]))
            self.median_epsilon.append(np.median(epsilon2[res2]))
        
        count=0
        for i in range(len(self.max_epsilon)):
            
            if count==count_range:
                count=0
            else:
                count=count
            
            #self.epsilon_theta_section[str(count)].append(self.max_epsilon[i])
            self.epsilon_theta_section_max[str(count)].append(self.max_epsilon[i])
            self.epsilon_theta_section_mean[str(count)].append(self.average_epsilon[i])
            self.epsilon_theta_section_median[str(count)].append(self.median_epsilon[i])
            count=count+1
            
            

         