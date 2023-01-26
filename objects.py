# -*- coding: utf-8 -*-
"""
Created on Wed Jul 21 14:14:49 2021

@author: dmulr
"""

import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
from scipy.interpolate import Rbf
from scipy.interpolate import RegularGridInterpolator
import matplotlib.pyplot as plt
from matplotlib import animation
import matplotlib.font_manager as fm
import matplotlib.patches as patches
import matplotlib
from matplotlib.patches import RegularPolygon
import random
import os
import csv
from csv import writer
import glob
import timeit
import cv2
from IPython.display import HTML
from shutil import copyfile
fm._rebuild()
#plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['font.family'] = 'Arial'
plt.rcParams['mathtext.fontset'] = 'dejavuserif'
plt.rcParams['font.size'] = 8
plt.rcParams['axes.linewidth'] = .1
plt.rcParams["text.usetex"] = True


class robots:
    def __init__(self,name,my_system,body_floor,path):
        self.name=name
        self.my_system = my_system
        self.body_floor = body_floor
        ###### Imported Variables #########
        self.path=path
        self.mainDirectory = self.path
        
        copyfile(__file__,self.mainDirectory+self.name+"/"+'objects.py')
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
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
        self.R = self.convert_dist*self.parameters['R']
        self.bot_volume=np.pi*self.bot_height*(self.bot_width/2)**2   # calculate volume
        self.membrane_width = self.convert_dist*self.parameters['membrane_width']
        self.membrane_type = self.parameters['membrane_type']
        self.skin_width = self.parameters['skin_width']
        self.ns = self.parameters['ns']
        self.spring_stiffness = self.parameters['spring_stiffness'] 
        self.spring_damping = self.parameters['spring_damping'] 
        
        
        
        self.lateralFriction = self.parameters['lateralFriction']
        self.spinningFriction = self.parameters['spinningFriction']
        self.rollingFriction = self.parameters['rollingFriction']
        self.dampingterm = self.parameters['dampingterm']
        self.Compliance_tangent = self.parameters['Ctr'] 
        self.Compliance = self.parameters['Cr'] 
        self.membrane_density = self.parameters['membrane_density']
        ####### Calculated variables ######
        self.bot_density=self.bot_mass/self.bot_volume # calculate density of robot 
        self.bot_material = self.Material(self.lateralFriction)
        #self.membrane_density = 2000
        self.countm = 0 
        self.rl = 0

        self.fixed = self.parameters['fixed']
        self.membrane_material = self.Material(self.lateralFriction)
        # Colors
        self.col_y = chrono.ChColorAsset(); self.col_y.SetColor(chrono.ChColor(1, 1, 0))       # Yellow
        self.col_b = chrono.ChColorAsset(); self.col_b.SetColor(chrono.ChColor(0, 0, 1))       # Blue
        self.col_g = chrono.ChColorAsset(); self.col_g.SetColor(chrono.ChColor(0, 1, 0))       # Green
        self.col_p = chrono.ChColorAsset(); self.col_p.SetColor(chrono.ChColor(0.44, .11, 52)) # Purple
        self.col_w = chrono.ChColorAsset(); self.col_p.SetColor(chrono.ChColor(.8, .8, .8)) # Purple
        
        
        self.skinM = []
        self.bots = []
        self.force = []
        self.Springs = []
        
        self.bot_xposition = {}
        self.bot_yposition = {}
        self.bot_zposition = {}
        
        self.bot_xvelocity = {}
        self.bot_yvelocity = {}
        self.bot_zvelocity = {}
        
        self.bot_xForcetotal = {}
        self.bot_yForcetotal = {}
        self.bot_zForcetotal = {}
        
        
        self.bot_xForcecontact = {}
        self.bot_yForcecontact = {}
        self.bot_zForcecontact = {}
        
        
        self.skin_xposition = {}
        self.skin_yposition = {}
        self.skin_zposition = {}
        self.R=self.R+.2
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
        
        
        
        
            # postion 
            theta=i*2*np.pi/self.nb # set angle
            
            x = self.R*np.cos(theta)+self.xcenter  # set x positon 
            y = self.bot_height/2                     # set y position 
            z = self.R*np.sin(theta)+self.zcenter  # set z position 
            # create body
            #### geometry of the robots to be cylinders
            if self.bot_geom=="cylinder":
                
                bot = chrono.ChBodyEasyCylinder(self.bot_width/2, self.bot_height,self.bot_density,True,True)
                # set position
                bot.SetPos(chrono.ChVectorD(x,y,z)) 
                bot.SetName("bot"+str(i)) # set name (important for contact tracking)
                bot.SetId(i)              # set id   (impoortant for contact tracking )  
                # material 
                bot.SetMaterialSurface(self.bot_material)  # set material 
                # rotate them so we can form the membrane
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
                
                # zeroth robot so we know which is which in the videos
                if i==0:   
                    bot.AddAsset(self.col_p)   
                if i>=20 and i<=25:
                    bot.AddAsset(self.col_w)
                if i==22:   
                    bot.AddAsset(self.col_y)  
                
                # set collision
                bot.SetCollide(True)
                
                
                # set fixed
                bot.SetBodyFixed(self.fixed)
                
                # link to floor
                #pt=chrono.ChLinkMatePlane()
                #pt.Initialize(self.body_floor,bot,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
                
                #add link to the system 
                #self.my_system.AddLink(pt)
                
                # add bot to series of array 
                self.my_system.Add(bot) # add bot to system 
                self.bots.append(bot) # add bot to bot array 
                
                              
            if self.membrane_type==1:
                b_ang=2*np.pi/self.nb                   # angle between centers of bots
                o_ang=np.arctan((self.bot_width/2)/self.R)   # angle offset for radius of bot
                p_ang=np.arctan((self.skin_width/2)/self.R)           # angle offset for radius of skin particle
                # Between this bot and last bot
                if i>=1 and i<self.nb:
                    for j in range(1,self.ns+1,1):
                        # Initial postion of each particle
                        theta=i*b_ang + j*(b_ang-o_ang-p_ang)/(self.ns) + p_ang 
                        x=self.R*np.cos(theta)+self.xcenter # x position 
                        y=self.bot_height/2              # y position 
                        z=self.R*np.sin(theta)+self.zcenter # z position  
                        
                        # create them and set position
                        #skinm = chrono.ChBodyEasySphere(self.skin_width/2,self.membrane_density,True,True)
                        skinm = chrono.ChBodyEasyCylinder(self.skin_width/2, .5*self.bot_height,self.membrane_density,True,True) # create particle
                        skinm.SetPos(chrono.ChVectorD(x,y,z)) # set position 
                        skinm.SetMaterialSurface(self.membrane_material) # add material 
                        skinm.SetNoGyroTorque(True) # no gyro toruqe 
                        skinm.SetName('skin'+str(i)) # create name 
                        skinm.SetId(i) # set id 
                                    # link to floor
                        pt=chrono.ChLinkMatePlane()
                        pt.Initialize(self.body_floor,skinm,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0)) 
                        self.my_system.AddLink(pt)
                        # rotate them bout y axis
                        rotation1 = chrono.ChQuaternionD()
                        rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0));  
                        skinm.SetRot(rotation1)
                        self.skin_xposition["skin_xposition{0}".format(self.countm)]=[]  #x position
                        self.skin_yposition["skin_yposition{0}".format(self.countm)]=[]  # y position
                        self.skin_zposition["skin_zposition{0}".format(self.countm)]=[]  # z position
                        
                        
                        skinm.SetBodyFixed(self.fixed)
                        self.countm=self.countm+1
                        # Attach springs if more then one was created    
                        if j>1:
                            ground=chrono.ChLinkSpring() # create spring 1
                            p1=0; p2=self.skin_width/2 # points where each spring is attatched 
                            p3=0; p4=-self.skin_width/2
                            h=self.bot_height/5
                            #h=self.bot_height/2
                            #h=0
                            ground.Initialize(self.skinM[-1], skinm,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),True) # link spring to particles
                            ground.Set_SpringK(self.spring_stiffness) # set spring constant
                            ground.Set_SpringR(self.spring_damping) # set damping constant
                            ground.Set_SpringRestLength(self.rl) # set resting length 
                            ground.AddAsset(self.col_p) # add color 
                            ground.AddAsset(chrono.ChPointPointSpring(.01,80,15)) # create visual 
                            self.my_system.AddLink(ground) # add spring to system 
                            self.Springs.append(ground)
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
                        theta=(i+1)*b_ang + j*(b_ang-o_ang-p_ang)/(self.ns) + p_ang
                        x=self.R*np.cos(theta)+self.xcenter
                        y=self.bot_height/2
                        z=self.R*np.sin(theta)+self.zcenter
                        self.skin_xposition["skin_xposition{0}".format(self.countm)]=[]  #x position
                        self.skin_yposition["skin_yposition{0}".format(self.countm)]=[]  # y position
                        self.skin_zposition["skin_zposition{0}".format(self.countm)]=[]  # z position
                        self.countm=self.countm+1
                        # Create particles
                        #skinm = chrono.ChBodyEasySphere(self.skin_width/2,self.membrane_density,True,True)
                        skinm = chrono.ChBodyEasyCylinder(self.skin_width/2, .5*self.bot_height,self.membrane_density,True,True)
                        skinm.SetPos(chrono.ChVectorD(x,y,z))
                        skinm.SetMaterialSurface(self.membrane_material)
                        skinm.SetNoGyroTorque(True)
                        skinm.SetName("skin"+str(i))
                        skinm.SetId(i)
                        # rotate them
                        rotation1 = chrono.ChQuaternionD()
                        rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0));  
                        skinm.SetRot(rotation1)
                        pt=chrono.ChLinkMatePlane()
                        pt.Initialize(self.body_floor,skinm,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
                        self.my_system.AddLink(pt)
                        skinm.SetBodyFixed(self.fixed)
                        # Attach springs    
                        if j>1:
                            ground=chrono.ChLinkSpring()
                            p1=0; p2=self.skin_width/2
                            p3=0; p4=-self.skin_width/2
                            #h=self.bot_height/5
                            h=self.bot_height/5
                            #h=0
                            ground.Initialize(self.skinM[-1], skinm,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),True)
                            ground.Set_SpringK(self.spring_stiffness)
                            ground.Set_SpringR(self.spring_damping)
                            ground.Set_SpringRestLength(self.rl)
                            ground.AddAsset(self.col_y)
                            ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                            self.my_system.AddLink(ground)
                            self.Springs.append(ground)
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
            
                
            
            
            if self.membrane_type==2:
                b_ang=2*np.pi/self.nb                   # angle between centers of bots
                o_ang=np.arctan((self.bot_width/2)/self.R)   # angle offset for radius of bot
                p_ang=np.arctan((self.skin_width/2)/self.R)           # angle offset for radius of skin particle
                # Between this bot and last bot
                if i>=1 and i<self.nb:
                    for j in range(1,self.ns+1,1):
                        # Initial postion of each particle
                        theta=i*b_ang + j*(b_ang-o_ang-p_ang)/(self.ns) + p_ang 
                        x=self.R*np.cos(theta)+self.xcenter # x position 
                        y=self.bot_height/2              # y position 
                        z=self.R*np.sin(theta)+self.zcenter # z position  
                        
                        # create them and set position
                        skinm = chrono.ChBodyEasySphere(self.skin_width/2,self.membrane_density,True,True)
                        #skinm = chrono.ChBodyEasyCylinder(self.skin_width/2, .95*self.bot_height,self.membrane_density,True,True) # create particle
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
                        self.skin_xposition["skin_xposition{0}".format(self.countm)]=[]  #x position
                        self.skin_yposition["skin_yposition{0}".format(self.countm)]=[]  # y position
                        self.skin_zposition["skin_zposition{0}".format(self.countm)]=[]  # z position
                        
                        
                        
                        self.countm=self.countm+1
                        # Attach springs if more then one was created    
                        if j>1:
                            ground=chrono.ChLinkSpring() # create spring 1
                            p1=0; p2=self.skin_width/2 # points where each spring is attatched 
                            p3=0; p4=-self.skin_width/2
                            #h=self.bot_height/5
                            #h=self.bot_height/2
                            h=0
                            ground.Initialize(self.skinM[-1], skinm,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),True) # link spring to particles
                            ground.Set_SpringK(self.spring_stiffness) # set spring constant
                            ground.Set_SpringR(self.spring_damping) # set damping constant
                            ground.Set_SpringRestLength(self.rl) # set resting length 
                            ground.AddAsset(self.col_p) # add color 
                            ground.AddAsset(chrono.ChPointPointSpring(.01,80,15)) # create visual 
                            self.my_system.AddLink(ground) # add spring to system 
                            self.Springs.append(ground)
                            # ground1=chrono.ChLinkSpring() # create spring 2
                            # ground1.Initialize(self.skinM[-1], skinm,True,chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),True) # link spring to particles 
                            # ground1.Set_SpringK(self.spring_stiffness) # set spring constant
                            # ground1.Set_SpringR(self.spring_damping) # set damping 
                            # ground1.AddAsset(self.col_p) # create color 
                            # ground1.AddAsset(chrono.ChPointPointSpring(.01,80,15)) # create visual 
                            # self.my_system.AddLink(ground1) # add to the system                                 
                            # self.Springs.append(ground1)
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
                        theta=(i+1)*b_ang + j*(b_ang-o_ang-p_ang)/(self.ns) + p_ang
                        x=self.R*np.cos(theta)+self.xcenter
                        y=self.bot_height/2
                        z=self.R*np.sin(theta)+self.zcenter
                        self.skin_xposition["skin_xposition{0}".format(self.countm)]=[]  #x position
                        self.skin_yposition["skin_yposition{0}".format(self.countm)]=[]  # y position
                        self.skin_zposition["skin_zposition{0}".format(self.countm)]=[]  # z position
                        self.countm=self.countm+1
                        # Create particles
                        skinm = chrono.ChBodyEasySphere(self.skin_width/2,self.membrane_density,True,True)
                        #skinm = chrono.ChBodyEasyCylinder(self.skin_width/2, .95*self.bot_height,self.membrane_density,True,True)
                        skinm.SetPos(chrono.ChVectorD(x,y,z))
                        skinm.SetMaterialSurface(self.membrane_material)
                        skinm.SetNoGyroTorque(True)
                        skinm.SetName("skin"+str(i))
                        skinm.SetId(i)
                        # rotate them
                        rotation1 = chrono.ChQuaternionD()
                        rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0));  
                        skinm.SetRot(rotation1)
                        pt=chrono.ChLinkMatePlane()
                        pt.Initialize(self.body_floor,skinm,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
                        #self.my_system.AddLink(pt)
                        # Attach springs    
                        if j>1:
                            ground=chrono.ChLinkSpring()
                            p1=0; p2=self.skin_width/2
                            p3=0; p4=-self.skin_width/2
                            #h=self.bot_height/5
                            h=self.bot_height/4
                            h=0
                            ground.Initialize(self.skinM[-1], skinm,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),True)
                            ground.Set_SpringK(self.spring_stiffness)
                            ground.Set_SpringR(self.spring_damping)
                            ground.Set_SpringRestLength(self.rl)
                            ground.AddAsset(self.col_y)
                            ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                            self.my_system.AddLink(ground)
                            self.Springs.append(ground)
                            # ground1=chrono.ChLinkSpring()
                            # ground1.Initialize(self.skinM[-1], skinm,True,chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),True)
                            # ground1.Set_SpringK(self.spring_stiffness)
                            # ground1.Set_SpringR(self.spring_damping)
                            # ground.Set_SpringRestLength(self.rl) # set resting length 
                            # ground1.AddAsset(self.col_y)
                            # ground1.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                            # self.my_system.AddLink(ground1)  
                            # self.Springs.append(ground1)
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
            
    # save velocity data
    def save_data_velocity(self):
        ''' save velocity of each bot '''
        for i in range(self.nb):
            self.bot_xvelocity["bot_xvelocity"+str(i)].append(self.bots[i].GetPos_dt().x)
            self.bot_yvelocity["bot_yvelocity"+str(i)].append(self.bots[i].GetPos_dt().y)
            self.bot_zvelocity["bot_zvelocity"+str(i)].append(self.bots[i].GetPos_dt().z)
            
                 # total forces
            self.bot_xForcetotal["bot_xForcetotal".format(i)]=[]  # Force x
            self.bot_yForcetotal["bot_yForcetotal".format(i)]=[]  # Force y
            self.bot_zForcetotal["bot_zForcetotal".format(i)]=[]  # force z
            
            
            # contact forces
            self.bot_xForcecontact["bot_xForcecontact".format(i)]=[]  # Force x
            self.bot_yForcecontact["bot_yForcecontact".format(i)]=[]  # Force y
            self.bot_zForcecontact["bot_zForcecontact".format(i)]=[]  # force z               
            
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
    def return_force_data(self):
        ''' return dictionary of each bot forces '''
        return(self.bot_xForcetotal,self.bot_yForcetotal,self.bot_zForcetotal)
  
    # return force data    
    def return_force_data_contact(self):
        ''' return dictionary of each bot forces '''
        return(self.bot_xForcecontact,self.bot_yForcecontact,self.bot_zForcecontact)
    
    
    
class Interiors:
    def __init__(self,name,my_system,body_floor,path):
        self.name=name
        self.my_system = my_system
        self.body_floor = body_floor    
        self.mainDirectory = path
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters=parameters.tolist()
        
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
        #self.R = self.convert_dist*self.parameters['R']
        self.scale_radius = self.parameters['scale_radius']
        self.R =  self.scale_radius*self.convert_dist*self.parameters['R']
        #print(self.R)
        self.interior_mode = self.parameters['interior_mode']
        #print(self.interior_mode)
        self.offset_radius=self.parameters['offset_radius']
        
        self.lateralFriction = self.parameters['lateralFriction']
        self.spinningFriction = self.parameters['spinningFriction']
        self.rollingFriction = self.parameters['rollingFriction']
        self.dampingterm = self.parameters['dampingterm']
        self.Compliance_tangent = self.parameters['Ctp'] 
        self.Compliance = self.parameters['Cp'] 
        self.particle_volume=np.pi*self.particle_height*(self.particle_width/2)**2   # calculate volume
        self.particle_density=self.particle_mass/self.particle_volume # calculate density of robot 
        #self.particle_material = self.Material()
        
        self.fixed = self.parameters['fixed']
        
        if self.interior_mode=="tunnel0":
            self.total_particles = 0
            self.particle_material = self.Material(0.01)
            self.parameters['n'] = [0]
            self.parameters['total_particles'] = self.total_particles
            self.parameters['Ri'] = [0]         
        
        elif self.interior_mode=="tunnel1":
            self.total_particles = 12
            self.particle_material = self.Material(0.01)
            self.parameters['n'] = [12]
            self.parameters['total_particles'] = self.total_particles
            self.parameters['Ri'] = [.28]         
     
        elif self.interior_mode=="tunnel2":
            self.total_particles = 24
            self.particle_material = self.Material(0.01)
            self.parameters['n'] = [12,12]
            self.parameters['total_particles'] = self.total_particles
            self.parameters['Ri'] = [0.28,0.18]                
     
        elif self.interior_mode=="experiment_shape":
            self.total_particles = 48
            self.particle_material = self.Material(0.01)
            self.parameters['n'] = [12,12,12,8,4]
            self.parameters['total_particles'] = self.total_particles
            self.parameters['Ri'] = [.38,.32,.27,.16,.07] 
        
        else:
            (self.n,self.Ri) = self.MaxValues()
            self.total_particles = np.sum(self.n)
            self.particle_material = self.Material(0.01)
            self.parameters['n'] = self.n
            self.parameters['total_particles'] = self.total_particles
            self.parameters['Ri'] = self.Ri            
            
        #self.particle_material = self.Material(0.1)            
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
        

        # with open(self.mainDirectory+name+"/Parameters.csv", 'a') as f_object:
        #     # Pass this file object to csv.writer()
        #     # and get a writer object
        #     writer_object = writer(f_object)
        #     #Pass the list as an argument into
        #     #the writerow()
        #     writer_object.writerow(['ring configuration',self.n])
        #     writer_object.writerow(['total number of particles',self.total_particles])
        #     #Close the file object
        #     f_object.close()        
        
        
        
        self.particle_xposition = {}
        self.particle_yposition = {}
        self.particle_zposition = {}
        
        
        self.particle_xvelocity = {}
        self.particle_yvelocity = {}
        self.particle_zvelocity = {}
        
        
        self.particle_xForcetotal = {}
        self.particle_yForcetotal = {}
        self.particle_zForcetotal = {}
        
        
        self.particle_xForcecontact = {}
        self.particle_yForcecontact = {}
        self.particle_zForcecontact = {}        
        
        
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
        
        # empty array
        self.particle_xposition = {}
        self.particle_yposition  = {}
        self.particle_zposition  = {}
        
        self.particle_xvelocity = {}
        self.particle_yvelocity  = {}
        self.particle_zvelocity  = {}
        


        #### mono-dispersion ####
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
                
                R2=self.radius2*self.n[i]/(np.pi) + const   
                
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


        #### bidispersion ####
        if self.interior_mode=="bidispersion": 
            count=0
            for i in range(len(self.n)):
                #print("i=",str(i))
                if i%2==0:
                    #self.radius2 = (self.particle_width/2)*(2**.5)
                    self.radius2 = (self.particle_width2/2)                    
                    con = 'b'
                    const = 0
                    self.N1 = 1+self.N1
                else:
                    self.radius2 = self.particle_width/2 
                    con = 'a'
                    const = 0
                    self.N2 = 1+self.N2
                    
                    
                R2=self.radius2*self.n[i]/(np.pi) + const
                # empty arrays of variables
                for j in range(self.n[i]):
                    self.particle_xposition["particle_xposition{0}".format(count)]=[] # x position  
                    self.particle_yposition["particle_yposition{0}".format(count)]=[] # y position 
                    self.particle_zposition["particle_zposition{0}".format(count)]=[] # z position 
                    self.particle_xvelocity["particle_xvelocity{0}".format(count)]=[] # x velocity
                    self.particle_yvelocity["particle_yvelocity{0}".format(count)]=[] # y velocity
                    self.particle_zvelocity["particle_zvelocity{0}".format(count)]=[] # z velocity 
                    
                    #R2=self.radius2*self.n[i]/(np.pi) + const# raidus of ring 
                    # x,y,z positions
                    x=(R2+.1)*np.cos(j*2*np.pi/self.n[i])+self.xcenter
                    y=.5*self.particle_height 
                    z=(R2+.1)*np.sin(j*2*np.pi/self.n[i])+self.zcenter
                    #print("j=",str(j),str(np.round(self.radius2,3)),"x,y",str(np.round(x,2)),str(np.round(z,2)))
                    self.Rm.append(self.radius2)
                    self.Area = self.Area + (np.pi)*(self.radius2)**2
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
                    #gran = chrono.ChBodyEasyCylinder(self.radius2 , self.particle_height ,self.particle_density,True,True)
                    gran =chrono.ChBodyEasyBox(2*self.radius2,self.particle_height,2*self.radius2,self.particle_density,True,True) # create object
                    gran.SetPos(chrono.ChVectorD(x,y,z))
                    gran.SetMaterialSurface(self.particle_material)
                    gran.SetName('gran'+str(con)+str(count))
                    gran.SetId(i)
                    gran.SetCollide(True)
                    gran.SetBodyFixed(self.fixed)
                    rotation1 = chrono.ChQuaternionD() # rotate the robots about y axis 
                    rotation1.Q_from_AngAxis(-j*2*np.pi/self.n[i], chrono.ChVectorD(0, 1, 0)) 
                    gran.SetRot(rotation1)                    
                    
                    
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
                    # pt=chrono.ChLinkMatePlane()
                    # pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
                    # self.my_system.AddLink(pt)
                    
                    # add to system
                    self.my_system.Add(gran) # add object to system 
                    self.particles.append(gran) 
        
                    
        
        #### experiment_shape ####
        if self.interior_mode=="experiment_shape":
            count=0
            data=np.load("F:/Soro_chrono/python/Pychrono/Strings/Strings_soro/particle_fill_starts/experiment_shape_start_position.npz")
            xx=data['Xp']
            zz=data['Yp']
            Rr=.01*data['R']
            for i in range(self.total_particles):
                        
                self.radius2=Rr[i]
                if Rr[i]==2:
                    con='b'
                    const=0
                else:
                    con='a'
                    const=0
                        
                # empty arrays of variables
                #print(count)
                self.particle_xposition["particle_xposition{0}".format(count)]=[] # x position  
                self.particle_yposition["particle_yposition{0}".format(count)]=[] # y position 
                self.particle_zposition["particle_zposition{0}".format(count)]=[] # z position 
                
                self.particle_xvelocity["particle_xvelocity{0}".format(count)]=[] # x velocity
                self.particle_yvelocity["particle_yvelocity{0}".format(count)]=[] # y velocity
                self.particle_zvelocity["particle_zvelocity{0}".format(count)]=[] # z velocity 

                count=count+1    
                y=0.5*self.radius2
                #y=0.5*self.particle_height
                self.Rm.append(self.radius2)
                # create body
                gran = chrono.ChBody()
                gran = chrono.ChBodyEasySphere(self.radius2,self.particle_density,True,True)
                #gran = chrono.ChBodyEasyCylinder(self.radius2, self.particle_height ,self.particle_density,True,True)
                gran.SetPos(chrono.ChVectorD(xx[i]*.01, y, zz[i]*.01))
                gran.SetMaterialSurface(self.particle_material)
                gran.SetName('gran'+str(con)+str(count))
                gran.SetId(i)
                gran.SetCollide(True)
                gran.SetBodyFixed(self.fixed)
                #pt=chrono.ChLinkMatePlane()
                #pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
                #self.my_system.AddLink(pt)
                    

                self.my_system.Add(gran)
                self.particles.append(gran)  

        #### tunnel0 ####
        if self.interior_mode=="tunnel0":
            pass
            # count=0
            # #data=np.load("F:/Soro_chrono/python/Pychrono/Strings/Strings_soro/particle_fill_starts/experiment_shape_start_position.npz")
            # xx=data['Xp']
            # zz=data['Yp']
            # Rr=.01*data['R']
            # for i in range(self.total_particles):
                        
            #     self.radius2=Rr[i]
            #     if Rr[i]==2:
            #         con='b'
            #         const=0
            #     else:
            #         con='a'
            #         const=0
                        
            #     # empty arrays of variables
            #     #print(count)
            #     self.particle_xposition["particle_xposition{0}".format(count)]=[] # x position  
            #     self.particle_yposition["particle_yposition{0}".format(count)]=[] # y position 
            #     self.particle_zposition["particle_zposition{0}".format(count)]=[] # z position 
                
            #     self.particle_xvelocity["particle_xvelocity{0}".format(count)]=[] # x velocity
            #     self.particle_yvelocity["particle_yvelocity{0}".format(count)]=[] # y velocity
            #     self.particle_zvelocity["particle_zvelocity{0}".format(count)]=[] # z velocity 

            #     count=count+1    
            #     y=0.5*self.radius2
            #     #y=0.5*self.particle_height
            #     self.Rm.append(self.radius2)
            #     # create body
            #     gran = chrono.ChBody()
            #     gran = chrono.ChBodyEasySphere(self.radius2,self.particle_density,True,True)
            #     #gran = chrono.ChBodyEasyCylinder(self.radius2, self.particle_height ,self.particle_density,True,True)
            #     gran.SetPos(chrono.ChVectorD(xx[i]*.01, y, zz[i]*.01))
            #     gran.SetMaterialSurface(self.particle_material)
            #     gran.SetName('gran'+str(con)+str(count))
            #     gran.SetId(i)
            #     gran.SetCollide(True)
            #     gran.SetBodyFixed(self.fixed)
            #     #pt=chrono.ChLinkMatePlane()
            #     #pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
            #     #self.my_system.AddLink(pt)
                    

            #     self.my_system.Add(gran)
            #     self.particles.append(gran) 

        #### tunnel1 ####
        if self.interior_mode=="tunnel1":
            count=0
            data=np.load("F:/Soro_chrono/python/Pychrono/Strings/Strings_soro/particle_fill_starts/experiment_tunnel_1_start_position.npz")
            xx=data['Xp']
            zz=data['Yp']
            Rr=.01*data['R']
            for i in range(self.total_particles):
                        
                self.radius2=Rr[i]
                if Rr[i]==2:
                    con='b'
                    const=0
                else:
                    con='a'
                    const=0
                        
                # empty arrays of variables
                #print(count)
                self.particle_xposition["particle_xposition{0}".format(count)]=[] # x position  
                self.particle_yposition["particle_yposition{0}".format(count)]=[] # y position 
                self.particle_zposition["particle_zposition{0}".format(count)]=[] # z position 
                
                self.particle_xvelocity["particle_xvelocity{0}".format(count)]=[] # x velocity
                self.particle_yvelocity["particle_yvelocity{0}".format(count)]=[] # y velocity
                self.particle_zvelocity["particle_zvelocity{0}".format(count)]=[] # z velocity 

                count=count+1    
                y=0.5*self.radius2
                #y=0.5*self.particle_height
                self.Rm.append(self.radius2)
                # create body
                gran = chrono.ChBody()
                gran = chrono.ChBodyEasySphere(self.radius2,self.particle_density,True,True)
                #gran = chrono.ChBodyEasyCylinder(self.radius2, self.particle_height ,self.particle_density,True,True)
                gran.SetPos(chrono.ChVectorD(xx[i]*.01, y, zz[i]*.01))
                gran.SetMaterialSurface(self.particle_material)
                gran.SetName('gran'+str(con)+str(count))
                gran.SetId(i)
                gran.SetCollide(True)
                gran.SetBodyFixed(self.fixed)
                #pt=chrono.ChLinkMatePlane()
                #pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
                #self.my_system.AddLink(pt)
                    

                self.my_system.Add(gran)
                self.particles.append(gran)        

        #### tunnel2 ####
        if self.interior_mode=="tunnel2":
            count=0
            data=np.load("F:/Soro_chrono/python/Pychrono/Strings/Strings_soro/particle_fill_starts/experiment_tunnel_2_start_position.npz")
            xx=data['Xp']
            zz=data['Yp']
            Rr=.01*data['R']
            for i in range(self.total_particles):
                        
                self.radius2=Rr[i]
                if Rr[i]==2:
                    con='b'
                    const=0
                else:
                    con='a'
                    const=0
                        
                # empty arrays of variables
                #print(count)
                self.particle_xposition["particle_xposition{0}".format(count)]=[] # x position  
                self.particle_yposition["particle_yposition{0}".format(count)]=[] # y position 
                self.particle_zposition["particle_zposition{0}".format(count)]=[] # z position 
                
                self.particle_xvelocity["particle_xvelocity{0}".format(count)]=[] # x velocity
                self.particle_yvelocity["particle_yvelocity{0}".format(count)]=[] # y velocity
                self.particle_zvelocity["particle_zvelocity{0}".format(count)]=[] # z velocity 

                count=count+1    
                y=0.5*self.radius2
                #y=0.5*self.particle_height
                self.Rm.append(self.radius2)
                # create body
                gran = chrono.ChBody()
                gran = chrono.ChBodyEasySphere(self.radius2,self.particle_density,True,True)
                #gran = chrono.ChBodyEasyCylinder(self.radius2, self.particle_height ,self.particle_density,True,True)
                gran.SetPos(chrono.ChVectorD(xx[i]*.01, y, zz[i]*.01))
                gran.SetMaterialSurface(self.particle_material)
                gran.SetName('gran'+str(con)+str(count))
                gran.SetId(i)
                gran.SetCollide(True)
                gran.SetBodyFixed(self.fixed)
                #pt=chrono.ChLinkMatePlane()
                #pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
                #self.my_system.AddLink(pt)
                    

                self.my_system.Add(gran)
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
        material.SetRollingFriction(self.rollingFriction)
        material.SetSpinningFriction(self.spinningFriction)
        # material.SetComplianceRolling(Cr)
        # material.SetComplianceSpinning(Cs)
        return (material)
        
        
        
        
    def MaxValues(self):
        # if self.interior_mode=='empty':
        #     N=[]
        #     R=0

        radius=self.bot_width/2
        radius2=self.particle_width/2 
        radius3=self.particle_width2/2 
        if self.interior_mode=="bidispersion":            
            Rin=self.R-radius
            S=2*radius2+2*radius3
            P=int(Rin/S)
            Ri=np.zeros(2*P)

            N=[]
            for i in range(P):
                #Ri[2*i]=Rin-np.sqrt(2)*radius2-i*S
                Ri[2*i]=Rin-radius3-i*S                
                Ri[2*i+1]=Rin-(2*radius3+radius2)-i*S
    

            for i in range(P):
                N.append(int((np.pi*2*Ri[2*i])/(2*radius3)))
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
            
        # if self.interior_mode=="Verify":
        #     N=0
        #     Ri=[0]
            
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
            
         

               
    # return position data
    def return_position_data(self):
        ''' return the dictionary of each bot '''
        return(self.particle_xposition,self.particle_yposition,self.particle_zposition)
    

      
    # return velocity data
    def return_velocity_data(self):
        ''' return the diction of each bot velocity '''
        return(self.particle_xvelocity,self.particle_yvelocity,self.particle_zvelocity)
        
  
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
        self.Compliance_tangent = self.parameters['Ctr'] 
        self.Compliance = self.parameters['Cr'] 
        
        self.wall = []
        
        
        self.material1 = self.Material(0.05)
        self.material2 = self.Material(0)
        self.material3 = self.Material(0.01)
        
        
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
        
 

        self.body_floor2 = chrono.ChBody()
        self.body_floor2.SetName('floor2')
        self.body_floor2.SetBodyFixed(True)
        self.body_floor2.SetPos(chrono.ChVectorD(0, self.floor_height+.1022, 0 ))
        self.body_floor2.SetMaterialSurface(self.material2)
        self.body_floor2.GetCollisionModel().ClearModel()
        self.body_floor2.GetCollisionModel().AddBox(self.floor_length, self.floor_height, self.floor_length) # hemi sizes
        self.body_floor2.GetCollisionModel().BuildModel()       
        self.body_floor2.SetCollide(True)
        self.my_system.Add(self.body_floor2)
        
        
        
        

        # #px=0
        # #pz=0
        # wall = chrono.ChBody()
        # # Attach a visualization shape .
        # # First load a .obj from disk into a ChTriangleMeshConnected:
        # path="F:/Soro_chrono/python/Pychrono/Strings/Strings_soro/shapes/"
        # mesh_for_visualization = chrono.ChTriangleMeshConnected()
        # mesh_for_visualization.LoadWavefrontMesh(path+'wall_7.obj')
        # mesh_for_visualization.Transform(chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(1))
        # visualization_shape = chrono.ChTriangleMeshShape()
        # visualization_shape.SetMesh(mesh_for_visualization)
        # wall.AddAsset(visualization_shape)

        # mesh_for_collision = chrono.ChTriangleMeshConnected()
        # mesh_for_collision.LoadWavefrontMesh(path+'wall_7.obj')
        # # Optionally: you can scale/shrink/rotate the mesh using this:
        # mesh_for_collision.Transform(chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(1))
        # wall.GetCollisionModel().ClearModel()
        # wall.GetCollisionModel().AddTriangleMesh(
        # mesh_for_collision, # the mesh 
        # False,  # is it static?
        # False)  # is it convex?
        # wall.GetCollisionModel().BuildModel()        
        # wall.SetPos(chrono.ChVectorD(0,-3.2,0))
        # wall.SetMass(16)
        # wall.SetInertiaXX(chrono.ChVectorD(0.270,0.400,0.427))
        # wall.SetInertiaXY(chrono.ChVectorD(0.057,0.037,-0.062))            
        # wall.SetBodyFixed(True)
        # col_y = chrono.ChColorAsset() # apply color
        # col_y.SetColor(chrono.ChColor(1, 1, 0))
        # wall.AddAsset(col_y)
        # wall.SetCollide(True) # set the collision mode
        # self.my_system.Add(wall)         
    
        
        
        
        
        
        
        
        
        
        
        
        
        
        # col_p = chrono.ChColorAsset()
        # col_p.SetColor(chrono.ChColor(1,0.1,0.5))        

        # w = .142
        # l = 0.65
        # gap = .30
        # xpos1_wall = -0.52
        # xpos2_wall = -0.52      
        # px1 = xpos1_wall #+ w/2
        # px2 = xpos2_wall #+ w/2
        # py1 = gap + l/2
        # py2 = -(gap + l/2)
        # print("xpos_1 wall:",(np.round(px1,2)))
        # print("ypos_1 wall:",(np.round(py1,2)))         
        
        # wall1 = chrono.ChBody()
        # wall1 = chrono.ChBodyEasyBox(w,w,l,1000,True,True) # create object
        # wall1.SetPos(chrono.ChVectorD(px1,w/2,py1)) # set its position 
        # wall1.SetMaterialSurface(self.material3) # apply material properties 
        # wall1.SetName('wall1')
        # wall1.SetBodyFixed(True)
        # wall1.AddAsset(col_p)
        # self.my_system.Add(wall1)
        # self.wall.append(wall1)        
        
        # wall2 = chrono.ChBody()
        # wall2 = chrono.ChBodyEasyBox(w,w,l,1000,True,True) # create object
        # wall2.SetPos(chrono.ChVectorD(px2,w/2,py2)) # set its position 
        # wall2.SetMaterialSurface(self.material3) # apply material properties 
        # wall2.SetName('wall2')
        # wall2.SetBodyFixed(True)
        # wall2.AddAsset(col_p)
        # self.my_system.Add(wall2)
        # self.wall.append(wall2)        
                

        # px3 = -1.448 #+ w/2
        # py3 = 0
      
        
        # wall3 = chrono.ChBody()
        # wall3 = chrono.ChBodyEasyBox(w,w,3*l,1000,True,True) # create object
        # wall3.SetPos(chrono.ChVectorD(px3,w/2,py3)) # set its position 
        # wall3.SetMaterialSurface(self.material3) # apply material properties 
        # wall3.SetName('wall1')
        # wall3.SetBodyFixed(True)
        # wall1.AddAsset(col_p)
        # self.my_system.Add(wall3)
        # self.wall.append(wall3)        
        







        
        
        # self.wall1 = chrono.ChBody()
        # self.wall1.SetName('floor')
        # self.wall1.SetBodyFixed(True)
        # self.wall1.SetPos(chrono.ChVectorD(px1, w/2, py1 ))
        # self.wall1.SetMaterialSurface(self.material3)
        # self.wall1.GetCollisionModel().ClearModel()
        # self.wall1.GetCollisionModel().AddBox(w,w,l) # hemi sizes
        # self.wall1.GetCollisionModel().BuildModel()       
        # self.wall1.SetCollide(True)
        # body_floor_shape = chrono.ChBoxShape()
        # body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(w/2,w/2,l)
        # self.wall1.GetAssets().push_back(body_floor_shape)
        # col_g = chrono.ChColorAsset()
        # col_g.SetColor(chrono.ChColor(1,0.1,0.5))
        # self.wall1.AddAsset(col_g)
        # self.my_system.Add(self.wall1)
        
        
        # self.wall2 = chrono.ChBody()
        # self.wall2.SetName('floor')
        # self.wall2.SetBodyFixed(True)
        # self.wall2.SetPos(chrono.ChVectorD(px2, w/2, py2 ))
        # self.wall2.SetMaterialSurface(self.material3)
        # self.wall2.GetCollisionModel().ClearModel()
        # self.wall2.GetCollisionModel().AddBox(w,w,l) # hemi sizes
        # self.wall2.GetCollisionModel().BuildModel()       
        # self.wall2.SetCollide(True)
        # body_floor_shape = chrono.ChBoxShape()
        # body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(w/2,w/2,l)
        # self.wall2.GetAssets().push_back(body_floor_shape)
        # col_g = chrono.ChColorAsset()
        # col_g.SetColor(chrono.ChColor(1,0.1,0.5))
        # self.wall2.AddAsset(col_g)
        # self.my_system.Add(self.wall2)       
        
        
        # self.my_system.Add(wall1)
        # self.wall.append(wall1)
        
        
        # wall2 = chrono.ChBody()
        # wall2 = chrono.ChBodyEasyBox(w,w,l,1000,True,True) # create object
        # wall2.SetPos(chrono.ChVectorD(px2,w,py2)) # set its position 
        # wall2.SetMaterialSurface(self.material3) # apply material properties 
        # wall2.SetName('wall2')
        # wall2.SetBodyFixed(True)
        # col_g = chrono.ChColorAsset()
        # col_g.SetColor(chrono.ChColor(1,0.1,0.5))
        # wall2.AddAsset(col_g)
        # self.my_system.Add(wall2)
        # self.wall.append(wall2)
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        # body_floor_shape2 = chrono.ChBoxShape()
        # body_floor_shape2.GetBoxGeometry().Size = chrono.ChVectorD(self.floor_length, self.floor_height, self.floor_length)
        # self.body_floor2.GetAssets().push_back(body_floor_shape2)
        # col_k = chrono.ChColorAsset()
        # col_k.SetColor(chrono.ChColor(0, 0, 0))
        # self.body_floor2.AddAsset(col_k)
        # self.my_system.Add(self.body_floor2)        
        # self.body_floor = chrono.ChBody()
        #self.body_floor.SetName('floor2')
        # self.body_floor.SetBodyFixed(True)
        # self.body_floor.SetPos(chrono.ChVectorD(0, self.tall+self.height2, 0 ))
        # self.body_floor.SetMaterialSurface(self.material2)
        # self.body_floor.GetCollisionModel().ClearModel()
        # self.body_floor.GetCollisionModel().AddBox(self.length, self.tall, self.length) # hemi sizes
        # self.body_floor_shape = chrono.ChBoxShape()
        
        
        # self.body_floor2 = chrono.ChBody()
        # self.body_floor2.SetName('floor2')
        # self.body_floor2.SetBodyFixed(True)
        # self.body_floor2.SetPos(chrono.ChVectorD(0, self.floor_height+self.bot_height, 0 ))
        # self.body_floor2.SetMaterialSurface(self.material2)
        # self.body_floor2.GetCollisionModel().ClearModel()
        # self.body_floor2.GetCollisionModel().AddBox(self.floor_length, self.floor_height+self.bot_height, self.floor_length) # hemi sizes
        # self.body_floor2.SetCollide(True)
        # self.my_system.Add(self.body_floor2) 
        
        #self.body_floor2_shape = chrono.ChBoxShape()
        # self.body_floor_shape2.GetBoxGeometry().Size = chrono.ChVectorD((self.floor_length, self.floor_tall, self.floor_length))
        # self.body_floor2.GetAssets().push_back(self.body_floor_shape2)
  
        # body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(self.length, self.tall, self.length)
        # self.body_floor.GetAssets().push_back(body_floor_shape)
        # col_g = chrono.ChColorAsset()
        # col_g.SetColor(chrono.ChColor(0, 0, 0))
        # self.body_floor.AddAsset(col_g)body_floor.GetCollisionModel().BuildModel()       
  
    
    
    def Material(self,lateralFriction):
        ''' Function that creates material object '''
        material = chrono.ChMaterialSurfaceNSC() # create material object
        material.SetFriction(lateralFriction) # set friction properties
        material.SetDampingF(self.dampingterm) # set damping properties
        material.SetCompliance(self.Compliance) # set compliance property
        material.SetComplianceT(self.Compliance_tangent) # set tangential property
        material.SetRollingFriction(self.rollingFriction)
        material.SetSpinningFriction(self.spinningFriction)
        # material.SetComplianceRolling(Cr)
        # material.SetComplianceSpinning(Cs)
        return (material)    
    
    
    def return_enviroment(self):
        return(self.my_system)
    
    
    
class simulate:
    def __init__(self,name,my_system,bots,particles,controller,my_rep,path,ball=None):
       
        self.name=name
        self.my_system = my_system
        self.Bots = bots
        self.particles = particles
        self.controller = controller
        self.my_rep = my_rep
        self.ball=ball
        ###### Imported Variables #########
        self.mainDirectory = path
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters=parameters.tolist()
        self.convert_dist=self.parameters['convert_dist']
        self.convert_mass=self.parameters['convert_mass']
        self.control_mode=self.parameters['control_mode']
        self.xcenter = self.parameters['xcenter']
        self.zcenter = self.parameters['zcenter']
        self.visual = self.parameters['visual']
        self.dt = self.parameters['dt']
        self.time_end = self.parameters['time_end']
        self.save_rate = self.parameters['save_rate']
        if self.control_mode=="shape_morphing":
            self.tcut = self.parameters['tcut']
            
        ###### Predefined variables ######
        self.myapplication=[]
        self.epoch = 0
        self.camx = self.xcenter
        self.camy = 3
        self.camz = self.zcenter
        self.camy_height =5
        self.save_video = False
        self.Trip=False
        self.sim_start=timeit.default_timer()
        self.change=0
        ###### Empty Arrays ######
        self.time = [] # time empty array
        self.time_contact = [] # contact time  empty array
        self.number_contacts = []
        self.Contact_points_x = []
        self.Contact_points_y = []
        self.Contact_points_z = []
        self.Contact_force_x = []
        self.Contact_force_y = []
        self.Contact_force_z = []
        self.bodiesA = []
        self.bodiesB = []
        self.bodiesA_ID = []
        self.bodiesB_ID = []
        
        
        
        
        
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
            #self.myapplication.SetContactsDrawMode(2)
            self.myapplication.SetPaused(self.Trip)
            self.myapplication.AddTypicalLights()
            self.myapplication.DrawAll               
            self.myapplication.AssetBindAll()
            self.myapplication.AssetUpdateAll()
            self.myapplication.AddShadowAll()
            self.count=0
            self.myapplication.SetTimestep(self.dt)
            self.myapplication.SetTryRealtime(False)
            ##### Run the sim
            while(self.myapplication.GetDevice().run()):
                #self.my_rep.ResetList()
                self.my_rep.ResetList()
                self.myapplication.BeginScene()
                self.myapplication.DrawAll()
                self.myapplication.DoStep()

                #if self.epoch%self.save_rate==0:      
                self.controller.run_controller()
                self.controller.get_position()

                time=np.round(self.my_system.GetChTime(),3)
                
                if self.control_mode=="shape_morphing":
                    t=np.round(self.my_system.GetChTime(),3)
                    
                    if t<=self.tcut[0]:
                        ft=np.round(self.controller.Psi.tanh(t),3)                    
                    
                    if self.tcut[0]<=t and t<=self.tcut[1]:
                        ft=np.round(self.controller.Psi.tanh(t-self.tcut[0]),3)
                        
                    if self.tcut[1]<=t:
                        ft=np.round(self.controller.Psi.tanh(t-self.tcut[1]),3)
                                    
                    print('time='+str(time), 'f(t)='+str(ft),end='\n')
                else:
                    print('time='+str(time))
                
                self.myapplication.EndScene()
                self.save_parameters()
                self.epoch = self.epoch + 1

                
                aaa=len(self.Bots.bots)
                cam_x=0.33*(self.Bots.bots[0].GetPos().x + self.Bots.bots[int(aaa/3)].GetPos().x + self.Bots.bots[int(2*aaa/3)].GetPos().x)
                cam_y=0.33*(self.Bots.bots[0].GetPos().y + self.Bots.bots[int(aaa/3)].GetPos().y + self.Bots.bots[int(2*aaa/3)].GetPos().y)
                cam_z=0.33*(self.Bots.bots[0].GetPos().z + self.Bots.bots[int(aaa/3)].GetPos().z + self.Bots.bots[int(2*aaa/3)].GetPos().z)
                self.myapplication.GetSceneManager().getActiveCamera().setPosition(chronoirr.vector3df(cam_x,cam_y+self.camy_height,cam_z))
                self.myapplication.GetSceneManager().getActiveCamera().setTarget(chronoirr.vector3df(cam_x,cam_y,cam_z))
                self.myapplication.SetVideoframeSave(self.save_video)
                self.myapplication.SetVideoframeSaveInterval(round(1/(self.dt*60)))
                # Close the simulation if time ends
                if self.my_system.GetChTime()> self.time_end :
                    self.myapplication.GetDevice().closeDevice()
            self.sim_end=timeit.default_timer()
            
            self.parameters['sime_time']=(self.sim_end-self.sim_start)/60
            self.number_parameters = self.parameters['number_parameters']
            
            
        #### pov   
        if self.visual=="pov": 
            while (self.my_system.GetChTime() < self.time_end): 
                self.my_rep.ResetList()
                self.my_system.DoStepDynamics(self.dt)
                #if self.epoch%self.save_rate==0:      
                self.controller.run_controller()
                self.controller.get_position()

                time=np.round(self.my_system.GetChTime(),3)
                
                if self.control_mode=="shape_morphing":
                    t=np.round(self.my_system.GetChTime(),3)
                    
                    if t<=self.tcut[0]:
                        ft=np.round(self.controller.Psi.tanh(t),3)                    
                    
                    if self.tcut[0]<=t and t<=self.tcut[1]:
                        ft=np.round(self.controller.Psi.tanh(t-self.tcut[0]),3)
                        
                    if self.tcut[1]<=t:
                        ft=np.round(self.controller.Psi.tanh(t-self.tcut[1]),3)
                                    
                    print('time='+str(time), 'f(t)='+str(ft),end='\n')
                else:
                    print('time='+str(time))
                
                self.save_parameters()
                self.epoch = self.epoch + 1
            self.sim_end=timeit.default_timer()     
            self.parameters['sime_time']=(self.sim_end-self.sim_start)/60
            self.number_parameters = self.parameters['number_parameters']

        #print(len(self.parameters),self.number_parameters+4)
        if len(self.parameters)==self.number_parameters+1:
        #print('save')
            np.save(self.mainDirectory+self.name+'/Parameters.npy',self.parameters)
        
        else:
            print('not save')
        
        
        with open(self.mainDirectory+self.name+"/Parameters.csv", 'w') as f:
            for key in self.parameters.keys():
                f.write("%s, %s\n" % (key, self.parameters[key]))

        print("Run time:",(self.sim_end-self.sim_start)/60) 
    def save_parameters(self):
        ''' Function that collects data of for the system '''
        if self.epoch%self.save_rate==0:
            print("saved into")
            self.time.append(np.round(self.my_system.GetChTime(),4))
            self.Bots.save_data_position()
            self.Bots.save_data_Forces()
            self.Bots.save_data_velocity()
            self.particles.save_data_position()
            self.particles.save_data_velocity()
            self.controller.save_field_value()
            self.my_system.GetContactContainer().ReportAllContacts(self.my_rep)
            crt_list = self.my_rep.GetList()
            self.number_contacts.append(self.my_system.GetContactContainer().GetNcontacts())
            if self.my_system.GetContactContainer().GetNcontacts()!=0:
                        self.time_contact.append(self.my_system.GetChTime())
                        self.Contact_points_x.append(crt_list[0])
                        self.Contact_points_y.append(crt_list[1])
                        self.Contact_points_z.append(crt_list[2])
                        self.Contact_force_x.append(crt_list[3])
                        self.Contact_force_y.append(crt_list[4])
                        self.Contact_force_z.append(crt_list[5])
                        self.bodiesA.append(crt_list[6])
                        self.bodiesB.append(crt_list[7])
                        self.bodiesA_ID.append(crt_list[8])
                        self.bodiesB_ID.append(crt_list[9])  
            if self.control_mode=="grasping":
                self.ball.save_data_position()
                self.ball.save_data_velocity()   
class Ball:
    def __init__(self,name,my_system,body_floor,path):
        self.name=name
        self.my_system = my_system
        self.body_floor = body_floor
        ###### Imported Variables #########
        self.path=path
        self.mainDirectory = self.path
        
        copyfile(__file__,self.mainDirectory+self.name+"/"+'objects.py')
        parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters=parameters.tolist()
        self.convert_dist=self.parameters['convert_dist']
        self.convert_mass=self.parameters['convert_mass']
        self.height=self.convert_dist*self.parameters['bot_height']
        self.y = self.convert_dist*self.parameters['bot_height']/2
        self.x =self.parameters['ballx']
        self.z =self.parameters['ballz']
        
        self.lateralFriction = self.parameters['lateralFriction']
        self.spinningFriction = self.parameters['spinningFriction']
        self.rollingFriction = self.parameters['rollingFriction']
        self.dampingterm = self.parameters['dampingterm']
        self.Compliance_tangent = self.parameters['Ctr'] 
        self.Compliance = self.parameters['Cr'] 
        self.material=self.Material(.1)
        self.geom = self.parameters['ball_geometry'] 
        self.radius = self.parameters['ball_radius']
        self.mb=3
        volume3=np.pi*self.height*(self.radius)**2   # calculate volume
        self.rho=self.mb/volume3 # density
        self.balls=[]
        self.obj=[]
        self.forceb=[]
        self.bx={}
        self.bz={}
        self.bvx={}
        self.bvz={}
        
        self.Fb={}
        self.PX={}
        self.PY={}
        self.TIME={}
        self.bx["ballx"]=[]
        self.bz["ballz"]=[]        
        self.bvx["ballvx"]=[]        
        self.bvz["ballvz"]=[]
        
        self.Fb["Fb"]=[]
        self.PX["PX"]=[]
        self.PY["PY"]=[]
        self.TIME["TIME"]=[]
        
        z2x = chrono.ChQuaternionD()
        z2x.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 1, 0))
        z2y = chrono.ChQuaternionD()
        z2y.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 0, 1))
        
        if self.geom=='circle':
            #Create ball
            ball = chrono.ChBody() # create ball object
            ball = chrono.ChBodyEasyCylinder(self.radius, self.height,self.rho,True,True) # specify properties and make it a cylinder
            # set position
            ball.SetPos(chrono.ChVectorD(self.x,self.y,self.z))
            ball.SetName("ball") # give it a name
            ball.SetId(10000) 
            ball.SetCollide(True) # set the collision mode
            ball.SetBodyFixed(False) # set if its fixed
            body_floor_texture = chrono.ChTexture()
            body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'bluwhite.png')
            ball.GetAssets().push_back(body_floor_texture)
            # pt=chrono.ChLinkMatePlane()
            # pt.Initialize(self.body_floor,ball,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
            # self.my_system.AddLink(pt) 
            #pt=chrono.ChLinkMatePlane() 
            #pt.Initialize(self.body_floor,ball,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
            #append the constraint and the ball to array of objects and system
            prismatic_ground_ball = chrono.ChLinkLockPrismatic()
            prismatic_ground_ball.SetName("prismatic_ground_ball")
            prismatic_ground_ball.Initialize(self.body_floor, ball, chrono.ChCoordsysD(chrono.ChVectorD(self.x, self.y, self.z), chrono.Q_ROTATE_X_TO_Z))
            self.my_system.Add(prismatic_ground_ball) 
            #self.my_system.AddLink(pt)           
        if self.geom=="square":
            const=self.radius*2*np.pi/4
            ball = chrono.ChBody() # create ball object
            ball = chrono.ChBodyEasyBox(const,self.height,const,self.rho,True,True) # create object # specify properties and make it a cylinder
            ball.SetPos(chrono.ChVectorD(self.x,self.y,self.z))
            ball.SetName("ball") # give it a name
            ball.SetId(10000) 
            ball.SetCollide(True) # set the collision mode
            ball.SetBodyFixed(False) # set if its fixed
            body_floor_texture = chrono.ChTexture()
            body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'bluwhite.png')
            ball.GetAssets().push_back(body_floor_texture)
            prismatic_ground_ball = chrono.ChLinkLockPrismatic()
            prismatic_ground_ball.SetName("prismatic_ground_ball")
            prismatic_ground_ball.Initialize(self.body_floor, ball, chrono.ChCoordsysD(chrono.ChVectorD(self.x, self.y, self.z), chrono.Q_ROTATE_X_TO_Z))
            self.my_system.Add(prismatic_ground_ball) 
            
        if self.geom=="triangle":     
            const=self.radius*2*np.pi/3
            r=const*np.sqrt(3)/3
            
            
            #r = .04/.5236
            x1=0
            y1=r
            x2=r*np.cos(7*np.pi/6)
            y2=r*np.sin(7*np.pi/6)
            x3=r*np.cos(11*np.pi/6)
            y3=r*np.sin(11*np.pi/6)
            pt_vect = chrono.vector_ChVectorD()
            const=.4627
            # creates bottom
            pt_vect.push_back(chrono.ChVectorD(y1,self.height/2,x1))
            pt_vect.push_back(chrono.ChVectorD(y2,self.height/2,x2))
            pt_vect.push_back(chrono.ChVectorD(y3,self.height/2,x3))
            
            pt_vect.push_back(chrono.ChVectorD(y1,-self.height/2,x1))
            pt_vect.push_back(chrono.ChVectorD(y2,-self.height/2,x2))
            pt_vect.push_back(chrono.ChVectorD(y3,-self.height/2,x3))            

            ball=chrono.ChBodyEasyConvexHull(pt_vect,self.rho,True,True)   
            ball.SetPos(chrono.ChVectorD(self.x,self.y,self.z))
            ball.SetName("ball") # give it a name
            ball.SetCollide(True) # set the collision mode
            ball.SetBodyFixed(True) # set if its fixed
            body_floor_texture = chrono.ChTexture()
            body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'bluwhite.png')
            ball.GetAssets().push_back(body_floor_texture)
            prismatic_ground_ball = chrono.ChLinkLockPrismatic()
            prismatic_ground_ball.SetName("prismatic_ground_ball")
            prismatic_ground_ball.Initialize(self.body_floor, ball, chrono.ChCoordsysD(chrono.ChVectorD(self.x, self.y, self.z), chrono.Q_ROTATE_X_TO_Z))
            self.my_system.Add(prismatic_ground_ball) 
            
            
            ball.GetCollisionModel().ClearModel()
            ball.GetCollisionModel().AddConvexHull(pt_vect)
            ball.GetCollisionModel().BuildModel()            
            
            
            # set position
        myforcez = chrono.ChForce() # create it 
        ball.AddForce(myforcez) # apply it to bot object
        myforcez.SetMode(chrono.ChForce.FORCE) # set the mode
        myforcez.SetDir(chrono.VECT_Z) # set direction 
        myforcez.SetVpoint(chrono.ChVectorD(0,0.05,0))
        self.forceb.append(myforcez) # add to force list

        #create constraint to fix it to the floor
        # pt=chrono.ChLinkMatePlane() 
        # pt.Initialize(self.body_floor,ball,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
        # prismatic_ground_ball = chrono.ChLinkLockPrismatic()
        # prismatic_ground_ball.SetName("prismatic_ground_ball")
        # prismatic_ground_ball.Initialize(self.body_floor, ball, chrono.ChCoordsysD(chrono.ChVectorD(self.x, self.y, self.z), chrono.Q_ROTATE_Z_TO_Y))
        # self.my_system.AddLink(prismatic_ground_ball)
        
        #self.my_system.Add(prismatic_ground_ball) 
                
        
        # material
        ball.SetMaterialSurface(self.material) # apply material
        self.obj.append(ball)
        self.balls.append(ball) 
        self.my_system.Add(ball)            
        
            
            #col_y = chrono.ChColorAsset() # apply color
            #col_y.SetColor(chrono.ChColor(1, 1, 0))
            #ball.AddAsset(col_y)            

    def Material(self,lateralFriction):
        ''' Function that creates material object '''
        material = chrono.ChMaterialSurfaceNSC() # create material object
        material.SetFriction(lateralFriction) # set friction properties
        material.SetDampingF(self.dampingterm) # set damping properties
        material.SetCompliance(self.Compliance) # set compliance property
        material.SetComplianceT(self.Compliance_tangent) # set tangential property
        material.SetRollingFriction(self.rollingFriction)
        material.SetSpinningFriction(self.spinningFriction)
        # material.SetComplianceRolling(Cr)
        # material.SetComplianceSpinning(Cs)
        return (material)            
        
            
        
    def save_data_position(self):
        ''' Function that saves the positon of the ball '''
        self.bx["ballx"].append(self.balls[0].GetPos().x) # x postion 
        self.bz["ballz"].append(self.balls[0].GetPos().z) # z postion 
    

    def save_data_velocity(self):
        ''' save the velocity of the ball '''
        self.bvx["ballvx"].append(self.balls[0].GetPos_dt().x) # x velocity
        self.bvz["ballvz"].append(self.balls[0].GetPos_dt().z)# z velocity 
        
    def save_contact_force(self):
        ''' save contact forces  '''
        self.bFx.append(self.balls[0].GetContactForce().x) # x contact force
        self.bFy.append(self.balls[0].GetContactForce().y) # y contact force
        self.bFz.append(self.balls[0].GetContactForce().z) # z contact force
     
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
    
    
    
    
                   
class controller():
    """ Class for storung all the controller information"""
    def __init__(self,name,my_system,bots,Psi,path,ball=None,interior=None):
        self.name = name # name of simulation
        self.my_system = my_system # the system object
        self.robots = bots # robot objets
        self.Psi = Psi # potential fields 
        self.ball = ball
        self.interior = interior

        ##### Extract variables from other imported objects #####
        self.forces = self.robots.force
        self.nb = self.robots.nb 
        self.forceb=self.ball.forceb
        ###### Imported Variables #########
        self.mainDirectory = path   # main directory 
        parameters = np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters = parameters.tolist()                 
        self.control_mode = self.parameters['control_mode']  # control mode 
        
        self.fxt = []
        self.fyt = []
        self.fzt = []
        
        self.T = 0
        self.tn = 0
        self.w = 5       
        
        self.pwm = self.parameters["pwm"]
        self.height = self.parameters['particle_height']        
        self.dt = self.parameters['dt']
        self.change = 0

        
        self.bot_position_x = 0
        self.bot_position_z = 0
        
        self.bot_velocitiy_x = 0
        self.bot_velocitiy_z = 0
        
        self.Field_value = {}
        
        for i in range(self.nb):
            self.Field_value["bot{0}".format(i)] = []            
        
        self.tarx=0
        self.tarz=0
        self.tarx2=0
        self.tarz2=0
        self.fb=0
        
        
        
        
        if self.control_mode=="grasping":
            
            self.alpha1 = self.parameters['alpha1']
            self.alpha2 = self.parameters['alpha2']
            
            self.beta1 = self.parameters['beta1']
            self.beta2 = self.parameters['beta2']
            
            self.tcut1 = self.parameters['tcut1']
            self.tcut2 =self.parameters['tcut2']
            self.xc1 = self.parameters["xc1"]
            self.zc1 = self.parameters["yc1"]
            self.xc2 = self.parameters["xc2"]
            self.zc2 = self.parameters["yc2"]            
            
        elif self.control_mode=="target_chasing":
            
            self.xc = self.parameters["xc"]
            self.zc = self.parameters["zc"]
            self.alpha = self.parameters['alpha']
            self.beta = self.parameters['beta']
            
        else:    
            self.alpha = self.parameters['alpha']
            self.beta = self.parameters['beta']
        

    def run_controller(self):
        """Function to run the controllers"""
        if self.control_mode =="shape_formation":
            
                (self.bot_position_x,self.bot_position_z) = self.get_position()     # get position of bots
                (self.bot_velocitiy_x,self.bot_velocitiy_z) = self.get_velocity()   # get velocity of bots
                (self.FX,self.FZ) = self.shape_controller() # run shape controller
                self.apply_force(self.FX,self.FZ)         # apply force
                
        if self.control_mode =="shape_morphing":
                (self.bot_position_x,self.bot_position_z) = self.get_position()     # get position of bots
                (self.bot_velocitiy_x,self.bot_velocitiy_z) = self.get_velocity()   # get velocity of bots
                (self.FX,self.FZ) = self.morph_controller() # run shape controller
                self.apply_force(self.FX,self.FZ)         # apply force

        if self.control_mode =="grasping":
                (self.bot_position_x,self.bot_position_z) = self.get_position()     # get position of bots
                (self.bot_velocitiy_x,self.bot_velocitiy_z) = self.get_velocity()   # get velocity of bots
                (self.FX,self.FZ) = self.grasping_controller() # run shape controller
                self.apply_force(self.FX,self.FZ)         # apply force    
                
        if self.control_mode =="target_chasing":
                (self.bot_position_x,self.bot_position_z) = self.get_position()     # get position of bots
                (self.bot_velocitiy_x,self.bot_velocitiy_z) = self.get_velocity()   # get velocity of bots
                (self.FX,self.FZ) = self.target_controller() # run shape controller
                self.apply_force(self.FX,self.FZ)         # apply force  
                
            
            
            
    def save_field_value(self):
        """ Function to save the potential field values """
        if self.control_mode == "shape_formation":  
            
            for i in range(self.nb):
               self.Field_value["bot"+str(i)].append(self.Psi.F_(100*self.bot_position_x[i],100*self.bot_position_z[i]))
        
        if self.control_mode == "grasping":
            time=np.round(self.my_system.GetChTime(),3)
            if time<self.Psi.tcut1:
                for i in range(self.nb):
                    self.Field_value["bot"+str(i)].append(self.Psi.F_(self.bot_position_x[i],self.bot_position_z[i],xc=self.Psi.xc1,yc=self.Psi.zc1,a=self.Psi.a1,b=self.Psi.b1))            
            else:            
                for i in range(self.nb):
                    self.Field_value["bot"+str(i)].append(self.Psi.F_(self.bot_position_x[i],self.bot_position_z[i],xc=self.Psi.xc2,yc=self.Psi.zc2,a=self.Psi.a2,b=self.Psi.b2))            


        if self.control_mode == "target":
            for i in range(self.nb):
               self.Field_value["bot"+str(i)].append(self.Psi.F_(self.bot_position_x[i],self.bot_position_z[i]))                   
            
    def shape_controller(self):
        """ Shape Controller """
        FX=[]
        FZ=[]
        for i in range(self.nb):
            Fx=self.Psi.FX_(100*self.bot_position_x[i],100*self.bot_position_z[i])
            Fz=self.Psi.FY_(100*self.bot_position_x[i],100*self.bot_position_z[i])
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

    def target_controller(self):
        """ target Controller """
        FX=[]
        FZ=[]
        for i in range(self.nb):
            Fx=self.Psi.FX_(100*self.bot_position_x[i],100*self.bot_position_z[i],100*self.xc,100*self.zc)
            Fz=self.Psi.FY_(100*self.bot_position_x[i],100*self.bot_position_z[i],100*self.xc,100*self.zc)
            mag=np.sqrt(Fx**2 + Fz**2)
            if mag==0:
                FXX=0
                FZZ=0
            else:
                FXX=Fx/mag
                FZZ=Fz/mag
            fx=-self.alpha*FXX-self.beta*self.bot_velocitiy_x[i]
            fz=-self.alpha*FZZ-self.beta*self.bot_velocitiy_z[i]
            
            ang2 = 20*np.sin(2*np.pi*np.round(self.my_system.GetChTime(),3))
            theta = np.nan_to_num(np.arctan2(FZZ, FXX)) % 2*np.pi
            theta=theta+ang2
            
            
            
            fz=-self.alpha*np.sin(theta)
            fx=-self.alpha*np.cos(theta)            
            FX.append(fx)
            FZ.append(fz)
        return(np.asarray(FX),np.asarray(FZ)) 


    def morph_controller(self):
        """ morph Controller """
        FX=[]
        FZ=[]
        for i in range(self.nb):
            Fx=self.Psi.FX_(100*self.bot_position_x[i],100*self.bot_position_z[i],xc=0,yc=0,a=0,b=0,t=np.round(self.my_system.GetChTime(),3))
            Fz=self.Psi.FY_(100*self.bot_position_x[i],100*self.bot_position_z[i],xc=0,yc=0,a=0,b=0,t=np.round(self.my_system.GetChTime(),3))            
            #Fx=self.Psi.Fmorphx(100*self.bot_position_x[i],100*self.bot_position_z[i])
            #Fz=self.Psi.Fmorphy(100*self.bot_position_x[i],100*self.bot_position_z[i])
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
        self.ball.balls[0].SetBodyFixed(False)
        if time<=self.Psi.tcut1:
            #self.tarx = self.ball.balls[0].GetPos().x
            #self.tarz = self.ball.balls[0].GetPos().z
            self.ball.balls[0].SetBodyFixed(True)
            self.tarx = self.xc1
            self.tarz = self.zc1           
            
        if time>=self.Psi.tcut1 and time<self.Psi.tcut2:
            self.ball.balls[0].SetBodyFixed(True)
            self.tarx = self.ball.balls[0].GetPos().x
            self.tarz = self.ball.balls[0].GetPos().z
            
            
        # if time>=self.Psi.tcut1 and time<self.Psi.tcut2 :
        #     self.tarx = self.ball.balls[0].GetPos().x
        #     self.tarz = self.ball.balls[0].GetPos().z
        #     self.ball.balls[0].SetBodyFixed(False)
           
        if time>self.Psi.tcut3:
            #self.alpha=80
            self.tarx = self.tarx
            self.tarz = self.tarz
            self.fb=self.fb+.001
            self.forceb[0].SetMforce(self.fb)
            self.forceb[0].SetDir(chrono.VECT_X)
            self.ball.balls[0].SetBodyFixed(False)            
            self.ball.Fb["Fb"].append(self.fb)
            self.ball.TIME["TIME"].append(time)
            self.ball.PX["PX"].append(self.ball.balls[0].GetPos().x)
            self.ball.PY["PY"].append(self.ball.balls[0].GetPos().z)           
            
        for i in range(self.nb):
            Fx1=self.Psi.FX_(100*self.bot_position_x[i],100*self.bot_position_z[i],xc=100*(self.tarx),yc=100*self.tarz,a=100*self.Psi.a1,b=100*self.Psi.b1)
            Fz1=self.Psi.FY_(100*self.bot_position_x[i],100*self.bot_position_z[i],xc=100*(self.tarx),yc=100*self.tarz,a=100*self.Psi.a1,b=100*self.Psi.b1)
            mag1=np.sqrt(Fx1**2 + Fz1**2)
            Fx1=Fx1/mag1
            Fz1=Fz1/mag1
            
            
            Fx2=self.Psi.FX_(100*self.bot_position_x[i],100*self.bot_position_z[i],xc=100*self.tarx,yc=100*self.tarz,a=100*self.Psi.a2,b=100*self.Psi.b2)
            Fz2=self.Psi.FY_(100*self.bot_position_x[i],100*self.bot_position_z[i],xc=100*self.tarx,yc=100*self.tarz,a=100*self.Psi.a2,b=100*self.Psi.b2)
            mag2=np.sqrt(Fx2**2 + Fz2**2)
            Fx2=Fx2/mag2
            Fz2=Fz2/mag2
            
            
            if time<=self.Psi.tcut1:
                Fx=Fx1
                Fz=Fz1
                mag=np.sqrt(Fx**2 + Fz**2)
                alpha_=self.alpha1
            else: 
            #elif time>=self.Psi.tcut2:
                Fx=Fx2
                Fz=Fz2
                mag=np.sqrt(Fx**2 + Fz**2)
                alpha_=self.alpha2
            if mag==0:
                FXX=0
                FZZ=0
            else:
                FXX=Fx/mag
                FZZ=Fz/mag
                
            fx=-alpha_*FXX-self.beta1*self.bot_velocitiy_x[i]
            fz=-alpha_*FZZ-self.beta1*self.bot_velocitiy_z[i]
            FX.append(fx)
            FZ.append(fz)
        return(np.asarray(FX),np.asarray(FZ)) 


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



    def apply_force(self,FX,FZ):  
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









    # def apply_force(self,FX,FZ):  
    #     """ Appy forces to robots """

    #     for i in range(self.nb):
    #         self.fxt.append(FX[i])
    #         self.fyt.append(0)
    #         self.fzt.append(self.FZ[i])
    #         self.forces[3*i].SetMforce(float(FX[i]))
    #         self.forces[3*i].SetDir(chrono.VECT_X)
    #         self.forces[3*i+2].SetMforce(float(FZ[i]))
    #         self.forces[3*i+2].SetDir(chrono.VECT_Z)
     

        
     
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
        self.ball=ball
        self.results_dir=self.mainDirectory+self.name+'/results'
        if not os.path.exists(self.results_dir):
            os.mkdir(self.results_dir)
        
        # time
        self.time = {'time':self.simulation.time} 
        
        # robot data
        (self.bot_xposition,self.bot_yposition,self.bot_zposition) = self.robots.return_position_data()
        (self.skin_xposition,self.skin_yposition,self.skin_zposition) = self.robots.return_position_membrane_data()
        (self.bot_xForcetotal,self.bot_yForcetotal,self.bot_zForcetotal) = self.robots.return_force_data()
        (self.bot_xvelocity,self.bot_yvelocity,self.bot_zvelocity) = self.robots.return_velocity_data()
        (self.bot_xForcecontact,self.bot_yForcecontact,self.bot_zForcecontact) = self.robots.return_force_data_contact()            

        # interior data
        (self.particle_xposition,self.particle_yposition,self.particle_zposition) = self.interior.return_position_data()
        (self.particle_xvelocity,self.particle_yvelocity,self.particle_zvelocity) = self.interior.return_velocity_data()
        
        
        # ball data        
        (self.ball_xposition,self.ball_zposition)=self.ball.return_position_data()
        (self.ball_xvelocity,self.ball_zvelocity)=self.ball.return_velocity_data()
        self.Fb=self.ball.Fb
        self.TIME=self.ball.TIME
        self.PX=self.ball.PX
        self.PY=self.ball.PY
        self.Field_value=self.controls.Field_value

    def export_data(self):  
        '''Export Bot positions '''
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
        
        file_name=self.results_dir+'/field_values.csv'
        with open(file_name, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
                
            # write x position to csv file
            for key, val in self.Field_value.items():
                w.writerow([key, *val])            
                
                
                
        '''Export membrane positions '''
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

                
             
        ''' Export Bot velocities '''        
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
        
        
        '''Export particle positions '''       
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
                
                    
        '''Export ball positions '''       
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
                
        '''Export ball velocity '''       
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
                
        '''Pull Force '''       
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
                
                
                
                
                
class MyReportContactCallback(chrono.ReportContactCallback):
    """ Class for reporting and storing the the contact forces and postions  """
    def __init__(self):

        chrono.ReportContactCallback.__init__(self)
        self.Contact_force_x=[]
        self.Contact_force_y=[]
        self.Contact_force_z=[]
        
        self.Contact_points_x= []
        self.Contact_points_y = []
        self.Contact_points_z = []
        
        self.bodiesA = []
        self.bodiesB = []
        
        self.bodiesA_ID=[]
        self.bodiesB_ID=[]

    def OnReportContact(self,vA,vB,cA,dist,rad,force,torque,modA,modB):
        bodyUpA = chrono.CastContactableToChBody(modA)
        nameA = bodyUpA.GetName()
        bodyUpB = chrono.CastContactableToChBody(modB)
        nameB = bodyUpB.GetName()
        
        IDA = bodyUpA.GetId()
        IDB = bodyUpB.GetId()
       
        self.Contact_points_x.append(vA.x)
        self.Contact_points_y.append(vA.y)
        self.Contact_points_z.append(vA.z)
        
        self.Contact_force_x.append(force.x)
        self.Contact_force_y.append(force.y)
        self.Contact_force_z.append(force.z)
        
        self.bodiesA.append(nameA)
        self.bodiesB.append(nameB)
        
        self.bodiesA_ID.append(IDA)
        self.bodiesB_ID.append(IDB)
        

        return True        # return False to stop reporting contacts

    # reset after every run 
    def ResetList(self):
        self.Contact_force_x=[]
        self.Contact_force_y=[]
        self.Contact_force_z=[]
        
        self.Contact_points_x= []
        self.Contact_points_y = []
        self.Contact_points_z = []
        
        self.bodiesA = []
        self.bodiesB = []
        
        self.bodiesA_ID=[]
        self.bodiesB_ID=[]

    # Get the points
    def GetList(self):
        return (self.Contact_points_x,
                self.Contact_points_y,
                self.Contact_points_z,
                self.Contact_force_x,
                self.Contact_force_y,
                self.Contact_force_z,
                self.bodiesA,
                self.bodiesB,
                self.bodiesA_ID,
                self.bodiesB_ID)


class Potential_fields():
    def __init__(self,name):
        self.direct = os.path.dirname(__file__)
        self.name = name
        ###### Imported Variables #########
        self.mainDirectory = self.direct+"/Experiments/"
        parameters = np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
        self.parameters = parameters.tolist()
        self.control_mode = self.parameters['control_mode']  # control mode
        
        #### Target
        if self.control_mode=="target_chasing":
            self.xc = self.parameters["xc"]
            self.yc = self.parameters["zc"]
                      
        #### Grasping
        if self.control_mode=="grasping":
            self.a1 = self.parameters['a1']
            self.b1 = self.parameters['b1']
            self.a2 = self.parameters['a1']
            self.b2 = self.parameters['b2']
            self.xc1 = self.parameters["xc1"]
            self.zc1 = self.parameters["yc1"]
            self.xc2 = self.parameters["xc2"]
            self.zc2 = self.parameters["yc2"]  
            self.tcut1 = self.parameters["tcut1"]
            self.tcut2 = self.parameters["tcut2"]
            self.tcut3 = self.parameters["tcut3"]            
            self.p = self.parameters["p"] 
            
        #### Shape formation 
        if self.control_mode=="shape_formation":
            self.geometry = self.parameters["geometry"]
            self.d = self.parameters['d']
            self.xc = self.parameters["xcenter"]
            self.yc = self.parameters["zcenter"]
            self.delta = self.parameters["delta"]
            #self.p = self.parameters["p"]
            
            self.xmin = -self.d + self.xc
            self.xmax = self.d + self.xc
            self.ymin = -self.d + self.yc
            self.ymax = self.d + self.yc
            self.xx = np.arange(np.round(self.xmin,3),np.round(self.xmax,3), self.delta)
            self.yy = np.arange(np.round(self.ymin,3),np.round(self.ymax,3), self.delta)
            self.X,self.Y = np.meshgrid(self.xx, self.yy)               
            
            if self.geometry=="oval":
                self.a = self.parameters['a']
                self.b = self.parameters['b']
                self.gap = self.parameters['gap']
                self.epsilon = self.parameters['epsilon']
                self.npoints = self.parameters['npoints']
                self.step = self.parameters['step']
                self.num_iter = int(self.npoints/self.step)
                (self.xt,self.yt,self.zt,self.xs,self.ys,self.zs) = self.points_oval_region(self.a + self.gap/2,self.b+self.gap/2,self.xc,self.yc,self.npoints,self.step,self.num_iter,self.epsilon,"out")
                (self.xt2,self.yt2,self.zt2,self.xs2,self.ys2,self.zs2) = self.points_oval_region(self.a - self.gap/2,self.b-self.gap/2,self.xc,self.yc,self.npoints,self.step,self.num_iter,self.epsilon,"in")  
                theta=np.linspace(0,2*np.pi,self.npoints)
                xt_desired = self.a*np.cos(theta)
                yt_desired = self.b*np.sin(theta)
                np.savez(self.mainDirectory+'/'+self.name+'/'+'shapes_'+self.geometry+'.npz',gap=self.gap,xt=self.xt,yt=self.yt,xt2=self.xt2,yt2=self.yt2,xt_desired=xt_desired,yt_desired=yt_desired)          

            if self.geometry=="square":
                self.Rshape = self.parameters['Rshape']
                self.gap = self.parameters['gap']
                self.epsilon = self.parameters['epsilon']
                self.npoints = self.parameters['npoints']
                self.step = self.parameters['step']
                self.num_iter = int(self.npoints/self.step)
                (self.xt,self.yt,self.zt,self.xs,self.ys,self.zs) = self.points_square_region(self.Rshape + self.gap/2,self.xc,self.yc,self.npoints,self.step,self.num_iter,self.epsilon,"out")
                (self.xt2,self.yt2,self.zt2,self.xs2,self.ys2,self.zs2) = self.points_square_region(self.Rshape - self.gap/2,self.xc,self.yc,self.npoints,self.step,self.num_iter,self.epsilon,"in")    
                
                xt1 = np.ones(int(self.npoints/4))
                xt2 = np.linspace(1.01,-1,int(self.npoints/4),endpoint=False)
                xt3 = -1*np.ones(int(self.npoints/4))
                xt4 = np.linspace(-1.01,1,int(self.npoints/4),endpoint=False)
        
                yt1 = np.linspace(-1.01,1,int(self.npoints/4),endpoint=False)
                yt2 = np.ones(int(self.npoints/4))
                yt3 = np.linspace(1.01,-1,int(self.npoints/4),endpoint=False)
                yt4 = -1*np.ones(int(self.npoints/4))
                
                xt=np.hstack([xt1,xt2,xt3,xt4])
                yt=np.hstack([yt1,yt2,yt3,yt4])
                xt_desired = np.dot(self.Rshape,xt)
                yt_desired = np.dot(self.Rshape,yt)                
                np.savez(self.mainDirectory+'/'+self.name+'/'+'shapes_'+self.geometry+'.npz',gap=self.gap,xt=self.xt,yt=self.yt,xt2=self.xt2,yt2=self.yt2,xt_desired=xt_desired,yt_desired=yt_desired)          
               
            if self.geometry=="triangle":
                self.Rshape = self.parameters['Rshape']
                self.gap = self.parameters['gap']
                self.epsilon = self.parameters['epsilon']
                self.npoints = self.parameters['npoints']
                self.step = self.parameters['step']
                self.num_iter = int(self.npoints/self.step)                
                (self.xt,self.yt,self.zt,self.xs,self.ys,self.zs) = self.points_triangle_region(self.Rshape + self.gap/2,self.xc,self.yc,self.npoints,self.step,self.num_iter,self.epsilon,"out")
                (self.xt2,self.yt2,self.zt2,self.xs2,self.ys2,self.zs2) = self.points_triangle_region(self.Rshape - self.gap/2,self.xc,self.yc,self.npoints,self.step,self.num_iter,self.epsilon,"in")      
            
                r = self.Rshape/.5236
                x1=0
                y1=r
                x2=r*np.cos(7*np.pi/6)
                y2=r*np.sin(7*np.pi/6)
                x3=r*np.cos(11*np.pi/6)
                y3=r*np.sin(11*np.pi/6)
        
        
                xt1=np.linspace(x3,x1,int(self.npoints/3),endpoint=True)
                yt1=np.linspace(y3,y1,int(self.npoints/3),endpoint=True)
                xt1=xt1[0:-1]
                yt1=yt1[0:-1]
        
                xt2=np.linspace(x1,x2,int(self.npoints/3),endpoint=True)
                yt2=np.linspace(y1,y2,int(self.npoints/3),endpoint=True)
                xt2=xt2[1:-1]
                yt2=yt2[1:-1]
        
                xt3=np.linspace(x2,x3,int(self.npoints/3),endpoint=True)
                yt3=np.linspace(y2,y3,int(self.npoints/3),endpoint=True)
                xt3=xt3[1:-1]
                yt3=yt3[1:-1]
        
                xt=np.hstack([xt1,xt2,xt3])
                yt=np.hstack([yt1,yt2,yt3])                
                xt_desired = xt
                yt_desired = yt             
                np.savez(self.mainDirectory+'/'+self.name+'/'+'shapes_'+self.geometry+'.npz',gap=self.gap,xt=self.xt,yt=self.yt,xt2=self.xt2,yt2=self.yt2,xt_desired=xt_desired,yt_desired=yt_desired)                      
            
            self.xp = np.hstack([self.xt,self.xs])
            self.yp = np.hstack([self.yt,self.ys])
            self.zp = np.hstack([self.zt,self.zs])
            self.rbf = Rbf(self.xp,self.yp,self.zp,function='thin_plate')  # radial basis function interpolator instance  
            
            self.Z = self.rbf(self.X,self.Y) # create zz grid
            self.Z=np.round(self.Z,3)
            self.Z=np.maximum(self.Z,0)
            
            self.xp2 = np.hstack([self.xt2,self.xs2])
            self.yp2 = np.hstack([self.yt2,self.ys2])
            self.zp2 = np.hstack([self.zt2,self.zs2])
            
            self.rbf2 = Rbf(self.xp2,self.yp2,self.zp2,function='thin_plate')  # radial basis function interpolator instance  
            self.Z2 = self.rbf2(self.X,self.Y) # create zz grid
            self.Z2=np.round(self.Z2,3)
            self.Z2=np.maximum(self.Z2,0)
               
            
            
            self.Z3 = self.Z + self.Z2
            self.Z3 = np.round(self.Z3,3)            
            self.Z3 = self.Z3/np.max(self.Z3)
            
            (self.fy,self.fx) = np.gradient(self.Z3) # take gradient 
            self.fx = self.fx/np.max(self.fx) # normaluze gradient x term
            self.fy = self.fy/np.max(self.fy) # normalize gradient y term 
            
            self.F = RegularGridInterpolator((self.yy,self.xx),self.Z3) # form potential function 
            self.Fx = RegularGridInterpolator((self.yy,self.xx),self.fx) # form gradient of potnetial function x
            self.Fy = RegularGridInterpolator((self.yy,self.xx),self.fy)             
        
        
        
        #### Shape morphing 
        if self.control_mode=="shape_morphing":            
            self.geometries = self.parameters["geometries"]  
            self.delta = self.parameters["delta"] 
            self.d = self.parameters["d"]
            self.p = self.parameters["p"]  
            self.xc = self.parameters["xcenter"]
            self.yc = self.parameters["zcenter"]
            self.scale = self.parameters["scale"] 
            self.alpha = self.parameters["alpha"] 
            self.beta = self.parameters["beta"] 
            self.tcut = self.parameters["tcut"] 
            self.xmin = -self.d + self.xc
            self.xmax = self.d + self.xc
            self.ymin = -self.d + self.yc
            self.ymax = self.d + self.yc
            self.xx = np.arange(np.round(self.xmin,3),np.round(self.xmax,3), self.delta)
            self.yy = np.arange(np.round(self.ymin,3),np.round(self.ymax,3), self.delta)
            self.X,self.Y = np.meshgrid(self.xx, self.yy)        
            # circle
            self.a1 = self.parameters["a1"] 
            self.b1 = self.parameters["b1"] 
            self.epsilon1 = self.parameters["epsilon1"] 
            self.npoints1 = self.parameters["npoints1"] 
            self.step1 = self.parameters["step1"] 
            self.gap1 = self.parameters["gap1"] 
            self.num_iter1 = int(self.npoints1/self.step1)  
            (self.xt,self.yt,self.zt,self.xs,self.ys,self.zs) = self.points_oval_region(self.a1 + self.gap1/2,self.b1+self.gap1/2,self.xc,self.yc,self.npoints1,self.step1,self.num_iter1,self.epsilon1,"out")
            (self.xt2,self.yt2,self.zt2,self.xs2,self.ys2,self.zs2) = self.points_oval_region(self.a1 - self.gap1/2,self.b1-self.gap1/2,self.xc,self.yc,self.npoints1,self.step1,self.num_iter1,self.epsilon1,"in")                 
            theta=np.linspace(0,2*np.pi,self.npoints1)
            xt_desired = self.a1*np.cos(theta)
            yt_desired = self.b1*np.sin(theta)
            np.savez(self.mainDirectory+'/'+self.name+'/'+'shapes_'+self.geometries[0]+'.npz',gap=self.gap1,xt=self.xt,yt=self.yt,xt2=self.xt2,yt2=self.yt2,xt_desired=xt_desired,yt_desired=yt_desired)          
           
            self.xp = np.hstack([self.xt,self.xs])
            self.yp = np.hstack([self.yt,self.ys])
            self.zp = np.hstack([self.zt,self.zs])
            self.rbf = Rbf(self.xp,self.yp,self.zp,function='thin_plate')  # radial basis function interpolator instance  
            
            self.Z = self.rbf(self.X,self.Y) # create zz grid
            self.Z=np.round(self.Z,3)
            self.Z=np.maximum(self.Z,0)
            
            self.xp2 = np.hstack([self.xt2,self.xs2])
            self.yp2 = np.hstack([self.yt2,self.ys2])
            self.zp2 = np.hstack([self.zt2,self.zs2])
            
            self.rbf2 = Rbf(self.xp2,self.yp2,self.zp2,function='thin_plate')  # radial basis function interpolator instance  
            self.Z2 = self.rbf2(self.X,self.Y) # create zz grid
            self.Z2=np.round(self.Z2,3)
            self.Z2=np.maximum(self.Z2,0)
               
            self.Z3 = self.Z + self.Z2
            self.Z3 = np.round(self.Z3,3)            
            self.Z3 = self.Z3/np.max(self.Z3)
            
            (self.fy,self.fx) = np.gradient(self.Z3) # take gradient 
            self.fx = self.fx/np.max(self.fx) # normaluze gradient x term
            self.fy = self.fy/np.max(self.fy) # normalize gradient y term 
            
            self.F1 = RegularGridInterpolator((self.yy,self.xx),self.Z3) # form potential function 
            self.Fx1 = RegularGridInterpolator((self.yy,self.xx),self.fx) # form gradient of potnetial function x
            self.Fy1 = RegularGridInterpolator((self.yy,self.xx),self.fy)             
                    
           
            
           
            
           
            # oval
            self.a2 = self.parameters["a2"] 
            self.b2 = self.parameters["b2"] 
            self.epsilon2 = self.parameters["epsilon2"] 
            self.npoints2 = self.parameters["npoints2"] 
            self.step2 = self.parameters["step2"] 
            self.gap2 = self.parameters["gap2"] 
            self.num_iter2 = int(self.npoints2/self.step2)  
            (self.xt,self.yt,self.zt,self.xs,self.ys,self.zs) = self.points_oval_region(self.a2 + self.gap2/2,self.b2 + self.gap2/2,self.xc,self.yc,self.npoints2,self.step2,self.num_iter2,self.epsilon2,"out")
            (self.xt2,self.yt2,self.zt2,self.xs2,self.ys2,self.zs2) = self.points_oval_region(self.a2 - self.gap2/2,self.b2 - self.gap2/2,self.xc,self.yc,self.npoints2,self.step2,self.num_iter2,self.epsilon2,"in")                 
            theta=np.linspace(0,2*np.pi,self.npoints2)
            xt_desired = self.a2*np.cos(theta)
            yt_desired = self.b2*np.sin(theta)
            np.savez(self.mainDirectory+'/'+self.name+'/'+'shapes_'+self.geometries[1]+'.npz',gap=self.gap1,xt=self.xt,yt=self.yt,xt2=self.xt2,yt2=self.yt2,xt_desired=xt_desired,yt_desired=yt_desired)          
                 
            self.xp = np.hstack([self.xt,self.xs])
            self.yp = np.hstack([self.yt,self.ys])
            self.zp = np.hstack([self.zt,self.zs])
            self.rbf = Rbf(self.xp,self.yp,self.zp,function='thin_plate')  # radial basis function interpolator instance  
            
            self.Z = self.rbf(self.X,self.Y) # create zz grid
            self.Z=np.round(self.Z,3)
            self.Z=np.maximum(self.Z,0)
            
            self.xp2 = np.hstack([self.xt2,self.xs2])
            self.yp2 = np.hstack([self.yt2,self.ys2])
            self.zp2 = np.hstack([self.zt2,self.zs2])
            
            self.rbf2 = Rbf(self.xp2,self.yp2,self.zp2,function='thin_plate')  # radial basis function interpolator instance  
            self.Z2 = self.rbf2(self.X,self.Y) # create zz grid
            self.Z2=np.round(self.Z2,3)
            self.Z2=np.maximum(self.Z2,0)
               
            self.Z3 = self.Z + self.Z2
            self.Z3 = np.round(self.Z3,3)            
            self.Z3 = self.Z3/np.max(self.Z3)
            
            (self.fy,self.fx) = np.gradient(self.Z3) # take gradient 
            self.fx = self.fx/np.max(self.fx) # normaluze gradient x term
            self.fy = self.fy/np.max(self.fy) # normalize gradient y term 
            
            self.F2 = RegularGridInterpolator((self.yy,self.xx),self.Z3) # form potential function 
            self.Fx2 = RegularGridInterpolator((self.yy,self.xx),self.fx) # form gradient of potnetial function x
            self.Fy2 = RegularGridInterpolator((self.yy,self.xx),self.fy)             
                    
            # square
            self.Rshape3 = self.parameters["Rshape3"] 
            self.epsilon3 = self.parameters["epsilon3"] 
            self.npoints3 = self.parameters["npoints3"] 
            self.step3 = self.parameters["step3"] 
            self.gap3 = self.parameters["gap3"] 
            self.num_iter3 = int(self.npoints3/self.step3) 
            (self.xt,self.yt,self.zt,self.xs,self.ys,self.zs) = self.points_square_region(self.Rshape3 + self.gap3/2,self.xc,self.yc,self.npoints3,self.step3,self.num_iter3,self.epsilon3,"out")
            (self.xt2,self.yt2,self.zt2,self.xs2,self.ys2,self.zs2) = self.points_square_region(self.Rshape3 - self.gap3/2,self.xc,self.yc,self.npoints3,self.step3,self.num_iter3,self.epsilon3,"in")                
            xt1 = np.ones(int(self.npoints3/4))
            xt2 = np.linspace(1.01,-1,int(self.npoints3/4),endpoint=False)
            xt3 = -1*np.ones(int(self.npoints3/4))
            xt4 = np.linspace(-1.01,1,int(self.npoints3/4),endpoint=False)
    
            yt1 = np.linspace(-1.01,1,int(self.npoints3/4),endpoint=False)
            yt2 = np.ones(int(self.npoints3/4))
            yt3 = np.linspace(1.01,-1,int(self.npoints3/4),endpoint=False)
            yt4 = -1*np.ones(int(self.npoints3/4))
            
            xt=np.hstack([xt1,xt2,xt3,xt4])
            yt=np.hstack([yt1,yt2,yt3,yt4])
            xt_desired = np.dot(self.Rshape3,xt)
            yt_desired = np.dot(self.Rshape3,yt)                
            np.savez(self.mainDirectory+'/'+self.name+'/'+'shapes_'+self.geometries[2]+'.npz',gap=self.gap3,xt=self.xt,yt=self.yt,xt2=self.xt2,yt2=self.yt2,xt_desired=xt_desired,yt_desired=yt_desired)                  
           
            self.xp = np.hstack([self.xt,self.xs])
            self.yp = np.hstack([self.yt,self.ys])
            self.zp = np.hstack([self.zt,self.zs])
            self.rbf = Rbf(self.xp,self.yp,self.zp,function='thin_plate')  # radial basis function interpolator instance  
            
            self.Z = self.rbf(self.X,self.Y) # create zz grid
            self.Z=np.round(self.Z,3)
            self.Z=np.maximum(self.Z,0)
            
            self.xp2 = np.hstack([self.xt2,self.xs2])
            self.yp2 = np.hstack([self.yt2,self.ys2])
            self.zp2 = np.hstack([self.zt2,self.zs2])
            
            self.rbf2 = Rbf(self.xp2,self.yp2,self.zp2,function='thin_plate')  # radial basis function interpolator instance  
            self.Z2 = self.rbf2(self.X,self.Y) # create zz grid
            self.Z2=np.round(self.Z2,3)
            self.Z2=np.maximum(self.Z2,0)
               
            self.Z3 = self.Z + self.Z2
            self.Z3 = np.round(self.Z3,3)            
            self.Z3 = self.Z3/np.max(self.Z3)
            
            (self.fy,self.fx) = np.gradient(self.Z3) # take gradient 
            self.fx = self.fx/np.max(self.fx) # normaluze gradient x term
            self.fy = self.fy/np.max(self.fy) # normalize gradient y term 
            
            self.F3 = RegularGridInterpolator((self.yy,self.xx),self.Z3) # form potential function 
            self.Fx3 = RegularGridInterpolator((self.yy,self.xx),self.fx) # form gradient of potnetial function x
            self.Fy3 = RegularGridInterpolator((self.yy,self.xx),self.fy)             
                    
                       
            
            # triangle
            self.Rshape4 = self.parameters["Rshape4"] 
            self.epsilon4 = self.parameters["epsilon4"] 
            self.npoints4 = self.parameters["npoints4"] 
            self.step4 = self.parameters["step4"] 
            self.epsilon4 = self.parameters["epsilon4"] 
            self.gap4 = self.parameters["gap4"]       
            self.num_iter4 = int(self.npoints4/self.step4)   
            (self.xt,self.yt,self.zt,self.xs,self.ys,self.zs) = self.points_triangle_region(self.Rshape4 + self.gap4/2,self.xc,self.yc,self.npoints4,self.step4,self.num_iter4,self.epsilon4,"out")
            (self.xt2,self.yt2,self.zt2,self.xs2,self.ys2,self.zs2) = self.points_triangle_region(self.Rshape4 - self.gap4/2,self.xc,self.yc,self.npoints4,self.step4,self.num_iter4,self.epsilon4,"in")      
            
            r = self.Rshape4/.5236
            x1=0
            y1=r
            x2=r*np.cos(7*np.pi/6)
            y2=r*np.sin(7*np.pi/6)
            x3=r*np.cos(11*np.pi/6)
            y3=r*np.sin(11*np.pi/6)
    
            xt1=np.linspace(x3,x1,int(self.npoints4/3),endpoint=True)
            yt1=np.linspace(y3,y1,int(self.npoints4/3),endpoint=True)
            xt1=xt1[0:-1]
            yt1=yt1[0:-1]
    
            xt2=np.linspace(x1,x2,int(self.npoints4/3),endpoint=True)
            yt2=np.linspace(y1,y2,int(self.npoints4/3),endpoint=True)
            xt2=xt2[1:-1]
            yt2=yt2[1:-1]
    
            xt3=np.linspace(x2,x3,int(self.npoints4/3),endpoint=True)
            yt3=np.linspace(y2,y3,int(self.npoints4/3),endpoint=True)
            xt3=xt3[1:-1]
            yt3=yt3[1:-1]
    
            xt=np.hstack([xt1,xt2,xt3])
            yt=np.hstack([yt1,yt2,yt3])                
            xt_desired = xt
            yt_desired = yt             
            np.savez(self.mainDirectory+'/'+self.name+'/'+'shapes_'+self.geometries[3]+'.npz',gap=self.gap4,xt=self.xt,yt=self.yt,xt2=self.xt2,yt2=self.yt2,xt_desired=xt_desired,yt_desired=yt_desired)                      
                
            self.xp = np.hstack([self.xt,self.xs])
            self.yp = np.hstack([self.yt,self.ys])
            self.zp = np.hstack([self.zt,self.zs])
            self.rbf = Rbf(self.xp,self.yp,self.zp,function='thin_plate')  # radial basis function interpolator instance  
            
            self.Z = self.rbf(self.X,self.Y) # create zz grid
            self.Z=np.round(self.Z,3)
            self.Z=np.maximum(self.Z,0)
            
            self.xp2 = np.hstack([self.xt2,self.xs2])
            self.yp2 = np.hstack([self.yt2,self.ys2])
            self.zp2 = np.hstack([self.zt2,self.zs2])
            
            self.rbf2 = Rbf(self.xp2,self.yp2,self.zp2,function='thin_plate')  # radial basis function interpolator instance  
            self.Z2 = self.rbf2(self.X,self.Y) # create zz grid
            self.Z2=np.round(self.Z2,3)
            self.Z2=np.maximum(self.Z2,0)
               
            self.Z3 = self.Z + self.Z2
            self.Z3 = np.round(self.Z3,3)            
            self.Z3 = self.Z3/np.max(self.Z3)
            
            (self.fy,self.fx) = np.gradient(self.Z3) # take gradient 
            self.fx = self.fx/np.max(self.fx) # normaluze gradient x term
            self.fy = self.fy/np.max(self.fy) # normalize gradient y term 
            
            self.F4 = RegularGridInterpolator((self.yy,self.xx),self.Z3) # form potential function 
            self.Fx4 = RegularGridInterpolator((self.yy,self.xx),self.fx) # form gradient of potnetial function x
            self.Fy4 = RegularGridInterpolator((self.yy,self.xx),self.fy)             
                    
           
        
            
            
            
            
    def points_oval_region(self,a,b,xc,yc,npoints,step,num_iter,epsilon,op):
        ''' Function to Create points for oval '''    
        theta=np.linspace(0,2*np.pi,npoints)
        xt = a*np.cos(theta)
        yt = b*np.sin(theta)
        zt = np.zeros(len(xt))
        xt = xt + xc
        yt = yt + yc 
        zt = np.zeros(len(xt))
        xs=[]
        ys=[]
        zs=[]
        for i in range(num_iter-step):
            if op=="out":
                theta=np.arctan2(yt[int(step*i)],xt[int(step*i)])
                rr=np.sqrt(yt[int(step*i)]**2 + xt[int(step*i)]**2)
                xs.append((rr-epsilon)*np.cos(theta))
                ys.append((rr-epsilon)*np.sin(theta))                
                zs.append(-0.25)
            if op=="in":
                theta=np.arctan2(yt[int(step*i)],xt[int(step*i)])
                rr=np.sqrt(yt[int(step*i)]**2 + xt[int(step*i)]**2)
                xs.append((rr-epsilon)*np.cos(theta))
                ys.append((rr-epsilon)*np.sin(theta))               
                zs.append(0.25)            


        return (xt,yt,zt,xs,ys,zs)


    def points_square_region(self,R,xc,yc,npoints,step,num_iter,epsilon,op):
        ''' Function to Create points for Square '''
        xt1 = np.ones(int(npoints/4))
        xt2 = np.linspace(1.01,-1,int(npoints/4),endpoint=False)
        xt3 = -1*np.ones(int(npoints/4))
        xt4 = np.linspace(-1.01,1,int(npoints/4),endpoint=False)

        yt1 = np.linspace(-1.01,1,int(npoints/4),endpoint=False)
        yt2 = np.ones(int(npoints/4))
        yt3 = np.linspace(1.01,-1,int(npoints/4),endpoint=False)
        yt4 = -1*np.ones(int(npoints/4))
        
        xt=np.hstack([xt1,xt2,xt3,xt4])
        yt=np.hstack([yt1,yt2,yt3,yt4])
        xt = np.dot(R,xt)
        yt = np.dot(R,yt)
        xt = xt + xc
        yt = yt + yc 
        zt = np.zeros(len(xt))
        xs=[]
        ys=[]
        zs=[]
        for i in range(num_iter-step):
            if op=="out":
                xs.append(xt[int(step*i)]-np.sign(xt[int(step*i)])*epsilon)
                ys.append(yt[int(step*i)]-np.sign(yt[int(step*i)])*epsilon)                  
                zs.append(-0.25)
            if op=="in":
                xs.append(xt[int(step*i)]-np.sign(xt[int(step*i)])*epsilon)
                ys.append(yt[int(step*i)]-np.sign(yt[int(step*i)])*epsilon)                  
                zs.append(0.25)     
        return (xt,yt,zt,xs,ys,zs)


    def points_triangle_region(self,R,xc,yc,npoints,step,num_iter,epsilon,op):
        ''' Function to Create points for Square '''
        r = R/.5236
        x1=0
        y1=r
        x2=r*np.cos(7*np.pi/6)
        y2=r*np.sin(7*np.pi/6)
        x3=r*np.cos(11*np.pi/6)
        y3=r*np.sin(11*np.pi/6)


        xt1=np.linspace(x3,x1,int(npoints/3),endpoint=True)
        yt1=np.linspace(y3,y1,int(npoints/3),endpoint=True)
        xt1=xt1[0:-1]
        yt1=yt1[0:-1]

        xt2=np.linspace(x1,x2,int(npoints/3),endpoint=True)
        yt2=np.linspace(y1,y2,int(npoints/3),endpoint=True)
        xt2=xt2[1:-1]
        yt2=yt2[1:-1]

        xt3=np.linspace(x2,x3,int(npoints/3),endpoint=True)
        yt3=np.linspace(y2,y3,int(npoints/3),endpoint=True)
        xt3=xt3[1:-1]
        yt3=yt3[1:-1]

        xt=np.hstack([xt1,xt2,xt3])
        yt=np.hstack([yt1,yt2,yt3])
        zt = np.zeros(len(xt))

        xt = xt + xc
        yt = yt + yc 
        zt = np.zeros(len(xt))

        xs=[]
        ys=[]
        zs=[]
        for i in range(num_iter-step):
            if op=="out":
                theta=np.arctan2(yt[int(step*i)],xt[int(step*i)])
                rr=np.sqrt(yt[int(step*i)]**2 + xt[int(step*i)]**2)
                xs.append((rr-epsilon)*np.cos(theta))
                ys.append((rr-epsilon)*np.sin(theta))               
                zs.append(-0.25)
            if op=="in":
                theta=np.arctan2(yt[int(step*i)],xt[int(step*i)])
                rr=np.sqrt(yt[int(step*i)]**2 + xt[int(step*i)]**2)
                xs.append((rr-epsilon)*np.cos(theta))
                ys.append((rr-epsilon)*np.sin(theta))              
                zs.append(0.25)             
        return (xt,yt,zt,xs,ys,zs)
    
    
    def d_(self,x,y,xc,yc,a,b):
        return(((x-xc)/a)**2 + ((y-yc)/b)**2)

    def dx_(self,x,y,xc,yc,a,b):
        return((2*x -2*xc) / a**2)

    def dy_(self,x,y,xc,yc,a,b):
        return((2*y -2*yc) / b**2)

    def F_grasp(self,x,y,xc,yc,a,b):
        d=self.d_(x,y,xc,yc,a,b)
        return(d**2 * np.log(d))

    def F2_grasp(self,x,y,xc,yc,a,b):
        return(self.F_grasp(x,y,xc,yc,a,b)**2)

    def Fx_grasp(self,x,y,xc,yc,a,b):
        dx = self.dx_(x,y,xc,yc,a,b)
        d = self.d_(x,y,xc,yc,a,b)    
        return(2*d*np.log(d)*dx + d*dx)

    def Fy_grasp(self,x,y,xc,yc,a,b):
        dy = self.dy_(x,y,xc,yc,a,b)
        d = self.d_(x,y,xc,yc,a,b)    
        return(2*d*np.log(d)*dy + d*dy)

    def Fx2_grasp(self,x,y,xc,yc,a,b):
        return(2*self.F_grasp(x,y,xc,yc,a,b)*self.Fx_grasp(x,y,xc,yc,a,b))

    def Fy2_grasp(self,x,y,xc,yc,a,b):
        return(2*self.F_grasp(x,y,xc,yc,a,b)*self.Fy_grasp(x,y,xc,yc,a,b))

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
    
    
    def F_(self,x,y,xc=0,yc=0,a=0,b=0,t=0):
        if self.control_mode=="shape_formation":
            F=self.F((y,x))
            
        if self.control_mode=="shape_morphing":
            F=self.F_morph(x,y,t)
                
        if self.control_mode=="grasping": 
            F=self.F_grasp(x,y,xc,yc,a,b)
            
        if self.control_mode=="target_chasing":
            F=self.F_point(x,y,xc,yc)
        return(F)

    def FX_(self,x,y,xc=0,yc=0,a=0,b=0,t=0):
        if self.control_mode=="shape_formation":
            FX = self.Fx((y,x)) 
            
        if self.control_mode=="shape_morphing":
            FX = self.F_morphx(x,y,t)            
            
        if self.control_mode=="grasping":
            FX = self.Fx2_grasp(x,y,xc,yc,a,b)
            
        if self.control_mode=="target_chasing":
            FX = self.Fx_point(x,y,xc,yc)
        return(FX)      

    def FY_(self,x,y,xc=0,yc=0,a=0,b=0,t=0):
        if self.control_mode=="shape_formation":
            FY = self.Fy((y,x))
            
        if self.control_mode=="shape_morphing":
            FY=self.F_morphy(x,y,t)            
            
        if self.control_mode=="grasping":
            FY = self.Fy2_grasp(x,y,xc,yc,a,b)
            
        if self.control_mode=="target_chasing":
            FY=self.Fy_point(x,y,xc,yc)            
        return(FY)  
    
    
    def tanh(self,t):
        """ tanh function """
        tanh=(np.exp(self.p*(t))-1)/(np.exp(self.p*(t))+1)
        return(tanh)      

    def F_morph(self,x,y,t):
        if t<=self.tcut[0]:
            F=(1-self.tanh(t))*self.F1((y,x)) + self.tanh(t)*self.F2((y,x))
        
        if self.tcut[0]<t and t<=self.tcut[1]:
            F=(1-self.tanh(t-self.tcut[0]))*self.F2((y,x)) + self.tanh(t-self.tcut[0])*self.F3((y,x))
        
        if self.tcut[1]<t:
            F=(1-self.tanh(t-self.tcut[1]))*self.F3((y,x)) + self.tanh(t-self.tcut[1])*self.F4((y,x))
        return(F)

    
    def F_morphx(self,x,y,t):
        if t<self.tcut[0]:
            FX=(1-self.tanh(t))*self.Fx1((y,x)) + self.tanh(t)*self.Fx2((y,x))
        
        if self.tcut[0]<=t and t<=self.tcut[1]:
            FX=(1-self.tanh(t-self.tcut[0]))*self.Fx2((y,x)) + self.tanh(t-self.tcut[0])*self.Fx3((y,x))
        
        if self.tcut[1]<=t:
            FX=(1-self.tanh(t-self.tcut[1]))*self.Fx3((y,x)) + self.tanh(t-self.tcut[1])*self.Fx4((y,x))
        return(FX)
    
    def F_morphy(self,x,y,t):
        if t<self.tcut[0]:
            FY=(1-self.tanh(t))*self.Fy1((y,x)) + self.tanh(t)*self.Fy2((y,x))
        
        if self.tcut[0]<=t and t<=self.tcut[1]:
            FY=(1-self.tanh(t-self.tcut[0]))*self.Fy2((y,x)) + self.tanh(t-self.tcut[0])*self.Fy3((y,x))
        
        if self.tcut[1]<=t:
            FY=(1-self.tanh(t-self.tcut[1]))*self.Fy3((y,x)) + self.tanh(t-self.tcut[1])*self.Fy4((y,x))
        return(FY)
    
    
    
class import_data:
     def __init__(self,name,path,wxmin,wxmax,wymin,wymax):
         self.name=name
         self.path=path
         self.mainDirectory = path   # main directory 
         parameters=np.load(self.mainDirectory+self.name+'/Parameters.npy',allow_pickle=True)
         data=np.load(self.mainDirectory+self.name+'/Radii'+self.name+'.npz',allow_pickle=True) 
         self.Rm=data['Rm'] 
         self.wxmin = wxmin
         self.wxmax = wxmax
         self.wymin = wymin
         self.wymax = wymax
         self.err = 5
       
        
         self.parameters=parameters.tolist()           
         self.nb=self.parameters['nb'] # number of bots
         self.ni=self.parameters['total_particles']
         self.ns=self.parameters['ns']
         self.nm=self.nb*self.ns
         self.bot_width=self.parameters['bot_width']
         self.particle_width=self.parameters['particle_width']
         self.control_mode=self.parameters['control_mode']
         self.skin_width=self.parameters['skin_width']
         
         if self.control_mode=="shape_formation":
             self.geometry = self.parameters['geometry']
             data2=np.load(self.mainDirectory+self.name+'/shapes_'+self.geometry+'.npz',allow_pickle=True)
             self.xt=data2['xt']
             self.yt=data2['yt']
             self.xt2=data2['xt2']
             self.yt2=data2['yt2']
             self.xt_desired=data2['xt_desired']
             self.yt_desired=data2['yt_desired']       
             self.gap=data2["gap"]
                  
         if self.control_mode=="shape_morphing":
             self.geometries = self.parameters["geometries"]  
             self.particle_width = self.parameters["particle_width"]  
             self.tcut = self.parameters["tcut"] 
             
             data1=np.load(self.mainDirectory+'/'+self.name+'/'+'shapes_'+self.geometries[1]+'.npz',allow_pickle=True)
             self.xt_desired1=data1['xt_desired']
             self.yt_desired1=data1['yt_desired']                

             data2=np.load(self.mainDirectory+'/'+self.name+'/'+'shapes_'+self.geometries[2]+'.npz',allow_pickle=True)
             self.xt_desired2=data2['xt_desired']
             self.yt_desired2=data2['yt_desired']                   
             
             data3=np.load(self.mainDirectory+'/'+self.name+'/'+'shapes_'+self.geometries[3]+'.npz',allow_pickle=True)         
             self.xt_desired3=data3['xt_desired']
             self.yt_desired3=data3['yt_desired']                   
         
            
         
         if self.control_mode=="target_chasing":
             self.xm = -1.34
             self.zm = -.0272
             self.rm = 0.08
             
 
         self.path=self.path+self.name+"/results/"
         os.chdir(self.path)
         self.files = sorted(os.listdir(os.getcwd()), key=os.path.getmtime)
         
         
         # Robot Position
         self.bot_position=np.genfromtxt(self.files[self.files.index('bot_position.csv') ] ,delimiter=',')
         (self.m1,self.n1)=np.shape(self.bot_position)
         self.bot_position=self.bot_position[:,1:self.n1]
         self.time=self.bot_position[0,:]
         self.bot_position_x=self.bot_position[1:self.nb+1,:]
         self.bot_position_y=self.bot_position[self.nb+1:2*self.nb+1,:]
         self.bot_position_z=self.bot_position[(2*self.nb)+1:3*self.nb+1,:] 
         
         # membrane_positions
         self.membrane_position=np.genfromtxt(self.files[self.files.index('membrane_position.csv') ] ,delimiter=',')
         (m,n)=np.shape(self.membrane_position)
         self.membrane_position=self.membrane_position[:,1:n]
         self.membrane_position_x=self.membrane_position[1:self.nm+1,:]
         self.membrane_position_y=self.membrane_position[self.nm+1:2*self.nm+1,:]
         self.membrane_position_z=self.membrane_position[(2*self.nm)+1:3*self.nm+1,:]  
         
         if self.control_mode=="shape_formation" or self.control_mode=="shape_morphing" :         
             self.Field_value=np.genfromtxt(self.files[self.files.index('field_values.csv') ] ,delimiter=',')
             (m,n)=np.shape(self.Field_value)
             self.Field_value=self.Field_value[:,1:n]
             self.Field_value_sum=[] 
             for i in range(len(self.time)):
                 self.Field_value_sum.append(np.sum(abs(self.Field_value[:,i])))
             
         if self.ni==0:
             pass
         else:
             
         # Particle Position
             self.particle_position=np.genfromtxt(self.files[self.files.index('particle_position.csv') ] ,delimiter=',')
             (self.m4a,self.n4a)=np.shape(self.particle_position)
             self.particle_position=self.particle_position[:,1:self.n4a]
             self.particle_position_x=self.particle_position[1:self.ni+1,:]
             self.particle_position_y=self.particle_position[self.ni+1:2*self.ni+1,:]
             self.particle_position_z=self.particle_position[(2*self.ni)+1:3*self.ni+1,:]

         if self.control_mode=="grasping":
             self.geom = self.parameters['ball_geometry'] 
             self.ball_radius = self.parameters['ball_radius']
             self.ball_position=np.genfromtxt(self.files[self.files.index('ball_position.csv') ] ,delimiter=',')            
             (self.m5a,self.n5a)=np.shape(self.ball_position)
             self.ballx_position=self.ball_position[1,:]
             self.ballz_position=self.ball_position[2,:]
             
             self.pull_data=np.genfromtxt(self.files[self.files.index('pull_force.csv') ] ,delimiter=',')            
             (self.m6a,self.n6a)=np.shape(self.pull_data)
             self.TIME=self.pull_data[0,:]
             self.PX=self.pull_data[1,:]
             self.PZ=self.pull_data[2,:]
             self.Fb=self.pull_data[3,:]   
             
             
             
              
     def create_frames_morph(self,membrane):
        ''' Create frames for a video '''
        direct = os.path.join(self.mainDirectory+self.name,'_frames')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        for i in range(len(self.time)-1):    
            fig = plt.figure(dpi=300)
            fig.set_size_inches(4, 4)
            
            ax = plt.axes(xlim=(self.wxmin,self.wxmax), ylim=(self.wymin, self.wymax))

            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]  
                patch = plt.Circle((100*x0, 100*y0),100*self.bot_width/2, fc='black')
                ax.add_patch(patch)
            
            for j in range(self.ni):
                x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]
                #print(self.Rm)
                #print(np.round(self.Rm[j],4))
                if self.Rm[j]==self.particle_width/2:
                    c='tab:blue'
                else:
                #if self.Rm[j]==self.particle_width*np.sqrt(2)/2:
                    c='tab:green'
                    
                patch = plt.Circle((100*x0, 100*y0),100*self.Rm[j], fc=c)
                ax.add_patch(patch)         
            t = self.time[i]
            if t<=self.tcut[0]:
                ax.plot(self.xt_desired1,self.yt_desired1,color='tab:red',linestyle='dashed',linewidth=2)
            
            if self.tcut[0]<t and t<=self.tcut[1]:
                ax.plot(self.xt_desired2,self.yt_desired2,color='tab:red',linestyle='dashed',linewidth=2)
            
            if self.tcut[1]<t:            
                ax.plot(self.xt_desired3,self.yt_desired3,color='tab:red',linestyle='dashed',linewidth=2)
            
          
            
            plt.title('Time= ' + str(np.round(self.time[i],3)),fontsize=8)
            plt.savefig(direct+"/frame%04d.jpg" % count)
            count=count+1 
            plt.close('all')   
    
     def create_frames(self,membrane,const,tunnel_=False):
        ''' Create frames for a video '''
        direct = os.path.join(self.mainDirectory+self.name,'_frames')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        for i in range(len(self.time)-1):    
            fig = plt.figure(dpi=300)
            fig.set_size_inches(4,2)
            
            ax = plt.axes(xlim=(self.wxmin,self.wxmax), ylim=(self.wymin, self.wymax))

            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]  
                patch = plt.Circle((const*x0, const*y0),const*self.bot_width/2, fc='black')
                ax.add_patch(patch)
            
                
            
            if membrane==True:
                for j in range(0,self.nm):
                    
                    x0,y0=self.membrane_position_x[j,i],self.membrane_position_z[j,i]  
                    patch = plt.Circle((const*x0, const*y0),const*self.skin_width/2, fc='tab:red')
                    ax.add_patch(patch)
                    
                    
                    
            if self.control_mode=="grasping":
                if self.geom=="circle":
                    x0,y0=self.ballx_position[i],self.ballz_position[i]
                    patch = plt.Circle((const*x0, const*y0),const*self.ball_radius, fc='tab:grey')
                    ax.add_patch(patch)
                if self.geom=="square":
                    const_=self.ball_radius*2*np.pi/4
                    x0,y0=self.ballx_position[i]-const_/2,self.ballz_position[i] - const_/2
                    patch = matplotlib.patches.Rectangle((const*x0, const*y0),const*const_, const*const_,fc='tab:grey',edgecolor='tab:grey')     
                    ax.add_patch(patch)
                if self.geom=="triangle":
                    const_=self.ball_radius*2*np.pi/3
                    r=const_*np.sqrt(3)/3
                    x0,y0=self.ballx_position[i],self.ballz_position[i]
                    patch = RegularPolygon((const*x0,const*y0),3,r,orientation=-np.pi/2,fc='tab:grey',edgecolor='tab:grey')
                    ax.add_patch(patch)                 
            if self.ni==0:
                pass
            else:
                
                for j in range(self.ni):
                    x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]
                    #print(self.Rm)
                    #print(np.round(self.Rm[j],4))
                    if np.round(self.Rm[j],4)==0.0508:
                        c='tab:blue'
                    else:
                        c='tab:green'
                    patch = plt.Circle((const*x0, const*y0),const*self.Rm[j], fc=c)
                    ax.add_patch(patch)              
            
            
            
            
            
            
            
            
            
            
            if tunnel_==True:
                # w = 14.2
                # l = 65
                # gap = 30
                # xpos1_wall = -52
                # xpos2_wall = -52      
                # px1 = xpos1_wall #+ w/2
                # px2 = xpos2_wall #+ w/2
                # py1 = gap
                # py2 = -gap-l 
    
                # rect1 = plt.Rectangle((px1, py1),w,l,facecolor="black", alpha=0.5,zorder=2)    
                # ax.add_patch(rect1)
    
                # rect2 = plt.Rectangle((px2, py2),w,l,facecolor="black", alpha=0.5,zorder=2)    
                # ax.add_patch(rect2)       
                # patch = plt.Circle((100*self.xm, 100*self.zm),100*self.rm, fc="tab:red")
                # ax.add_patch(patch)   
                
                # c1=0
                # c2=10            
                # x2=np.linspace(c1,c2,100)
                # x1=np.linspace(c1,c2,100)                
                # line1=np.tanh(x1-5)-3.75-.25
                # line2=np.tanh(x2-5)-2.25+.25 
                # plt.fill_between(np.flipud(x1),np.flipud(line2),color='tab:grey',alpha=1)
                # plt.fill_between(np.flipud(x1),np.flipud(line1),-6.25,color='tab:grey',alpha=1)   
                
                
                c1=0
                c2=10 
                x2=np.linspace(c1,c2,1000)
                x1=np.linspace(c1,c2,1000)                
                line1=np.sin(1.745*x1)-4
                line2=np.sin(1.745*x1)-1.5
                plt.fill_between(np.flipud(x1),np.flipud(line2),color='tab:grey',alpha=1)
                plt.fill_between(np.flipud(x1),np.flipud(line1),-6.25,color='tab:grey',alpha=1)      
                plt.plot(x1,line1,color='k',linewidth=2)
                plt.plot(x2,line2,color='k',linewidth=2)            
            
                
                
                
                
                
                
                #plt.plot(x1,line1,color='k',linewidth=2)
                #plt.plot(x2,line2,color='k',linewidth=2)
                ax.scatter(12, -2, s=80, c="tab:cyan", marker=(5, 1))
       
            #ax.plot(self.xt,self.yt,color='tab:red',linestyle='dashed',linewidth=2)
            #ax.plot(self.xt_desired,self.yt_desired,color='tab:red',linestyle='dashed',linewidth=2)            
            #ax.plot(self.xt2,self.yt2,color='tab:red',linestyle='dashed',linewidth=2)            
            plt.title('Time= ' + str(np.round(self.time[i],3)),fontsize=8)
            plt.savefig(direct+"/frame%04d.jpg" % count)
            count=count+1 
            plt.close('all')          

     def plot_field_values(self):
         fig, ax = plt.subplots(figsize=(2, 2),dpi=300)      
         ax.plot(self.time,self.Field_value_sum,color='red',linewidth=1)
        
         x_ticks = np.linspace(self.time[0], self.time[-1],5,endpoint=True)
         y_ticks = np.linspace(np.min(self.Field_value_sum), np.max(self.Field_value_sum),5,endpoint=True)
         ax.set_xticks(np.round(x_ticks,2))
         ax.set_yticks(np.round(y_ticks,2))
         ax.xaxis.set_tick_params(width=.25,length=2)
         ax.yaxis.set_tick_params(width=.25,length=2)
         ax.set_ylabel(r"$\Sigma \phi$")
         ax.set_xlabel("Time (seconds)")
         ax.set_title("Field Values")
         ax.grid(True)
         plt.tight_layout()
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'field_value.jpg')
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'field_value.svg')    
         plt.savefig(self.mainDirectory+'/'+self.name+'/'+'field_value.pdf')        

     def create_snap_shot_morph(self,entry,const,membrane):
        ''' Create snapshots  '''
        i=entry
        fig, ax = plt.subplots(figsize=(1.0,1.0),dpi=300)
        ax = plt.axes(xlim=(self.wxmin,self.wxmax), ylim=(self.wymin, self.wymax))
        for j in range(0,self.nb):
            x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]  
            patch = plt.Circle((const*x0, const*y0),const*self.bot_width/2, fc='black')
            ax.add_patch(patch)
            
        for j in range(self.ni):
            x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

            if np.round(self.Rm[j],4)==0.0508:
                c='tab:blue'
            else:
                c='tab:green'
            patch = plt.Circle((const*x0, const*y0),const*self.Rm[j], fc=c)
            ax.add_patch(patch)         
     
            t = self.time[i]
            if t<=self.tcut[0]:
                ax.plot(self.xt_desired1,self.yt_desired1,color='tab:red',linestyle='dashed',linewidth=0.5)
            
            if self.tcut[0]<t and t<=self.tcut[1]:
                ax.plot(self.xt_desired2,self.yt_desired2,color='tab:red',linestyle='dashed',linewidth=0.5)
            
            if self.tcut[1]<t:            
                ax.plot(self.xt_desired3,self.yt_desired3,color='tab:red',linestyle='dashed',linewidth=0.5)
        plt.axis('off')   
        ax.axis('equal')  
        plt.title('Time= ' + str(np.round(self.time[i],0)),fontsize=8)
        plt.savefig(self.mainDirectory+'/'+self.name+'/'+'time'+str(np.round(self.time[i],2))+'.jpg')
        plt.savefig(self.mainDirectory+'/'+self.name+'/'+'time'+str(np.round(self.time[i],2))+'.svg')    
        plt.savefig(self.mainDirectory+'/'+self.name+'/'+'time'+str(np.round(self.time[i],2))+'.pdf')
    
       # plt.close('all')   
       




     def create_snap_shot(self,entry,const,membrane,tunnel_=False):
        ''' Create snapshots  '''
        i=entry
        #ratio =(self.wymax-self.wymin)/(self.wxmax-self.wxmin)
        height_=2
        fig, ax = plt.subplots(figsize=(2,2),dpi=300)
        #plt.axis('off')
        ax = plt.axes(xlim=(self.wxmin,self.wxmax), ylim=(self.wymin, self.wymax))
        for j in range(0,self.nb):
            x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]  
            patch = plt.Circle((const*x0,const*y0),const*self.bot_width/2, fc='black')
            ax.add_patch(patch)
            
        if self.control_mode=="grasping":
            if self.geom=="circle":
                x0,y0=self.ballx_position[i],self.ballz_position[i]
                patch = plt.Circle((const*x0, const*y0),const*self.ball_radius, fc='tab:grey')
                ax.add_patch(patch)
            if self.geom=="square":
                const_=self.ball_radius*2*np.pi/4
                x0,y0=self.ballx_position[i]-const_/2,self.ballz_position[i] - const_/2
                patch = matplotlib.patches.Rectangle((const*x0, const*y0),const*const_, const*const_,fc='tab:grey',edgecolor='tab:grey')     
                ax.add_patch(patch)
            if self.geom=="triangle":
                const_=self.ball_radius*2*np.pi/3
                r=const_*np.sqrt(3)/3
                x0,y0=self.ballx_position[i],self.ballz_position[i]
                patch = RegularPolygon((const*x0,const*y0),3,r,orientation=-np.pi/2,fc='tab:grey',edgecolor='tab:grey')
                ax.add_patch(patch)             
        if self.ni==0:
            pass
        else:
                            
            for j in range(self.ni):
                x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]
    
                if np.round(self.Rm[j],4)==0.0508:
                    c='tab:blue'
                else:
                    c='tab:green'
                patch = plt.Circle((const*x0, const*y0),const*self.Rm[j], fc=c)
                ax.add_patch(patch)         
     
       
     
        
     
        #ax.plot(self.xt,self.yt,color='tab:red',linestyle='dashed',linewidth=0.5,zorder=0)
        #ax.plot(self.xt_desired,self.yt_desired,color='tab:red',linestyle='dashed',linewidth=0.5)        
        #ax.plot(self.xt2,self.yt2,color='tab:red',linestyle='dashed',linewidth=0.5,zorder=0) 
        #plt.title('Time = ' + str(np.round(self.time[i],1))+" s",fontsize=8)
        #plt.axis('off')
        #ax.axis('equal')
        plt.axis('off')        
        if tunnel_==True:
            w = 14.2
            l = 65
            gap = 30
            xpos1_wall = -52
            xpos2_wall = -52      
            px1 = xpos1_wall #+ w/2
            px2 = xpos2_wall #+ w/2
            py1 = gap
            py2 = -gap-l 

            rect1 = plt.Rectangle((px1, py1),w,l,facecolor="black", alpha=0.5,zorder=2)    
            ax.add_patch(rect1)

            rect2 = plt.Rectangle((px2, py2),w,l,facecolor="black", alpha=0.5,zorder=2)    
            ax.add_patch(rect2)       
            patch = plt.Circle((const*self.xm, const*self.zm),const*self.rm, fc="tab:red")
            ax.add_patch(patch)   
            
            # c1=0
            # c2=10            
            # x2=np.linspace(c1,c2,100)
            # x1=np.linspace(c1,c2,100)                
            # line1=np.tanh(x1-5)-3.75-.25
            # line2=np.tanh(x2-5)-2.25+.25 
            # plt.fill_between(np.flipud(x1),np.flipud(line2),color='tab:grey',alpha=1)
            # plt.fill_between(np.flipud(x1),np.flipud(line1),-6.25,color='tab:grey',alpha=1)            
            # plt.plot(np.flipud(x1),np.flipud(line2),color='k',linewidth=1)
            # plt.plot(np.flipud(x1),np.flipud(line1),color='k',linewidth=1)
            # ax.scatter(12, -2, s=80, c="tab:cyan", marker=(5, 1))       
        
            # c1=0
            # c2=10 
            # x2=np.linspace(c1,c2,1000)
            # x1=np.linspace(c1,c2,1000)                
            # line1=np.sin(1.745*x1)-3.75
            # line2=np.sin(1.745*x1)-2.0
            # plt.fill_between(x1,line2,color='tab:grey',alpha=1)
            # plt.fill_between(x1,line1,-6.25,color='tab:grey',alpha=1)      
            # plt.plot(x1,line1,color='k',linewidth=2)
            # plt.plot(x2,line2,color='k',linewidth=2)            
        
        
        
        
        
        
        
        # xticks = np.linspace(-50, 50,3,endpoint=True)
        # yticks = np.linspace(-50, 50,3,endpoint=True)        
        # #xticks = np.linspace(self.wxmin, self.wxmax,3,endpoint=True)
        # #yticks = np.linspace(self.wymin, self.wymax,3,endpoint=True)
        # ax.set_xticks(np.round(xticks,2))
        # ax.set_yticks(np.round(yticks,2))        
        # ax.xaxis.set_tick_params(width=.25,length=2)
        # ax.yaxis.set_tick_params(width=.25,length=2)
        # ax.set_xlim([self.wxmin,self.wxmax])
        # ax.set_ylim([self.wymin,self.wymax])
        # #ax.set_ylabel("$y$ (m)",size=6)
        # #ax.set_xlabel("$x$ (m)",size=6)
        # #ax.axis('equal')
        # #plt.tight_layout()
        # #plt.axis('scaled')
        # #plt.axis('square')
        # ax.set_aspect('equal', 'box')
        plt.savefig(self.mainDirectory+'/'+self.name+'/'+'time'+str(np.round(self.time[i],2))+'.jpg')
        plt.savefig(self.mainDirectory+'/'+self.name+'/'+'time'+str(np.round(self.time[i],2))+'.svg')    
        plt.savefig(self.mainDirectory+'/'+self.name+'/'+'time'+str(np.round(self.time[i],2))+'.pdf')
    
       # plt.close('all')   
       
     def create_snap_shot_field(self,entry,membrane,Z,X,Y):
        ''' Create snapshots  '''
        i=entry
        fig, ax = plt.subplots(nrows=1, ncols=1,figsize=(1.5,1.5),dpi=300)
        #plt.axis('off')
        ax = plt.axes(xlim=(self.wxmin,self.wxmax), ylim=(self.wymin, self.wymax))
        im1=ax.contourf(X,Y,Z,cmap = 'jet',levels=200,alpha=1,linestyles='solid')
        ax.contour(X,Y,Z,levels = 10,colors=('k',),linestyles=('-',),linewidths=(.1,))

        for j in range(0,self.nb):
            x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]  
            patch = plt.Circle((100*x0, 100*y0),100*self.bot_width/2, fc='black')
            ax.add_patch(patch)
            
        for j in range(self.ni):
            x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]

            if np.round(self.Rm[j],4)==0.0508:
                c='tab:blue'
            else:
                c='tab:green'
            patch = plt.Circle((100*x0, 100*y0),100*self.Rm[j], fc=c)
            ax.add_patch(patch)         
     
        #plt.title('Time = ' + str(np.round(self.time[i],1))+" s",fontsize=8)
        #ax.axis('equal')
        xticks = np.linspace(-50, 50,3,endpoint=True)
        yticks = np.linspace(-50, 50,3,endpoint=True) 
        ax.set_xticks(np.round(xticks,2))
        ax.set_yticks(np.round(yticks,2))        
        ax.xaxis.set_tick_params(width=.25,length=2)
        ax.yaxis.set_tick_params(width=.25,length=2)
        ax.set_xlim([self.wxmin,self.wxmax])
        ax.set_ylim([self.wymin,self.wymax])
        #ax.set_ylabel("$y$ (m)",size=6)
        #ax.set_xlabel("$x$ (m)",size=6)
        #ax.axis('equal')
        #plt.tight_layout()
        #plt.axis('scaled')
        #plt.axis('square')
        for c in im1.collections:
            c.set_edgecolor("face")
        cbar=fig.colorbar(im1, ax=ax,shrink=0.86,pad=0.01)
        cbar.set_ticks([0.0,0.2,0.4,0.6,0.8,1])
        cbar.ax.tick_params(labelsize=8,width=.25,pad=1,direction='out',length=1)          
        
        
        plt.gca().set_aspect('equal', adjustable='box')
        plt.savefig(self.mainDirectory+'/'+self.name+'/'+'time_field'+str(np.round(self.time[i],2))+'.jpg')
        plt.savefig(self.mainDirectory+'/'+self.name+'/'+'time_field'+str(np.round(self.time[i],2))+'.svg')    
        plt.savefig(self.mainDirectory+'/'+self.name+'/'+'time_field'+str(np.round(self.time[i],2))+'.pdf')
                
   
     def create_video(self):
         #import pdb    
         img_array = []
         for index, filename in enumerate(glob.glob(self.mainDirectory+'/'+self.name+'/_frames/'+'/*.jpg')):
             #pdb.set_trace()
             img = cv2.imread(filename)
             height, width, layers = img.shape
             size = (width,height)
             img_array.append(img)
         out = cv2.VideoWriter(self.mainDirectory+'/'+self.name+'/video.mp4',cv2.VideoWriter_fourcc(*'DIVX'), 15, (width,height))    

         for i in range(len(img_array)):
            out.write(img_array[i])
         out.release()          
      
         
      
        
      
        
      
     def create_snap_shot_tunnel(self,entry,const,membrane,tunnel_=False):
        ''' Create snapshots  '''
        i=entry
        ratio =(self.wymax-self.wymin)/(self.wxmax-self.wxmin)
        height_=2
        fig, ax = plt.subplots(figsize=(3.5,1.54),dpi=300)
        #plt.axis('off')
        ax = plt.axes(xlim=(self.wxmin,self.wxmax), ylim=(self.wymin, self.wymax))
        
        for i in entry:
            
            for j in range(0,self.nb):
                x0,y0=self.bot_position_x[j,i],self.bot_position_z[j,i]  
                patch = plt.Circle((-const*x0,const*y0),const*self.bot_width/2, fc='black')
                ax.add_patch(patch)
            if self.ni==0:
                pass
            else:
                                
                for j in range(self.ni):
                    x0,y0=self.particle_position_x[j,i],self.particle_position_z[j,i]
        
                    if np.round(self.Rm[j],4)==0.0508:
                        c='tab:blue'
                    else:
                        c='tab:green'
                    patch = plt.Circle((-const*x0, const*y0),const*self.Rm[j], fc=c)
                    ax.add_patch(patch)         
            ax.text(-const*x0, const*y0,str(np.round(self.time[i],1)),color="k")
    
            #plt.title('Time = ' + str(np.round(self.time[i],1))+" s",fontsize=8)
            #plt.axis('off')
            #ax.axis('equal')
            plt.axis('off')        
            if tunnel_==True:
                # w = 14.2
                # l = 65
                # gap = 30
                # xpos1_wall = -52
                # xpos2_wall = -52      
                # px1 = xpos1_wall #+ w/2
                # px2 = xpos2_wall #+ w/2
                # py1 = gap
                # py2 = -gap-l 
    
                # rect1 = plt.Rectangle((px1, py1),w,l,facecolor="black", alpha=0.5,zorder=2)    
                # ax.add_patch(rect1)
    
                # rect2 = plt.Rectangle((px2, py2),w,l,facecolor="black", alpha=0.5,zorder=2)    
                # ax.add_patch(rect2)       
                # patch = plt.Circle((const*self.xm, const*self.zm),const*self.rm, fc="tab:red")
                # ax.add_patch(patch)   
                
                # c1=0
                # c2=10            
                # x2=np.linspace(c1,c2,100)
                # x1=np.linspace(c1,c2,100)                
                # line1=np.tanh(x1-5)-3.75-.25
                # line2=np.tanh(x2-5)-2.25+.25 
                # plt.fill_between(np.flipud(x1),np.flipud(line2),color='tab:grey',alpha=1)
                # plt.fill_between(np.flipud(x1),np.flipud(line1),-6.25,color='tab:grey',alpha=1)            
                # plt.plot(np.flipud(x1),np.flipud(line2),color='k',linewidth=1)
                # plt.plot(np.flipud(x1),np.flipud(line1),color='k',linewidth=1)
                ax.scatter(12, -2, s=80, c="tab:cyan", marker=(5, 1))       
            
                c1=0
                c2=10 
                x2=np.linspace(c1,c2,1000)
                x1=np.linspace(c1,c2,1000)                
                line1=np.sin(1.745*x1)-4
                line2=np.sin(1.745*x1)-1.5
                plt.fill_between(np.flipud(x1),np.flipud(line2),color='tab:grey',alpha=1)
                plt.fill_between(np.flipud(x1),np.flipud(line1),-6.25,color='tab:grey',alpha=1)      
                #plt.plot(x1,line1,color='k',linewidth=2)
                #plt.plot(x2,line2,color='k',linewidth=2)           
            


        plt.savefig(self.mainDirectory+'/'+self.name+"/_tunnel_"+'.jpg')
        plt.savefig(self.mainDirectory+'/'+self.name+"/_tunnel_"+'.svg')    
        plt.savefig(self.mainDirectory+'/'+self.name+"/_tunnel_"+'.pdf')
    
       # plt.close('all')           
      
        
      
        
     # def create_video(self):         
     #    nsteps=len(self.time)
     #    tend=self.time[-2]
     #    FPS = 60
     #    #framesNum = int(FPS*tend)
     #    framesNum=len(self.time)
     #    fig = plt.figure(figsize=(6,6))
     #    #fig.set_size_inches(10, 10)
    
     #    ax = plt.axes(xlim=(self.wxmin,self.wxmax), ylim=(self.wymin, self.wymax))
     #    bots=[]
     #    for i in range(self.nb):
     #        x,y = self.bot_position_x[i,0],self.bot_position_z[i,0]
     #        patch = plt.Circle((x, y), self.bot_width/2, fc='k')
     #        bots.append(patch)
    
        
     #    def init():
     #        ax.grid(True)
     #        for i in range (0,len(bots)):
     #            ax.add_patch(bots[i])
                
     #        return []
    
     #    def animatePatches(i,bots):
     #        for j in range (0,len(bots)):
     #            x,y = self.bot_position_x[j,i],self.bot_position_z[j,i]
     #            bots[j].center=(x, y)
                
    
     #        return bots,
    
    
     #    def animationManage(i,bots):
     #        animatePatches(i,bots)
     #        plt.title('time= ' + str(np.round(self.time[i],3)))
     #        return []
    
     #    anim = animation.FuncAnimation(fig, animationManage,init_func=init,interval=100,fargs=(bots,))

     #    #plt.show()
     #    #return(HTML(anim.to_html5_video()))    