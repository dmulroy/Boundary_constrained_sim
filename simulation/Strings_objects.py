# -*- coding: utf-8 -*-
"""
Created on Fri Oct 16 16:03:14 2020

@author: dmulr
"""
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess
import os
import numpy as np
from numpy import savetxt
from scipy.interpolate import Rbf
from scipy.interpolate import RegularGridInterpolator
from matplotlib.patches import RegularPolygon
import csv
import random
import matplotlib.pyplot as plt
import math
from scipy import signal
#Robot object
class robot:
    def __init__(self,nb,radius,height,rowr,material,k,rl,body_floor,my_system,fixed,type_spring,obj,R,geom,xc,zc,skind,ratioM):
        # number of interior
        self.nb=nb 
        # starting positions x and z
        self.xc=xc ; self.zc=zc
        # spring rest legnth 
        self.rl=rl
        # diameter
        self.radius=radius
        #height
        self.height=height
        # density
        self.rowr=rowr
        # surface properties
        self.material=material
        # floor
        self.body_floor=body_floor
        # radius of ring
        self.R=R
        # robot position
        self.xb={}; self.yb={}; self.zb={}
        # membrane position
        self.xbm={}; self.ybm={}; self.zbm={}        
        # velocity position
        self.xvb={}; self.yvb={}; self.zvb={}
        # total force
        self.Ftxb={}; self.Ftyb={}; self.Ftzb={}
        
        self.Ftxcb={}; self.Ftycb={}; self.Ftzcb={}        
        # empty object arrays
        self.my_system=my_system
        self.bots=[]; self.Springs=[]
        self.obj=obj; self.force=[]
        # is object fixed
        self.fixed=fixed
        # Geometry of robots
        self.geom=geom
        self.Springs=[]
        '''
        # geometry of robots
        square: robot is cube  
        cylinder: robot is cylinder
        sphere : make them spheres
        '''


        # spring rleated things for membrane particles
        self.km=k; self.bm=5
        self.type_spring=type_spring
        # points for where each spirng is attatched
        self.p1=0; self.p2=self.radius/2
        self.p3=0; self.p4=-self.radius/2
        self.h=0; 
        self.mem=3 # membrane mode
        self.skinrho=4000 # membrane density
        self.skind = skind   # diameter of cylinders for skin particles
        self.ratioM = ratioM   # ratio of membrane skin particles to big bots
        self.skinM=[]        # empty matrix of membrane skin cylinders        
        self.countm=0
        # Colors
        col_y = chrono.ChColorAsset(); col_y.SetColor(chrono.ChColor(1, 1, 0))       # Yellow
        col_b = chrono.ChColorAsset(); col_b.SetColor(chrono.ChColor(0, 0, 1))       # Blue
        col_g = chrono.ChColorAsset(); col_g.SetColor(chrono.ChColor(0, 1, 0))       # Green
        col_p = chrono.ChColorAsset(); col_p.SetColor(chrono.ChColor(0.44, .11, 52)) # Purple
        col_w = chrono.ChColorAsset(); col_p.SetColor(chrono.ChColor(.8, .8, .8)) # Purple
        # data arrays of robots
        for i in range(self.nb):
            # positions
            self.xb["botx{0}".format(i)]=[]  #x position
            self.yb["boty{0}".format(i)]=[]  # y position
            self.zb["botz{0}".format(i)]=[]  # z position 
            
            # velocities
            self.xvb["botx{0}".format(i)]=[]  # x velocity
            self.yvb["boty{0}".format(i)]=[]  # y velocity
            self.zvb["botz{0}".format(i)]=[]  # z velocity
            
            # forces 
            self.Ftxb["botx{0}".format(i)]=[]  # Force x
            self.Ftyb["boty{0}".format(i)]=[]  # Force y
            self.Ftzb["botz{0}".format(i)]=[]  # force z
            
            self.Ftxcb["botx{0}".format(i)]=[]  # Force x
            self.Ftycb["boty{0}".format(i)]=[]  # Force y
            self.Ftzcb["botz{0}".format(i)]=[]  # force z            
        
            # postion 
            theta=i*2*np.pi/self.nb # set angle
            x=self.R*np.cos(theta)+self.xc  # set x positon 
            y=.5*height                     # set y position 
            z=self.R*np.sin(theta)+self.zc  # set z position 
            # create body
            #### Want the geometry of the robots to be cylinders
            if self.geom=="cylinder":
                
                bot = chrono.ChBodyEasyCylinder(self.radius, self.height,self.rowr,True,True)
                # set position
                bot.SetPos(chrono.ChVectorD(x,y,z)) 
                bot.SetName("bot"+str(i)) # set name (important for contact tracking)
                bot.SetId(i)              # set id   (impoortant for contact tracking )  
                # material 
                bot.SetMaterialSurface(self.material)  # set material 
                # rotate them so we can form the membrane
                rotation1 = chrono.ChQuaternionD() # rotate the robots about y axis 
                rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0)) 
                bot.SetRot(rotation1)
                
                
                ##### Add force components to each bot 
                # x forces 
                myforcex = chrono.ChForce()  # create it 
                bot.AddForce(myforcex) # apply it to bot object
                myforcex.SetMode(chrono.ChForce.FORCE) # set the mode 
                myforcex.SetDir(chrono.VECT_X) # set direction 
                self.force.append(myforcex) # add to force list
                
                # y forces    
                myforcey = chrono.ChForce() # create it 
                bot.AddForce(myforcey) # apply it to bot object
                myforcey.SetMode(chrono.ChForce.FORCE) # set the mode
                myforcey.SetDir(chrono.VECT_Y) # set direction 
                self.force.append(myforcey) # add to force list
                
                # z forces            
                myforcez = chrono.ChForce() # create it 
                bot.AddForce(myforcez) # apply it to bot object
                myforcez.SetMode(chrono.ChForce.FORCE) # set the mode
                myforcez.SetDir(chrono.VECT_Z) # set direction 
                self.force.append(myforcez) # add to force list
                
                # set max speed (Not always needed but it helps)
                bot.SetMaxSpeed(2)
                bot.SetLimitSpeed(False)


            #### if we want the robot to be a square 
            if self.geom=="square":
                bot = chrono.ChBodyEasyBox(2*self.radius,self.height,2*self.radius,self.rowr,True,True) # create object
                bot.SetPos(chrono.ChVectorD(x,y,z)) # set its position 
                bot.SetMaterialSurface(self.material) # apply material properties 
                bot.SetName('bot'+str(i))             # set name
                bot.SetId(i)                          # set id
                # rotate them
                rotation1 = chrono.ChQuaternionD()
                rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0)) 
                bot.SetRot(rotation1)
                
            #### If we want the robot to have sphere geometry 
            if self.geom=="sphere":
                bot = chrono.ChBodyEasySphere(self.diameter,self.rowr,True,True)  # create object
                bot.SetPos(chrono.ChVectorD(x,y,z)) # set position 
                bot.SetMaterialSurface(self.material) # apply material 
                bot.SetName('bot') # set name
                bot.SetId(i) # set id 
                # rotate them
                rotation1 = chrono.ChQuaternionD()
                rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0)) 
                bot.SetRot(rotation1)
            
            
            # make the color blue 
            bot.AddAsset(col_b)
            
            # zeroth robot so we know which is which in the videos
            if i==0:   
                bot.AddAsset(col_p)   
            if i>=20 and i<=25:
                bot.AddAsset(col_w)
            if i==22:   
                bot.AddAsset(col_y)  
            # set collision
            bot.SetCollide(True)
            # set fixed
            bot.SetBodyFixed(self.fixed)
            
            # link to floor
            pt=chrono.ChLinkMatePlane()
            pt.Initialize(self.body_floor,bot,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
            # add link to the system 
            self.my_system.AddLink(pt)
            
            # add bot to series of array 
            self.my_system.Add(bot) # add bot to system 
            self.bots.append(bot) # add bot to bot array 
            self.obj.append(bot) # add bot to obj array 
            
            # if no springs 
            if self.type_spring=="None":         
                print('None')
          
            # constant force spring ( Force is constant but again no pseudo membrane)
            if self.type_spring=="const" and self.mem!=3:
                
                # link springs
                if i>=1: # if we created at least two robots
                    ground=chrono.ChLinkSpring() # create spring 
                    ground.Initialize(self.bots[i-1], bot,True,chrono.ChVectorD(self.p1,self.h,self.p2), chrono.ChVectorD(self.p3,self.h,self.p4),False) # iniitizialize position 
                    ground.Set_SpringF(self.k) # set spring force
                    ground.Set_SpringRestLength(self.rl) # set resting length 
                    ground.AddAsset(col_b) # add color
                    ground.AddAsset(chrono.ChPointPointSpring(.01,80,15)) # create visual 
                    self.my_system.AddLink(ground) # add to system  
                    self.Springs.append(ground)  # add to spring array 
                    
                # Last spring
                if i==self.nb-1:  # last spring 
                    ground=chrono.ChLinkSpring() # create spring 
                    ground.Initialize(bot, self.bots[0], True, chrono.ChVectorD(self.p1,self.h,self.p2), chrono.ChVectorD(self.p3,self.h,self.p4),False) # set position 
                    ground.Set_SpringF(self.k) # set spring force 
                    ground.Set_SpringRestLength(self.rl) # set resting legnth 
                    ground.AddAsset(col_b) # add color 
                    ground.AddAsset(chrono.ChPointPointSpring(.01,80,15)) # create visual 
                    self.my_system.AddLink(ground) # add to system 
                    self.Springs.append(ground)    # add spring to spring array          
            
        
            #### (MODE 3) membrane particles (Creates a membrane)
            if self.mem==3 and self.type_spring!="None":
                b_ang=2*np.pi/self.nb                   # angle between centers of bots
                o_ang=np.arctan(self.radius/self.R)   # angle offset for radius of bot
                p_ang=np.arctan(self.skind/self.R)           # angle offset for radius of skin particle
                # Between this bot and last bot
                if i>=1 and i<self.nb:
                    for j in range(1,self.ratioM+1,1):
                        # Initial postion of each particle
                        theta=i*b_ang + j*(b_ang-o_ang-p_ang)/(self.ratioM) + p_ang 
                        x=self.R*np.cos(theta)+self.xc # x position 
                        y=self.height/2              # y position 
                        z=self.R*np.sin(theta)+self.zc # z position  
                        
                        # create them and set position
                        skinm = chrono.ChBodyEasyCylinder(self.skind/2, .95*self.height,self.skinrho,True,True) # create particle
                        skinm.SetPos(chrono.ChVectorD(x,y,z)) # set position 
                        skinm.SetMaterialSurface(self.material) # add material 
                        skinm.SetNoGyroTorque(True) # no gyro toruqe 
                        skinm.SetName('skin'+str(i)) # create name 
                        skinm.SetId(i) # set id 
                        
                        
                        # link to floor
                        pt=chrono.ChLinkMatePlane()
                        pt.Initialize(self.body_floor,skinm,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
                        
                        
                        # add link to the system 
                        self.my_system.AddLink(pt)
                        
                        
                        # rotate them bout y axis
                        rotation1 = chrono.ChQuaternionD()
                        rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0));  
                        skinm.SetRot(rotation1)
                        self.xbm["botmx{0}".format(self.countm)]=[]  #x position
                        self.ybm["botmy{0}".format(self.countm)]=[]  # y position
                        self.zbm["botmz{0}".format(self.countm)]=[]  # z position
                        self.countm=self.countm+1
                       
                        # Attach springs if more then one was created    
                        if j>1:
                            ground=chrono.ChLinkSpring() # create spring 1
                            p1=0; p2=self.skind/2 # points where each spring is attatched 
                            p3=0; p4=-self.skind/2
                            h=self.height/5
                            
                            ground.Initialize(self.skinM[-1], skinm,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),True) # link spring to particles
                            ground.Set_SpringK(self.km) # set spring constant
                            ground.Set_SpringR(self.bm) # set damping constant
                            ground.Set_SpringRestLength(self.rl) # set resting length 
                            ground.AddAsset(col_p) # add color 
                            ground.AddAsset(chrono.ChPointPointSpring(.01,80,15)) # create visual 
                            self.my_system.AddLink(ground) # add spring to system 
                            self.Springs.append(ground)
                            ground1=chrono.ChLinkSpring() # create spring 2
                            ground1.Initialize(self.skinM[-1], skinm,True,chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),True) # link spring to particles 
                            ground1.Set_SpringK(self.km) # set spring constant
                            ground1.Set_SpringR(self.bm) # set damping 
                            ground1.AddAsset(col_p) # create color 
                            ground1.AddAsset(chrono.ChPointPointSpring(.01,80,15)) # create visual 
                            self.my_system.AddLink(ground1) # add to the system                                 
                            self.Springs.append(ground1)
                        
                        #Link to cylinder 
                        if j==1:
                            skinm.AddAsset(col_p) # add color
                            glue=chrono.ChLinkMateFix() # cretae fix constraint
                            glue.Initialize(skinm,self.bots[i]) # fix object to bot
                            self.my_system.AddLink(glue) # add to system 
                            # Link last particle with this bot
                            if i>=2:
                                glue=chrono.ChLinkMateFix() # create the constraint 
                                glue.Initialize(self.skinM[-1],self.bots[-1]) # add constraint 
                                self.my_system.AddLink(glue) # add to the system 
                        if j==self.ratioM:
                            skinm.AddAsset(col_p)
                        self.my_system.Add(skinm)
                        self.skinM.append(skinm)
                    
                # Between this bot and first bot
                if i==self.nb-1:
                    for j in range(1,self.ratioM+1,1):
                        # Initial postion of each particle
                        theta=(i+1)*b_ang + j*(b_ang-o_ang-p_ang)/(self.ratioM) + p_ang
                        x=self.R*np.cos(theta)+self.xc
                        y=self.height/2
                        z=self.R*np.sin(theta)+self.zc
                        self.xbm["botmx{0}".format(self.countm)]=[]  #x position
                        self.ybm["botmy{0}".format(self.countm)]=[]  # y position
                        self.zbm["botmz{0}".format(self.countm)]=[]  # z position
                        self.countm=self.countm+1
                        # Create particles
                        #skinm = chrono.ChBodyEasySphere(self.skind/2,self.skinrho,True,True)
                        skinm = chrono.ChBodyEasyCylinder(self.skind/2, .95*self.height,self.skinrho,True,True)
                        skinm.SetPos(chrono.ChVectorD(x,y,z))
                        skinm.SetMaterialSurface(self.material)
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
                        # Attach springs    
                        if j>1:
                            ground=chrono.ChLinkSpring()
                            p1=0; p2=self.skind/2
                            p3=0; p4=-self.skind/2
                            h=self.height/5
                    
                            ground.Initialize(self.skinM[-1], skinm,True,chrono.ChVectorD(p1,h,p2), chrono.ChVectorD(p3,h,p4),True)
                            ground.Set_SpringK(self.km)
                            ground.Set_SpringR(self.bm)
                            ground.Set_SpringRestLength(self.rl)
                            ground.AddAsset(col_y)
                            ground.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                            self.my_system.AddLink(ground)
                            self.Springs.append(ground)
                            ground1=chrono.ChLinkSpring()
                            ground1.Initialize(self.skinM[-1], skinm,True,chrono.ChVectorD(p1,-h,p2), chrono.ChVectorD(p3,-h,p4),True)
                            ground1.Set_SpringK(self.km)
                            ground1.Set_SpringR(self.bm)
                            ground.Set_SpringRestLength(self.rl) # set resting length 
                            ground1.AddAsset(col_y)
                            ground1.AddAsset(chrono.ChPointPointSpring(.01,80,15))
                            self.my_system.AddLink(ground1)  
                            self.Springs.append(ground1)
                        #Link to cylinder
                        if j==1:
                            skinm.AddAsset(col_p)
                            glue=chrono.ChLinkMateFix()
                            glue.Initialize(skinm,self.bots[0])
                            self.my_system.AddLink(glue)
                            glue=chrono.ChLinkMateFix()
                            glue.Initialize(self.skinM[-1],self.bots[0])
                            self.my_system.AddLink(glue)
                         
                        if j==self.ratioM:
                            skinm.AddAsset(col_p)
                            glue=chrono.ChLinkMateFix()
                            glue.Initialize(skinm,self.bots[1])
                            self.my_system.AddLink(glue)
                            
                        self.my_system.Add(skinm)
                        self.skinM.append(skinm)

    # return system (Helps with adding to the data extractor, simulaor, and controllers)
    def return_system(self):
        ''' Return system, springs, bots, obj, force '''
        return(self.my_system,self.Springs,self.bots,self.obj,self.force)

    # save position data
    def save_data_position(self):
        ''' Save position of each bot '''
        for i in range(self.nb):
            self.xb['botx'+str(i)].append(self.bots[i].GetPos().x)
            self.yb['boty'+str(i)].append(self.bots[i].GetPos().y)
            self.zb['botz'+str(i)].append(self.bots[i].GetPos().z)
            
        for i in range(self.countm):
            self.xbm['botmx'+str(i)].append(self.skinM[i].GetPos().x)
            self.ybm['botmy'+str(i)].append(self.skinM[i].GetPos().y)
            self.zbm['botmz'+str(i)].append(self.skinM[i].GetPos().z)
            
    # save velocity data
    def save_data_velocity(self):
        ''' save velocity of each bot '''
        for i in range(self.nb):
            self.xvb['botx'+str(i)].append(self.bots[i].GetPos_dt().x)
            self.yvb['boty'+str(i)].append(self.bots[i].GetPos_dt().y)
            self.zvb['botz'+str(i)].append(self.bots[i].GetPos_dt().z)    
            
    # save force data            
    def save_data_Forces(self):
        ''' Save total force on each bot '''
        for i in range(self.nb):
            self.Ftxb["botx"+str(i)].append(self.bots[i].GetContactForce().x)
            self.Ftyb["boty"+str(i)].append(self.bots[i].GetContactForce().y)
            self.Ftzb["botz"+str(i)].append(self.bots[i].GetContactForce().z)
    
    def save_data_Forces_contact(self):
        ''' Save total force on each bot '''
        for i in range(self.nb):
            self.Ftxcb["botx"+str(i)].append(self.bots[i].Get_Xforce().x)
            self.Ftycb["boty"+str(i)].append(self.bots[i].Get_Xforce().y)
            self.Ftzcb["botz"+str(i)].append(self.bots[i].Get_Xforce().z)
            
            
    # return position data
    def return_position_data(self):
        ''' return the dictionary of each bot '''
        return(self.xb,self.yb,self.zb)
    
    # return position membrane data
    def return_position_membrane_data(self):
        ''' return the dictionary of each bot '''
        return(self.xbm,self.ybm,self.zbm)  
      
    # return velocity data
    def return_velocity_data(self):
        ''' return the diction of each bot velocity '''
        return(self.xvb,self.yvb,self.zvb)
        
    # return force data    
    def return_force_data(self):
        ''' return dictionary of each bot forces '''
        return(self.Ftxb,self.Ftyb,self.Ftzb)
  
    # return force data    
    def return_force_data_contact(self):
        ''' return dictionary of each bot forces '''
        return(self.Ftxcb,self.Ftycb,self.Ftzcb)
    
    
    
    # return last position
    def return_last_position(self):
        ''' return the last position of each bot '''
        for i in range(self.nb):
            self.XL.append(self.xb['botx'+str(i)][-1])
            self.ZL.append(self.zb['botz'+str(i)][-1])
        np.savez("points.npz",allow_pickle=True,XL=self.XL,ZL=self.ZL)
        return(self.XL,self.ZL)        
    

#Interior Particles
'''
Interior Particles
'''
class Interiors:
    def __init__(self,nb,radius,rowp,height,my_system,obj,body_floor,material,fixed,mode,R,xc,zc,n,sim):
        
        self.xc=xc # x positon off set if we dont want it to start at (0,0)
        self.zc=zc # z positon off set if we dont want it to start at (0,0)
        self.radius=radius # interior particle radius
        self.nb=nb  # number of robots   
        self.sim=sim
        self.R=R 
        self.path='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/'
        self.Rm=[]
        self.rowp=rowp # density of particle
        self.height=height # height of particle
        self.particles=[] # empty array to store them all in 
        self.obj=obj # object array all the robots and particles are being stored in 
        self.body_floor=body_floor # this is the floor
        self.my_system=my_system # my system 
        self.material=material # material 
        self.fixed=True # fixed if True the particles are fixed to the ground 
        self.off=0.0
        # empty position dictionaries
        self.xp={}; self.yp={}; self.zp={}

        # empty velocity dictionaries 
        self.xvp={}; self.yvp={}; self.zvp={}
        
        # empty forces dictionaries
        self.Ftxp={}; self.Ftyp={}; self.Ftzp={}
        self.Pphi={}; self.Ptheta={}; self.Ppsi={}
        self.mode=mode # granular mode 
        
        # colors
        col_y = chrono.ChColorAsset(); col_y.SetColor(chrono.ChColor(1, 1, 0))       # Yellow
        col_b = chrono.ChColorAsset(); col_b.SetColor(chrono.ChColor(0, 0, 1))       # Blue
        col_g = chrono.ChColorAsset(); col_g.SetColor(chrono.ChColor(0, 1, 0))       # Green
        col_p = chrono.ChColorAsset(); col_p.SetColor(chrono.ChColor(0.44, .11, 52)) # Purple
        
        # no interiors
        if self.mode=='empty':
            self.ni=0
            self.n=np.array([0])
        
        # not max interiors
        if self.mode=="nmax":
            (self.n)=n
            (self.ni)=np.sum(self.n)
        
        # max non homo interiors
        if self.mode=="nonhnmax":
            (self.n)=n # array of number of interior at each ring
            (self.ni)=np.sum(self.n) # sum up the interiors

        

        #### tunnel verify
        if self.mode=="tunnel_verify":
            count=0
            data=np.load("tunnel_start_positions.npz")
            xx=data['X']
            zz=data['Y']
            Rr=data['R']
            Xc=data['Xc']
            Yc=data['Yc']
            self.ni=len(Rr)
            print(count)
            self.n=0
            for i in range(self.ni):        
                self.radius2=Rr[i]*.01
                if Rr[i]==5.08:
                    con='b'
                    const=0
                else:
                    con='a'
                    const=0
                        
                # empty arrays of variables
                print(count)
                self.xp["gransx{0}".format(count)]=[]
                self.yp["gransy{0}".format(count)]=[]
                self.zp["gransz{0}".format(count)]=[]
                self.xvp["gransvx{0}".format(count)]=[]
                self.yvp["gransvy{0}".format(count)]=[]
                self.zvp["gransvz{0}".format(count)]=[]
                self.Ftxp["gransFx{0}".format(count)]=[]
                self.Ftyp["gransFy{0}".format(count)]=[]
                self.Ftzp["gransFz{0}".format(count)]=[]   
                count=count+1    
                y=.5*self.height
                self.Rm.append(self.radius2)
                # create body
                gran = chrono.ChBody()
                gran = chrono.ChBodyEasyCylinder(self.radius2-self.off, self.height,self.rowp,True,True)
                gran.SetPos(chrono.ChVectorD((xx[i]-Xc)*.01,y,(zz[i]-Yc)*.01))
                gran.SetMaterialSurface(self.material)
                gran.SetName('gran'+str(con)+str(count))
                gran.SetId(i)
                gran.SetCollide(False)
                gran.SetBodyFixed(self.fixed)
                pt=chrono.ChLinkMatePlane()
                pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
                self.my_system.AddLink(pt)
                self.my_system.Add(gran)
                self.obj.append(gran)
                self.particles.append(gran)              
           
        #### grasp verify
        if self.mode=="grasp_verify":
            count=0
            data=np.load("grasp_start_positions.npz")
            xx=data['X']
            zz=data['Y']
            Rr=data['R']
            Xc=data['Xc']
            Yc=data['Yc']
            self.ni=len(Rr)
            print(count)
            self.n=0
            for i in range(self.ni):
                        
                self.radius2=Rr[i]*.01
                if Rr[i]==5.08:
                    con='b'
                    const=0
                else:
                    con='a'
                    const=0
                        
                # empty arrays of variables
                print(count)
                self.xp["gransx{0}".format(count)]=[]
                self.yp["gransy{0}".format(count)]=[]
                self.zp["gransz{0}".format(count)]=[]
                self.xvp["gransvx{0}".format(count)]=[]
                self.yvp["gransvy{0}".format(count)]=[]
                self.zvp["gransvz{0}".format(count)]=[]
                self.Ftxp["gransFx{0}".format(count)]=[]
                self.Ftyp["gransFy{0}".format(count)]=[]
                self.Ftzp["gransFz{0}".format(count)]=[]   

                count=count+1    
                y=.5*self.height
                self.Rm.append(self.radius2)
                # create body
                gran = chrono.ChBody()
                gran = chrono.ChBodyEasyCylinder(self.radius2-self.off, self.height,self.rowp,True,True)
                gran.SetPos(chrono.ChVectorD((xx[i])*.01,y,(zz[i]-Yc)*.01))
                gran.SetMaterialSurface(self.material)
                gran.SetName('gran'+str(con)+str(count))
                gran.SetId(i)
                gran.SetCollide(False)
                gran.SetBodyFixed(self.fixed)
                pt=chrono.ChLinkMatePlane()
                pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
                self.my_system.AddLink(pt)
                    

                self.my_system.Add(gran)
                self.obj.append(gran)
                self.particles.append(gran)              


        #### verify 
        if self.mode=="target_verify":
            count=0
            data=np.load("target_track_start_position.npz")
            xx=data['X']
            zz=data['Y']
            Rr=data['R']
            print(Rr)
            Xc=data['Xc']
            Yc=data['Yc']
            self.ni=len(Rr)
            print(count)
            self.n=0
            for i in range(self.ni):
                        
                self.radius2=Rr[i]*.0254
                if Rr[i]==2:
                    con='b'
                    const=0
                else:
                    con='a'
                    const=0
                        
                # empty arrays of variables
                print(count)
                self.xp["gransx{0}".format(count)]=[]
                self.yp["gransy{0}".format(count)]=[]
                self.zp["gransz{0}".format(count)]=[]
                self.xvp["gransvx{0}".format(count)]=[]
                self.yvp["gransvy{0}".format(count)]=[]
                self.zvp["gransvz{0}".format(count)]=[]
                self.Ftxp["gransFx{0}".format(count)]=[]
                self.Ftyp["gransFy{0}".format(count)]=[]
                self.Ftzp["gransFz{0}".format(count)]=[]   

                count=count+1    
                y=.5*self.height
                self.Rm.append(self.radius2)
                # create body
                gran = chrono.ChBody()
                gran = chrono.ChBodyEasyCylinder(self.radius2-self.off, self.height,self.rowp,True,True)
                gran.SetPos(chrono.ChVectorD(xx[i]*.01 + self.xc,y,zz[i]*.01 + self.zc))
                gran.SetMaterialSurface(self.material)
                gran.SetName('gran'+str(con)+str(count))
                gran.SetId(i)
                gran.SetCollide(False)
                gran.SetBodyFixed(self.fixed)
                pt=chrono.ChLinkMatePlane()
                pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
                self.my_system.AddLink(pt)
                    

                self.my_system.Add(gran)
                self.obj.append(gran)
                self.particles.append(gran)  

        
        #### mono-dispersion 
        if self.mode=="nmax":  
            count=0
            for i in range(self.n.size):
                for j in range(self.n[i]):
                     
                    self.xp["gransx{0}".format(count)]=[] # x position  
                    self.yp["gransy{0}".format(count)]=[] # y position 
                    self.zp["gransz{0}".format(count)]=[] # z position 
                    self.xvp["gransvx{0}".format(count)]=[] # x velocity
                    self.yvp["gransvy{0}".format(count)]=[] # y velocity
                    self.zvp["gransvz{0}".format(count)]=[] # z velocity
                    self.Ftxp["gransFx{0}".format(count)]=[] # total force x
                    self.Ftyp["gransFy{0}".format(count)]=[] # total force y
                    self.Ftzp["gransFz{0}".format(count)]=[] # total force z
                    count=count+1
                    # temporary radius
                    R2=self.radius*self.n[i]/(np.pi)
                    # position
                    x=R2*np.cos(j*2*np.pi/self.n[i])+self.xc # x position 
                    y=.5*self.height                         # y position 
                    z=R2*np.sin(j*2*np.pi/self.n[i])+self.zc # z position 
                    # create granular
                    gran = chrono.ChBodyEasyCylinder(self.radius-self.off, self.height,self.rowp,True,True)
                    # rotate them
                    # rotation1 = chrono.ChQuaternionD()
                    # rotation1.Q_from_AngAxis(-theta, chrono.ChVectorD(0, 1, 0)) 
                    # gran.SetRot(rotation1)
                    # gran = chrono.ChBodyEasySphere(self.radius,self.rowp,True,True)
                    # gran.SetPos(chrono.ChVectorD(x,y,z))   # add position 
                    gran.SetMaterialSurface(self.material) # add material 
                    gran.SetName("grana"+str(i))                   # set name 
                    gran.SetId(i)                          # add id 
                    gran.SetCollide(True)                  # create collision   
                    gran.SetBodyFixed(self.fixed)          # Add body fixed 
                    
                    # add color
                    col_r = chrono.ChColorAsset()          # apply color 
                    col_r.SetColor(chrono.ChColor(1, 0, 0)) 
                    gran.AddAsset(col_r)
                    
                    # mate to floor
                    pt=chrono.ChLinkMatePlane() 
                    pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,-1, 0),chrono.ChVectorD(0,1, 0))
                    self.my_system.AddLink(pt)
                    
                    # set speed limit ( Helps but not always needed)
                    #gran.SetMaxSpeed(2)
                    #gran.SetLimitSpeed(False)
                    
                    # add to system
                    self.my_system.Add(gran) # add object to system 
                    self.obj.append(gran)# add granular to object
                    self.particles.append(gran)   
                    
        #### nonhnmax             
        if self.mode=="nonhnmax": 
            count=0
            for i in range(len(self.n)):
                print("i=",str(i))
                if i%2==0:
                    self.radius2=(self.radius)*(2**.5)
                    con='b'
                    const=0
                else:
                    self.radius2=self.radius
                    con='a'
                    const=0
                R2=self.radius2*self.n[i]/(np.pi) + const
                
                # empty arrays of variables
                for j in range(self.n[i]):
                    self.xp["gransx{0}".format(count)]=[]
                    self.yp["gransy{0}".format(count)]=[]
                    self.zp["gransz{0}".format(count)]=[]
                    self.xvp["gransvx{0}".format(count)]=[]
                    self.yvp["gransvy{0}".format(count)]=[]
                    self.zvp["gransvz{0}".format(count)]=[]
                    self.Ftxp["gransFx{0}".format(count)]=[]
                    self.Ftyp["gransFy{0}".format(count)]=[]
                    self.Ftzp["gransFz{0}".format(count)]=[]    
                    self.Pphi["gransPphi{0}".format(count)]=[]
                    self.Ptheta["gransPtheta{0}".format(count)]=[]
                    self.Ppsi["gransPpsi{0}".format(count)]=[]  
                    
                    #R2=self.radius2*self.n[i]/(np.pi) + const# raidus of ring 
                    # x,y,z positions
                    x=R2*np.cos(j*2*np.pi/self.n[i])+self.xc
                    y=.5*self.height
                    z=R2*np.sin(j*2*np.pi/self.n[i])+self.zc
                    print("j=",str(j),str(np.round(self.radius2,3)),"x,y",str(np.round(x,2)),str(np.round(z,2)))
                    self.Rm.append(self.radius2)
                    # create body
                    gran = chrono.ChBody()
                    #gran = chrono.ChBodyEasySphere(self.radius2-self.off,self.rowp,True,True)
                    const=.5*np.pi*(self.radius2-self.off)
                    #gran = chrono.ChBodyEasyBox(const,self.height,const,self.rowp,True,True) # create object # specify properties and make it a cylinder
                    gran = chrono.ChBodyEasyCylinder(self.radius2-self.off, self.height,self.rowp,True,True)
                    
    
               
                    gran.SetPos(chrono.ChVectorD(x,y,z))
                    gran.SetMaterialSurface(self.material)
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
                    
                    # link to plane    
                    pt=chrono.ChLinkMatePlane()
                    pt.Initialize(self.body_floor,gran,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
                    self.my_system.AddLink(pt)
                    
                    # add to system
                    self.my_system.Add(gran)
                    self.obj.append(gran)
                    self.particles.append(gran)
        np.savez(self.path+'radius/'+'R'+self.sim+'.npz',Rm=self.Rm)      

    def save_angle_data(self):
        for i in range(self.ni):
            temp=self.particles[i].GetRot()  
            q0=temp.e0
            q1=temp.e1
            q2=temp.e2
            q3=temp.e3        
            self.Pphi["gransPphi"+str(i)].append(np.arctan2(2*(q0*q1+q2*q3),1-2*(q1**2 +q2**2)))
            self.Ptheta["gransPtheta"+str(i)].append(math.asin(2*(q0*q2-q3*q1)))
            self.Ppsi["gransPpsi"+str(i)].append(np.arctan2(2*(q0*q3+q1*q2),1-2*(q2**2 +q3**2)))
            
            
            
    def return_angle_data(self):
        ''' return angle information '''
        return(self.Pphi,self.Ptheta,self.Ppsi)    
    
    # return system
    def return_system(self):
        ''' Return objects from the interior particles '''
        return(self.my_system,self.particles,self.obj)
    
    # save position data
    def save_data_position(self):
        ''' Save postions of the interior '''
        for i in range(self.ni):
            self.xp["gransx"+str(i)].append(self.particles[i].GetPos().x) # save x position
            self.yp["gransy"+str(i)].append(self.particles[i].GetPos().y) # save y position
            self.zp["gransz"+str(i)].append(self.particles[i].GetPos().z) # save z position
        return(self.xp,self.yp,self.zp)
        
    # save velocity data
    def save_data_velocity(self):
        ''' save velocities of the interiors '''
        for i in range(self.ni):                 
            self.xvp['gransvx'+str(i)].append(self.particles[i].GetPos_dt().x) # save x velocity
            self.yvp['gransvy'+str(i)].append(self.particles[i].GetPos_dt().y) # save y velocity
            self.zvp['gransvz'+str(i)].append(self.particles[i].GetPos_dt().z) # save z velocity    
            
    # save force data            
    def save_data_Forces(self):
        ''' Save the total forces on the interiors'''
        for i in range(self.ni):
            self.Ftxp["gransFx"+str(i)].append(self.particles[i].Get_Xforce().x) # save x force
            self.Ftyp["gransFy"+str(i)].append(self.particles[i].Get_Xforce().y) # save y force
            self.Ftzp["gransFz"+str(i)].append(self.particles[i].Get_Xforce().z) # save z force  
    
  # return position data      
    def return_position_data(self):
        ''' Return the positon data '''
        return(self.xp,self.yp,self.zp)
        
    # return velocity data
    def return_velocity_data(self):
        ''' return the velocity data '''
        return(self.xvp,self.yvp,self.zvp)
        
    # return force data    
    def return_force_data(self):
        ''' return the force data '''
        return(self.Ftxp,self.Ftyp,self.Ftzp)   
    
    
'''
Creates a ball for grabbing 
'''  
class Ball:
    def __init__(self,control_type,my_system,body_floor,obj,material,height=None,R=None,rho=None,zball=None,xball=None):
        self.control_type=control_type # control type
        self.my_system=my_system # the system 
        self.body_floor=body_floor # the body floor
        self.obj=obj # array of objects 
        self.material=material # material properties
        self.balls=[] # empty array of balls in case we make multiple
        self.ballx=[]; self.ballz=[] 
        self.Fballx=0; self.Fballz=0
        self.nsides=3 # nuber of sides if its a poylggon shaped
        self.bforce=[]
        self.geom="verify"
        self.fixed=True
        self.path='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/shapes/'
        self.bx=[]
        self.by=[]
        self.bz=[]
        self.bvx=[]
        self.bvy=[]
        self.bvz=[]
        self.bFx=[]
        self.bFy=[]
        self.bFz=[]
        self.bphi=[]
        self.btheta=[]
        self.bpsi=[]
        self.height=height
        self.radius=R
        self.rho=rho
        self.x=xball
        self.y=self.height/2
        self.z=zball
        self.forceb=[]
        z2x = chrono.ChQuaternionD()
        z2x.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 1, 0))
        z2y = chrono.ChQuaternionD()
        z2y.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 0, 1))
        
        if self.geom=='verify':
        #Create ball
            ball = chrono.ChBody() # create ball object
            ball = chrono.ChBodyEasyCylinder(self.radius, self.height,self.rho,True,True) # specify properties and make it a cylinder
            # set position
            ball.SetPos(chrono.ChVectorD(self.x,self.y,self.z))
            ball.SetName("ball") # give it a name
            ball.SetId(10000) 
            ball.SetCollide(True) # set the collision mode
            ball.SetBodyFixed(True) # set if its fixed
            body_floor_texture = chrono.ChTexture()
            body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'bluwhite.png')
            ball.GetAssets().push_back(body_floor_texture)
            # material
            ball.SetMaterialSurface(self.material) # apply material
            #col_y = chrono.ChColorAsset() # apply color
            #col_y.SetColor(chrono.ChColor(1, 1, 0))
            #ball.AddAsset(col_y)            
            
        if self.geom=="circle":
        #Create ball
            ball = chrono.ChBody() # create ball object
            ball = chrono.ChBodyEasyCylinder(self.radius, self.height,self.rho,True,True) # specify properties and make it a cylinder
            # set position
            ball.SetPos(chrono.ChVectorD(self.x,self.y,self.z))
            ball.SetName("ball") # give it a name
            ball.SetId(10000) 
            ball.SetCollide(True) # set the collision mode
            ball.SetBodyFixed(self.fixed) # set if its fixed
            body_floor_texture = chrono.ChTexture()
            body_floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'bluwhite.png')
            ball.GetAssets().push_back(body_floor_texture)
            # material
            ball.SetMaterialSurface(self.material) # apply material
            #col_y = chrono.ChColorAsset() # apply color
            #col_y.SetColor(chrono.ChColor(1, 1, 0))
            #ball.AddAsset(col_y)

        if self.geom=="square":
        #Create ball
            ball = chrono.ChBody() # create ball object
            const=.5*np.pi*self.radius
            ball = chrono.ChBodyEasyBox(const,self.height,const,self.rho,True,True) # create object # specify properties and make it a cylinder
            # set position
            ball.SetPos(chrono.ChVectorD(self.x,self.y,self.z))
            ball.SetName("ball") # give it a name
            ball.SetCollide(True) # set the collision mode
            ball.SetBodyFixed(self.fixed) # set if its fixed
            # material
            ball.SetMaterialSurface(self.material) # apply material
            col_y = chrono.ChColorAsset() # apply color
            col_y.SetColor(chrono.ChColor(1, 1, 0))
            ball.AddAsset(col_y)

        if self.geom=="polygon":
        # create points for convex hull
            pt_vect = chrono.vector_ChVectorD()
            const=.4627
            # creates bottom
            for i in range(self.nsides):
                pt_vect.push_back(chrono.ChVectorD((const)*np.sin(i*2*np.pi/self.nsides),self.height/2,(const)*np.cos(i*2*np.pi/self.nsides)))
                #create top 
            for i in range(self.nsides):
                pt_vect.push_back(chrono.ChVectorD((const)*np.sin(i*2*np.pi/self.nsides),-self.height/2,(const)*np.cos(i*2*np.pi/self.nsides)))
        
            ball=chrono.ChBodyEasyConvexHull(pt_vect,self.rho,True,True)   
            ball.SetPos(chrono.ChVectorD(self.x,self.y,self.z))
            ball.SetName("ball") # give it a name
            ball.SetCollide(True) # set the collision mode
            ball.SetBodyFixed(self.fixed) # set if its fixed
            ball.GetCollisionModel().ClearModel()
            ball.GetCollisionModel().AddConvexHull(pt_vect)
            ball.GetCollisionModel().BuildModel()
            print(ball.GetMass())
     
        if self.geom=="star":

            ball = chrono.ChBody()
            ball.SetPos(chrono.ChVectorD(self.x,.05,self.z))
            ball.SetMass(30)
            ball.SetInertiaXX(chrono.ChVectorD(0.0344868, 0.0344869, 0.0683785))
            ball.SetInertiaXY(chrono.ChVectorD(0,0,0)) 
            
            # Attach a visualization shape .
            # First load a .obj from disk into a ChTriangleMeshConnected:
            mesh_for_visualization = chrono.ChTriangleMeshConnected()
            mesh_for_visualization.LoadWavefrontMesh(('shapes/star34.obj'))
            mesh_for_visualization.Transform(chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(1))
            visualization_shape = chrono.ChTriangleMeshShape()
            visualization_shape.SetMesh(mesh_for_visualization)
            ball.AddAsset(visualization_shape)
            
            mesh_for_collision = chrono.ChTriangleMeshConnected()
            mesh_for_collision.LoadWavefrontMesh(('shapes/star34.obj'))
            # Optionally: you can scale/shrink/rotate the mesh using this:
            mesh_for_collision.Transform(chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(1))
            ball.GetCollisionModel().ClearModel()
            ball.GetCollisionModel().AddTriangleMesh(
            mesh_for_collision, # the mesh 
            False,  # is it static?
            False)  # is it convex?
            ball.GetCollisionModel().BuildModel()                  
            ball.SetBodyFixed(self.fixed)
            col_y = chrono.ChColorAsset() # apply color
            col_y.SetColor(chrono.ChColor(1, 1, 0))
            ball.AddAsset(col_y)
            ball.SetCollide(True) # set the collision mode
            print(ball.GetMass())

        #z forces            
        myforcez = chrono.ChForce() # create it 
        ball.AddForce(myforcez) # apply it to bot object
        myforcez.SetMode(chrono.ChForce.FORCE) # set the mode
        myforcez.SetDir(chrono.VECT_Z) # set direction 
        myforcez.SetVpoint(chrono.ChVectorD(0,0.05,0))
        self.forceb.append(myforcez) # add to force list

        #create constraint to fix it to the floor
        
        pt=chrono.ChLinkMatePlane() 
        pt.Initialize(self.body_floor,ball,False,chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,0,0),chrono.ChVectorD(0,1, 0),chrono.ChVectorD(0,-1, 0))
        #append the constraint and the ball to array of objects and system
        prismatic_ground_ball = chrono.ChLinkLockPrismatic()
        prismatic_ground_ball.SetName("prismatic_ground_ball")
        prismatic_ground_ball.Initialize(self.body_floor, ball, chrono.ChCoordsysD(chrono.ChVectorD(self.x, self.y, self.z), chrono.Q_ROTATE_X_TO_Y))
        my_system.AddLink(prismatic_ground_ball)
        
        self.my_system.Add(prismatic_ground_ball) 
        
        self.my_system.AddLink(pt)
        self.obj.append(ball)
        self.balls.append(ball) 
        self.my_system.Add(ball)
            
        
            
        
    def save_data_position(self):
        ''' Function that saves the positon of the ball '''
        self.bx.append(self.balls[0].GetPos().x) # x postion 
        self.bz.append(self.balls[0].GetPos().z) # z postion 
    

    def save_data_velocity(self):
        ''' save the velocity of the ball '''
        self.bvx.append(self.balls[0].GetPos_dt().x) # x velocity
        self.bvz.append(self.balls[0].GetPos_dt().z)# z velocity 
        
    def save_contact_force(self):
        ''' save contact forces  '''
        self.bFx.append(self.balls[0].GetContactForce().x) # x contact force
        self.bFy.append(self.balls[0].GetContactForce().y) # y contact force
        self.bFz.append(self.balls[0].GetContactForce().z) # z contact force
     
        
     
    def save_angle_data(self):
        temp=self.balls[0].GetRot()  
        q0=temp.e0
        q1=temp.e1
        q2=temp.e2
        q3=temp.e3        
        self.bphi.append(0)
        self.btheta.append(0)
        self.bpsi.append(0)        
    
        
      
        
    def return_angle_data(self):
        ''' return angle information '''
        return(self.bphi,self.btheta,self.bpsi)
    
    
    def return_position_data(self):
        ''' return ball position '''
        return(self.bx,self.bz)
    
    def return_force_data(self):
        ''' return force data of ball '''
        return(self.bFx,self.bFz)
    
    def return_velocity_data(self):
        ''' return velocity data '''
        return(self.bvx,self.bvz)    

    # return system    
    def return_system(self):
        ''' return system  '''
        return(self.my_system,self.forceb)    

#Create material
def Material(mu_f,mu_b,mu_r,mu_s,C,Ct,Cr,Cs):
    ''' Function that creates material object '''
    material = chrono.ChMaterialSurfaceNSC() # create material object
    material.SetFriction(mu_f) # set friction properties
    material.SetDampingF(mu_b) # set damping properties
    material.SetCompliance (C) # set compliance property
    material.SetComplianceT(Ct) # set tangential property
    # material.SetRollingFriction(mu_r)
    # material.SetSpinningFriction(mu_s)
    # material.SetComplianceRolling(Cr)
    # material.SetComplianceSpinning(Cs)
    return material

# environment           
class enviroment:
    ''' class that creates enviroment ie floor tunnel obstactles etc '''
    def __init__(self,my_system,material,material2,length,tall,height2,env_mode=None,gapw=None,R=None,X=None,Z=None,px=None,py=None):
        self.my_system=my_system
        self.material=material
        self.material2=material2
        self.length=length
        self.tall=tall
        self.R=R
        self.X=X
        self.Z=Z
        self.radius=1
        self.height=.06
        self.height2=height2
        self.rho=1000
        self.y=self.height/2
        self.Obst=[]
        self.px=px
        self.py=py
        self.path='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/shapes/'
        self.wall=[]
        #self.height=height
        if env_mode is not None:
            self.env_mode=env_mode
        else:
            self.env_mode=None
        if gapw is not None:
            self.gapw=gapw
        else:
            self.gapw=None    
            
            
        self.body_floor = chrono.ChBody()
        self.body_floor.SetName('floor')
        self.body_floor.SetBodyFixed(True)
        self.body_floor.SetPos(chrono.ChVectorD(0, -self.tall, 0 ))
        self.body_floor.SetMaterialSurface(self.material)
        self.body_floor.GetCollisionModel().ClearModel()
        self.body_floor.GetCollisionModel().AddBox(self.length, self.tall, self.length) # hemi sizes
        self.body_floor.GetCollisionModel().BuildModel()       
        self.body_floor.SetCollide(False)
        body_floor_shape = chrono.ChBoxShape()
        body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(self.length, self.tall, self.length)
        self.body_floor.GetAssets().push_back(body_floor_shape)
        col_g = chrono.ChColorAsset()
        col_g.SetColor(chrono.ChColor(0, 0, 0))
        self.body_floor.AddAsset(col_g)
        self.my_system.Add(self.body_floor)
        
        
        self.body_floor = chrono.ChBody()
        self.body_floor.SetName('floor2')
        self.body_floor.SetBodyFixed(True)
        self.body_floor.SetPos(chrono.ChVectorD(0, self.tall+self.height2, 0 ))
        self.body_floor.SetMaterialSurface(self.material2)
        self.body_floor.GetCollisionModel().ClearModel()
        self.body_floor.GetCollisionModel().AddBox(self.length, self.tall, self.length) # hemi sizes
        self.body_floor_shape = chrono.ChBoxShape()      
        self.body_floor.SetCollide(False)
        self.my_system.Add(self.body_floor)  
    
        if self.env_mode=='verify_tunnel':    
            data=np.load("tunnel_start_positions.npz")
            Xc=data['Xc']
            Yc=data['Yc']
            const1 = 2.54
            const2 = .01
            w = const2*const1*5.5
            l = const2*const1*25.5
            xpos1 = ( -35.4840177524972)/100
            ypos1 = (29.713987884937147)/100
            xpos2 = (-36.4840177524972)/100
            ypos2 = (-106.91221851298883)/100        
            px1= xpos1 - w
            px1 = -.57
            pz1 = ypos1 + l/2 
            px2 = -.57
            pz2 = ypos2 + l/2 
            wall = chrono.ChBody()
            
            wall = chrono.ChBodyEasyBox(w,w,l,1000,True,True) # create object
            wall.SetPos(chrono.ChVectorD(px1,w/2,pz1)) # set its position 
            wall.SetMaterialSurface(self.material) # apply material properties 
            wall.SetName('wall')
            wall.SetBodyFixed(True)
            col_g = chrono.ChColorAsset()
            col_g.SetColor(chrono.ChColor(1,0.1,0.5))
            wall.AddAsset(col_g)
            self.my_system.Add(wall)
            self.wall.append(wall)
            
            
            wall = chrono.ChBodyEasyBox(w,w,l,1000,True,True) # create object
            wall.SetPos(chrono.ChVectorD(px2,w/2,pz2)) # set its position 
            wall.SetMaterialSurface(self.material) # apply material properties 
            wall.SetName('wall')
            wall.SetBodyFixed(True)
            col_g = chrono.ChColorAsset()
            col_g.SetColor(chrono.ChColor(1,0.1,0.5))
            wall.AddAsset(col_g)
            self.my_system.Add(wall)
            self.wall.append(wall)        
            
            z2x = chrono.ChQuaternionD()
            z2x.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 1, 0))
            z2y = chrono.ChQuaternionD()
            z2y.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 0, 1))
            px=5
            pz=10
            w=10
            l=.5
            wall = chrono.ChBody()
            wall.SetName('wall')
            wall.SetBodyFixed(True)
            wall.SetPos(chrono.ChVectorD(px,.25,pz))
            wall.SetDensity(1000)
            wall.SetMaterialSurface(self.material)
            wall.GetCollisionModel().ClearModel()
            wall.GetCollisionModel().AddBox(w,.25,l)
            wall.SetCollide(True)
            wall_shape=chrono.ChBoxShape()
            wall_shape.GetBoxGeometry().Size = chrono.ChVectorD(w,.25,l)
            wall.GetAssets().push_back(wall_shape)
            col_g = chrono.ChColorAsset()
            col_g.SetColor(chrono.ChColor(1,0.1,0.5))
            wall.AddAsset(col_g)
            self.my_system.Add(wall)
       
            
        if self.env_mode=='import_tunnel':    

            rotate = 15*np.pi/180
            w=2
            l=.5
            l2=l/np.cos(rotate)
            px=0
            pz=0
            wall = chrono.ChBody()
            # Attach a visualization shape .
            # First load a .obj from disk into a ChTriangleMeshConnected:
            mesh_for_visualization = chrono.ChTriangleMeshConnected()
            mesh_for_visualization.LoadWavefrontMesh(self.path+'wall_5.obj')
            mesh_for_visualization.Transform(chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(1))
            visualization_shape = chrono.ChTriangleMeshShape()
            visualization_shape.SetMesh(mesh_for_visualization)
            wall.AddAsset(visualization_shape)

            mesh_for_collision = chrono.ChTriangleMeshConnected()
            mesh_for_collision.LoadWavefrontMesh(self.path+'wall_5.obj')
            # Optionally: you can scale/shrink/rotate the mesh using this:
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
            
            
        if self.env_mode=='tunnel':             
            
            # left wall
            wall = chrono.ChBody()
            wall.SetName('wall')
            wall.SetBodyFixed(True)
            wall.SetPos(chrono.ChVectorD(px,.25,pz))
            wall.SetDensity(1000)
            wall.SetMaterialSurface(self.material)
            wall.GetCollisionModel().ClearModel()
            wall.GetCollisionModel().AddBox(w,.25,l)
            wall.SetCollide(True)
            wall_shape=chrono.ChBoxShape()
            wall_shape.GetBoxGeometry().Size = chrono.ChVectorD(w,.25,l)
            wall.GetAssets().push_back(wall_shape)
            col_g = chrono.ChColorAsset()
            col_g.SetColor(chrono.ChColor(1,0.1,0.5))
            wall.AddAsset(col_g)
            self.my_system.Add(wall)
            
            # right wall 
            wall2 = chrono.ChBody()
            wall2.SetName('wall2')
            wall2.SetBodyFixed(False)
            wall2.SetDensity(10000)
            wall2.SetPos(chrono.ChVectorD(px,.25,-pz))
            wall2.SetMaterialSurface(self.material)
            wall2.GetCollisionModel().ClearModel()
            wall2.GetCollisionModel().AddBox(w,.25,l)
            wall2.SetCollide(True)
            wall_shape=chrono.ChBoxShape()
            wall_shape.GetBoxGeometry().Size = chrono.ChVectorD(w, .25,l)
            wall2.GetAssets().push_back(wall_shape)
            col_g = chrono.ChColorAsset()
            col_g.SetColor(chrono.ChColor(1,0.1,0.5))
            wall2.AddAsset(col_g)
            prismatic_ground_wall = chrono.ChLinkLockPrismatic()
            prismatic_ground_wall.SetName("prismatic_ground_ball")
            prismatic_ground_wall.Initialize(self.body_floor, wall2, chrono.ChCoordsysD(chrono.ChVectorD(px,.25,-pz), z2y))

            # Funnel right 
            wall3 = chrono.ChBody()
            wall3.SetName('wall3')
            wall3.SetBodyFixed(True)
            wall3.SetPos(chrono.ChVectorD((px-(w+w*np.cos(rotate))+l2*np.sin(rotate)),.25, -pz-1*np.sin(rotate)*w ))
            wall3.SetRot(chrono.Q_from_AngY(-rotate))
            wall3.SetMaterialSurface(self.material)
            wall3.GetCollisionModel().ClearModel()
            wall3.GetCollisionModel().AddBox(w,.25,l2)
            wall3.SetCollide(True)
            wall_shape=chrono.ChBoxShape()
            wall_shape.GetBoxGeometry().Size = chrono.ChVectorD(w, .25,l2)
            wall3.GetAssets().push_back(wall_shape)
            col_g = chrono.ChColorAsset()
            col_g.SetColor(chrono.ChColor(1,0.4,0.5))
            wall3.AddAsset(col_g)
            self.my_system.Add(wall3) 

            # funnel left
            wall4 = chrono.ChBody()
            wall4.SetName('wall4')
            wall4.SetBodyFixed(True)
            wall4.SetPos(chrono.ChVectorD((px-(w+w*np.cos(rotate))+l2*np.sin(rotate)),.25, pz+1*np.sin(rotate)*w))
            wall4.SetRot(chrono.Q_from_AngY(rotate))
            wall4.SetMaterialSurface(self.material)
            wall4.GetCollisionModel().ClearModel()
            wall4.GetCollisionModel().AddBox(w,.25,l2)
            wall4.SetCollide(True)
            
            wall_shape=chrono.ChBoxShape()
            wall_shape.GetBoxGeometry().Size = chrono.ChVectorD(w, .25,l2)
            wall4.GetAssets().push_back(wall_shape)
            col_g = chrono.ChColorAsset()
            col_g.SetColor(chrono.ChColor(1,0.4,0.5))
            wall4.AddAsset(col_g)
            self.my_system.Add(wall4) 

    def return_env(self):
        return(self.my_system)
    
# Controller 
class Controls:
    # **We should really put variable=None for all the optional arguments so we don't need to init everything every single time**
    def __init__(self,forces,bots,interiors,Springs,my_system,nb,control_type,balls,tstep,my_rep,w,tn,tpull,radius2,n,R,enviroment,forceb,boundary,*args,**kwargs):
        # objects
        
        #### Initialize some parameters for all cases of the controller
        self.forces=forces          # array of forces 
        self.forceb=forceb
        self.bots=bots              # array of all our boundary robots
        self.interiors=interiors    # array of all the interior particles
        self.Springs=Springs        # array of all the spring objects
        self.my_system=my_system    # the simulation system
        self.balls=balls            # balls if they are there 
        self.tstep=tstep           # time step of simulation
        self.control_type=control_type # controller type                  
        self.nb=nb                  # number of robots
        self.w=w                    # frequency
        self.tn=tn                  # time interval
        self.T=0                    # internal time 
        self.time=0                 # empty time variable 
        self.tpull=tpull            # VARIABLE FOR WHEN TO LET THE BALL BE FREE THIS applies to the grab sims 
        self.enviroment=enviroment
        self.Obst=self.enviroment.Obst
        self.tt=[6]
        self.boundary=boundary
        
        # save controller forces and error
        self.Faxc={};self.Fayc={};self.Fazc={}; self.E=[]
        
        # these are for multi shape formation. There variables that assist with the fields so there only made once
        self.trig=1
        self.trig2=1
        self.trig3=1
        self.trig4=1
        self.trig5=1
        self.trigspring=0
        
        # temporary arrays
        self.fxt=[]; self.fyt=[]; self.fzt=[]
        
        # contact class 
        self.my_rep=my_rep 
        self.count=0
        self.xc=0
        self.zc=0
        self.xb=0
        self.zb=0
        self.p=2
        self.Fb=[]
        self.PX=[]
        self.PY=[]
        self.MB=[]
        self.ALPHA=[]
        self.change=0
        
        # create controller forces applied 
        for i in range(self.nb):
            self.Faxc["botx{0}".format(i)]=[]
            self.Fayc["boty{0}".format(i)]=[]
            self.Fazc["botz{0}".format(i)]=[] 
       
        
##############################################################################            
        
        ##### SHAPE_FORM  PARAMETERS
        if self.control_type=='shape_form':
            self.phi=args[0][0]   # potential field object
            self.f=self.phi.f     # potential field function
            self.fnx=self.phi.fnx # gradient in x direction  
            self.fny=self.phi.fny # gradient in the y direction             
            self.alpha=args[0][1] # proportional gain 
            self.beta=args[0][2]  # damping term 
            self.b=args[0][3]     # width of field               
            self.FX=0             # empty force X
            self.FZ=0             # empty force Z
            self.E=[]             # empty error array  
            
##############################################################################            
        
        ##### SHAPE_FORM_analytic  PARAMETERS
        if self.control_type=='shape_form_analytic':
            self.phi=args[0][0]   # potential field object
            self.f=self.phi.f     # potential field function
            self.fnx=self.phi.fnx # gradient in x direction  
            self.fny=self.phi.fny # gradient in the y direction             
            self.alpha=args[0][1] # proportional gain 
            self.beta=args[0][2]  # damping term 
            self.b=args[0][3]     # width of field               
            self.FX=0             # empty force X
            self.FZ=0             # empty force Z
            self.E=[]             # empty error array                  
            
    
##############################################################################   
        
        ##### POT_FIELD_GRAB PARAMETERS        
        if self.control_type=='pot_field_grab':
            self.phi=args[0][0]   # potential field object
            self.f=self.phi.f     # potential field function
            self.fnx=self.phi.fnx # gradient in x direction  
            self.fny=self.phi.fny # gradient in the y direction
            self.Obst=self.enviroment.Obst           
            self.alpha=args[0][1] # proportional gain 
            self.beta=args[0][2]  # damping term 
            self.b=args[0][3]     # width of field               
            self.FX=0             # empty force X
            self.FZ=0             # empty force Z    
            self.cond=False
            self.cn=0
            self.num=10
            self.tcut=0
            self.mode=0
            self.fb=0
            self.countb=0
            self.ALPHA=[]


##############################################################################   
        
        ##### GRASP PARAMETERS        
        if self.control_type=="GRASP":
            self.Obst=self.enviroment.Obst
            self.phi=args[0][0]
            self.alpha=args[0][1] # proportional gain 
            self.beta=args[0][2]  # damping term 
            self.b=args[0][3]     # width of field               
            self.FX=0             # empty force X
            self.FZ=0             # empty force Z    
            self.cond=False
            self.cn=0
            self.num=15
            self.tcut=0
            self.mode=0                
            self.fb=0            
            
##############################################################################
            
        ##### SHAPE_WARP PARAMETERS
        if self.control_type=='image_warp':
            self.phi=args[0][0]   # potential field object
            self.t=0
            self.alpha=args[0][1] # proportional gain 
            self.beta=args[0][2]  # damping term 
            self.b=args[0][3]     # width of field               
            self.FX=0             # empty force X
            self.FZ=0             # empty force Z
            self.E=[]             # empty error array     
            self.count=0
            self.trig=1
            self.trig2=1
            self.tt = [10,20,30]
            self.tcut=0
            self.mode=0
            
############################################################################### 

        ##### LETTER FIELD
        if self.control_type=="import_field_letters":
            
            self.phi=args[0][0]   # potential field object
            self.t=0
                   # A M  O  E  B  A 
            self.tt=[6,12,18,24,32,38]
            #self.tt=[10,20,30,40,50,60]
            self.alpha=args[0][1] # proportional gain 
            self.beta=args[0][2]  # damping term               
            self.FX=0             # empty force X
            self.FZ=0             # empty force Z
            self.E=[]             # empty error array     
            self.count=0
            
            self.trig1=1
            self.trig2=1
            self.trig3=1 
            self.trig4=1
            self.trig5=1
            self.trig6=1             
            self.trig7=1
            self.trig8=1  
            
            self.tcut=0
            self.mode=1            
            
###############################################################################           
        
        ##### TUNNELING    
        if self.control_type=="tunnel": 
            self.phi=args[0][0]     # potential field object (contains all the pot field stuff)  
            #self.phi1 = self.phi[0]
            #self.phi2 = self.phi[1]
            self.fny=self.phi.fny   # gradient in the y direction of field 
            self.fnx=self.phi.fnx   # gradinen in the x direction of field             
            # self.fny1=self.phi1.fny   # gradient in the y direction of field 
            # self.fnx1=self.phi1.fnx   # gradinen in the x direction of field 
            # self.fny2=self.phi2.fny   # gradient in the y direction of field 
            # self.fnx2=self.phi2.fnx   # gradinen in the x direction of field 
            self.alpha=args[0][1]   # proportional gain 
            self.beta=args[0][2]    # damping term 
            self.FX=0               # empty force array x direction 
            self.FZ=0               # empty force array z direction    
            
###############################################################################   
        
        ##### VERIFY  TUNNEL 
        if self.control_type=="tunnel_verify": 
            self.phi=args[0][0]     # potential field object (contains all the pot field stuff)  
            self.alpha=args[0][1]   # proportional gain 
            self.beta=args[0][2]    # damping term 
            self.FX=0               # empty force array x direction 
            self.FZ=0               # empty force array z direction  
            data=np.load("tunnel_start_positions.npz")
            self.Xp=data['Xb']
            self.Yp=data['Yb']
            self.xc=data['Xc']
            self.zc=data['Yc']            
            self.fb=0            
                        
###############################################################################            
        
        ##### GRASP  VERIFY
        if self.control_type=="grasp_verify":
            self.Obst=self.enviroment.Obst
            self.phi=args[0][0]
            self.phi1=self.phi[0]
            self.phi2=self.phi[1]
            
            self.alpha=args[0][1] # proportional gain 
            self.beta=args[0][2]  # damping term 
            self.b=args[0][3]     # width of field               
            self.FX=0             # empty force X
            self.FZ=0             # empty force Z    
            self.cond=False
            self.cn=0
            self.num=15
            self.tcut=0
            self.mode=0 
            data=np.load("grasp_start_positions.npz")
            self.Xp=data['Xb']
            self.Yp=data['Yb']
            self.xc=data['Xc']
            self.zc=data['Yc'] 
            self.fb=0  
               
###############################################################################            
        
        ##### VERIFY TARGET
        if self.control_type=="target_verify": 
            self.phi=args[0][0]     # potential field object (contains all the pot field stuff)  
            self.alpha=args[0][1]   # proportional gain 
            self.beta=args[0][2]    # damping term 
            self.FX=0               # empty force array x direction 
            self.FZ=0               # empty force array z direction  
            data=np.load("target_track_start_position.npz")
            self.Xp=data['Xb']
            self.Yp=data['Yb']
            self.xc=data['Xc']
            self.zc=data['Yc'] 
            self.Xm=data['Xm']
            self.Ym=data['Ym']   
            self.Tm=data['Tm']
            self.fb=0                  
           


##############################################################################


   
    # run controller
    def run_controller(self):
        #### SHAPE_FORM : Run the shape formation controller 
        if self.control_type=="shape_form":
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            (self.FX,self.FZ)=self.shape_controller()
            self.apply_force(self.FX,self.FZ)
            
        #### SHAPE_FORM : Run the shape formation analytic controller
        if self.control_type=="shape_form_analytic":
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            (self.FX,self.FZ)=self.shape_controller()
            self.apply_force(self.FX,self.FZ)

        ##### POT FIELD : grab a fixed object
        if self.control_type=="pot_field_grab":
            (self.xb,self.zb)=self.get_position() # get the position of the bots
            (self.xbv,self.zbv)=self.get_velocity() # get the velocity of the bots
            (self.FX,self.FZ)=self.grab_controller1() # run the grab controller
            #(self.FX,self.FZ)=self.grab_drag_controller()
            #(self.FX,self.FZ)=self.grab_controller_warp()
            #(self.FX,self.FZ)=self.grab_controller_warp_2()
            self.apply_force(self.FX,self.FZ) # apply the force to the bots


        ##### GRAB WARP : grab a fixed object by warping
        if self.control_type=="GRASP":
            (self.xb,self.zb)=self.get_position() # get the position of the bots
            (self.xbv,self.zbv)=self.get_velocity() # get the velocity of the bots
            (self.FX,self.FZ)=self.grab_controller5() # run the grab controller
            self.apply_force(self.FX,self.FZ) # apply the force to the bots
             
        #### SHAPE WARP : Run the shape warping controller
        if self.control_type=="image_warp":
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            (self.FX,self.FZ)=self.warp_controller1()
            self.apply_force(self.FX,self.FZ)     

        #### IMPORT IMAGE LETTERS : run the imported field letters controller   
        if self.control_type=="import_field_letters":
            (self.xb,self.zb)=self.get_position()
            (self.xbv,self.zbv)=self.get_velocity()
            (self.FX,self.FZ)=self.warp_controller2()
            self.apply_force(self.FX,self.FZ)            
            

        #### TUNNEL : Run the tunneling controller     
        if self.control_type=="tunnel":
            (self.xb,self.zb)=self.get_position() # get the positions of the bots
            (self.xbv,self.zbv)=self.get_velocity() # get the velocities of the bots
            (self.FX,self.FZ)=self.tunnel_controller() # run the tunneling controller 
            self.apply_force(self.FX,self.FZ)  # apply the force to the bots

        #### TUNNEL VERIFY : Run the tunneling controller     
        if self.control_type=="tunnel_verify":
            (self.xb,self.zb)=self.get_position() # get the positions of the bots
            (self.xbv,self.zbv)=self.get_velocity() # get the velocities of the bots
            (self.FX,self.FZ)=self.tunnel_verify_controller() # run the tunneling controller 
            self.apply_force(self.FX,self.FZ)  # apply the force to the bots

        #### GRASP VERIFY : Run the tunneling controller     
        if self.control_type=="grasp_verify":
            (self.xb,self.zb)=self.get_position() # get the positions of the bots
            (self.xbv,self.zbv)=self.get_velocity() # get the velocities of the bots
            (self.FX,self.FZ)=self.grasp_verify_controller() # run the tunneling controller 
            self.apply_force(self.FX,self.FZ)  # apply the force to the bots
            
        #### TARGET_TRACK VERIFY : Run the tunneling controller     
        if self.control_type=="target_verify":
            (self.xb,self.zb)=self.get_position() # get the positions of the bots
            (self.xbv,self.zbv)=self.get_velocity() # get the velocities of the bots
            (self.FX,self.FZ)=self.Verify_controller() # run the tunneling controller 
            self.apply_force(self.FX,self.FZ)  # apply the force to the bots            
       
##################################################################   
# controlling a shape     
    def shape_controller(self):
        FX=[]
        FZ=[]
        #     self.phi.update_field()
        #     self.f=self.phi.f
        #     self.fnx=self.phi.fnx
        #     self.fny=self.phi.fny 
        for i in range(self.nb):
            Fx=self.fny([self.xb[i],self.zb[i]])
            Fz=self.fnx([self.xb[i],self.zb[i]])
            mag=np.sqrt(Fx**2+Fz**2)
            if mag[0]==0:
                FXX=np.array([0])
                FZZ=np.array([0])
            else:
                FXX=Fx/mag
                FZZ=Fz/mag
            fx=-self.alpha*FXX-self.beta*self.xbv[i]
            fz=-self.alpha*FZZ-self.beta*self.zbv[i] 
            
            FX.append(fx[0])
            FZ.append(fz[0])
        return(np.asarray(FX),np.asarray(FZ))

##############################################################################            

   #  Grabbing a ball controller 
    def grab_controller1(self):
        ''' This controller is used for the tug test'''
        FX=[]
        FZ=[]
        #self.phi.py=self.balls.balls[0].GetPos().x
        #self.phi.px=self.balls.balls[0].GetPos().z
        t=np.round(self.my_system.GetChTime(),4) 
        if t>2.5:
            self.balls.balls[0].SetBodyFixed(False)
        if t>5.0:
            #self.alpha=80
            self.fb=self.fb+.05
        # # self.countb=self.countb+1
            self.forceb[0].SetMforce(self.fb)
            self.forceb[0].SetDir(chrono.VECT_Z)
            # for i in range(self.nb):
            #     if i==21 or i==22 or i==23:
            #         self.bots[i].SetBodyFixed(True)
            
        #(self.f,self.fny,self.fnx)=self.phi.update_field_gradient()
        for i in range(self.nb):
            Fx=self.fny([self.xb[i],self.zb[i]])
            Fz=self.fnx([self.xb[i],self.zb[i]])
            mag=np.sqrt(Fx**2+Fz**2)
            FXX=Fx/mag
            FZZ=Fz/mag
            fz=-self.alpha*FZZ-self.beta*self.zbv[i] 
            fx=-self.alpha*FXX-self.beta*self.xbv[i]                
            FX.append(fx[0])
            FZ.append(fz[0])
            #FZ.append(70)
        return(np.asarray(FX),np.asarray(FZ))

    def grab_controller5(self):
        ''' This controller is used GRASPING BY WARPING AROUND THE OBJECT'''

        FX=[]
        FZ=[]
        #self.phi.py=self.balls.balls[0].GetPos().x
        #self.phi.px=self.balls.balls[0].GetPos().z
        t=np.round(self.my_system.GetChTime(),4) 
        
        
        if t>10:
            self.alpha=self.phi.alpha2
            self.phi.a=0.25*self.phi.Rb 
            self.phi.c=0.25*self.phi.Rb 
            self.phi.px=self.phi.bx
            self.phi.py=self.phi.by

        if t>11:
            self.balls.balls[0].SetBodyFixed(False)
        if t>12:
            #self.alpha=80
            self.fb=self.fb+.05
            self.forceb[0].SetMforce(self.fb)
            self.forceb[0].SetDir(chrono.VECT_Z)
   
        for i in range(self.nb):
            Fx=self.phi.Fy2(self.zb[i],self.xb[i],self.phi.px,self.phi.py,self.phi.a,self.phi.c)
            Fz=self.phi.Fx2(self.zb[i],self.xb[i],self.phi.px,self.phi.py,self.phi.a,self.phi.c)
            mag=np.sqrt(Fx**2+Fz**2)
            FXX=Fx/mag
            FZZ=Fz/mag
            fz=-self.alpha*FZZ-self.beta*self.zbv[i] 
            fx=-self.alpha*FXX-self.beta*self.xbv[i]                
            FX.append([fx])
            FZ.append([fz])
        return(np.asarray(FX),np.asarray(FZ))





##############################################################################      
    def warp_controller1(self):
        ''' Controller for making it warp between 3 shapes '''
        t=np.round(self.my_system.GetChTime(),4)    
        
        if t>self.tt[0] and self.tanh(t-self.tcut)>.90 and self.trig!=0:
            self.trig=0
            self.tcut=t
            self.mode=1
            
        if t>self.tt[1] and self.tanh(t-self.tcut)>.90 and self.trig2!=0:
            self.trig2=0
            self.tcut=t
            self.mode=2
            
        if self.count%10==0: # check on it every 10 times for efficiency
            # e=[] # empty error array 
            # # check error #
            # for i in range(self.nb):
            #     e.append(.5*(self.phi.M3(t-self.tcut,self.zb[i],self.xb[i],self.mode)**2)) # calculate error
            # error=np.round(np.sum(e),3) # round it for precision  
            # print('error=',error)
            print('tanh',self.tanh(t-self.tcut))
        FX=[]
        FZ=[]

        for i in range(self.nb):
            Fx=self.phi.Mx3(self.tanh(t-self.tcut),self.zb[i],self.xb[i],self.mode)
            Fz=self.phi.My3(self.tanh(t-self.tcut),self.zb[i],self.xb[i],self.mode)
            mag=np.sqrt(Fx**2+Fz**2)
            if mag[0]==0:
                FXX=np.array([0])
                FZZ=np.array([0])  
            else:
                FXX=Fx/mag
                FZZ=Fz/mag

            fx=(-self.alpha*FXX-self.beta*self.xbv[i])
            fz=(-self.alpha*FZZ-self.beta*self.zbv[i])
            FX.append(fx[0])
            FZ.append(fz[0])
        self.count=self.count+1    
        return(np.asarray(FX),np.asarray(FZ))  

##############################################################################      
    def warp_controller2(self):
        ''' Controller for making it warp between 3 shapes '''
        t=np.round(self.my_system.GetChTime(),4)    
        
        
        # shape change A
        if t>self.tt[0] and self.tanh(t-self.tcut)>.90 and self.trig1!=0:
            self.trig1=0
            self.tcut=t
            self.mode=2
            
        # shape change M   
        if t>self.tt[1] and self.tanh(t-self.tcut)>.90 and self.trig2!=0:
            self.trig2=0
            self.tcut=t
            self.mode=3
            
        # shape change O
        if t>self.tt[2] and self.tanh(t-self.tcut)>.90 and self.trig3!=0:
            self.trig3=0
            self.tcut=t
            self.mode=4
            
        # shape change E    
        if t>self.tt[3] and self.tanh(t-self.tcut)>.90 and self.trig4!=0:
            self.trig4=0
            self.tcut=t
            self.mode=5
        
        # shape change B
        if t>self.tt[4] and self.tanh(t-self.tcut)>.90 and self.trig5!=0:
            self.trig5=0
            self.tcut=t
            self.mode=6
            
        # shape change A
        if t>self.tt[5] and self.tanh(t-self.tcut)>.90 and self.trig6!=0:
            self.trig6=0
            self.tcut=t
            self.mode=7            

        if self.count%10==0: # check on it every 10 times for efficiency
            # e=[] # empty error array 
            # # check error #
            # for i in range(self.nb):
            #     e.append(.5*(self.phi.M3(t-self.tcut,self.zb[i],self.xb[i],self.mode)**2)) # calculate error
            # error=np.round(np.sum(e),3) # round it for precision  
            # print('error=',error)
            print('tanh',self.tanh(t-self.tcut))
        FX=[]
        FZ=[]

        for i in range(self.nb):
            Fx=self.phi.Mx(self.tanh(t-self.tcut),self.zb[i],self.xb[i],self.mode)
            Fz=self.phi.My(self.tanh(t-self.tcut),self.zb[i],self.xb[i],self.mode)
            mag=np.sqrt(Fx**2+Fz**2)
            if mag[0]==0:
                FXX=np.array([0])
                FZZ=np.array([0])  
            else:
                FXX=Fx/mag
                FZZ=Fz/mag

            fx=(-self.alpha*FXX-self.beta*self.xbv[i])
            fz=(-self.alpha*FZZ-self.beta*self.zbv[i])
            FX.append(fx[0])
            FZ.append(fz[0])
        self.count=self.count+1    
        return(np.asarray(FX),np.asarray(FZ))  
    

##############################################################################    
    def tunnel_controller(self):
        FX=[]
        FZ=[]
        t = np.round(self.my_system.GetChTime(),4)
        # if np.round(self.my_system.GetChTime(),4)>2:
        #     self.fnx=self.fnx2
        #     self.fny=self.fny2
        # else:
        #     self.fnx=self.fnx1
        #     self.fny=self.fny1
            
        for i in range(self.nb):
            Fx=self.fny([self.xb[i],self.zb[i]])
            Fz=self.fnx([self.xb[i],self.zb[i]])
            mag=np.sqrt(Fx**2+Fz**2)
            if mag[0]==0:
                FXX=np.array([0])
                FZZ=np.array([0])
            else:
                FXX=Fx/mag
                FZZ=Fz/mag
            fx=-self.alpha*FXX-self.beta*self.xbv[i] #+ 1*self.alpha*np.cos(20*(t+np.random.random_sample()))

            fz=-self.alpha*FZZ-self.beta*self.zbv[i] #+ 1*self.alpha*np.sin(20*(t+np.random.random_sample()))
            
            FX.append(fx[0])
            FZ.append(fz[0])
        return(np.asarray(FX),np.asarray(FZ))   
    








    

    def Verify_controller(self):
        ''' This controller is used GRASPING'''
        FX=[]
        FZ=[]
        tt=np.round(self.my_system.GetChTime(),2)
        #print(t)
        if tt<11:
            for i in range(self.nb):
                Y=(self.Yp[i]-self.zc)*.01
                X=(self.Xp[i]-self.xc)*.01
                Fx=self.phi.Fy(self.zb[i],self.xb[i],Y,X)
                Fz=self.phi.Fx(self.zb[i],self.xb[i],Y,X)
                mag=np.sqrt(Fx**2+Fz**2)
                FXX=Fx/mag
                FZZ=Fz/mag
                fz=-self.alpha*FZZ-self.beta*self.zbv[i] 
                fx=-self.alpha*FXX-self.beta*self.xbv[i]   
                FX.append([fx])
                FZ.append([fz])   
        else:
            for i in range(len(self.interiors.particles)):
                self.interiors.particles[i].SetCollide(True)
                self.interiors.particles[i].SetBodyFixed(False)            
            
            (x,y)=self.change_marker(tt)
            self.phi.px=x#*.01
            self.phi.py=y#*.01
            print(self.change)
            #print(self.phi.px,self.phi.py)
            self.alpha=1
            for i in range(self.nb):
                Fx=self.phi.Fy(self.zb[i],self.xb[i],self.phi.py,self.phi.px)
                Fz=self.phi.Fx(self.zb[i],self.xb[i],self.phi.py,self.phi.px)
                mag=np.sqrt(Fx**2+Fz**2)
                FXX=Fx/mag
                FZZ=Fz/mag
                fz=-self.alpha*FZZ#-self.beta*self.zbv[i] 
                fx=-self.alpha*FXX#-self.beta*self.xbv[i]                
                FX.append([fx])
                FZ.append([fz])
        return(np.asarray(FX),np.asarray(FZ))


    def tunnel_verify_controller(self):
        ''' This controller is used GRASPING'''
        FX=[]
        FZ=[]
        tt = np.round(self.my_system.GetChTime(),2)
        if tt<4:
            for i in range(self.nb):
                Y=(self.Yp[i]-self.zc)*.01
                X=(self.Xp[i]-self.xc)*.01
                Fx=self.phi.Fy(self.zb[i],self.xb[i],Y,X)
                Fz=self.phi.Fx(self.zb[i],self.xb[i],Y,X)
                mag=np.sqrt(Fx**2+Fz**2)
                FXX=Fx/mag
                FZZ=Fz/mag
                fz=-self.alpha*FZZ-self.beta*self.zbv[i] 
                fx=-self.alpha*FXX-self.beta*self.xbv[i]   
                FX.append([fx])
                FZ.append([fz])
        else:
            self.enviroment.wall[0].SetCollide(True)
            self.enviroment.wall[1].SetCollide(True)
            
            
            for i in range(len(self.interiors.particles)):
                self.interiors.particles[i].SetCollide(True)
                self.interiors.particles[i].SetBodyFixed(False)
            for i in range(self.nb):
                self.alpha = .4
                Fx=self.phi.Fy(self.zb[i],self.xb[i],.1,-1.6)
                Fz=self.phi.Fx(self.zb[i],self.xb[i],.1,-1.6)
                mag=np.sqrt(Fx**2+Fz**2)
                FXX=Fx/mag
                FZZ=Fz/mag
                #print(FXX,FZZ)             
                MAG=np.sqrt(FXX*2+FZZ**2)
                ang2 = signal.square(2 * np.pi * .2 * tt)*(1*np.pi/3)
                theta = np.nan_to_num(np.arctan2(FZZ, FXX)) % 2*np.pi
                theta=theta#+ang2
                
                
                
                fz=-self.alpha*np.sin(theta)
                fx=-self.alpha*np.cos(theta)
                
                
                # #print(ang2)
                # fX = mag*np.cos(ang2) 
                # fZ = np.sin(ang2)  
                # Fz = Fz + fZ
                # mag = np.sqrt(Fx**2+Fz**2)
                
                # FXX=Fx/mag
                # FZZ=Fz/mag
                # #print(FXX,FZZ)
                # fz=-self.alpha*FZZ#-self.beta*self.zbv[i] 
                # fx=-self.alpha*FXX#-self.beta*self.xbv[i]                  
                
                FX.append([fx])
                FZ.append([fz])
                                
                
        return(np.asarray(FX),np.asarray(FZ))

    def grasp_verify_controller(self):
        ''' This controller is used GRASPING'''
        
        FX=[]
        FZ=[]
        #self.phi.py=self.balls.balls[0].GetPos().x
        #self.phi.px=self.balls.balls[0].GetPos().z
        tt = np.round(self.my_system.GetChTime(),2)
        if tt<8:
            for i in range(self.nb):
                Y=(self.Yp[i]-self.zc)*.01
                X=(self.Xp[i]-self.xc)*.01
                Fx=self.phi1.Fy(self.zb[i],self.xb[i],Y,X)
                Fz=self.phi1.Fx(self.zb[i],self.xb[i],Y,X)
                mag=np.sqrt(Fx**2+Fz**2)
                FXX=Fx/mag
                FZZ=Fz/mag
                fz=-self.alpha*FZZ-self.beta*self.zbv[i] 
                fx=-self.alpha*FXX-self.beta*self.xbv[i]   
                FX.append([fx])
                FZ.append([fz])        

        else:  
            self.alpha=.4
            for i in range(len(self.interiors.particles)):
                self.interiors.particles[i].SetCollide(True)
                self.interiors.particles[i].SetBodyFixed(False)            
            
            if tt>18:
                self.alpha=.4
                self.phi2.a=.01
                self.phi2.c=.01
                self.phi2.px=self.phi2.bx
                self.phi2.py=self.phi2.by
    
    
            for i in range(self.nb):
                Fx=self.phi2.Fy2(self.zb[i],self.xb[i],self.phi2.px,self.phi2.py,self.phi2.a,self.phi2.c)
                Fz=self.phi2.Fx2(self.zb[i],self.xb[i],self.phi2.px,self.phi2.py,self.phi2.a,self.phi2.c)
                mag=np.sqrt(Fx**2+Fz**2)
                FXX=Fx/mag
                FZZ=Fz/mag
                fz=-self.alpha*FZZ#-self.beta*self.zbv[i] 
                fx=-self.alpha*FXX#-self.beta*self.xbv[i]                
                FX.append([fx])
                FZ.append([fz])
        return(np.asarray(FX),np.asarray(FZ))

    def tanh(self,t):

        tanh=(np.exp(self.p*(t))-1)/(np.exp(self.p*(t))+1)
        #print('tanh=',tanh)
        return(tanh)   

    def change_marker(self,time):
        res=np.where(self.Tm==np.round(time,2))
        #print(res)
        if len(res[0])==0:
            #print('na')
            x=self.balls.balls[0].GetPos().x
            z=self.balls.balls[0].GetPos().z
        else:
            self.change=self.change+1
            x=self.Xm[res[0][0]]*.01
            z=self.Ym[res[0][0]]*.01
        #print(x,z)
        
            self.balls.balls[0].SetPos(chrono.ChVectorD(x,.06,z))
        return(x,z)


 
    
##############################################################################                
    def apply_force(self,FX,FZ):  
        
        tt=np.round(self.my_system.GetChTime(),2)

        # if tt>4:
        #     pwm=155# 0-255
        #     w=5 # freqency
        #     tn=(pwm/255)/w 
            
            
            
        self.T=self.tstep+self.T
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
            self.fxt.append(self.FX[i])
            self.fyt.append(0)
            self.fzt.append(self.FZ[i])
            self.forces[3*i].SetMforce(float(self.FX[i]))
            self.forces[3*i].SetDir(chrono.VECT_X)
            self.forces[3*i+2].SetMforce(float(self.FZ[i]))
            self.forces[3*i+2].SetDir(chrono.VECT_Z)
            
# get current position         
    def get_position(self):
        self.xb=[]        
        self.zb=[]
        for i in range(len(self.bots)):
            self.xb.append(self.bots[i].GetPos().x)
            self.zb.append(self.bots[i].GetPos().z)
        return(self.xb,self.zb)

    def get_centroid(self):
        self.xc=np.sum(self.xb)/self.nb
        self.zc=np.sum(self.zb)/self.nb
        return(self.xc,self.zc)            
# Get ball Position
    def get_ball_position(self):
        self.ballx=self.balls[0].GetPos().x
        self.ballz=self.balls[0].GetPos().z
        return(self.ballx,self.ballz)
    
    def get_error(self):
        t = np.round(self.my_system.GetChTime(),4)

        if self.control_type=='shape_form':
            e=[]
            for i in range(self.nb):
                e.append((self.f([self.xb[i],self.zb[i]])))
            self.E.append(np.sum(e))
            
        if self.control_type=='image_warp': 
            e=[]
            for i in range(self.nb):
                if t<=self.tt[0]:
                    F=self.phi.F[1]
                if t>self.tt[0] and t<=self.tt[1]:
                    F=self.phi.F[2]
                if t>self.tt[1]:
                    F=self.phi.F[3]
                    
                e.append((F([self.xb[i],self.zb[i]]))) # calculate error
                    
            self.E.append(np.sum(e))
            



        
    def save_pull_test(self):
        self.Fb.append(self.fb)
        self.PX.append(self.phi.px)
        self.PY.append(self.phi.py)
        self.MB.append(self.balls.balls[0].GetMass())
        
        
 # get current velocity       
    def get_velocity(self):
        self.xbv=[]
        self.zbv=[]
        for i in range(len(self.bots)):
            self.xbv.append(self.bots[i].GetPos_dt().x)
            self.zbv.append(self.bots[i].GetPos_dt().z)
        return(self.xbv,self.zbv)
# save the variables        
    def save_data_Forces(self):
       #if len(self.fxt)<self.nb:
        for i in range(self.nb):
             self.fxt.append(0)
             self.fyt.append(0)
             self.fzt.append(0)                
        for i in range(self.nb):
           self.Faxc["botx"+str(i)].append(self.fxt[i])
           self.Fayc["boty"+str(i)].append(self.fyt[i])
           self.Fazc["botz"+str(i)].append(self.fzt[i])    

    def clear_temp_forces(self):
        self.fxt=[]
        self.fyt=[]
        self.fzt=[]
        
# return force data for controllers     
    def return_force_data(self):
        return(self.Faxc,self.Fayc,self.Fazc)  


    def return_pull_test(self):
        return(self.Fb,self.PX,self.PY,self.MB)
  

# In[Potential Fields]

# Shape fields
class Shape_fields:
    def __init__(self,R,px,py,b,res,shape,sim):
        self.type='Shape_Fields'
        self.px=px # center x
        self.py=py # center y
        self.R=R
        self.sim=sim
        self.b=b # range of field
        self.res=res # resolution of the field so how big are the squares 
        self.shape=shape
        self.xmin=self.px-self.b # x max so how far to the right x
        self.xmax=self.px+self.b # x min so how far to the left x
        self.ymin=self.py-self.b # y max so how far to the right y
        self.ymax=self.py+self.b # y min so how far to the left y
        self.path='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/Desfield/'
        self.nr=[0,1,2]
        self.xcount=int(round((self.xmax-self.xmin)/self.res)) # number of x points 
        self.ycount=int(round((self.ymax-self.ymin)/self.res)) # number of y points
        
        self.xp=np.linspace(self.xmin,self.xmax,self.xcount) # create x axis point
        self.yp=np.linspace(self.ymin,self.ymax,self.ycount) # create y axis point
        self.xx,self.yy=np.meshgrid(self.xp,self.yp) # create grid for the field    
        (self.x,self.y,self.z)=self.points_for_region()
        self.XT=self.x[0:int(len(self.x)/len(self.nr))]
        self.YT=self.y[0:int(len(self.y)/len(self.nr))]
        np.savez(self.path+self.sim+'.npz',XT=self.XT,YT=self.YT)
        self.rbf = Rbf(self.x,self.y,self.z,function='thin_plate')  # radial basis function interpolator instance    
        self.zz = self.rbf(self.xx, self.yy) # create zz grid
        self.zz=np.nan_to_num(self.zz/np.max(self.zz)) # normalize the zz grid
        U, S, VT = np.linalg.svd(self.zz,full_matrices=False)
        S = np.diag(S)
        r=10
        self.zz = U[:,:r] @ S[0:r,:r] @ VT[:r,:]  
        (self.fy,self.fx)=np.gradient(self.zz**2)
        self.f= RegularGridInterpolator((self.yp,self.xp),self.zz)
        self.fnx= RegularGridInterpolator((self.yp,self.xp),self.fx)
        self.fny= RegularGridInterpolator((self.yp,self.xp),self.fy)   
        
    def update_field_gradient(self):
        self.xmin=self.px-self.b # x max so how far to the right x
        self.xmax=self.px+self.b # x min so how far to the left x
        self.ymin=self.py-self.b # y max so how far to the right y
        self.ymax=self.py+self.b # y min so how far to the left y
        self.xcount=int(round((self.xmax-self.xmin)/self.res)) # number of x points 
        self.ycount=int(round((self.ymax-self.ymin)/self.res)) # number of y points
        
        self.xp=np.linspace(self.xmin,self.xmax,self.xcount) # create x axis point
        self.yp=np.linspace(self.ymin,self.ymax,self.ycount) # create y axis point
        self.xx,self.yy=np.meshgrid(self.xp,self.yp) # create grid for the field    
        self.zz=np.nan_to_num(self.zz/np.max(self.zz)) # normalize the zz grid
        (self.fy,self.fx)=np.gradient(self.zz**2)
        self.f= RegularGridInterpolator((self.yp,self.xp),self.zz)
        self.fnx= RegularGridInterpolator((self.yp,self.xp),self.fx)
        self.fny= RegularGridInterpolator((self.yp,self.xp),self.fy)  
        return(self.f,self.fnx,self.fny) 
       
    def points_for_region(self):
        
        # create points for square
        if self.shape=='Square':
            (x,y,z)=self.points_square_region()
            
        # create points for triangle    
        if self.shape=='Triangle':
            (x,y,z)=self.points_triangle_region()  
            
        # create points for Circle    
        if self.shape=='Circle':
            (x,y,z)=self.points_circle_region() 
            
        # create points oval
        if self.shape=='Oval':
            (x,y,z)=self.points_Oval_region() 

        # create points star    
        if self.shape=='Star':              
            (x,y,z)=self.points_star_region()
            
        return(x,y,z)
    
    def points_square_region(self):
        ''' Function to Create points for Square '''
        xt=np.array([1,1,1,1,1,.75,.5,.25,0,-.25,-.5,-.75,-1,-1,-1,-1,-1,-1,-1,-1,-1,-.75,-.5,-.25,0,.25,.5,.75,1,1,1,1])
        yt=np.array([0,.25,.5,.75,1,1,1,1,1,1,1,1,1,.75,.5,.25,0,-.25,-.5,-.75,-1,-1,-1,-1,-1,-1,-1,-1,-1,-.75,-.5,-.25])
        x=np.zeros(len(self.nr)*len(xt))
        y=np.zeros(len(self.nr)*len(xt))
        z=np.zeros(len(self.nr)*len(xt))
        for j in range(len(self.nr)):
            for i in range(len(xt)):
                x[i+len(xt)*j]=(self.R+self.nr[j])*xt[i]
                y[i+len(xt)*j]=(self.R+self.nr[j])*yt[i]
                if j==0:
                    z[i+len(xt)*j]=0
                else:
                    z[i+len(xt)*j]=j       
        return (x,y,z) 
    
    def points_triangle_region(self):
        ''' Function to Create points for Traingle '''        
        #n=3
        m=3
        n1=4.5
        n2=10
        n3=10
        a=1.1
        b=1
        theta=np.linspace(0,2*np.pi,100)
        #(xt,yt)=self.super_ellipse(m,n1,n2,n3,a,b,theta)
        n=3
        r=np.cos(np.pi/n)/np.cos((theta%(2*np.pi/n))-(np.pi/n))
        xt=r*np.cos(theta)
        yt=r*np.sin(theta)
        x=np.zeros(len(self.nr)*len(xt))
        y=np.zeros(len(self.nr)*len(xt))
        z=np.zeros(len(self.nr)*len(xt))

        for j in range(len(self.nr)):
            for i in range(len(xt)):
                x[i+len(xt)*j]=(self.nr[j]+self.R)*xt[i]
                y[i+len(xt)*j]=(self.nr[j]+self.R)*yt[i]
                if j==0:
                    z[i+len(xt)*j]=0
                else:
                    z[i+len(xt)*j]=j 
                           
        return (x,y,z)  


    def points_star_region(self):
        ''' Function to Create points for Star '''        
        m=5
        n1=2
        n2=7
        n3=7
        a=.9
        b=a
        C=m/4
        theta=np.linspace(0,2*np.pi,100)
        R = (abs(np.cos(C*theta)/a)**n2 + abs(np.sin(C*theta)/b)**n3)**(-1/n1)
        xt=R*np.cos(theta)
        yt=R*np.sin(theta)
        nr=np.linspace(1,5,5)
        x=np.zeros(len(nr)*len(xt))
        y=np.zeros(len(nr)*len(xt))
        z=np.zeros(len(nr)*len(xt))

        for j in range(len(nr)):
            for i in range(len(xt)):
                x[i+len(xt)*j]=(nr[j])*xt[i]
                y[i+len(xt)*j]=(nr[j])*yt[i]
                if j==0:
                    z[i+len(xt)*j]=0
                else:
                    z[i+len(xt)*j]=j  
                      
        return (y,x,z) 


 

    
    def points_circle_region(self):
        ''' Function to Create points for Circle'''        
        theta=np.linspace(0,2*np.pi,30)
        xt=self.R*np.cos(theta)
        yt=self.R*np.sin(theta)
        x=np.zeros(len(self.nr)*len(xt))
        y=np.zeros(len(self.nr)*len(xt))
        z=np.zeros(len(self.nr)*len(xt))
        for j in range(len(self.nr)):
            for i in range(len(xt)):
                x[i+len(xt)*j]=(self.R+self.nr[j])*xt[i]
                y[i+len(xt)*j]=(self.R+self.nr[j])*yt[i]
                if j==0:
                    z[i+len(xt)*j]=0
                else:
                    z[i+len(xt)*j]=j       
        return (x,y,z) 



    def points_Oval_region(self):
        ''' Function to Create points for Oval '''
        theta=np.linspace(0,2*np.pi,30)
        xt=self.R*np.cos(theta)
        yt=self.R*np.sin(theta)
        x=np.zeros(len(self.nr)*len(xt))
        y=np.zeros(len(self.nr)*len(xt))
        z=np.zeros(len(self.nr)*len(xt))
        for j in range(len(self.nr)):
            for i in range(len(xt)):
                x[i+len(xt)*j]=(1.25*self.R+self.nr[j])*xt[i]
                y[i+len(xt)*j]=(.55*self.R+self.nr[j])*yt[i]
                if j==0:
                    z[i+len(xt)*j]=0
                else:
                    z[i+len(xt)*j]=j       
        return (x,y,z) 



            
    def plot_2D(self):
        fig = plt.figure(figsize = (8, 8))     # Set the figure size to square
        plt.pcolor(self.xx,self.yy, self.zz, cmap = 'jet')         # Pseudocolor plot of data and choose colormap
        #plt.plot(self.xt,self.yt,color='white')
        plt.colorbar()  
        plt.title('plot')
        plt.xlabel("$x$")
        plt.ylabel("$y$")
        plt.show()


    def plot_2D_lines(self):
        fig, ax = plt.subplots(figsize = (8, 8))
        CS = ax.contour(self.xx,self.yy,self.zz,10)
        ax.clabel(CS, inline=1, fontsize=10)
        plt.title('plot')
        plt.xlabel("$x$")
        plt.ylabel("$y$")

    
# Analytic fields         
class analytic_field:
    def __init__(self,a,c,px,py,theta,b,res):
        self.type='Shape_Fields_analytic'
        self.px=px # center x
        self.py=py # center y
        self.a=a # radii 1
        self.b=b # range of field
        self.c=c # radii 2
        self.theta=theta # rotation
        self.res=res # resolution of the field so how big are the squares 

        self.xmin=self.px-self.b # x max so how far to the right x
        self.xmax=self.px+self.b # x min so how far to the left x
        self.ymin=self.py-self.b # y max so how far to the right y
        self.ymax=self.py+self.b # y min so how far to the left y
        
        self.xcount=int(round((self.xmax-self.xmin)/self.res)) # number of x points 
        self.ycount=int(round((self.ymax-self.ymin)/self.res)) # number of y points
        
        self.xp=np.linspace(self.xmin,self.xmax,self.xcount) # create x axis point
        self.yp=np.linspace(self.ymin,self.ymax,self.ycount) # create y axis point
        self.xt=np.linspace(-self.b,self.b,self.xcount)
        self.yt=np.linspace(-self.b,self.b,self.ycount)
        self.xxt,self.yyt=np.meshgrid(self.xt,self.yt) # create grid for the field 
        self.xx=0
        self.yy=0
        Y=(self.yyt)*np.sin(self.theta)+(self.yyt)*np.cos(self.theta) # Y value of distance
        X=(self.xxt)*np.cos(self.theta)-(self.yyt)*np.sin(self.theta) # X value of distance
        d=np.sqrt((X**2) / (self.a**2) + (Y**2)/(self.c**2)) # d value 
        self.zz = d**2 * np.log(d) # create field gird form 
        self.zz=self.zz/np.max(self.zz)
        (self.fy,self.fx)=np.gradient(self.zz**2) # take gradient 
        self.fx=self.fx/np.max(self.fx) # normaluze gradient x term
        self.fy=self.fy/np.max(self.fy) # normalize gradient y term 
        self.f = RegularGridInterpolator((self.yp,self.xp),self.zz) # form potential function 
        self.fnx = RegularGridInterpolator((self.yp,self.xp),self.fx) # form gradient of potnetial function x
        self.fny = RegularGridInterpolator((self.yp,self.xp),self.fy)  # form gradient of potential function y
        thetatemp=np.linspace(0,2*np.pi,100)
        xt=self.a*np.cos(thetatemp)
        xy=self.c*np.sin(thetatemp)

        #np.savez('F:/Soro_chrono/python/Pychrono/Strings/Strings_final/Desfield/'+self.sim+'.npz',XT=self.XT,YT=self.YT)
    def update_field_gradient(self):
        self.xmin=self.px-self.b # x max so how far to the right x
        self.xmax=self.px+self.b # x min so how far to the left x
        self.ymin=self.py-self.b # y max so how far to the right y
        self.ymax=self.py+self.b # y min so how far to the left y
        
        self.xcount=int(round((self.xmax-self.xmin)/self.res)) # number of x points 
        self.ycount=int(round((self.ymax-self.ymin)/self.res)) # number of y points
        
        self.xp=np.linspace(self.xmin,self.xmax,self.xcount) # create x axis point
        self.yp=np.linspace(self.ymin,self.ymax,self.ycount) # create y axis point
        self.xx,self.yy=np.meshgrid(self.xp,self.yp) # create grid for the field 
        self.f = RegularGridInterpolator((self.yp,self.xp),self.zz) # form potential function 
        self.fnx = RegularGridInterpolator((self.yp,self.xp),self.fx) # form gradient of potnetial function x
        self.fny = RegularGridInterpolator((self.yp,self.xp),self.fy)  # form gradient of potential function y
        return(self.f,self.fny,self.fnx)
    
class GRASPING_FIELD:
    def __init__(self,a,c,px,py,Rb,bx,by,alpha2):
        self.type='GRASP FIELD'
        self.px=px # center x
        self.py=py # center y
        self.bx=bx
        self.by=by
        self.a=a # radii 1
        self.c=c # radii 2
        self.Rb=Rb
        self.alpha2=alpha2
    def F(self,x,y,px,py,a,c):
        Y=(y-py)
        X=(x-px)
        
        d=np.sqrt((X**2) / (a**2) + (Y**2)/(c**2)) 
        return(d**2 * np.log(d)) # create field gird form      
  
    def Fx(self,x,y,px,py,a,c):
        Y=(y-py)
        X=(x-px)
        d=np.sqrt((X**2)/(a**2) + (Y**2)/(c**2)) 
        return(((2*x-2*px)/a**2)*np.log(d) + (2*x-2*px)/2*(a)**2)
  
    def Fy(self,x,y,px,py,a,c):
        Y=(y-py)
        X=(x-px)
        d=np.sqrt((X**2)/(a**2) + (Y**2)/(c**2)) 
        return(((2*y-2*py)/c**2)*np.log(d) + (2*y-2*py)/2*(c)**2)    
    
    def Fx2(self,x,y,px,py,a,c):
        return(self.F(x,y,px,py,a,c)*self.Fx(x,y,px,py,a,c))
    
    def Fy2(self,x,y,px,py,a,c):
        return(self.F(x,y,px,py,a,c)*self.Fy(x,y,px,py,a,c)) 
    
    
#Import fields#
class imported_fields:
    def __init__(self,field):
        self.type='import_field'
        self.field=field
        self.path='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/fields/'
        data = np.load(self.path+self.field+'.npz') # specify data file 
        self.zz=data['zz'] # z values in grid
        self.xx=data['xx'] # x points in grid
        self.yy=data['yy'] # y points in grid      
        self.xp=data['xp']
        self.yp=data['yp']
        self.zz=self.zz/np.max(self.zz)
        (self.fy,self.fx)=np.gradient(self.zz**2) # take gradient 
        self.fx=self.fx/np.max(self.fx) # normaluze gradient x term
        self.fy=self.fy/np.max(self.fy) # normalize gradient y term 
        self.f = RegularGridInterpolator((self.yp,self.xp),self.zz) # form potential function 
        self.fnx = RegularGridInterpolator((self.yp,self.xp),self.fx) # form gradient of potnetial function x
        self.fny = RegularGridInterpolator((self.yp,self.xp),self.fy)  # form gradient of potential function y       

class imported_fields_letters:
    def __init__(self,field,sim):
        self.type='imported_letters'
        self.field=field
        self.path='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/fields/'
        data = np.load(self.path+self.field+'.npz') # specify data file 
        self.Z=data['Z'] # z values in grid
        self.xx=data['X'] # x points in grid
        self.yy=data['Y'] # y points in grid      
        self.XP=data['XP']
        self.YP=data['YP']
        self.xp=data['xp']
        self.yp=data['yp']
        self.sim=sim
        self.px=0
        self.py=0
        self.FY=[] # empty array for arrayu for gradient Y field points
        self.FX=[] # empty array for each array for gradient X field points
        self.F=[] # Empty array for continous function 
        self.FNX=[] # empty array for continous gradient x
        self.FNY=[] # empty array for continous gradient y
        self.create_gradients()
        np.savez('F:/Soro_chrono/python/Pychrono/Strings/Strings_final/Desfield/'+self.sim+'.npz',XT=self.XP,YT=self.YP)
    def create_gradients(self):
        for i in range(len(self.Z)):
            (fy,fx)=np.gradient(self.Z[i]) # take gradient 
            fx=fx/np.max(fx) # normaluze gradient x term
            fy=fy/np.max(fy) # normalize gradient y term 
            f = RegularGridInterpolator((self.yp,self.xp),self.Z[i]) # form potential function 
            fnx = RegularGridInterpolator((self.yp,self.xp),fx) # form gradient of potnetial function x
            fny = RegularGridInterpolator((self.yp,self.xp),fy)  # form gradient of potential function y 
            self.FX.append(fx)
            self.FY.append(fy)
            self.F.append(f)
            self.FNX.append(fnx)
            self.FNY.append(fny)


    def M(self,t,x,z,mode): 
        F1=self.F[0]
        F2=self.F[1]
        F3=self.F[2]
        F4=self.F[3]
        F5=self.F[4]
        F6=self.F[5]
        F7=self.F[6]   
        F8=self.F[7]
        # J
        if mode==1:
            M=(1-t)*F1([x,z])+t*F2([x,z])    
        # A
        if mode==2:
            M=(1-t)*F2([x,z])+t*F3([x,z])    
        # M
        if mode==3:
            M=(1-t)*F3([x,z])+t*F4([x,z])    
        # O
        if mode==4:
            M=(1-t)*F4([x,z])+t*F5([x,z])    
        # E    
        if mode==5:
            M=(1-t)*F5([x,z])+t*F6([x,z])    
        # B
        if mode==6:
            M=(1-t)*F6([x,z])+t*F7([x,z])    
        # A
        if mode==7:
            M=(1-t)*F7([x,z])+t*F8([x,z])    
          
        return(M)

    
    def Mx(self,t,x,z,mode):
        
        F1=self.F[0]
        F2=self.F[1]
        F3=self.F[2]
        F4=self.F[3]
        F5=self.F[4]
        F6=self.F[5]
        F7=self.F[6]   
        F8=self.F[7]
        
        fnx1=self.FNX[0]
        fnx2=self.FNX[1]   
        fnx3=self.FNX[2]
        fnx4=self.FNX[3] 
        fnx5=self.FNX[4]
        fnx6=self.FNX[5] 
        fnx7=self.FNX[6]
        fnx8=self.FNX[7]         
        
        if mode==1:
            Mx=((1-t)**2)* F1([x,z])*fnx1([x,z])+(t**2)*F2([x,z])*fnx2([x,z])+t*(1-t)*F2([x,z])*fnx1([x,z]) + 2*t*(1-t)*F1([x,z])*fnx2([x,z])
        
        if mode==2:        
            Mx=((1-t)**2)* F2([x,z])*fnx2([x,z])+(t**2)*F3([x,z])*fnx3([x,z])+t*(1-t)*F3([x,z])*fnx2([x,z]) + 2*t*(1-t)*F2([x,z])*fnx3([x,z])
        
        if mode==3:        
            Mx=((1-t)**2)* F3([x,z])*fnx3([x,z])+(t**2)*F4([x,z])*fnx4([x,z])+t*(1-t)*F4([x,z])*fnx3([x,z]) + 2*t*(1-t)*F3([x,z])*fnx4([x,z])        
        
        if mode==4:        
            Mx=((1-t)**2)* F4([x,z])*fnx4([x,z])+(t**2)*F5([x,z])*fnx5([x,z])+t*(1-t)*F5([x,z])*fnx4([x,z]) + 2*t*(1-t)*F4([x,z])*fnx5([x,z])
        
        if mode==5:        
            Mx=((1-t)**2)* F5([x,z])*fnx5([x,z])+(t**2)*F6([x,z])*fnx6([x,z])+t*(1-t)*F6([x,z])*fnx5([x,z]) + 2*t*(1-t)*F5([x,z])*fnx6([x,z])
        
        if mode==6:       
            Mx=((1-t)**2)* F6([x,z])*fnx6([x,z])+(t**2)*F7([x,z])*fnx7([x,z])+t*(1-t)*F7([x,z])*fnx6([x,z]) + 2*t*(1-t)*F6([x,z])*fnx7([x,z])
        
        if mode==7:
            Mx=((1-t)**2)* F7([x,z])*fnx7([x,z])+(t**2)*F8([x,z])*fnx8([x,z])+t*(1-t)*F8([x,z])*fnx7([x,z]) + 2*t*(1-t)*F7([x,z])*fnx8([x,z])        
        
        return(Mx)


    def My(self,t,x,z,mode):
        
        F1=self.F[0]
        F2=self.F[1]
        F3=self.F[2]
        F4=self.F[3]
        F5=self.F[4]
        F6=self.F[5]
        F7=self.F[6]   
        F8=self.F[7]
        
        fny1=self.FNY[0]
        fny2=self.FNY[1]   
        fny3=self.FNY[2]
        fny4=self.FNY[3] 
        fny5=self.FNY[4]
        fny6=self.FNY[5] 
        fny7=self.FNY[6]
        fny8=self.FNY[7]         
        
        if mode==1:
            My=((1-t)**2)* F1([x,z])*fny1([x,z])+(t**2)*F2([x,z])*fny2([x,z])+t*(1-t)*F2([x,z])*fny1([x,z]) + 2*t*(1-t)*F1([x,z])*fny2([x,z])
        
        if mode==2:        
            My=((1-t)**2)* F2([x,z])*fny2([x,z])+(t**2)*F3([x,z])*fny3([x,z])+t*(1-t)*F3([x,z])*fny2([x,z]) + 2*t*(1-t)*F2([x,z])*fny3([x,z])
        
        if mode==3:        
            My=((1-t)**2)* F3([x,z])*fny3([x,z])+(t**2)*F4([x,z])*fny4([x,z])+t*(1-t)*F4([x,z])*fny3([x,z]) + 2*t*(1-t)*F3([x,z])*fny4([x,z])        
        
        if mode==4:        
            My=((1-t)**2)* F4([x,z])*fny4([x,z])+(t**2)*F5([x,z])*fny5([x,z])+t*(1-t)*F5([x,z])*fny4([x,z]) + 2*t*(1-t)*F4([x,z])*fny5([x,z])
        
        if mode==5:        
            My=((1-t)**2)* F5([x,z])*fny5([x,z])+(t**2)*F6([x,z])*fny6([x,z])+t*(1-t)*F6([x,z])*fny5([x,z]) + 2*t*(1-t)*F5([x,z])*fny6([x,z])
        
        if mode==6:       
            My=((1-t)**2)* F6([x,z])*fny6([x,z])+(t**2)*F7([x,z])*fny7([x,z])+t*(1-t)*F7([x,z])*fny6([x,z]) + 2*t*(1-t)*F6([x,z])*fny7([x,z])
        
        if mode==7:
            My=((1-t)**2)* F7([x,z])*fny7([x,z])+(t**2)*F8([x,z])*fny8([x,z])+t*(1-t)*F8([x,z])*fny7([x,z]) + 2*t*(1-t)*F7([x,z])*fny8([x,z]) 
            
        return(My)


    
   
    



#Image warping import 
class image_warping_import:
    def __init__(self,name,R,px,py):
        self.type='import_field_complex'
        self.name=name
        self.path='F:/Soro_chrono/python/Pychrono/Strings/Strings_final/fields/'
        data = np.load(self.path+self.name+'.npz') # specify data file 
        self.zz=data['zz'] # z values in grid
        self.xx=data['xx'] # x points in grid
        self.yy=data['yy'] # y points in grid      
        self.xp=data['xp']
        self.yp=data['yp']
        self.Xmin=data['ymin']
        self.Ymin=data['ymin']
        self.Xmax=data['ymax']
        self.Ymax=data['ymax']
        self.res=data['res']
        self.R=R
        self.px=px
        self.py=py
        self.theta=0

        self.xcount=int(round(len(self.xp))) # number of x points 
        self.ycount=int(round(len(self.yp))) # number of y points
        
        self.xmin=self.px - abs(self.Xmin) # x max so how far to the right x
        self.xmax=self.px + self.Xmax# x min so how far to the left x

        
        self.ymin=self.py-abs(self.Ymin) # y max so how far to the right y
        self.ymax=self.py+self.Ymax # y min so how far to the left y

        self.xp=np.linspace(self.xmin,self.xmax,self.xcount) # create x axis point
        self.yp=np.linspace(self.ymin,self.ymax,self.ycount) # create y axis point
        #print(self.xp)
        self.xx,self.yy=np.meshgrid(self.xp,self.yp) # create grid for the field 

        #p1=(self.xx)**2+(self.yy)**2-(self.R)**2
        #p1=self.k*np.maximum(0,f1)
        #p1=p1/np.max(p1)
        Y=(self.yy) # Y value of distance
        X=(self.xx)
        d=np.sqrt((((self.xx)**2)/(self.R**2))+(((self.yy)**2)/(self.R**2))) # d value 
        zz = np.nan_to_num((d**2)*np.log(d)) # create field gird form 
        zz=zz/np.max(zz)        
        self.ZZ=[]
        self.ZZ.append(zz)
        self.ZZ.append(self.zz)

        self.FY=[] # empty array for arrayu for gradient Y field points
        self.FX=[] # empty array for each array for gradient X field points
        self.F=[] # Empty array for continous function 
        self.FNX=[] # empty array for continous gradient x
        self.FNY=[] # empty array for continous gradient y        
        self.create_gradients()

    def create_gradients(self):
        for i in range(2):
            print('field_num=',i)
            (fy,fx)=np.gradient(self.ZZ[i]) # take gradient 
            fx=fx/np.max(fx) # normaluze gradient x term
            fy=fy/np.max(fy) # normalize gradient y term 
            f = RegularGridInterpolator((self.yp,self.xp),self.ZZ[i]) # form potential function 
            fnx = RegularGridInterpolator((self.yp,self.xp),fx) # form gradient of potnetial function x
            fny = RegularGridInterpolator((self.yp,self.xp),fy)  # form gradient of potential function y 
            self.FX.append(fx)
            self.FY.append(fy)
            self.F.append(f)
            self.FNX.append(fnx)
            self.FNY.append(fny)

        
    # def create_gradients(self):
    #     for i in range(2):
    #         (fy,fx)=np.gradient(self.ZZ[i]**2) # take gradient 
    #         fx=fx/np.max(fx) # normaluze gradient x term
    #         fy=fy/np.max(fy) # normalize gradient y term 
    #         f = RegularGridInterpolator((self.yp,self.xp),self.ZZ[i]) # form potential function 
    #         fnx = RegularGridInterpolator((self.yp,self.xp),fx) # form gradient of potnetial function x
    #         fny = RegularGridInterpolator((self.yp,self.xp),fy)  # form gradient of potential function y 
    #         self.FX.append(fx)
    #         self.FY.append(fy)
    #         self.F.append(f)
    #         self.FNX.append(fnx)
    #         self.FNY.append(fny)

    def update_field(self):
        self.xmin=self.px - abs(self.Xmin) # x max so how far to the right x
        self.xmax=self.px + self.Xmax# x min so how far to the left x

        
        self.ymin=self.py-abs(self.Ymin) # y max so how far to the right y
        self.ymax=self.py+self.Ymax # y min so how far to the left y

        self.xp=np.linspace(self.xmin,self.xmax,self.xcount) # create x axis point
        self.yp=np.linspace(self.ymin,self.ymax,self.ycount) # create y axis point
        #print(self.xp)
        self.xx,self.yy=np.meshgrid(self.xp,self.yp) # create grid for the field        
        for i in range(2):
            (fy,fx)=np.gradient(self.ZZ[i]) # take gradient 
            fx=fx/np.max(fx) # normaluze gradient x term
            fy=fy/np.max(fy) # normalize gradient y term 
            f = RegularGridInterpolator((self.yp,self.xp),self.ZZ[i]) # form potential function 
            fnx = RegularGridInterpolator((self.yp,self.xp),fx) # form gradient of potnetial function x
            fny = RegularGridInterpolator((self.yp,self.xp),fy)  # form gradient of potential function y 
            self.FX.append(fx)
            self.FY.append(fy)
            self.F.append(f)
            self.FNX.append(fnx)
            self.FNY.append(fny)     
        
    def M(self,t,x,z): 
        F1=self.F[0]
        F2=self.F[1]
        M=(1-t)*F1([x,z])+t*F2([x,z])    
        return(M)

    
    def Mx(self,t,x,z):
        F1=self.F[0]
        F2=self.F[1]
        fnx1=self.FNX[0]
        fnx2=self.FNX[1]        
        Mx=((1-t)**2)* F1([x,z])*fnx1([x,z])+(t**2)*F2([x,z])*fnx2([x,z])+t*(1-t)*F2([x,z])*fnx1([x,z]) + 2*t*(1-t)*F1([x,z])*fnx2([x,z])
        return(Mx)
    
    def My(self,t,x,z):
        F1=self.F[0]
        F2=self.F[1]
        fny1=self.FNY[0]
        fny2=self.FNY[1]        
        My=((1-t)**2)*F1([x,z])*fny1([x,z])+(t**2)*F2([x,z])*fny2([x,z]) + t*(1-t)*F2([x,z])*fny1([x,z]) + 2*t*(1-t)*F1([x,z])*fny2([x,z])
        return(My)    
    
    



# Image warping 
class image_warping:
    def __init__(self,px,py,b,res,shape,sim,a=None,c=None,theta=None,R=None,R2=None):
        self.type='Warping field'
        self.shape=shape
        self.px=px # center x
        self.py=py # center y
        self.R=R
        self.R2=R2
        self.a=a # radii 1
        self.c=c # radii 2
        self.theta=theta # rotation
        self.sim=sim
        self.nr=[0,1,2,3] # ring numbers
        self.b=b # range of field
        self.res=res # resolution of the field so how big are the squares 
        self.xmin=self.px-self.b # x max so how far to the right x
        self.xmax=self.px+self.b # x min so how far to the left x
        self.ymin=self.py-self.b # y max so how far to the right y
        self.ymax=self.py+self.b # y min so how far to the left y
        
        self.xcount=int(round((self.xmax-self.xmin)/self.res)) # number of x points 
        self.ycount=int(round((self.ymax-self.ymin)/self.res)) # number of y points
        
        self.xp=np.linspace(self.xmin,self.xmax,self.xcount) # create x axis point
        self.yp=np.linspace(self.ymin,self.ymax,self.ycount) # create y axis point
        self.xx,self.yy=np.meshgrid(self.xp,self.yp) # create grid for the field 
        self.XT=[]
        self.YT=[]
        self.ZZ=[] # Empty array for each field grid points 
        self.FY=[] # empty array for arrayu for gradient Y field points
        self.FX=[] # empty array for each array for gradient X field points
        self.F=[] # Empty array for continous function 
        self.FNX=[] # empty array for continous gradient x
        self.FNY=[] # empty array for continous gradient y
        self.t=np.linspace(0,1,100)
        self.fields()
        self.create_gradients()
        np.savez('F:/Soro_chrono/python/Pychrono/Strings/Strings_final/Desfield/'+self.sim+'.npz',XT=self.XT,YT=self.YT)
        #(self.fy,self.fx)=np.gradient(self.zz**2) # take gradient 
        #self.fx=self.fx/np.max(self.fx) # normaluze gradient x term
        #self.fy=self.fy/np.max(self.fy) # normalize gradient y term 
        #self.f = RegularGridInterpolator((self.yp,self.xp),self.zz) # form potential function 
        #self.fnx = RegularGridInterpolator((self.yp,self.xp),self.fx) # form gradient of potnetial function x
        #self.fny = RegularGridInterpolator((self.yp,self.xp),self.fy)  # form gradient of potential function y
            
    def fields(self):
        for i in range(len(self.shape)):
            
            if self.shape[i]=='Circle':
                (zz)=self.circle_field() 
                self.ZZ.append(zz)
                
            if self.shape[i]=='Square':
                (zz)=self.square_field()
                self.ZZ.append(zz)
                
            if self.shape[i]=='Triangle':
                (zz)=self.triangle_field()
                self.ZZ.append(zz)
            
            if self.shape[i]=='EllipseS':
                (zz)=self.EllipseS_field()
                self.ZZ.append(zz)                
    
            if self.shape[i]=='EllipseL':
                (zz)=self.EllipseL_field()
                self.ZZ.append(zz)  
                
            if self.shape[i]=='Star':
                (zz)=self.Star_field()
                self.ZZ.append(zz)
                
                
    def create_gradients(self):
        for i in range(len(self.shape)):
            (fy,fx)=np.gradient(self.ZZ[i]) # take gradient 
            fx=fx/np.max(fx) # normaluze gradient x term
            fy=fy/np.max(fy) # normalize gradient y term 
            f = RegularGridInterpolator((self.yp,self.xp),self.ZZ[i]) # form potential function 
            fnx = RegularGridInterpolator((self.yp,self.xp),fx) # form gradient of potnetial function x
            fny = RegularGridInterpolator((self.yp,self.xp),fy)  # form gradient of potential function y 
            self.FX.append(fx)
            self.FY.append(fy)
            self.F.append(f)
            self.FNX.append(fnx)
            self.FNY.append(fny)


    def M3(self,t,x,z,mode):    
        F1=self.F[0]
        F2=self.F[1] 
        F3=self.F[2] 
        F4=self.F[3]
        if mode==0:
            M=(1-t)*F1([x,z])+t*F2([x,z])
        if mode==1:
            M=(1-t)*F2([x,z])+t*F3([x,z])
        if mode==2:
            M=(1-t)*F3([x,z])+t*F4([x,z])        
        return(M)

    def plot_2D(self,i):
        fig = plt.figure(figsize = (8, 8))     # Set the figure size to square
        plt.pcolor(self.xx,self.yy, self.ZZ[i], cmap = 'jet')         # Pseudocolor plot of data and choose colormap
        #plt.plot(self.xt,self.yt,color='white')
        plt.colorbar()  
        plt.title('plot')
        plt.xlabel("$x$")
        plt.ylabel("$y$")
        plt.show()


    def plot_2D_lines(self,i):
        fig, ax = plt.subplots(figsize = (8, 8))
        CS = ax.contour(self.xx,self.yy,self.ZZ[i],20)
        ax.clabel(CS, inline=1, fontsize=10)
        plt.title('plot')
        plt.xlabel("$x$")
        plt.ylabel("$y$")
        
    def Mx3(self,t,x,z,mode):
        
        F1=self.F[0]
        F2=self.F[1]
        F3=self.F[2]
        F4=self.F[3]
        
        fnx1=self.FNX[0]
        fnx2=self.FNX[1]  
        fnx3=self.FNX[2] 
        fnx4=self.FNX[3]
        
        if mode==0:
            Mx=((1-t)**2)* F1([x,z])*fnx1([x,z])+(t**2)*F2([x,z])*fnx2([x,z])+t*(1-t)*F2([x,z])*fnx1([x,z]) + 2*t*(1-t)*F1([x,z])*fnx2([x,z])
        
        if mode==1:
            Mx=((1-t)**2)* F2([x,z])*fnx2([x,z])+(t**2)*F3([x,z])*fnx3([x,z])+t*(1-t)*F3([x,z])*fnx2([x,z]) + 2*t*(1-t)*F2([x,z])*fnx3([x,z])            
        
        if mode==2:
            Mx=((1-t)**2)* F3([x,z])*fnx3([x,z])+(t**2)*F4([x,z])*fnx4([x,z])+t*(1-t)*F4([x,z])*fnx3([x,z]) + 2*t*(1-t)*F3([x,z])*fnx4([x,z])             
        return(Mx)
    
    def My3(self,t,x,z,mode):
        
        F1=self.F[0]
        F2=self.F[1]
        F3=self.F[2]
        F4=self.F[3]
        fny1=self.FNY[0]
        fny2=self.FNY[1]  
        fny3=self.FNY[2] 
        fny4=self.FNY[3]
        
        if mode==0:
            My=((1-t)**2)* F1([x,z])*fny1([x,z])+(t**2)*F2([x,z])*fny2([x,z])+t*(1-t)*F2([x,z])*fny1([x,z]) + 2*t*(1-t)*F1([x,z])*fny2([x,z])
        
        if mode==1:
            My=((1-t)**2)* F2([x,z])*fny2([x,z])+(t**2)*F3([x,z])*fny3([x,z])+t*(1-t)*F3([x,z])*fny2([x,z]) + 2*t*(1-t)*F2([x,z])*fny3([x,z])            

        if mode==2:
            My=((1-t)**2)* F3([x,z])*fny3([x,z])+(t**2)*F4([x,z])*fny4([x,z])+t*(1-t)*F4([x,z])*fny3([x,z]) + 2*t*(1-t)*F3([x,z])*fny4([x,z])            
        
        return(My) 


    def M(self,t,x,z):    
        F1=self.F[0]
        F2=self.F[1] 
        M=(1-t)*F1([x,z])+t*F2([x,z])
        return(M)


    def Mx(self,t,x,z):
        
        F1=self.F[0]
        F2=self.F[1]
        fnx1=self.FNX[0]
        fnx2=self.FNX[1]  
 
        Mx=((1-t)**2)* F1([x,z])*fnx1([x,z])+(t**2)*F2([x,z])*fnx2([x,z])+t*(1-t)*F2([x,z])*fnx1([x,z]) + 2*t*(1-t)*F1([x,z])*fnx2([x,z])
        return(Mx)
    
    def My(self,t,x,z):
        
        F1=self.F[0]
        F2=self.F[1]

        
        fny1=self.FNY[0]
        fny2=self.FNY[1]  
        My=((1-t)**2)* F1([x,z])*fny1([x,z])+(t**2)*F2([x,z])*fny2([x,z])+t*(1-t)*F2([x,z])*fny1([x,z]) + 2*t*(1-t)*F1([x,z])*fny2([x,z])
        
        return(My) 
            
    # def M(self,t,x,z): 
    #     F1=self.F[0]
    #     F2=self.F[1]
    #     M=(1-t)*F1([x,z])+t*F2([x,z])    
    #     return(M)

    
    # def Mx(self,t,x,z):
    #     fnx1=self.FNX[0]
    #     fnx2=self.FNX[1]        
    #     Mx=(1-t)*fnx1([x,z])+t*fnx2([x,z])
    #     return(Mx)
    
    # def My(self,t,x,z):
    #     fny1=self.FNY[0]
    #     fny2=self.FNY[1]
    #     My=(1-t)*fny1([x,z])+t*fny2([x,z])
    #     return(My)
    
    def EllipseS_field(self):    
        Y=(self.xx)*np.sin(self.theta)+(self.yy)*np.cos(self.theta) # Y value of distance
        X=(self.xx)*np.cos(self.theta)-(self.yy)*np.sin(self.theta) # X value of distance
        d=np.sqrt((X**2) / (self.a**2) + (Y**2)/(self.c**2)) # d value 
        theta=np.linspace(0,2*np.pi,100)
        self.XT.append(self.a*np.cos(theta))
        self.YT.append(self.c*np.sin(theta))
        zz = d**2 * np.log(d) # create field gird form 
        zz=zz/np.max(zz)
        return(zz) 

    def EllipseL_field(self):    
        Y=(self.xx)*np.sin(self.theta)+(self.yy)*np.cos(self.theta) # Y value of distance
        X=(self.xx)*np.cos(self.theta)-(self.yy)*np.sin(self.theta) # X value of distance
        d=np.sqrt((X**2) / (self.c**2) + (Y**2)/(self.a**2)) # d value 
        theta=np.linspace(0,2*np.pi,100)
        self.XT.append(self.c*np.cos(theta))
        self.YT.append(self.a*np.sin(theta))
        zz = d**2 * np.log(d) # create field gird form 
        zz=zz/np.max(zz)
        return(zz) 
    
        
    def circle_field(self):
        Y=(self.yy-self.py)
        X=(self.xx-self.px)
        d=np.sqrt((X**2)/(self.R**2) + (Y**2)/(self.R**2)) # d value 
        theta=np.linspace(0,2*np.pi,100)
        self.XT.append(self.R*np.cos(theta))
        self.YT.append(self.R*np.sin(theta))
        zz = d**2 * np.log(d) # create field gird form 
        zz=zz/np.max(zz)
        return(zz)
 

    def triangle_field(self):
        m=3
        n1=4.5
        n2=10
        n3=10
        a=1
        b=1
        theta=np.linspace(0,2*np.pi,100)
        #(xt,yt)=self.super_ellipse(m,n1,n2,n3,a,b,theta)
        n=3
        r=np.cos(np.pi/n)/np.cos((theta%(2*np.pi/n))-(np.pi/n))
        xt=r*np.cos(theta)
        yt=r*np.sin(theta)
        x=np.zeros(len(self.nr)*len(xt))
        y=np.zeros(len(self.nr)*len(xt))
        z=np.zeros(len(self.nr)*len(xt))
        for j in range(len(self.nr)):
            for i in range(len(xt)):
                x[i+len(xt)*j]=(self.R2+self.nr[j])*xt[i]
                y[i+len(xt)*j]=(self.R2+self.nr[j])*yt[i]
                if j==0:
                    z[i+len(xt)*j]=0
                else:
                    z[i+len(xt)*j]=j   
                    
        self.XT.append(x[0:int(len(x)/len(self.nr))])
        self.YT.append(y[0:int(len(y)/len(self.nr))])            
        rbf = Rbf(x,y,z,function='thin_plate')  # radial basis function interpolator instance    
        zz = rbf(self.xx, self.yy) # create zz grid
        zz=np.nan_to_num(zz/np.max(zz)) # normalize the zz grid            
        return(zz)
    

    def square_field(self):
        xt=np.array([1,1,1,1,1,.75,.5,.25,0,-.25,-.5,-.75,-1,-1,-1,-1,-1,-1,-1,-1,-1,-.75,-.5,-.25,0,.25,.5,.75,1,1,1,1])
        yt=np.array([0,.25,.5,.75,1,1,1,1,1,1,1,1,1,.75,.5,.25,0,-.25,-.5,-.75,-1,-1,-1,-1,-1,-1,-1,-1,-1,-.75,-.5,-.25])
        x=np.zeros(len(self.nr)*len(xt))
        y=np.zeros(len(self.nr)*len(xt))
        z=np.zeros(len(self.nr)*len(xt))
        for j in range(len(self.nr)):
            for i in range(len(xt)):
                x[i+len(xt)*j]=(self.R+self.nr[j])*xt[i]
                y[i+len(xt)*j]=(self.R+self.nr[j])*yt[i]
                if j==0:
                    z[i+len(xt)*j]=0
                else:
                    z[i+len(xt)*j]=j          
        
        self.XT.append(x[0:int(len(x)/len(self.nr))])
        self.YT.append(y[0:int(len(y)/len(self.nr))])
        rbf = Rbf(x,y,z,function='thin_plate')  # radial basis function interpolator instance    
        zz = rbf(self.xx, self.yy) # create zz grid
        zz=np.nan_to_num(zz/np.max(zz)) # normalize the zz grid            
        return(zz)

    def Star_field(self):
        m=5
        n1=2
        n2=7
        n3=7
        a=.75
        b=a
        C=m/4
        theta=np.linspace(0,2*np.pi,100)
        R = (abs(np.cos(C*theta)/a)**n2 + abs(np.sin(C*theta)/b)**n3)**(-1/n1)
        xt=R*np.cos(theta)
        yt=R*np.sin(theta)
        nr=np.linspace(1,5,5)
        #nr=[1,2,3]
        x=np.zeros(len(nr)*len(xt))
        y=np.zeros(len(nr)*len(xt))
        z=np.zeros(len(nr)*len(xt))
        for j in range(len(nr)):
            for i in range(len(xt)):
                x[i+len(xt)*j]=(nr[j])*xt[i]
                y[i+len(xt)*j]=(nr[j])*yt[i]
                if j==0:
                    z[i+len(xt)*j]=0
                else:
                    z[i+len(xt)*j]=j  
                    
        rbf = Rbf(x,y,z,function='thin_plate')  # radial basis function interpolator instance    
        zz = rbf(self.xx, self.yy) # create zz grid
        zz=np.nan_to_num(zz/np.max(zz)) # normalize the zz grid    
        U, S, VT = np.linalg.svd(zz,full_matrices=False)
        S = np.diag(S)
        r=10
        zz = U[:,:r] @ S[0:r,:r] @ VT[:r,:]  
        return (zz) 


    def super_ellipse(self,m,n1,n2,n3,a,b):
        C=m/4
        R = (abs(np.cos(C*self.theta)/a)**n2 + abs(np.sin(C*self.theta)/b)**n3)**(-1/n1)
        x=R*np.cos(self.theta)
        y=R*np.sin(self.theta)
        return(x,y,R)  

    

class grasp_warping:
    def __init__(self,px,py,a,c):
        self.type='Warping field grasp'
        self.px=px # center x
        self.py=py # center y
        self.a=a # radii 1
        self.c=c # radii 
        self.p=2
        self.type='GRASP field'
    def F(self,x,y,px,py,a,c):
        Y=(y-py)
        X=(x-px)

        d=np.sqrt((X**2) / (a**2) + (Y**2)/(c**2)) 
        return(d**2 * np.log(d)) # create field gird form      
        
    def Fx(self,x,y,px,py,a,c):
        Y=(y-py)
        X=(x-px)
        d=np.sqrt((X**2)/(a**2) + (Y**2)/(c**2)) 
        return(((2*x-2*px)/a**2)*np.log(d) + (2*x-2*px)/2*(a)**2)
    
    def Fy(self,x,y,px,py,a,c):
        Y=(y-py)
        X=(x-px)
        d=np.sqrt((X**2)/(a**2) + (Y**2)/(c**2)) 
        return(((2*y-2*py)/c**2)*np.log(d) + (2*y-2*py)/2*(c)**2)    
        

    def M(self,t,x,y,mode):  

    
        #print(self.px[0],self.py[0],self.a[0],self.c[0])
        F1=self.F(x,y,self.px[0],self.py[0],self.a[0],self.c[0])
        F2=self.F(x,y,self.px[1],self.py[1],self.a[1],self.c[1])
        F3=self.F(x,y,self.px[2],self.py[2],self.a[2],self.c[2])
        if mode==0:
            M=(1-t)*F1+t*F2
        if mode==1:
            M=(1-t)*F2+t*F3     
        return(M)
    
    def My(self,t,x,y,mode):

        F1=self.F(x,y,self.px[0],self.py[0],self.a[0],self.c[0])
        F2=self.F(x,y,self.px[1],self.py[1],self.a[1],self.c[1])
        F3=self.F(x,y,self.px[2],self.py[2],self.a[2],self.c[2])
        fny1=self.Fy(x,y,self.px[0],self.py[0],self.a[0],self.c[0])
        fny2=self.Fy(x,y,self.px[1],self.py[1],self.a[1],self.c[1])
        fny3=self.Fy(x,y,self.px[2],self.py[2],self.a[2],self.c[2])

        
        if mode==0:
            My=((1-t)**2)*F1*fny1 + (t**2)*F2*fny2 + t*(1-t)*F2*fny1 + 2*t*(1-t)*F1*fny2
        
        if mode==1:
            My=((1-t)**2)*F2*fny2 + (t**2)*F3*fny3 + t*(1-t)*F3*fny2 + 2*t*(1-t)*F2*fny3            

        return(My) 


    def Mx(self,t,x,y,mode):
        
        F1=self.F(x,y,self.px[0],self.py[0],self.a[0],self.c[0])
        F2=self.F(x,y,self.px[1],self.py[1],self.a[1],self.c[1])
        F3=self.F(x,y,self.px[2],self.py[2],self.a[2],self.c[2])
        fnx1=self.Fx(x,y,self.px[0],self.py[0],self.a[0],self.c[0])
        fnx2=self.Fx(x,y,self.px[1],self.py[1],self.a[1],self.c[1])
        fnx3=self.Fx(x,y,self.px[2],self.py[2],self.a[2],self.c[2])

        
        if mode==0:
            Mx=((1-t)**2)*F1*fnx1 + (t**2)*F2*fnx2 + t*(1-t)*F2*fnx1 + 2*t*(1-t)*F1*fnx2
        
        if mode==1:
            Mx=((1-t)**2)*F2*fnx2 + (t**2)*F3*fnx3 + t*(1-t)*F3*fnx2 + 2*t*(1-t)*F2*fnx3            
        return(Mx) 



#point fields                    
class point_field:
    def __init__(self,px,py,res,b):    
        self.type='point_field'
        self.px=px # center x
        self.py=py # center y
        self.b=b # range of field
        self.res=res # resolution of the field so how big are the squares 
        self.xmin=self.px-self.b # x max so how far to the right x
        self.xmax=self.px+self.b # x min so how far to the left x
        self.ymin=self.py-self.b # y max so how far to the right y
        self.ymax=self.py+self.b # y min so how far to the left y
        
        self.xcount=int(round((self.xmax-self.xmin)/self.res)) # number of x points 
        self.ycount=int(round((self.ymax-self.ymin)/self.res)) # number of y points
        
        self.xp=np.linspace(self.xmin,self.xmax,self.xcount) # create x axis point
        self.yp=np.linspace(self.ymin,self.ymax,self.ycount) # create y axis point
        self.xx,self.yy=np.meshgrid(self.xp,self.yp) # create grid for the field     
        r=np.sqrt((self.xx-self.px)**2+(self.yy-self.py)**2)
        self.zz=r
        self.zz=self.zz/np.max(self.zz)
        (self.fy,self.fx)=np.gradient(self.zz)
        self.fx=self.fx/np.max(self.fx)
        self.fy=self.fy/np.max(self.fy)
        self.f = RegularGridInterpolator((self.yp,self.xp),self.zz)
        self.fnx = RegularGridInterpolator((self.yp,self.xp),self.fx)
        self.fny = RegularGridInterpolator((self.yp,self.xp),self.fy) 

class source_field:
    def __init__(self,px,py):
        self.px=px
        self.py=py
        self.type='Source_field'
    def Fx(self,x,y,px,py):
        return((x-px)/(np.sqrt((x-px)**2 + (y-py)**2)))
    
    
    def Fy(self,x,y,px,py):
        return((y-py)/(np.sqrt((x-px)**2 + (y-py)**2)))
        

# In[Simulate]
class simulate:
    def __init__(self,my_system,bond,particles,balls,controls,Springs,obj,my_rep,sim,tstep,tend,visual,data_path):   
        self.visual=visual # visualization method
        self.my_system=my_system # my system
        self.sim=sim # sim number 
        self.tstep=tstep # step size
        self.tend=tend # tend of simulation
        self.data_path=data_path # data path
        self.particles=particles # interior particles
        self.bond=bond # bots
        self.obj=obj # object array
        self.balls= balls # ball array 
        self.Springs=Springs # spring array 
        self.controls=controls # controls
        self.time=[] # empty time array 
        self.timec=[] # empty time array for contacts
        self.my_rep=my_rep # contact collectors
        # empty positions forces and number of contact arrays
        self.nc=[]; self.cx=[]; self.cy=[]; self.cz=[]
        self.Fxct=[]; self.Fyct=[]; self.Fzct=[]
        self.bodiesA=[]; self.bodiesB=[]
        self.BID=[] ; self.AID=[]
        self.Trip=False
        self.myapplication=[]
        self.count=0
        self.camx=0#self.controls.phi.py
        #self.camx=0
        self.camy=10
        self.camz=0#self.controls.phi.px
        #self.camz=0
        self.Fb=[]
        self.sim_cond=False
        

    # simulate the robot
    def simulate(self):
        #### Irrrlecnt
        #  Create an Irrlicht application to visualize the system
        if self.visual=="irr":
            self.myapplication = chronoirr.ChIrrApp(self.my_system, self.sim , chronoirr.dimension2du(800,600))
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
            self.myapplication.SetTimestep(self.tstep)
            self.myapplication.SetTryRealtime(False)
            ##### Run the sim
            while(self.myapplication.GetDevice().run()):
                self.my_rep.ResetList()
                self.myapplication.BeginScene()
                self.myapplication.DrawAll()
                self.myapplication.DoStep()
                self.controls.run_controller()
                self.controls.get_position()
                #(xc,zc)=self.controls.get_centroid()
                #print(xc,zc)
                #self.controls.clear_temp_forces()
                print ('time=', np.round(self.my_system.GetChTime(),4))

                if self.count%200==0:
                    t=np.round(self.my_system.GetChTime(),4)
                    # if t>1.0:
                    #     self.controls.alpha=self.controls.alpha+1
                    self.controls.ALPHA.append(self.controls.alpha)    
                    #print('center = ',np.round(xc,2),np.round(zc,2))
                    self.time.append(self.my_system.GetChTime())
                    self.controls.save_data_Forces()
                    self.controls.clear_temp_forces()
                    self.controls.get_error()
                    
                    self.bond.save_data_position()
                    self.bond.save_data_Forces()
                    self.bond.save_data_velocity()
                    self.particles.save_data_position()
                    self.particles.save_data_velocity()
                    self.particles.save_data_Forces()
                    #self.particles.save_angle_data()
                    # self.my_system.GetContactContainer().ReportAllContacts(self.my_rep)
                    # crt_list = self.my_rep.GetList()
                    # self.nc.append(self.my_system.GetContactContainer().GetNcontacts())
                    # if self.my_system.GetContactContainer().GetNcontacts()!=0:
                    #       self.timec.append(self.my_system.GetChTime())
                    #       self.cx.append(crt_list[0])
                    #       self.cy.append(crt_list[1])
                    #       self.cz.append(crt_list[2])
                    #       self.Fxct.append(crt_list[3])
                    #       self.Fyct.append(crt_list[4])
                    #       self.Fzct.append(crt_list[5])
                    #       self.bodiesA.append(crt_list[6])
                    #       self.bodiesB.append(crt_list[7])
                    #       self.AID.append(crt_list[8])
                    #       self.BID.append(crt_list[9])   
                #save ball position if it exists
                if self.balls!=None:
                    if self.count%50==0:
                        self.balls.save_data_position()
                        self.balls.save_contact_force()
                        self.balls.save_data_velocity()
                        #self.controls.save_ball_applied_force()
                        #self.controls.save_pull_test()
                self.controls.clear_temp_forces()
                self.count=self.count+1
                self.myapplication.EndScene()
                # save data
                aaa=len(self.bond.bots)
                cam_x=0.33*(self.bond.bots[0].GetPos().x + self.bond.bots[int(aaa/3)].GetPos().x + self.bond.bots[int(2*aaa/3)].GetPos().x)
                cam_y=0.33*(self.bond.bots[0].GetPos().y + self.bond.bots[int(aaa/3)].GetPos().y + self.bond.bots[int(2*aaa/3)].GetPos().y)
                cam_z=0.33*(self.bond.bots[0].GetPos().z + self.bond.bots[int(aaa/3)].GetPos().z + self.bond.bots[int(2*aaa/3)].GetPos().z)
                self.myapplication.GetSceneManager().getActiveCamera().setPosition(chronoirr.vector3df(cam_x,cam_y+2,cam_z))
                self.myapplication.GetSceneManager().getActiveCamera().setTarget(chronoirr.vector3df(cam_x,cam_y,cam_z))
                self.myapplication.SetVideoframeSave(False)
                self.myapplication.SetVideoframeSaveInterval(round(1/(self.tstep*60)))
                #zc=self.balls.balls[0].GetPos().z
                #vzc=self.balls.balls[0].GetPos_dt().z
                #print(zc)
                # Close the simulation if time ends
                if self.my_system.GetChTime()> self.tend :
                    self.myapplication.GetDevice().closeDevice()
        
        #### Pov ray
        if self.visual=="pov": 
            

            pov_exporter = postprocess.ChPovRay(self.my_system)
            # Sets some file names for in-out processes.
            pov_exporter.SetTemplateFile("F:/data/_template_POV.pov")
            pov_exporter.SetOutputScriptFile("rendering_frames"+self.sim+".pov")
            if not os.path.exists("output"+self.sim):
                os.mkdir("output"+self.sim)
            if not os.path.exists("anim"+self.sim):
                os.mkdir("anim"+self.sim)
            pov_exporter.SetOutputDataFilebase("output"+self.sim+"/my_state")
            pov_exporter.SetPictureFilebase("anim"+self.sim+"/picture")
            pov_exporter.SetCamera(chrono.ChVectorD(self.camx,self.camy,self.camz),chrono.ChVectorD(self.camx,0,self.camz),90)
            pov_exporter.SetLight(chrono.ChVectorD(self.camx,self.camy,self.camz), chrono.ChColor(1.1,1.1,1.1), True)
            pov_exporter.SetAmbientLight(chrono.ChColor(1,1,0.9))
            #pov_exporter.SetLight(chrono.ChVectorD(-2,2,-1), chrono.ChColor(0.9,0.9,1.1), True)
            #pov_exporter.SetAmbientLight(chrono.ChColor(2,2,2))                  
            pov_exporter.AddAll()
            pov_exporter.ExportScript()
            pov_exporter.SetCustomPOVcommandsScript(
                '''
                light_source{ <1,3,1.5> color rgb<0.9,0.9,0.8> }
                ''')
            self.count=0
            ##### Run the sim 
            while (self.my_system.GetChTime() < self.tend): 
                
                # if self.controls.control_type=='pot_field_grab':
                #     zc=abs(self.balls.balls[0].GetPos().z-self.controls.phi.px)
                #     if zc>self.balls.radius:
                #         self.sim_cond=True
                #else:
                #if(self.my_system.GetChTime() < self.tend) :  
                    #self.sim_cond=True
                
                self.my_rep.ResetList()
                self.my_system.DoStepDynamics(self.tstep)
                # run the controllers if they exist
                self.controls.run_controller()
                #self.controls.clear_temp_forces()
                
                t=np.round(self.my_system.GetChTime(),4)
                sample=50
                print ('time=', self.my_system.GetChTime())
                if self.count%sample==0:
                    #print('center = ',np.round(xc,2),np.round(zc,2))
                    #if self.controls.control_type=='pot_field_grab':
                        
                        #if t>1.0:
                            #self.controls.alpha=self.controls.alpha+2
                    self.controls.ALPHA.append(self.controls.alpha)  
                    self.time.append(self.my_system.GetChTime())
                    self.controls.save_data_Forces()
                    self.controls.clear_temp_forces()
                    self.controls.get_error()
                    self.bond.save_data_position()
                    self.bond.save_data_Forces()
                    self.bond.save_data_velocity()
                    self.bond.save_data_Forces_contact()                    
                    self.particles.save_data_position()
                    self.particles.save_data_velocity()
                    self.particles.save_data_Forces()
                    self.particles.save_angle_data()
                    self.my_system.GetContactContainer().ReportAllContacts(self.my_rep)
                    crt_list = self.my_rep.GetList()
                    self.nc.append(self.my_system.GetContactContainer().GetNcontacts())
                    if self.my_system.GetContactContainer().GetNcontacts()!=0:
                          self.timec.append(self.my_system.GetChTime())
                          self.cx.append(crt_list[0])
                          self.cy.append(crt_list[1])
                          self.cz.append(crt_list[2])
                          self.Fxct.append(crt_list[3])
                          self.Fyct.append(crt_list[4])
                          self.Fzct.append(crt_list[5])
                          self.bodiesA.append(crt_list[6])
                          self.bodiesB.append(crt_list[7])
                          self.AID.append(crt_list[8])
                          self.BID.append(crt_list[9])                          
                #save ball position if it exists
                    if self.balls!=None:
                        if self.count%sample==0:
                            self.balls.save_data_position()
                            self.balls.save_contact_force()
                            self.balls.save_data_velocity()
                            self.balls.save_angle_data()
                            #self.controls.save_ball_applied_force()
                            self.controls.save_pull_test()
                    (xc,zc)=self.controls.get_centroid()
                    pov_exporter.ExportData()    
                self.count=self.count+1
                self.controls.clear_temp_forces()
        return(self.bond,self.time,self.controls,self.cx,self.cy,self.cz,self.Fxct,self.Fyct,self.Fzct,self.nc,self.bodiesA,self.bodiesB,self.AID,self.BID)     

# In[Create rings]

def MaxValues(R,radius,radius2,nb,mode):
    if mode=='empty':
        N=[]
        A=0
    if mode=='nonhnmax':
        Rin=R-radius
        S=2*radius2+2*np.sqrt(2)*radius2
        P=int(Rin/S)
        X=np.zeros(2*P)
        Y=np.zeros(2*P)
        R=np.zeros(2*P)
        A=np.zeros(2*P)
        N=[]
        for i in range(P):
            R[2*i]=np.sqrt(2)*radius2
            R[2*i+1]=radius2
            X[2*i]=Rin-np.sqrt(2)*radius2-i*S
            X[2*i+1]=Rin-(2*np.sqrt(2)*radius2+radius2)-i*S

        N1=0
        N2=0
        for i in range(P):
            N.append(int((np.pi*2*X[2*i])/(2*np.sqrt(2)*radius2)))
            N1=N1+int((np.pi*2*X[2*i])/(2*np.sqrt(2)*radius2))
            N.append(int((np.pi*2*X[2*i+1])/(2*radius2)))
            N2=N2+int((np.pi*2*X[2*i+1])/(2*radius2))
            print(N1,N2)
            A[2*i]=np.pi * R[2*i]**2 * N[i]
            A[2*i+1]=np.pi * R[2*i+1]**2 *N[i+1]
       
        A=np.sum(A)
        print(N1,N2) 
    if mode=='nmax':
        Rin=R-radius-radius2
        ngrans1=int(Rin/(2*radius2))
        ri=np.zeros((1,ngrans1))
        ni=np.zeros((1,ngrans1))
        radii=Rin-(2*radius2)
        for i in range(ngrans1):
            remainder=((2*radius2))*i
            ri[:,i]=radii-remainder
            ni[:,i]=np.floor(((ri[:,i]*np.pi)/radius2))
        n=np.asarray(ni,dtype=int)
        N=n[0]
        A=0
    if mode=='verify':
        N=16
        A=0 
    if mode=='tunnel_verify':
        N=16
        A=0 
        
    if mode=='grasp_verify':
        N=18
        A=0     
        
    if mode=='target_verify':
        N=17
        A=0              
    return(N,A)

# In[report contact callback]
class MyReportContactCallback(chrono.ReportContactCallback):

    def __init__(self):

        chrono.ReportContactCallback.__init__(self)
        self.Fxc=[]
        self.Fyc=[]
        self.Fzc=[]
        self.pointx = []
        self.pointy = []
        self.pointz = []
        self.bodiesA = []
        self.bodiesB = []
        self.bodiesAID=[]
        self.bodiesBID=[]
        self.CA=[]

    def OnReportContact(self,vA,vB,cA,dist,rad,force,torque,modA,modB):
        bodyUpA = chrono.CastContactableToChBody(modA)
        nameA = bodyUpA.GetName()
        bodyUpB = chrono.CastContactableToChBody(modB)
        nameB = bodyUpB.GetName()
        IDA = bodyUpA.GetId()
        IDB = bodyUpB.GetId()
        #AA=cA*chrono.ChVectorD(vA.x,vA.y,vA.z)
        #BB=cA*chrono.ChVectorD(force.x,force.y,force.z)
        #self.pointx.append(AA.x)
        #self.pointy.append(AA.y)
        #self.pointz.append(AA.z)
#        self.Fxc.append(BB.x)
#        self.Fyc.append(BB.y)
#        self.Fzc.append(BB.z)        
        self.pointx.append(vA.x)
        self.pointy.append(vA.y)
        self.pointz.append(vA.z)
        self.Fxc.append(force.x)
        self.Fyc.append(force.y)
        self.Fzc.append(force.z)
        self.bodiesA.append(nameA)
        self.bodiesB.append(nameB)
        self.bodiesAID.append(IDA)
        self.bodiesBID.append(IDB)
        self.CA.append(cA)
        return True        # return False to stop reporting contacts

    # reset after every run 
    def ResetList(self):
        self.pointx = []
        self.pointy = []
        self.pointz = [] 
        self.Fxc=[]
        self.Fyc=[]
        self.Fzc=[]
        self.bodiesA=[]
        self.bodiesB=[]
        self.bodiesAID=[]
        self.bodiesBID=[]
        self.CA=[]
    # Get the points
    def GetList(self):
        return (self.pointx,self.pointy,self.pointz,self.Fxc,self.Fyc,self.Fzc,self.bodiesA,self.bodiesB,self.bodiesAID,self.bodiesBID)


# In[report contact callback2]                
class MyReportContactCallback2(chrono.ReportContactCallback):

    def __init__(self):

        chrono.ReportContactCallback.__init__(self)
        self.Fxc=[]
        self.Fyc=[]
        self.Fzc=[]
        self.pointx = []
        self.pointy = []
        self.pointz = []
        self.bodiesA = []
        self.bodiesB = []
        self.bodiesAID=[]
        self.bodiesBID=[]
        self.CA=[]
        
    def OnReportContact(self,vA,vB,cA,dist,rad,force,torque,modA,modB):
        bodyUpA = chrono.CastContactableToChBody(modA)
        nameA = bodyUpA.GetName()
        IDA = bodyupA.GetID()
        IDB =bodyupA.GetID()
        bodyUpB = chrono.CastContactableToChBody(modB)
        nameB = bodyUpB.GetName()
        self.pointx.append(vA.x)
        self.pointy.append(vA.y)
        self.pointz.append(vA.z)
        self.Fxc.append(force.x)
        self.Fyc.append(force.y)
        self.Fzc.append(force.z)
        self.bodiesA.append(nameA)
        self.bodiesB.append(nameB)
        self.CA.append(cA)
        return True        # return False to stop reporting contacts

    # reset after every run 
    def ResetList(self):
        self.pointx = []
        self.pointy = []
        self.pointz = [] 
        self.Fxc=[]
        self.Fyc=[]
        self.Fzc=[]
        self.bodiesA=[]
        self.bodiesB=[]
        self.CA=[]
        
    # Get the points
    def GetList(self):
        return (self.pointx,self.pointy,self.pointz,self.Fxc,self.Fyc,self.Fzc,self.bodiesA,self.bodiesB,self.CA) 


# In[Export data]
class export_data():
    def __init__(self,bots,interior,ball,phi,controller,tend,time,sim,nb,mr,mp,mu_f,mu_b,mu_r,mu_s,C,Ct,Cr,Cs,cx,cy,cz,Fxct,Fyct,Fzct,nc,bodiesA,bodiesB,pwm,w,tn,gapw,env_mode,save_data,tr,AID,BID):
        
    
        self.bots=bots # bots
        self.tr=tr # time transformations 
        
        if interior is not None: # interiors
            self.interior=interior
        else:
            self.interior=None
            
    
        if ball is not None: # if there is a ball
            self.ball=ball
        else:               # if there is not a ball
            self.ball=None
            
        # controller
        if controller is not None:
            self.controller=controller
        else:
            self.controller=None
            
        # potential field
        if phi is not None:
            self.phi=phi
        else:
            self.phi=None
            
           
        self.control_type=self.controller.control_type  # control type 
        self.field_type=self.phi.type                   # potential field type
        #self.field_type=self.phi[0].type
        self.tend=tend                                  # end time
        self.time={'time': time}                        # time array
        self.count=len(time)                            # count number this i
        self.sim=sim                                    # simulation name
        self.nb=nb                                      # number of robots
        self.mr=mr                                      # mass of robots
        self.mp=mp                                      # mass of particles
        self.mu_f=mu_f                                  # friction 
        self.mu_b=mu_b                                  # damping 
        self.mu_r=mu_r                                  # rolling friction 
        self.mu_s=mu_s                                  # sliding friction
        self.C=C                                        # compliance 
        self.Ct=Ct                                      # compliance tangent
        self.Cr=Cr                                      # compliance rolling
        self.Cs=Cs                                      # compliance sliding
        # # # contact locations
        # self.cx=cx # contact location x 
        # self.cy=cy # contact location y
        # self.cz=cz # contact location z
        # self.nc=np.asarray(nc) # number of contacts
        # #self.lengthm=np.amax(self.nc) # length of nc
        
        # #empty array to fill contact locations in each time step
        # self.xc=np.zeros((self.lengthm,self.count)) # x points
        # self.yc=np.zeros((self.lengthm,self.count)) # y points 
        # self.zc=np.zeros((self.lengthm,self.count)) # z points
        
        # self.AN={} # empty array of names of contact bodies
        # self.BN={} # empty array of names of contact bodes
        # self.aid=AID
        # self.bid=BID
        # self.bodiesA=bodiesA # bodies that made contact
        # self.bodiesB=bodiesB # bodies that made contact

        # for i in range(len(self.nc)):        
        #     self.AN["AN{0}".format(i)]=self.bodiesA[i]  # A names
        #     self.BN["BN{0}".format(i)]=self.bodiesB[i]  # B names
            
        # # contact of forces
        # self.Fxct=Fxct ; self.Fyct=Fyct ; self.Fzct=Fzct
        # self.Fcx=np.zeros((self.lengthm,self.count))
        # self.Fcy=np.zeros((self.lengthm,self.count))
        # self.Fcz=np.zeros((self.lengthm,self.count))
        # self.AID=np.zeros((self.lengthm,self.count)) # object A id
        # self.BID=np.zeros((self.lengthm,self.count)) # object B id
        
        self.tn=tn # range of time applied
        self.w=w   # freqency

        self.pwm=pwm # pwm 
         
        self.gapw=gapw # gap width 
        self.env_mode=env_mode # if yhrtr is a tunnel or obstacle fiels 
        
        # save data array
        self.save_data=save_data # matrix indicating what to save its made in the config file 

        ###### Export variables ######
        self.results_dir = os.path.join('robot_data_'+self.sim+'/') 
        if not os.path.isdir(self.results_dir):
            os.makedirs(self.results_dir)
            
        self.file_name0=self.results_dir+'variables.csv'

        with open(self.file_name0, 'w', newline='') as fout:
            w = csv.writer(fout)
        
            # sim 
            w.writerow(['sim',self.sim])
            # time run 
            w.writerow(['time run (minutes)',self.tr])
            # control type
            w.writerow(['control_type',self.control_type])
            # number of bots
            w.writerow(['number of bots', self.nb])
            # geometry of bots
            w.writerow(['geometry of bot',self.bots.geom])
            # radius of bots
            w.writerow(['radius of bots',self.bots.radius])
            # starting radius
            w.writerow(['starting radius', self.bots.R])
            # robot unit mass
            w.writerow(['robot mass', self.mr])           
            # ring rumber
            w.writerow(['ring numbers', self.interior.n])
            # number of interiors
            w.writerow(['number of interior', self.interior.ni])
            # radius of interiors 
            w.writerow(['radius of interior', self.interior.radius])
            # particles mass
            w.writerow(['particle mass', self.mp])
            # granular mode
            w.writerow(['gran mode',self.interior.mode])
            # offset radius 
            w.writerow(['offset radius',self.interior.off])
            # net mass
            w.writerow(['net_mass',self.mp*self.interior.ni+self.mr*self.nb])
            # frequency 
            w.writerow(['frequency', self.w])
            # pwm 
            w.writerow(['pwm',self.pwm])
            # time active
            w.writerow(['time active', self.tn])
            # simulation end
            w.writerow(['tend', self.tend])    
            # spring stiffness 
            w.writerow(['spring stiffness',self.bots.km])
            # gap 
            w.writerow(['radius_offset',self.interior.off])
            if self.bots.mem==3:
                w.writerow(['km',self.bots.km]) # membrane spring stiffness
                w.writerow(['bm',self.bots.bm]) # membrane damping 
                w.writerow(['rationM',self.bots.ratioM]) # number of particles between bots
                w.writerow(['skinrho',self.bots.skinrho])# densitiy
                
            
            w.writerow(['alpha', self.controller.alpha])# alpha
            w.writerow(['beta', self.controller.beta])  # beta
            w.writerow(['field type', self.field_type]) # field type
            w.writerow(['tanh coefficient',self.controller.p]) # tanh term
            
        
            if self.field_type=='Shape_Fields':
                w.writerow(['R', self.phi.R]) # radius of shape   
                w.writerow(['res',self.phi.res]) # resolution
                w.writerow(['range',self.phi.b]) # boundaries of field
                w.writerow(['Shape',self.phi.shape]) # shape of contour
                
            # if self.field_type=='Shape_Fields_analytic':
            #     w.writerow(['a', self.phi.a]) # radius of shape  
            #     w.writerow(['c', self.phi.c]) # radius of shape
            #     w.writerow(['theta', self.phi.theta]) # radius of shape
            #     w.writerow(['res',self.phi.res]) # resolution
            #     w.writerow(['range',self.phi.b]) # boundaries of field 
                
            # if self.field_type=='import_field_complex':
            #    w.writerow(['field name',self.phi.name])
             
            # friction
            w.writerow(['mu_f', self.mu_f])
            # damping 
            w.writerow(['mu_b', self.mu_b])
            # rolling 
            w.writerow(['mu_r', self.mu_r])
            # spinning
            w.writerow(['mu_s', self.mu_s])
           
            # Cohesive stuff
            w.writerow(['C', self.C])
            w.writerow(['Ct', self.Ct])
            w.writerow(['Cr', self.Cr])
            w.writerow(['Cs', self.Cs])
            
            # tunnel mode 
            w.writerow(['env_mode',self.env_mode])
            w.writerow(['gapw',self.gapw])
            
              
            
        # # fill the arrays with contact information 
        # for i in range(self.count):
        #     ind=self.nc[i]
        #     tryme=self.cx[i]
        #     tryme2=self.cy[i]
        #     tryme3=self.cz[i]
        #     tryme4=self.Fxct[i]
        #     tryme5=self.Fyct[i]
        #     tryme6=self.Fzct[i]
        #     tryme7=self.aid[i]
        #     tryme8=self.bid[i]
            
        #     # convert to array
        #     tryme=np.asarray(tryme)
        #     tryme2=np.asarray(tryme2)
        #     tryme3=np.asarray(tryme3)
        #     tryme4=np.asarray(tryme4)
        #     tryme5=np.asarray(tryme5)
        #     tryme6=np.asarray(tryme6)
        #     tryme7=np.asarray(tryme7)
        #     tryme7=np.asarray(tryme8)
        #     #fill array position
        #     self.xc[0:ind,i]=np.transpose(tryme)
        #     self.yc[0:ind,i]=np.transpose(tryme2)
        #     self.zc[0:ind,i]=np.transpose(tryme3)

        #     # Fill array forces
        #     self.Fcx[0:ind,i]=np.transpose(tryme4)
        #     self.Fcy[0:ind,i]=np.transpose(tryme5)
        #     self.Fcz[0:ind,i]=np.transpose(tryme6)  
        #     self.AID[0:ind,i]=np.transpose(tryme7)
        #     self.BID[0:ind,i]=np.transpose(tryme8)
        #### return data robots
        (self.xb,self.yb,self.zb)=self.bots.return_position_data()
        (self.xbm,self.ybm,self.zbm)=self.bots.return_position_membrane_data()
        (self.Faxb,self.Fayb,self.Fazb)=self.bots.return_force_data()
        (self.xvb,self.yvb,self.zvb)=self.bots.return_velocity_data()
        (self.Faxc,self.Fayc,self.Fazc)=self.controller.return_force_data()
        (self.Faxcb,self.Faycb,self.Fazcb)=self.bots.return_force_data_contact()        

        #### Return Particle data
        #if self.interior is not None:
        (self.xp,self.yp,self.zp)=self.interior.return_position_data()
        (self.Faxp,self.Fayp,self.Fazp)=self.interior.return_force_data()
        (self.xvp,self.yvp,self.zvp)=self.interior.return_velocity_data() 
        (self.Pphi,self.Ptheta,self.Ppsi)=self.interior.return_angle_data()
        
        
        #### ball properties    
        if self.ball is not None:
            (self.bx,self.bz)=self.ball.return_position_data()
            (self.bFx,self.bFz)=self.ball.return_force_data()
            (self.bvx,self.bvz)=self.ball.return_velocity_data()
            (self.Fb,self.PX,self.PY,self.MB)=self.controller.return_pull_test()
            (self.bphi,self.btheta,self.bpsi)=self.ball.return_angle_data()
            
        # # names of objects in contact 
        # self.file_name111=self.results_dir+'/AN.csv'
        # with open(self.file_name111, 'w', newline='') as fout:
        #     w = csv.writer(fout)
        #     # write time to csv file
        #     for key, val in self.AN.items():
        #         w.writerow([key,*val])
        
        # # names of objects in contact        
        # self.file_name112=self.results_dir+'/BN.csv'
        # with open(self.file_name112, 'w', newline='') as fout:
        #     w = csv.writer(fout)
        #     # write time to csv file
        #     for key, val in self.BN.items():
        #         w.writerow([key,*val])            

 
    
        #### position data           
        if self.save_data[0]==True:        
            self.file_name1=self.results_dir+'/bot_position.csv'

            with open(self.file_name1, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.xb.items():
                    w.writerow([key, *val])
                
            # write y position to csv file    
                for key, val in self.yb.items():
                    w.writerow([key, *val]) 
                
            # write z position to csv file     
                for key, val in self.zb.items():
                    w.writerow([key, *val])   
                    
                    
            self.file_name1=self.results_dir+'/membrane_position.csv'

            with open(self.file_name1, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.xbm.items():
                    w.writerow([key, *val])
                
            # write y position to csv file    
                for key, val in self.ybm.items():
                    w.writerow([key, *val]) 
                
            # write z position to csv file     
                for key, val in self.zbm.items():
                    w.writerow([key, *val]) 
                    
                    
        #### bot velocity                    
        if self.save_data[1]==True:        
        
            self.file_name1=self.results_dir+'/bot_velocity.csv'

            with open(self.file_name1, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.xvb.items():
                    w.writerow([key, *val])
                
                # write y position to csv file    
                for key, val in self.yvb.items():
                    w.writerow([key, *val]) 
                
                # write z position to csv file     
                for key, val in self.zvb.items():
                    w.writerow([key, *val])      
                    
        #### bot Forces
        if self.save_data[2]==True:
         
            self.file_name2=self.results_dir+'/bot_TotalForces.csv'

            with open(self.file_name2, 'w', newline='') as fout:
                w = csv.writer(fout)
                    # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.Faxb.items():
                    w.writerow([key, *val])
                
                # write y position to csv file    
                for key, val in self.Fayb.items():
                    w.writerow([key, *val]) 
                
                # write z position to csv file     
                for key, val in self.Fazb.items():
                    w.writerow([key, *val])
                    
                    
            self.file_name2_2=self.results_dir+'/bot_TotalForces_contact.csv'

            with open(self.file_name2_2, 'w', newline='') as fout:
                w = csv.writer(fout)
                    # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.Faxcb.items():
                    w.writerow([key, *val])
                
                # write y position to csv file    
                for key, val in self.Faycb.items():
                    w.writerow([key, *val]) 
                
                # write z position to csv file     
                for key, val in self.Fazcb.items():
                    w.writerow([key, *val])                    
                    
                    
                
        #### Controller force
        if self.save_data[3]==True:
            
            self.file_name3=self.results_dir+'/Force_controller.csv'

            with open(self.file_name3, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.Faxc.items():
                    w.writerow([key, *val])
                
                # write y position to csv file    
                for key, val in self.Fayc.items():
                    w.writerow([key, *val]) 
                
                # write z position to csv file     
                for key, val in self.Fazc.items():
                    w.writerow([key, *val])
            
        #### [Contact points]
        if self.save_data[4]==True:
            # Contact A
            
            # contact points x
            self.file_name4=self.results_dir+'/x contact points.csv' 
            savetxt(self.file_name4,self.xc, delimiter=',')
        
            # contact points y
            self.file_name5=self.results_dir+'/y contact points.csv' 
            savetxt(self.file_name5,self.yc, delimiter=',')
        
            # contact points z
            self.file_name6=self.results_dir+'/z contact points.csv' 
            savetxt(self.file_name6,self.zc, delimiter=',')
        
            # contact force x
            self.file_name7=self.results_dir+'/x contact force.csv' 
            savetxt(self.file_name7,self.Fcx, delimiter=',')
        
            # contact force y
            self.file_name8=self.results_dir+'/y contact force.csv' 
            savetxt(self.file_name8,self.Fcy, delimiter=',')
        
            # contact force z
            self.file_name9=self.results_dir+'/z contact force.csv' 
            savetxt(self.file_name9,self.Fcz, delimiter=',')
        
            # number of contacts
            self.file_name10=self.results_dir+'/nc.csv' 
            savetxt(self.file_name10,self.nc, delimiter=',')
            
            # contact A id
            self.file_name101=self.results_dir+'/AID.csv' 
            savetxt(self.file_name101,self.AID, delimiter=',')
                        
            # contact B id
            self.file_name102=self.results_dir+'/BID.csv' 
            savetxt(self.file_name102,self.BID, delimiter=',')   
            
        # particle position
        #if self.save_data[5]==True:
        self.file_name13=self.results_dir+'/particle_position.csv'
        with open(self.file_name13, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
                
            # write x position to csv file
            for key, val in self.xp.items():
                w.writerow([key, *val])
                
                # write y position to csv file    
            for key, val in self.yp.items():
                w.writerow([key, *val]) 
                
                # write z position to csv file     
            for key, val in self.zp.items():
                w.writerow([key, *val])  


#(self.Pphi,self.Ptheta,self.Ppsi)
        self.file_name133=self.results_dir+'/particle_angles.csv'
        with open(self.file_name133, 'w', newline='') as fout:
            w = csv.writer(fout)
            # write time to csv file
            for key, val in self.time.items():
                w.writerow([key,*val])
                
            # write x position to csv file
            for key, val in self.Pphi.items():
                w.writerow([key, *val])
                
                # write y position to csv file    
            for key, val in self.Ptheta.items():
                w.writerow([key, *val]) 
                
                # write z position to csv file     
            for key, val in self.Ppsi.items():
                w.writerow([key, *val])  



                    
        # Particle Velocity                    
        if self.save_data[6]==True:
            self.file_name16=self.results_dir+'/particle_velocity.csv'
            with open(self.file_name16, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.xvp.items():
                    w.writerow([key, *val])
                
            # write y position to csv file    
                for key, val in self.yvp.items():
                    w.writerow([key, *val]) 
                
            # write z position to csv file     
                for key, val in self.zvp.items():
                    w.writerow([key, *val])
                    
        if self.save_data[7]==True:
            self.file_name17=self.results_dir+'/particle_forces.csv'
            with open(self.file_name17, 'w', newline='') as fout:
                w = csv.writer(fout)
                # write time to csv file
                for key, val in self.time.items():
                    w.writerow([key,*val])
                
                # write x position to csv file
                for key, val in self.xvp.items():
                    w.writerow([key, *val])
                
            # write y position to csv file    
                for key, val in self.yvp.items():
                    w.writerow([key, *val]) 
                
            # write z position to csv file     
                for key, val in self.zvp.items():
                    w.writerow([key, *val])                    
                
        # # ball variables 
        # if self.save_data[8]==True:
        #     # ball positions x
        #     self.file_name18=self.results_dir+'/ballx.csv' 
        #     savetxt(self.file_name18,self.bx, delimiter=',')
        #     # ball position z
        #     self.file_name19=self.results_dir+'/ballz.csv' 
        #     savetxt(self.file_name19,self.bz, delimiter=',')
        #     # ball velocity x
        #     self.file_name20=self.results_dir+'/ballvx.csv' 
        #     savetxt(self.file_name20,self.bvx, delimiter=',')
        #     # ball velocity z
        #     self.file_name21=self.results_dir+'/ballvz.csv' 
        #     savetxt(self.file_name21,self.bvz, delimiter=',')
        #     # ball force x
        #     self.file_name22=self.results_dir+'/ballFx.csv' 
        #     savetxt(self.file_name22,self.bFx, delimiter=',')
        #     # ball force z
        #     self.file_name23=self.results_dir+'/ballFz.csv' 
        #     savetxt(self.file_name23,self.bFz, delimiter=',')
            
            
        #     ### Pull test ###
        #     # ball force applied
        #     self.file_name233=self.results_dir+'/ballFb.csv' 
        #     savetxt(self.file_name233,self.Fb, delimiter=',')            

        #     # center of field X
        #     self.file_name234=self.results_dir+'/PX.csv' 
        #     savetxt(self.file_name234,self.PX, delimiter=',')      

        #     # center of field Y
        #     self.file_name235=self.results_dir+'/PY.csv' 
        #     savetxt(self.file_name235,self.PY, delimiter=',')      

        #     # mass of ball 
        #     self.file_name236=self.results_dir+'/MB.csv' 
        #     savetxt(self.file_name236,self.MB, delimiter=',') 
            
        #     # phi of ball 
        #     self.file_name237=self.results_dir+'/bphi.csv' 
        #     savetxt(self.file_name237,self.bphi, delimiter=',')             
            
        #     # theta of ball 
        #     self.file_name238=self.results_dir+'/btheta.csv' 
        #     savetxt(self.file_name238,self.btheta, delimiter=',') 
            
        #     # psi of ball 
        #     self.file_name239=self.results_dir+'/bpsi.csv' 
        #     savetxt(self.file_name239,self.bpsi, delimiter=',')             
            
            
            
        self.file_name246=self.results_dir+'/ALPHA.csv'
        savetxt(self.file_name246,self.controller.ALPHA, delimiter=',')            
        # error of robot
        if self.save_data[9]==True:
            self.file_name24=self.results_dir+'/error.csv'
            savetxt(self.file_name24,self.controller.E, delimiter=',')
            

            
        #     self.file_name26=self.results_dir+'/xd2.csv' 
        #     savetxt(self.file_name26,self.phi.xd2, delimiter=',')            
            
        #     self.file_name27=self.results_dir+'/yd2.csv' 
        #     savetxt(self.file_name27,self.phi.yd2, delimiter=',')  