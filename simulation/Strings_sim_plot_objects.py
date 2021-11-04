# -*- coding: utf-8 -*-
"""
Created on Tue May 12 12:53:23 2020

@author: dmulr
"""
import numpy as np
import math as math
import os
import csv
import glob
import statistics 
import cv2
import pandas as pd

import matplotlib
import matplotlib.pyplot as plt
from matplotlib import colors as colors
import matplotlib as mpl
from matplotlib import animation
import matplotlib.cm as cm
import matplotlib.patches as mpl_patches
from scipy.spatial import Voronoi, voronoi_plot_2d,ConvexHull
from matplotlib.patches import RegularPolygon
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
import matplotlib.patches as patches
from mpl_toolkits.axes_grid.inset_locator import (inset_axes, InsetPosition,mark_inset)


from scipy import interpolate
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import RegularGridInterpolator
from scipy.interpolate import interp1d

  
#[Compare error]
class compare_grasp:
    def __init__(self,nam,path,ni,nb,radius,radius2,height,Rb):
        self.path=path # path array
        self.nam=nam # name array
        self.ni=np.sum(ni) # number of interiors
        self.nb=nb         # number of bots
        self.radius=radius # radius of bots 
        self.radius2=radius2 # radius of interiors
        self.Rb=Rb # ball radius
        self.height=height # height of objects
        self.filename1=[] # ball force
        self.filename2=[] # ball position
        self.filename3=[] # position and time
        self.filename4=[] # ALPHA
        self.filename5=[] # x contact force
        self.filename6=[] # z contact force
        self.filename7=[] # particle position
        self.filename8=[] # AN
        self.filename9=[] # BN
        self.filename10=[] # nc   
        self.filename11=[] # ballx    
        self.filename12=[] # bot velocity
        self.FB=[]
        self.BALLZ=[]
        self.BALLX=[]        
        self.TIME=[]
        self.ALPHA=[]
        self.XB=[]
        self.ZB=[]
        self.XP=[]
        self.ZP=[]
        self.FXC=[]
        self.FZC=[]
        self.AN=[]
        self.BN=[]
        self.NC=[]
        self.MAGV=[]        
        self.ANGLE=[]
        self.XVB=[]
        self.ZVB=[]
        
        self.name1='/ballFb.csv'
        self.name2='/ballz.csv'
        self.name3='/bot_position.csv'
        self.name4='/ALPHA.csv'
        self.name5="/x contact force.csv"
        self.name6="/z contact force.csv"
        self.name7='/particle_position.csv'
        self.name8="/AN.csv"
        self.name9="/BN.csv"
        self.name10="/nc.csv"   
        self.name11='/ballx.csv'
        self.name12='/bot_velocity.csv'
        self.head='robot_data_'
        self.color=['tab:blue','tab:green','tab:red','tab:cyan','tab:orange','tab:purple','tab:brown','tab:olive']
        
 
        for i in range(len(self.nam)):
            self.filename1.append(self.head+self.path[i]+self.name1) # ball force
            self.filename2.append(self.head+self.path[i]+self.name2) # ball position
            self.filename3.append(self.head+self.path[i]+self.name3) # position and time
            self.filename4.append(self.head+self.path[i]+self.name4) # ALPHA
            self.filename5.append(self.head+self.path[i]+self.name5) # x contact force
            self.filename6.append(self.head+self.path[i]+self.name6) # z contact force
            self.filename7.append(self.head+self.path[i]+self.name7) # particle position
            self.filename8.append(self.head+self.path[i]+self.name8) # AN
            self.filename9.append(self.head+self.path[i]+self.name9) # BN
            self.filename10.append(self.head+self.path[i]+self.name10) # nc
            self.filename11.append(self.head+self.path[i]+self.name11) # ballx           
            self.filename12.append(self.head+self.path[i]+self.name12) # ballx

  
    
    def smooth(self,y, box_pts):
        ''' smooth the function'''
        box = np.ones(box_pts)/box_pts
        y_smooth = np.convolve(y, box, mode='same')
        return y_smooth          
    
        
    def Pull_force_vs_position(self,titl,xticks,yticks): 
        import matplotlib.font_manager as fm
        # Rebuild the matplotlib font cache
        fm._rebuild()
        mpl.rcParams['font.family'] = 'sans-serif'
        plt.rcParams['font.size'] = 10
        plt.rcParams['axes.linewidth'] = 1         
        fig, ax = plt.subplots(figsize=(1.625, 1.625),dpi=300)
        #fig.subplots_adjust(top=0.85,bottom=0.2,left=0.25,right=0.95,hspace=0,wspace=0)
        fig.subplots_adjust(top=0.88,bottom=0.20,left=0.275,right=0.95,hspace=0,wspace=0)

        # plot 1
        #ax.set_title(titl)
        ax.grid(True)
        # for i in range(len(self.nam)):
        #      ax.plot(self.FB[i],self.BALLZ[i],color=self.color[i],label=self.nam[i])

        #entry = 60
        entry = 25
        
        for i in range(len(self.nam)):
            #ax.plot(self.BALLZ[i],self.FB[i],color=self.color[i],label=self.nam[i])
            #print(i,self.BALLZ[i])
            BZ=self.BALLZ[i]-self.BALLZ[i][entry]
            #print(i,'BZ',BZ,'FB',self.FB[i])
            ax.plot(self.FB[i][entry:-1],BZ[entry:-1],color=self.color[i],label=self.nam[i])
            np.savez(titl+str(i)+'.npz',FB=self.FB[i][entry:-1],BZ=BZ[entry:-1])
        ax.set_xticks(xticks)
        ax.set_yticks(yticks)
        ax.xaxis.set_tick_params(labelsize=8)
        ax.yaxis.set_tick_params(labelsize=8)
        ax.set_xlabel('Pull Force (N)',fontsize=8,labelpad=1)
        ax.set_ylabel('Position (m)',fontsize=8,labelpad=1)
        ax.set_ylim([yticks[0]-.1,yticks[-1]])
        ax.set_xlim([xticks[0]-10,xticks[-1]])
        #ax.legend(prop={'size': 8})             
        #ax.legend(loc="lower center", bbox_to_anchor=(0.5, -0.05),ncol=3,prop={'size': 10})
        plt.savefig(titl+"_tug_test.svg") 
        plt.savefig(titl+"_tug_test.pdf") 
        plt.savefig(titl+"_tug_test.png") 
        
        

    def moving_average(self,a, n=3):
        ret = np.cumsum(a, dtype=float)
        ret[n:] = ret[n:] - ret[:-n]
        return ret[n - 1:] / n    



    def plot_mag_velocity(self,name):
        #direct = os.path.join(self.results_dir,'magnitude of velocity')
        #if not os.path.isdir(direct):
            #os.makedirs(direct)
        fig, ax = plt.subplots(figsize=(5,5))
        fig.subplots_adjust(top=0.9,bottom=0.1,left=0.1,right=0.9,hspace=0.3,wspace=0.2)
        MAGV=[]
        ax.grid(True)

        for i in range(len(self.nam)):
            xvel=self.XVB[i]
            zvel=self.ZVB[i]
            magv=[]
            for j in range(len(self.TIME[i])):
                temp=[]
                for k in range(self.nb):
                    #temp.append(zvel[k,j])
                    temp.append(np.sqrt(xvel[k,j]**2+zvel[k,j]**2))
                magv.append(np.mean(temp))
                
            MAGV.append(magv)
                
        for i in range(len(self.nam)):
            ax.plot(self.TIME[i],MAGV[i],color=self.color[i],label=self.nam[i])                
            ax.plot(self.TIME[i][0:len(self.moving_average(MAGV[i]))],self.moving_average(MAGV[i]),color=self.color[i],label=self.nam[i]) 
        ax.set_title(name+"mag")
        #plt.grid(True)        
        #plt.title("velocity vs Time (s) for Bot "+self.sim )
        #plt.xlabel('time (seconds)')
        #plt.ylabel('velocity (m/s)')
        plt.savefig(name+'_avg_magnitude_velocity'+".png") 
    
    
    def plot_compx_velocity(self,name):
        #direct = os.path.join(self.results_dir,'magnitude of velocity')
        #if not os.path.isdir(direct):
            #os.makedirs(direct)
        fig, ax = plt.subplots(figsize=(5,5))
        fig.subplots_adjust(top=0.9,bottom=0.1,left=0.1,right=0.9,hspace=0.3,wspace=0.2)
        MAGV=[]
        ax.grid(True)

        for i in range(len(self.nam)):
            xvel=self.XVB[i]
            zvel=self.ZVB[i]
            magv=[]
            for j in range(len(self.TIME[i])):
                temp=[]
                for k in range(self.nb):
                    #temp.append(zvel[k,j])
                    temp.append(xvel[k,j])
                magv.append(np.sum(temp))
                
            MAGV.append(magv)
                
        for i in range(len(self.nam)):
            ax.plot(self.TIME[i],MAGV[i],color=self.color[i],label=self.nam[i])                
            ax.plot(self.TIME[i][0:len(self.moving_average(MAGV[i]))],self.moving_average(MAGV[i]),color=self.color[i],label=self.nam[i],linestyle='dashed') 
        ax.set_title(name+"Xvel")
        #plt.grid(True)        
        #plt.title("velocity vs Time (s) for Bot "+self.sim )
        #plt.xlabel('time (seconds)')
        #plt.ylabel('velocity (m/s)')
        plt.savefig(name+'_sum_x_velocity'+".png") 


    def plot_compz_velocity(self,name):
        #direct = os.path.join(self.results_dir,'magnitude of velocity')
        #if not os.path.isdir(direct):
            #os.makedirs(direct)
        fig, ax = plt.subplots(figsize=(5,5))
        fig.subplots_adjust(top=0.9,bottom=0.1,left=0.1,right=0.9,hspace=0.3,wspace=0.2)
        MAGV=[]
        ax.grid(True)

        for i in range(len(self.nam)):
            xvel=self.XVB[i]
            zvel=self.ZVB[i]
            magv=[]
            for j in range(len(self.TIME[i])):
                temp=[]
                for k in range(self.nb):
                    #temp.append(zvel[k,j])
                    temp.append(zvel[k,j])
                magv.append(np.sum(temp))
                
            MAGV.append(magv)
                
        for i in range(len(self.nam)):
            ax.plot(self.TIME[i],MAGV[i],color=self.color[i],label=self.nam[i])                
            ax.plot(self.TIME[i][0:len(self.moving_average(MAGV[i]))],self.moving_average(MAGV[i]),color=self.color[i],label=self.nam[i],linestyle='dashed',linewidth=3) 
        ax.set_title(name+"Zvel")
        #plt.grid(True)        
        #plt.title("velocity vs Time (s) for Bot "+self.sim )
        #plt.xlabel('time (seconds)')
        #plt.ylabel('velocity (m/s)')
        plt.savefig(name+'_sum_z_velocity'+".png") 

    
def create_video(name,directory,export,file):
    for index, filename in enumerate(glob.glob(directory+'/*.'+file)):
        if index>1: break
        img = cv2.imread(filename)
        height, width, layers = img.shape
        size = (width,height)
        
    out = cv2.VideoWriter(export+'.mp4',cv2.VideoWriter_fourcc(*'DIVX'), 15, size)    
    for filename in glob.glob(directory+'/*.'+file):
        img = cv2.imread(filename)

        out.write(img)
    out.release()     



class grasping_analysis:
    def __init__(self,filename1,filename2,filename3,filename4,filename5,filename6,filename7,filename8,filename9,filename10,filename11,filename12,filename13,filename14,filename15,filename16,filename17,filename18,filename19,filename20,filename21,filename22,filename23,filename24,result_dir,nb,ni,radius,radius2,Rb,name,width,err,height):    
        self.nb=nb # number of robots
        self.ni=ni # number of interiors
        self.nm=int(self.nb*4)
        self.Rb=Rb
        self.radius=radius
        self.radius2=radius2
        self.radius3=.03/2
        self.height=height
        self.err=0
        self.name=name # name of simulation 
        
        self.width=width # width for images
        self.widx=width # width x of images
        self.widy=width # width y of images
        self.result_dir=result_dir # result directory where to export images
        ### File direction for specific data ###
        self.filename1=filename1 # name for robot
        self.filename2=filename2 # name for interior
        self.filename3=filename3 # name for radius data
        self.filename4=filename4 # ball x position
        self.filename5=filename5 # ball z position
        self.filename6=filename6  # force controllers   
        self.filename7=filename7 # pull force of ball   
        self.filename8=filename8 # x contact force         
        self.filename9=filename9 # y contact force         
        self.filename10=filename10 # y contact force
        self.filename11=filename11 # AN
        self.filename12=filename12 # BN
        self.filename13=filename13 # x contact point       
        self.filename14=filename14 # y contact point    
        self.filename15=filename15 # z contact point
        self.filename16=filename16 # nc       
        self.filename17=filename17 # phi angle of ball
        self.filename18=filename18 # theta angle of ball
        self.filename19=filename19 # psi angle of ball
        self.filename20=filename20 # contact ID A
        self.filename21=filename21 # contact ID B
        self.filename22=filename22 # bot total force 
        self.filename23=filename23
        self.filename24=filename24

        self.data1 = np.genfromtxt(self.filename1,delimiter=',') # extract data for robots
        self.data2 = np.genfromtxt(self.filename2,delimiter=',') # extract data for particles        
        self.data3 = np.load(self.filename3) # radius of each bot interor positon 
        self.data4 = np.genfromtxt(self.filename4,delimiter=',') # extract data ballx
        self.data5 = np.genfromtxt(self.filename5,delimiter=',') # extract data ballz 
        self.data6 = np.genfromtxt(self.filename6,delimiter=',') # force controllers   
        self.data7 = np.genfromtxt(self.filename7,delimiter=',') # extract pull force           
        self.data8 = np.genfromtxt(self.filename8,delimiter=',')  # x contact force    
        self.FCX = self.data8 # rename it this corresponds to control force X
        self.data9 = np.genfromtxt(self.filename9,delimiter=',')  # y contact force
        self.data10 = np.genfromtxt(self.filename10,delimiter=',') # z contact force
        self.FCZ = self.data10 # rename it this corresponds to control force Z
        self.xpos=0
        self.zpos=0
        
        self.xposp=0
        self.zposp=0
        
        self.xposm=0
        self.zposm=0
        
        self.AN=[] # empty array of contact ID A
        infile = open(self.filename11, 'r') # now we fill the array AN
        for row in csv.reader(infile):
            self.AN.append(row[1:])
        self.BN=[] # empty array of contact ID B
        infile = open(self.filename12, 'r') # now we fill the array AN

        for row in csv.reader(infile):
            self.BN.append(row[1:])
        


        self.data13 = np.genfromtxt(self.filename13,delimiter=',') # x contact point
        self.xcontact=self.data13  # rename x contact point      
        self.data14 = np.genfromtxt(self.filename14,delimiter=',') # y contact point
        self.data15 = np.genfromtxt(self.filename15,delimiter=',') # z contact point
        self.zcontact=self.data15  # rename z contact point  
        self.data16 = np.genfromtxt(self.filename16,delimiter=',') # nc        
        self.nc=self.data16 # rename nc number of contacts 
        self.data17 = np.genfromtxt(self.filename17,delimiter=',')        
        self.phib=self.data17 # phi angle of ball
        self.data18 = np.genfromtxt(self.filename18,delimiter=',')
        self.thetab=self.data18 # theta angle of ball
        self.data19 = np.genfromtxt(self.filename19,delimiter=',')
        self.psib=self.data19 # psi angle of ball
        self.AID = np.genfromtxt(self.filename20,delimiter=',') # ID of contact body  
        self.BID = np.genfromtxt(self.filename21,delimiter=',') # ID of other contact body
        self.Rm=self.data3['Rm']  # radiuses of each interior particle needed for bi dispersion
        self.ballx=self.data4 # ball x position
        self.ballz=self.data5 # ball z position       
        self.FB=self.data7 # ball pull force
        self.data22=np.genfromtxt(self.filename22,delimiter=',') # total bot forces
        self.data23 = np.genfromtxt(self.filename23,delimiter=',') # extract data for robots
        self.data24 = np.genfromtxt(self.filename24,delimiter=',') # extract data for particle_angles      
        
        self.F2=[]
        
        self.FBX=[] 
        self.FBZ=[]
        
        self.MAG=[]
        
        self.XC=[] # empty array of contact points of x
        self.ZC=[] # empty array of contact points of z
        
        self.M=[] # empty array of moments
        self.Mo=[]
        
        self.Xb={} # empty list of robot x position
        self.Yb={} # empty list of robot y position 
        
        self.Xp={} # empty list of particle positions
        self.Yp={} # empty list of particle position
        
        self.Xbm={}
        self.Ybm={}
        
        self.Fcx={} # empty list of x controller forc
        self.Fcy={} # empty list of x controller forc
        self.Fcz={} # empty list of z controller force        

        self.Ftotalx={}
        self.Ftotaly={}
        self.Ftotalz={}
  
        self.Pphi={}
        self.Ptheta={}
        self.Ppsi={}
        
    def sort_data(self):
        ''' Sort position data of robots and particles '''
        (m1,n1)=np.shape(self.data1)
        self.data1=self.data1[:,1:n1]
        self.time=self.data1[0,:]
        Xpos=self.data1[1:self.nb+1,:]
        Ypos=self.data1[self.nb+1:2*self.nb+1,:]
        Zpos=self.data1[(2*self.nb)+1:3*self.nb+1,:] 
        self.xpos=Xpos
        self.zpos=Zpos
        
        (m2,n2)=np.shape(self.data2)        
        self.data2=self.data2[:,1:n2]        
        Xposp=self.data2[1:self.ni+1,:]
        Yposp=self.data2[self.ni+1:2*self.ni+1,:]
        Zposp=self.data2[(2*self.ni)+1:3*self.ni+1,:]  
        self.xposp=Xposp
        self.zposp=Zposp
        
        (m23,n23)=np.shape(self.data23)
        self.data1=self.data23[:,1:n23]
        Xposm=self.data23[1:self.nm+1,:]
        Yposm=self.data23[self.nm+1:2*self.nm+1,:]
        Zposm=self.data23[(2*self.nm)+1:3*self.nm+1,:] 
        self.xposm=Xposm
        self.zposm=Zposm
        
        (m24,n24)=np.shape(self.data24)
        self.data24=self.data24[:,1:n1]
        self.time=self.data24[0,:]
        Pphi=self.data24[1:self.ni+1,:]
        Ptheta=self.data24[self.ni+1:2*self.ni+1,:]
        Ppsi=self.data24[(2*self.ni)+1:3*self.ni+1,:] 
       

        for i in range(self.ni):
            self.Pphi["Pphi{0}".format(i)]=Pphi[i,:]
            self.Ptheta["Ptheta{0}".format(i)]=Ptheta[i,:]
            self.Ppsi["Ppsi{0}".format(i)]=Ppsi[i,:]
        
        for i in range(self.nb):
            self.Xb["Xb{0}".format(i)]=Xpos[i,:]
            self.Yb["Yb{0}".format(i)]=Zpos[i,:]

        for i in range(self.nm):
            self.Xbm["Xbm{0}".format(i)]=Xposm[i,:]
            self.Ybm["Ybm{0}".format(i)]=Zposm[i,:]
            
        for i in range(self.ni):
            self.Xp["Xp{0}".format(i)]=Xposp[i,:]
            self.Yp["Yp{0}".format(i)]=Zposp[i,:]
            
        
            
    def sort_data_forces(self):
        ''' sort control force  '''
        (m7,n7)=np.shape(self.data6)
        self.data6=self.data6[:,1:n7]
        Fcx=self.data6[1:self.nb+1,:]
        Fcy=self.data6[self.nb+1:2*self.nb+1,:]
        Fcz=self.data6[(2*self.nb)+1:3*self.nb+1,:] 
        
        Ftotalx=self.data22[1:self.nb+1,:]
        Ftotaly=self.data22[self.nb+1:2*self.nb+1,:]
        Ftotalz=self.data22[(2*self.nb)+1:3*self.nb+1,:]
        
        for i in range(self.nb):
            self.Ftotalx["Ftotalx{0}".format(i)]=Ftotalx[i,:]
            self.Ftotaly["Ftotaly{0}".format(i)]=Ftotaly[i,:]
            self.Ftotalz["Ftotalz{0}".format(i)]=Ftotalz[i,:]   
            
            self.Fcx["Fcx{0}".format(i)]=Fcx[i,:]
            self.Fcy["Fcy{0}".format(i)]=Fcy[i,:]
            self.Fcz["Fcz{0}".format(i)]=Fcz[i,:]   
            

   
    

            
                    
    def create_images_ball(self,geom):
        '''Create the images for grasping simuation that will be compiled into a video'''
        direct = os.path.join(self.result_dir,self.name+'_video_')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0

        for i in range(len(self.time)):    
            fig = plt.figure(dpi=300)
            fig.set_size_inches(10, 10)
            widx=self.widx
            widy=self.widy
            ax = plt.axes(xlim=(-widx,widx), ylim=(-widy, widy))          
            bots=[]
            Xo=[]
            Yo=[]

            for j in range(0,len(self.Xb)):
                x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]    
                patch = plt.Circle((y0, x0),self.radius-self.err, fc='black')
                ax.add_patch(patch)
                Yo.append(y0)
                Xo.append(x0)
                
            for j in range(len(self.Xp)):
                x0=self.Xp['Xp'+str(j)][i]
                y0=self.Yp['Yp'+str(j)][i]
                if self.Rm[j]==self.radius2:
                    c='tab:blue'
                if self.Rm[j]==self.radius2*np.sqrt(2):
                    c='tab:green' 
                patch = plt.Circle((y0, x0),self.Rm[j]-self.err, fc=c)
                ax.add_patch(patch)
                    
            # if grasping a circle
            if geom=="circle":
                patch = plt.Circle((self.data5[i], self.data4[i]),self.Rb, fc='tab:grey',edgecolor='tab:grey',linewidth=1)
                ax.add_patch(patch)            
            
            # if grasping a square
            if geom=="square":
                const=.5*np.pi*self.Rb
                px=self.data5[i]-const/2 
                py=self.data4[i] -const/2
                patch = matplotlib.patches.Rectangle((px, py),const, const,fc='tab:grey',edgecolor='tab:grey')     
                ax.add_patch(patch)
            # if grasping a triangle
            if geom=="triangle":
                patch = RegularPolygon((self.data5[i], self.data4[i]),3,.4627,orientation=-np.pi/2,fc='tab:grey',edgecolor='tab:grey')
                ax.add_patch(patch)  
            
            
            plt.title('t= ' + str(np.round(self.time[i],3)))
            plt.savefig(direct+"/frame%04d.jpg" % count)
            count=count+1 
            print(str(i)+ "of"+ str(len(self.time)))
            plt.close('all')

    def create_images_ball_entry(self,entry,geom):
        ''' shows the forces beign applied '''
        direct = os.path.join(self.result_dir,self.name+'_video_force')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        i=entry
  
        fig = plt.figure(dpi=300)
        fig.set_size_inches(2, 2)

        widx=self.widx
        widy=self.widy
        ax = plt.axes(xlim=(-widx,widx), ylim=(-widy, widy))
        plt.axis('off')
        
        xl=[]
        yl=[]
        

        #ax=plt.axes()
        bots=[]
        Xo=[]
        Yo=[]
        


        for j in range(0,len(self.Xb)):
            x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]    
            patch = plt.Circle((y0, x0),self.radius-self.err, fc='k')
            ax.add_patch(patch)
            
            
        for j in range(len(self.Xp)):
            x0=self.Xp['Xp'+str(j)][i]
            y0=self.Yp['Yp'+str(j)][i]    
            if self.Rm[j]==self.radius2:
                c='tab:blue'
            if self.Rm[j]==self.radius2*np.sqrt(2):
                c='tab:green' 
    
            patch = plt.Circle((y0, x0),self.Rm[j]-self.err, fc=c)
            ax.add_patch(patch)
            
        if geom=="circle":
            patch = plt.Circle((self.data5[i], self.data4[i]),self.Rb, fc='tab:grey',edgecolor='tab:grey',linewidth=1)
            ax.add_patch(patch)            
        

        if geom=="square":
            self.Rb=.5*np.pi*self.Rb
            px=self.data5[i] - self.Rb/2
            py=self.data4[i] - self.Rb/2
            patch = matplotlib.patches.Rectangle((px, py),self.Rb,self.Rb,fc='tab:grey',edgecolor='tab:grey')     
            ax.add_patch(patch)
            
            
        if geom=="triangle":
            patch = RegularPolygon((self.data5[i], self.data4[i]),3,.4627,orientation=-np.pi/2,fc='tab:grey',edgecolor='tab:grey')
            ax.add_patch(patch)
                         
        #plt.xlim([0,2.0])
        plt.axis('equal')
        #patch = plt.Circle((self.data5[i], self.data4[i]),self.Rb*1.1, fc='none',edgecolor='tab:orange',linestyle='--',linewidth=1)        
        #ax.add_patch(patch)
        #print(self.GRASP_CON[i])
        #plt.title('t= ' + str(np.round(self.time[i],3)))
       
    def smooth(self,y, box_pts):
        ''' smooth the function'''
        box = np.ones(box_pts)/box_pts
        y_smooth = np.convolve(y, box, mode='same')
        return y_smooth        
        
        

class create_videos_irr:
    def __init__(self,filename,filename2,filename3,filename4,filename5,filename6,filename7,filename8,filename9,result_dir,nb,ni,mode,R,radius,radius2,name,width,err,tim,Rb):

        self.nb=nb # number of robots
        self.ni=ni # number of interiors      
        self.mode=mode  # simulation mode      
        self.R=R # initial radius of system
        self.radius=radius # robot radius
        self.radius2=radius2 # radius of interior
        self.err=err # if we spaced out the rings more 
        self.widx=width # width of the screen x
        self.widy=width # width of the screen y
        self.name=name # name of simulation to be processed
        self.tim=tim
        self.Rb=Rb  
        self.Xm=0
        self.Ym=0
        self.Tm=0
        self.xp=0
        self.yp=0
        self.markerx = 0
        self.markery = 0
        
        
        self.Xb={} # empty list of robot x position
        self.Yb={} # empty list of robot y position 
        
        self.Xbm={} # empty list of robot x position
        self.Ybm={} # empty list of robot y position         
        
        self.Xp={} # empty list of particle positions
        self.Yp={} # empty list of particle position
        
        self.Fcx={} # empty list of x controller forc
        self.Fcy={} # empty list of x controller forc
        self.Fcz={} # empty list of z controller force        
        
        
        
        self.filename=filename # name for robot
        self.filename2=filename2 # name for interior
        self.filename3=filename3# outline of desried shape
        self.filename4=filename4 # name for radius data
        self.filename5=filename5 # ball x position
        self.filename6=filename6 # ball z position
        self.filename7=filename7  # force controllers   
        self.filename8=filename8 # pull force of ball
        self.filename9=filename9
        self.data1 = np.genfromtxt(self.filename,delimiter=',') # extract data for robots
        self.data2 = np.genfromtxt(self.filename2,delimiter=',') # extract data for particles
        
        if self.filename9!=None:
            self.data9 = np.genfromtxt(self.filename9,delimiter=',') 
        

        if self.filename3!=None:
            self.data3 = np.load(self.filename3,mmap_mode='r',allow_pickle=True)
            self.XT=self.data3["XT"]
            self.YT=self.data3["YT"]

        #radius of each interior more important for bi dispersion
        self.data4=np.load(self.filename4) 
        self.Rm=self.data4['Rm'] 
        
        # if there is a ball
        if self.filename5!=None:
            self.data5 = np.genfromtxt(self.filename5,delimiter=',') # extract data ballx
            self.data6 = np.genfromtxt(self.filename6,delimiter=',') # extract data ballz  
            self.ballx=self.data5 # ball x position
            self.ballz=self.data6 # ball z position 
            self.data8 = np.genfromtxt(self.filename8,delimiter=',') # extract pull force           
            self.FB=self.data8
        
        
        #   If there is controller force
        if self.filename7!=None:
            self.data7 = np.genfromtxt(self.filename7,delimiter=',')      

        self.result_dir=result_dir # result directory where to export images

 
    def sort_data(self):
        ''' sort data'''
        (m1,n1)=np.shape(self.data1)
        self.data1=self.data1[:,1:n1]
        self.time=self.data1[0,:]
        Xpos=self.data1[1:self.nb+1,:]
        Ypos=self.data1[self.nb+1:2*self.nb+1,:]
        Zpos=self.data1[(2*self.nb)+1:3*self.nb+1,:] 
        
        (m2,n2)=np.shape(self.data2)        
        self.data2=self.data2[:,1:n2]        
        Xposp=self.data2[1:self.ni+1,:]
        Yposp=self.data2[self.ni+1:2*self.ni+1,:]
        Zposp=self.data2[(2*self.ni)+1:3*self.ni+1,:]  
        if self.filename9!=None:
            (m9,n9)=np.shape(self.data9)
            self.data9=self.data9[:,1:n9]
            Xposm=self.data9[1:self.nm+1,:]
            Yposm=self.data9[self.nm+1:2*self.nm+1,:]
            Zposm=self.data9[(2*self.nm)+1:3*self.nm+1,:] 
            for i in range(self.nm):
                self.Xbm["Xbm{0}".format(i)]=Xposm[i,:]
                self.Ybm["Ybm{0}".format(i)]=Zposm[i,:]
        for i in range(self.nb):
            self.Xb["Xb{0}".format(i)]=Xpos[i,:]
            self.Yb["Yb{0}".format(i)]=Zpos[i,:]

        
        for i in range(self.ni):
            self.Xp["Xp{0}".format(i)]=Xposp[i,:]
            self.Yp["Yp{0}".format(i)]=Zposp[i,:]

            
    def sort_data_forces(self):
        ''' sort control force  '''
        (m7,n7)=np.shape(self.data7)
        self.data7=self.data7[:,1:n7]
        Fcx=self.data7[1:self.nb+1,:]
        Fcy=self.data7[self.nb+1:2*self.nb+1,:]
        Fcz=self.data7[(2*self.nb)+1:3*self.nb+1,:] 
        
        for i in range(self.nb):
            self.Fcx["Fcx{0}".format(i)]=Fcx[i,:]
            self.Fcy["Fcy{0}".format(i)]=Fcy[i,:]
            self.Fcz["Fcz{0}".format(i)]=Fcz[i,:]
    
      
##############################################################################

    def create_images_nmax_1shape(self):
        ''' Create images for video of robot forming shapes '''
        direct = os.path.join(self.result_dir,self.name+'_video')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        for i in range(len(self.time)-1):    
            fig = plt.figure(dpi=300)
            fig.set_size_inches(10, 10)
            
            ax = plt.axes(xlim=(-(self.widx),self.widx), ylim=(-(self.widy), self.widy))

            for j in range(0,len(self.Xb)):
                x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]    
                patch = plt.Circle((y0, x0),self.radius-self.err, fc='black')
                ax.add_patch(patch)
                
            for j in range(len(self.Xp)):
                x0,y0=self.Xp['Xp'+str(j)][i],self.Yp['Yp'+str(j)][i]
                if self.Rm[j]==self.radius2:
                    c='tab:blue'
                if self.Rm[j]==self.radius2*np.sqrt(2):
                    c='tab:green'
                patch = plt.Circle((y0, x0),self.Rm[j]-self.err, fc=c)
                ax.add_patch(patch)
            
            plt.plot(self.XT,self.YT,color='tab:red',linestyle='dashed',linewidth=2) 
                
            plt.title('t= ' + str(np.round(self.time[i],3)))
            plt.savefig(direct+"/frame%04d.jpg" % count)
            count=count+1 
            plt.close('all')


    def create_images_nmax_snapshot_1shape(self,entry,titl,xticks,yticks):
        ''' create single entry of robot forming shape '''
        count=0
        import matplotlib.font_manager as fm
        # Rebuild the matplotlib font cache
        fm._rebuild()
        mpl.rcParams['font.family'] = 'sans-serif'
        plt.rcParams['font.size'] = 10
        plt.rcParams['axes.linewidth'] = 1         
        fig, ax = plt.subplots(figsize=(2.16, 2.16),dpi=300)
        fig.subplots_adjust(top=0.85,bottom=0.3,left=0.30,right=0.85,hspace=0,wspace=0)
        ax.axis('equal')
        i=entry
        for j in range(0,len(self.Xb)):
            x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]    
            patch = plt.Circle((y0, x0),self.radius-self.err, fc='black')
            ax.add_patch(patch)

        for j in range(len(self.Xp)):
            x0=self.Xp['Xp'+str(j)][i]
            y0=self.Yp['Yp'+str(j)][i]
            if self.Rm[j]==self.radius2:
                c='tab:blue'
            if self.Rm[j]==self.radius2*np.sqrt(2):
                c='tab:green'                
            patch = plt.Circle((y0, x0),self.Rm[j]-self.err, fc=c)
            ax.add_patch(patch)
            
        ax.set_xticks(xticks)
        ax.set_yticks(yticks)
        ax.xaxis.set_tick_params(labelsize=8)
        ax.yaxis.set_tick_params(labelsize=8)
        ax.set_xlabel('$x$ (Meters)',labelpad=1,fontsize=8)
        ax.set_ylabel('$y$ (Meters)',labelpad=1,fontsize=8)  
        ax.plot(self.XT,self.YT,color='tab:red',linestyle='dashed',zorder=0) 
        ax.set_title(titl)
        count=count+1 
        plt.savefig(titl+'.svg')
        #plt.close('all')


##############################################################################
    def create_images_nmax_letter(self):
        '''Create images for the letter morphing '''
        direct = os.path.join(self.result_dir,self.name+'_video')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        count=0
        for i in range(len(self.time)-1):    
            fig = plt.figure(dpi=150)
            fig.set_size_inches(10, 10)
            
            ax = plt.axes(xlim=(-(self.widx),self.widx), ylim=(-(self.widy), self.widy))
            #ax=plt.axes()
            for j in range(0,len(self.Xb)):
                x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]    
                patch = plt.Circle((y0, x0),self.radius-self.err, fc='black')
                ax.add_patch(patch)
                
            for j in range(len(self.Xp)):
                x0=self.Xp['Xp'+str(j)][i]
                y0=self.Yp['Yp'+str(j)][i]
                if self.Rm[j]==self.radius2:
                    c='tab:blue'
                if self.Rm[j]==self.radius2*np.sqrt(2):
                    c='tab:green'
                patch = plt.Circle((y0, x0),self.Rm[j]-self.err, fc=c)
                ax.add_patch(patch)
                # J
                if self.time[i]<self.tim[0]:
                    plt.plot(self.XT[1],self.YT[1],color='tab:red',linestyle='dashed',linewidth=1,zorder=0) 
                # A    
                if  self.time[i]>self.tim[0] and self.time[i]<self.tim[1]:
                    plt.plot(self.XT[2],self.YT[2],color='tab:red',linestyle='dashed',linewidth=1,zorder=0)
                # M    
                if  self.time[i]>self.tim[1] and self.time[i]<self.tim[2]:
                    plt.plot(self.XT[3],self.YT[3],color='tab:red',linestyle='dashed',linewidth=1,zorder=0)
                # O   
                if  self.time[i]>self.tim[2] and self.time[i]<self.tim[3]:
                    plt.plot(self.XT[4],self.YT[4],color='tab:red',linestyle='dashed',linewidth=1,zorder=0)                    
                # E   
                if  self.time[i]>self.tim[3] and self.time[i]<self.tim[4]:
                    plt.plot(self.XT[5],self.YT[5],color='tab:red',linestyle='dashed',linewidth=1,zorder=0) 
                # B    
                if  self.time[i]>self.tim[4] and self.time[i]<self.tim[5]:
                    plt.plot(self.XT[6],self.YT[6],color='tab:red',linestyle='dashed',linewidth=1,zorder=0)                      
                # A
                if  self.time[i]>self.tim[5]:
                    plt.plot(self.XT[7],self.YT[7],color='tab:red',linestyle='dashed',linewidth=1,zorder=0)                    
                 
            plt.title('t= ' + str(np.round(self.time[i],3)))
            plt.savefig(direct+"/frame%04d.jpg" % count)
                 
            count=count+1 
            
            plt.close('all')           

    def create_images_nmax_letter_snap_shot(self,entry):
        ''' Create images for specific leter '''
        direct = os.path.join(self.result_dir,self.name+'_video')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        i=entry

        import matplotlib.font_manager as fm
        fm._rebuild()
        mpl.rcParams['font.family'] = 'sans-serif'
        plt.rcParams['font.size'] = 10
        plt.rcParams['axes.linewidth'] = 1         
        fig, ax = plt.subplots(figsize=(2, 2),dpi=300)
        
        fig.subplots_adjust(top=0.9,bottom=0.049,left=0.001,right=1,hspace=0,wspace=0)
        ax = plt.axes(xlim=(-(self.widx),self.widx), ylim=(-(self.widy), self.widy))
        plt.axis('off')
        for j in range(0,len(self.Xb)):
            x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]    
            patch = plt.Circle((y0, x0),self.radius-self.err, fc='black')
            ax.add_patch(patch)
            
        for j in range(len(self.Xp)):
            x0,y0=self.Xp['Xp'+str(j)][i],self.Yp['Yp'+str(j)][i]
            
            if self.Rm[j]==self.radius2:
                c='tab:blue'
            if self.Rm[j]==self.radius2*np.sqrt(2):
                c='tab:green'
                
            patch = plt.Circle((y0, x0),self.Rm[j]-self.err, fc=c)
            ax.add_patch(patch)
        
        ax.set_aspect('equal', 'box')

        plt.savefig(str(entry)+'.svg')
          
##############################################################################
    def create_images_morphing(self):
        ''' Create images for morphing'''
        direct = os.path.join(self.result_dir,self.name+'_video')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
            
        count=0
        for i in range(len(self.time)-1):    
            fig = plt.figure(dpi=300)
            fig.set_size_inches(10, 10)
            ax = plt.axes(xlim=(-(self.widx),self.widx), ylim=(-(self.widy), self.widy))
            
            for j in range(0,len(self.Xb)):
                x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]    
                patch = plt.Circle((x0, y0),self.radius-self.err, fc='black')
                ax.add_patch(patch)
                
            for j in range(len(self.Xp)):
                x0=self.Xp['Xp'+str(j)][i]
                y0=self.Yp['Yp'+str(j)][i]
                
                if self.Rm[j]==self.radius2:
                    c='tab:blue'
                if self.Rm[j]==self.radius2*np.sqrt(2):
                    c='tab:green'
                
                patch = plt.Circle((x0, y0),self.Rm[j]-self.err, fc=c)
                ax.add_patch(patch)

                if self.time[i]<10:
                    plt.plot(self.XT[1],self.YT[1],color='tab:red',linestyle='dashed',linewidth=2) 
                if  self.time[i]>10 and self.time[i]<20:
                    plt.plot(self.XT[2],self.YT[2],color='tab:red',linestyle='dashed',linewidth=2)
                if  self.time[i]>20:
                    plt.plot(self.XT[3],self.YT[3],color='tab:red',linestyle='dashed',linewidth=2)  
        
            plt.title('t= ' + str(np.round(self.time[i],3)))
            plt.savefig(direct+"/frame%04d.jpg" % count)
            count=count+1 
            plt.close('all')

    def create_images_morphing_snap_shot(self,entry,xticks,yticks):
        ''' Create morphing snap shots  '''
        i=entry
        import matplotlib.font_manager as fm
        fm._rebuild()
        mpl.rcParams['font.family'] = 'sans-serif'
        plt.rcParams['font.size'] = 10
        plt.rcParams['axes.linewidth'] = 1         
        fig, ax = plt.subplots(figsize=(2, 2),dpi=300)
        fig.subplots_adjust(top=0.85,bottom=0.2,left=0.25,right=0.90,hspace=0,wspace=0)
        #plt.axis('off')
        #ax.axis('equal')
        ax = plt.axes(xlim=(-(self.widx),self.widx), ylim=(-(self.widy), self.widy))
        for j in range(0,len(self.Xb)):
            x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]    
            patch = plt.Circle((x0, y0),self.radius-self.err, fc='black')
            ax.add_patch(patch)
   
        for j in range(len(self.Xp)):
            x0=self.Xp['Xp'+str(j)][i]
            y0=self.Yp['Yp'+str(j)][i]
            if self.Rm[j]==self.radius2:
                c='tab:blue'
            if self.Rm[j]==self.radius2*np.sqrt(2):
                c='tab:green'
            patch = plt.Circle((x0, y0),self.Rm[j]-self.err, fc=c)
            ax.add_patch(patch)

            if self.time[i]<10:
                plt.plot(self.XT[1],self.YT[1],color='tab:red',linestyle='dashed',linewidth=1,zorder=0) 
            if  self.time[i]>10 and self.time[i]<20:
                plt.plot(self.XT[2],self.YT[2],color='tab:red',linestyle='dashed',linewidth=1,zorder=0)
            if  self.time[i]>20:
                plt.plot(self.XT[3],self.YT[3],color='tab:red',linestyle='dashed',linewidth=1,zorder=0)  
        
        ax.set_xticks(xticks)
        ax.set_yticks(yticks)
        ax.xaxis.set_tick_params(labelsize=8)
        ax.yaxis.set_tick_params(labelsize=8)
        ax.set_xlabel('$x$ (Meters)',labelpad=1,fontsize=8)
        ax.set_ylabel('$y$ (Meters)',labelpad=1,fontsize=8)
        ax.set_title(str(np.round(self.time[i],1))+" s")
        


##############################################################################

    def create_images_nmax_tunnel(self):
        ''' create images for tunnel video '''
        direct = os.path.join(self.result_dir,self.name+'_tunnel_video')    
        if not os.path.isdir(direct):
            os.makedirs(direct)
        
        ycenter=-4
        xcenter=8
        ranged=10
        count=0
        xtop=xcenter+ranged
        xbottom=xcenter-ranged
        ytop=ycenter+ranged
        ybottom=ycenter-ranged
        
        for i in range(len(self.time)-1):    
            fig = plt.figure(dpi=150)
            fig.set_size_inches(10, 10)
            
            ax = plt.axes(xlim=(xbottom,xtop), ylim=(ybottom,ytop))
            #ax=plt.axes()
            for j in range(0,len(self.Xb)):
                x0,y0=self.Xb['Xb'+str(j)][i],self.Yb['Yb'+str(j)][i]    
                patch = plt.Circle((-x0, y0),self.radius-self.err, fc='black')
                ax.add_patch(patch)
                
            for j in range(len(self.Xp)):
                x0=self.Xp['Xp'+str(j)][i]
                y0=self.Yp['Yp'+str(j)][i]
                if np.round(self.Rm[j],3)==.04:
                    c='tab:blue'
                else:
                    c='tab:green' 
                patch = plt.Circle((-x0, y0),self.Rm[j]-self.err, fc=c)
                ax.add_patch(patch)
                
            c1=0
            c2=10
            # x2=np.linspace(c1,c2,100)
            # x1=np.linspace(c1,c2,100)                
            # line1=np.tanh(x1-5)-3.75
            # line2=np.tanh(x2-5)-2.25 
            # plt.fill_between(x1,line2,color='tab:grey',alpha=1)
            # plt.fill_between(x1,line1,-6.25,color='tab:grey',alpha=1)            
            # plt.plot(x1,line1,color='k',linewidth=2)
            # plt.plot(x2,line2,color='k',linewidth=2)
            
        
            x2=np.linspace(c1,c2,1000)
            x1=np.linspace(c1,c2,1000)                
            line1=np.sin(1.745*x1)-3.75
            line2=np.sin(1.745*x1)-2.0
            plt.fill_between(x1,line2,color='tab:grey',alpha=1)
            plt.fill_between(x1,line1,-6.25,color='tab:grey',alpha=1)      
            plt.plot(x1,line1,color='k',linewidth=2)
            plt.plot(x2,line2,color='k',linewidth=2)            
            
            # plt.hlines(y=0, xmin=0, xmax=10, linewidth=2, color='k')
            # plt.hlines(y=-6.25, xmin=0, xmax=10, linewidth=2, color='k') 
            # plt.vlines(x=0, ymin=-3.00, ymax=0, linewidth=2, color='k')
            # plt.vlines(x=0, ymin=-6.25, ymax=-5, linewidth=2, color='k')
            # plt.vlines(x=10, ymin=-6.25, ymax=-3.00, linewidth=2, color='k')
            # plt.vlines(x=10, ymin=-1, ymax=0, linewidth=2, color='k')
      
            plt.title('t= ' + str(np.round(self.time[i],3)))
            plt.savefig(direct+"/frame%04d.jpg" % count)
            count=count+1 
            plt.close('all')      

##############################################################################   
    def create_images_nmax_tunnel_entry(self,entry):
        ''' Create imahes of tunnel series of entries '''

        import matplotlib.font_manager as fm
        fm._rebuild()
        mpl.rcParams['font.family'] = 'sans-serif'
        plt.rcParams['font.size'] = 10
        plt.rcParams['axes.linewidth'] = 1 
        const=2.66   
        fx=6
        fy=fx/const
        fig, ax = plt.subplots(figsize=(fx,fy),dpi=300)
        fig.subplots_adjust(top=1,bottom=0.0,left=0.0,right=1,hspace=0,wspace=0)
        #ax = plt.axes(xlim=(-3,15), ylim=(-13,5))
        ax = plt.axes(xlim=(-3,15.5), ylim=(-6.35,.15))
        plt.axis('off')
        plt.gca().set_aspect('equal')
        for k in range(len(entry)):
            for j in range(0,len(self.Xb)):
                x0,y0=self.Xb['Xb'+str(j)][entry[k]],self.Yb['Yb'+str(j)][entry[k]]    
                patch = plt.Circle((-x0, y0),self.radius-self.err, fc='black')
                ax.add_patch(patch)
    
                
            for j in range(len(self.Xp)):
                x0=self.Xp['Xp'+str(j)][entry[k]]
                y0=self.Yp['Yp'+str(j)][entry[k]]
                if np.round(self.Rm[j],3)==.04:
                    c='tab:blue'
                else:
                    c='tab:green' 
                    
                patch = plt.Circle((-x0, y0),self.Rm[j]-self.err, fc=c)
                ax.add_patch(patch)
                
        c1=0
        c2=10
        
        # x2=np.linspace(c1,c2,1000)
        # x1=np.linspace(c1,c2,1000)                
        # line1=np.sin(1.745*x1)-3.75
        # line2=np.sin(1.745*x1)-2.0
        # plt.fill_between(x1,line2,color='tab:grey',alpha=0.5)
        # plt.fill_between(x1,line1,-6.25,color='tab:grey',alpha=0.5)
        # plt.plot(x1,line1,color='k')
        # plt.plot(x2,line2,color='k')
        # plt.hlines(y=0, xmin=0, xmax=10, color='k')
        # plt.hlines(y=-6.25, xmin=0, xmax=10,color='k') 
        # plt.vlines(x=0, ymin=-3.2, ymax=0,color='k')
        # plt.vlines(x=0, ymin=-6.25, ymax=-4.75,color='k')
        # plt.vlines(x=10, ymin=-6.25, ymax=-2.75,color='k')
        # plt.vlines(x=10, ymin=-1.25, ymax=0,color='k')         
        
        x2=np.linspace(c1,c2,100)
        x1=np.linspace(c1,c2,100)                
        line1=np.tanh(x1-5)-3.75
        line2=np.tanh(x2-5)-2.25 
        
        plt.fill_between(x1,line2,color='tab:grey',alpha=0.5)
        plt.fill_between(x1,line1,-6.25,color='tab:grey',alpha=0.5)            
        plt.plot(x1,line1,color='k')
        plt.plot(x2,line2,color='k')
        
    
        plt.hlines(y=0, xmin=0, xmax=10, color='k')
        plt.hlines(y=-6.25, xmin=0, xmax=10,color='k') 
        plt.vlines(x=0, ymin=-3.2, ymax=0,color='k')
        plt.vlines(x=0, ymin=-6.25, ymax=-4.75,color='k')
        plt.vlines(x=10, ymin=-6.25, ymax=-2.75,color='k')
        plt.vlines(x=10, ymin=-1.25, ymax=0,color='k')   
        plt.scatter(15,-2.5,marker="*",c="tab:cyan",s=100)
        plt.savefig("Tunnel_snap_shots.svg")  
        plt.savefig("Tunnel_snap_shots.png")  

                 