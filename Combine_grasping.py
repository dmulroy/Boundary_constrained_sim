# -*- coding: utf-8 -*-
"""
Created on Mon Aug  9 11:56:04 2021

@author: dmulr
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors as colors
import matplotlib as mpl
from matplotlib import animation
import matplotlib
import matplotlib.cm as cm
import matplotlib.patches as mpl_patches
from scipy.spatial import Voronoi, voronoi_plot_2d,ConvexHull
from matplotlib.patches import RegularPolygon
from numpy import diff
FBC=[]
BZC=[]

FBS=[]
BZS=[]

FBT=[]
BZT=[]

colors=['tab:blue','tab:green','tab:red']
yticks=[0,.1,.2,.3,.4,.5,.6]
#yticks=[1.7,1.8,1.9,2.0,2.1,2.2,2.3]
xticks=[0,200,400,600,800]

for i in range(len(colors)):
        data = np.load('Circle'+str(i)+'.npz',mmap_mode='r',allow_pickle=True)
        FBC.append(data['FB'])
        BZC.append(data['BZ'])







        # data = np.load('Square'+str(i)+'.npz',mmap_mode='r',allow_pickle=True)
        # FBS.append(data['FB'])
        # BZS.append(data['BZ'])

        # data = np.load('Triangle'+str(i)+'.npz',mmap_mode='r',allow_pickle=True)
        # FBT.append(data['FB'])
        # BZT.append(data['BZ'])


for i in range(len(FBC)):
    dydx=diff(BZC[i])/diff(FBC[i])
    ax.plot(FBC[i][:-1],dydx,color=colors[i],marker='o',ms=size,markevery=case,linewidth=width)
    dydx=diff(BZS[i])/diff(FBS[i])    
    ax.plot(FBS[i][:-1],dydx,color=colors[i],marker='s',ms=size,markevery=case,linewidth=width)
    dydx=diff(BZT[i])/diff(FBT[i])  
    ax.plot(FBT[i][:-1],dydx,color=colors[i],marker='^',ms=size,markevery=case,linewidth=width)
    #ax.plot(FBC[i],BZC[i],color=colors[i],marker='o',ms=size,markevery=case,linewidth=width)
    # ax.plot(FBS[i],BZS[i],color=colors[i],marker='s',ms=size,markevery=case,linewidth=width)
    # ax.plot(FBT[i],BZT[i],color=colors[i],marker='^',ms=size,markevery=case,linewidth=width)
      





import matplotlib.font_manager as fm
# Rebuild the matplotlib font cache
fm._rebuild()
mpl.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.size'] = 10
plt.rcParams['axes.linewidth'] = .5       
fig, ax = plt.subplots(figsize=(4, 2),dpi=300)
fig.subplots_adjust(top=0.88,bottom=0.20,left=0.120,right=0.95,hspace=0,wspace=0)
ax.grid(True)

# case=10
# width=.5
# size=2


# for i in range(len(FBC)):
#     dydx=diff(BZC[i])/diff(FBC[i])
#     ax.plot(FBC[i][:-1],dydx,color=colors[i],marker='o',ms=size,markevery=case,linewidth=width)
#     dydx=diff(BZS[i])/diff(FBS[i])    
#     ax.plot(FBS[i][:-1],dydx,color=colors[i],marker='s',ms=size,markevery=case,linewidth=width)
#     dydx=diff(BZT[i])/diff(FBT[i])  
#     ax.plot(FBT[i][:-1],dydx,color=colors[i],marker='^',ms=size,markevery=case,linewidth=width)
#     #ax.plot(FBC[i],BZC[i],color=colors[i],marker='o',ms=size,markevery=case,linewidth=width)
#     # ax.plot(FBS[i],BZS[i],color=colors[i],marker='s',ms=size,markevery=case,linewidth=width)
#     # ax.plot(FBT[i],BZT[i],color=colors[i],marker='^',ms=size,markevery=case,linewidth=width)
      
ax.set_xticks(xticks)
ax.set_yticks(yticks)
ax.xaxis.set_tick_params(labelsize=8)
ax.yaxis.set_tick_params(labelsize=8)
ax.set_xlabel('Pull Force (N)',fontsize=8,labelpad=1)
ax.set_ylabel('Position (M)',fontsize=8,labelpad=1)
ax.set_ylim([yticks[0]-.1,yticks[-1]])
ax.set_xlim([xticks[0]-10,xticks[-1]])        
        

    
# fig, ax = plt.subplots(figsize=(4, 2),dpi=300)
# fig.subplots_adjust(top=0.88,bottom=0.20,left=0.120,right=0.95,hspace=0,wspace=0)
# ax.grid(True)

# case=10
# width=.5
# size=2

# for i in range(len(FBC)):
#     ax.plot(FBC[i],BZC[i],color=colors[i],marker='o',ms=size,markevery=case,linewidth=width)
#     ax.plot(FBS[i],BZS[i],color=colors[i],marker='s',ms=size,markevery=case,linewidth=width)
#     ax.plot(FBT[i],BZT[i],color=colors[i],marker='^',ms=size,markevery=case,linewidth=width)
    



# ax.set_xticks(xticks)
# ax.set_yticks(yticks)
# ax.xaxis.set_tick_params(labelsize=8)
# ax.yaxis.set_tick_params(labelsize=8)
# ax.set_xlabel('Pull Force (N)',fontsize=8,labelpad=1)
# ax.set_ylabel('Position (M)',fontsize=8,labelpad=1)
# ax.set_ylim([yticks[0]-.1,yticks[-1]])
# ax.set_xlim([xticks[0]-10,xticks[-1]]) 



