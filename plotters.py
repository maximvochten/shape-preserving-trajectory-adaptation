#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Sep  5 09:52:48 2019

@author: Zeno Gillis, Victor Van Wymeersch, Maxim Vochten

Functions for plotting invariant signatures and trajectories

"""

from mpl_toolkits import mplot3d

import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
sns.set(style='whitegrid',context='paper')

def plotPose(pose, figure = '', label = '', c='b', m='.', orientation = False):
    """
    plots the given trajectory, positions and rotations are indicated
    @param: trajectory = list of pose matrices
    """

    if figure == '':
        fig = plt.figure()
    else:
        fig = figure
    ax = fig.gca(projection='3d')

    arrow_length = 0.01

    xq = pose[0,3]
    yq = pose[1,3]
    zq = pose[2,3]

    if orientation:
        #plot RED arrows at START
        ux = pose[0,0]
        uy = pose[1,0]
        uz = pose[2,0]

        vx = pose[0,1]
        vy = pose[1,1]
        vz = pose[2,1]

        wx = pose[0,2]
        wy = pose[1,2]
        wz = pose[2,2]

        # Make the direction data for the arrows
        a1 = ax.quiver(xq, yq, zq, ux, uy, uz, length=arrow_length, normalize=True, color = c)
        a2 = ax.quiver(xq, yq, zq, vx, vy, vz, length=arrow_length, normalize=True, color = c)
        a3 = ax.quiver(xq, yq, zq, wx, wy, wz, length=arrow_length, normalize=True, color = c)

    #plots the pose
    a4 = ax.scatter(xq, yq, zq, c=c, marker=m, label=label)

    if orientation:
        p = [a1,a2,a3,a4] #save the axis so that this pose can be removed again!!
    else:
        p = [a4]

    #labels, etc
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
#    ax.view_init(45, 45)
    ax.legend()

#    plt.autoscale(True, 'both', True)
#    plt.ion()
    plt.show()

    return fig, p



def plotTrajectory(trajectory, figure = None, label = "trajectory", title = '', c = 'b', m ='', mark = False):
    """
    plots the given trajectory, positions and rotations are indicated
    @param: trajectory = list of pose matrices
    """
    plt.ion()
    if figure == None:
        fig = plt.figure( num = title ,figsize=(15, 13), dpi=120, facecolor='w', edgecolor='k')
    else:
        fig = figure
    ax = fig.gca(projection='3d')
#    ax = plt.axes(projection='3d')
    x = []
    y = []
    z = []

    limit_value = 0.0

#    arrow_length = 0.1

    p_lst = []
    for i in range(len(trajectory)):
        xq = trajectory[i][0,3]
        yq = trajectory[i][1,3]
        zq = trajectory[i][2,3]

        x.append(xq)
        y.append(yq)
        z.append(zq)

        if (np.abs(min([limit_value, xq,yq,zq])) > max([limit_value, xq,yq,zq])):
            limit_value = min([limit_value, xq,yq,zq])
        else:
            limit_value = max([limit_value, xq,yq,zq])



        if ((i == len(trajectory)/4) or (i == len(trajectory)/2) or (i == 3*len(trajectory)/4)) and mark:
            fig, p = plotPose(trajectory[i], fig, orientation=True, c=c)

        elif (i== 0) and mark:
            fig, p = plotPose(trajectory[i], fig, c= 'g', m = 'x', orientation=True, label='Start pose')

        elif (i == len(trajectory) -1)and mark:
            fig, p = plotPose(trajectory[i], fig, c= 'r', m = 'x', orientation=True, label='End pose')

        else:
            fig, p = plotPose(trajectory[i], fig, c = c)

        p_lst.append(p)
    #plots the line
    a1, = ax.plot(x, y, z, label = label, c = c, linestyle=m)

    #for removing every second tick if the graphs are too busy
#    for label in ax.xaxis.get_ticklabels()[::2]:
#        label.set_visible(False)
#    for label in ax.yaxis.get_ticklabels()[::2]:
#        label.set_visible(False)
#    for label in ax.zaxis.get_ticklabels()[::2]:
#        label.set_visible(False)
    p_lst.append([a1])

#    #plots the points
#    ax.scatter(x, y, z, c='k', marker='.', label = 'datapoints')
#    ax.scatter(x[0], y[0], z[0], c='r', marker='x', label = 'start')
#    ax.scatter(x[-1], y[-1], z[-1], c='g', marker='x', label = 'end')

    #labels, etc
#    ax.set_xlim(-limit_value, limit_value)
#    ax.set_ylim(-limit_value, limit_value)
#    ax.set_zlim(-limit_value, limit_value)
    ax.zaxis.set_rotate_label(False)  # disable automatic rotation
    ax.set_xlabel('$X$ [m]')
    ax.set_ylabel('$Y$ [m]')
    ax.set_zlabel('$Z$ [m]', rotation = 90)

    ax.view_init(13,-170)
#    ax.view_init(-135,135)
    ax.axis('equal')

    plt.legend(loc=2, prop={'size': 15})


#    plt.title(title)
    plt.show()

    return fig, p_lst

def plotInvariantSignature(invariantSignature, title = 'Invariant Signature'):
    plt.figure( num = title ,figsize=(15, 13), dpi=120, facecolor='w', edgecolor='k')
    N = len(invariantSignature['U1']) + 1
    x_temp = np.linspace(0,1,N-1)
    plt.subplot(231)
    plt.plot(x_temp,[round(elem,6) for elem in invariantSignature['U1']],'r-')
    plt.title("U1 - Ir1 [rad]")
    plt.subplot(232)
    plt.plot(x_temp,invariantSignature['U2'],'r-')
    plt.title("U2 - Ir2 [rad]")
    plt.subplot(233)
    plt.plot(x_temp,invariantSignature['U3'],'r-')
    plt.title("U3 - Ir3 [rad]")
    plt.subplot(234)
    plt.plot(x_temp,[round(elem,6) for elem in invariantSignature['U4']],'r-')
    plt.title("U4 - It1 [m]")
    plt.subplot(235)
    plt.plot(x_temp,invariantSignature['U5'],'r-')
    plt.title("U5 - It2 [rad]")
    plt.subplot(236)
    plt.plot(x_temp,invariantSignature['U6'],'r-')
    plt.title("U6 - It3 [rad]")

def removeAxis(ax):
    ax.remove()

def removeMultipleAxis(pList):
    for i in range(len(pList)):
        if len(pList[i]) > 1:
            for ax in pList[i]:
                removeAxis(ax)
        else:
            removeAxis(pList[i][0])
