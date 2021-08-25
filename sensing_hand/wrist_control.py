import numpy as np
import time
import serial
import socket
import math
import json
import _thread
from math import pi
import scipy.optimize as optimize

import waypoints as wp

class wrist_control():
    def __init__(self,robot):
        self.name = "test.json"
        self.robot = robot

    def shift_play(self,name="",dx=0,dy=0,dz=0,dr=0):
        if name == "":
            name = self.name
        sequence = json.load(open(name))
        print(len(sequence))
        print("average timestep: ",sequence[-1][1]/(len(sequence)-2))

        for i in range(0,len(sequence)):
            sequence[i][0][0] += dx
            sequence[i][0][1] += dy
            sequence[i][0][2] += dz

        self.robot.movej(wp.grasp_wpj)

        self.robot.movel(sequence[0][0])
        toc = time.time()
        for i in range(1,len(sequence)-1):
            self.robot.servoj(sequence[i][0],control_time=sequence[i][1],lookahead_time=0.008,gain=300)

        self.robot.stopl(0.5)
        tic = time.time()
        self.robot.socket_ping()
        print("recorded ",sequence[-1][1],"secs")
        print("executed in ",tic-toc,"secs")
        print("recorded end_pos: ",sequence[-1][0])
        print("actual end_pos:",self.robot.getl())

    def lin_play_1b(self,dx=0,dy=0,dz=0,dr=0,dl=0):
        #grasp_centre = [-0.4546+dx,-0.0846+dy,0.1394+dz,0.9860,-1.4931,-0.5669]
        #modify to ensure corrent thumb and finger contact
        #self.robot.movej(wp.grasp_wpj)
        self.robot.movel([-0.4637+dx,-0.0560+dy,0.2532+dz+dl,0.6514,-1.5089,-0.5977])#added waypoint to prevent collisions
        self.robot.movel([-0.4637+dx+dr,-0.0560+dy+dr,0.2132+dz+dl,0.6514,-1.5089,-0.5977])#thumb contact
        self.robot.movel([-0.4633+dx-dr,-0.0838+dy-dr,0.2057+dz+dl,0.6706,-1.5065,-0.4185])#stretch to palm
        self.robot.movel([-0.4546+dx-dr,-0.0846+dy-dr,0.1394+dz+dl/2,0.9860,-1.4931,-0.5669])#finger contact
        self.robot.movel([-0.4610+dx,-0.0834+dy-dr,0.1447+dz+dl/2,0.6684+dr*30*(0.9860-0.6684),-1.1774+dr*30*(-1.4931+1.1774),-0.5650+dr*30*(-0.5669+0.5650)])#re-orient
        self.robot.movel([-0.4618+dx,-0.0839+dy-dr,0.1974+dz+dl/2,0.6797+dr*30*(0.9860-0.6684),-1.1800+dr*30*(-1.4931+1.1774),-0.5636+dr*30*(-0.5669+0.5650)])#lift

    def lin_play_1c(self,dx=0,dy=0,dz=0,dr=0):
        if dr>=0:
            self.robot.movel([-0.5170+dx-dr,-0.0577+dy-dr,0.1824+dz,-0.4513,-1.5405,0.3436])#avoid collision
            self.robot.movel([-0.5030+dx-dr,-0.0565+dy-dr,0.1380+dz,-0.4023,-1.6682,0.2875])#finger contact
            self.robot.movel([-0.4723+dx-dr,-0.0522+dy-dr,0.1371+dz,-0.2949,-1.8249,0.2885])#stretch finger
            self.robot.movel([-0.4740+dx-2*dr,-0.0522+dy-dr,0.1347+dz,-0.0467+dr*0*(-0.2949+0.0467),-1.8339+dr*0*(-1.8249+1.8339),0.1449+dr*0*(0.2885-0.1449)])#rotate
            self.robot.movel([-0.4798+dx-2*dr,-0.0561+dy-dr,0.1850+dz,-0.0577+dr*0*(-0.2949+0.0467),-1.7996+dr*0*(-1.8249+1.8339),0.1619+dr*0*(0.2885-0.1449)])#lift

        else:
            self.robot.movel([-0.5170+dx-dr,-0.0577+dy-dr,0.1824+dz,-0.4513,-1.5405,0.3436])#avoid collision
            self.robot.movel([-0.5030+dx-dr,-0.0565+dy+dr,0.1380+dz+dr,-0.4023,-1.6682,0.2875])#finger contact
            self.robot.movel([-0.4723+dx-dr,-0.0522+dy+dr,0.1371+dz+dr,-0.2949,-1.8249,0.2885])#stretch finger
            self.robot.movel([-0.4740+dx-dr,-0.0522+dy+dr,0.1347+dz,-0.0467+dr*10*(-0.2949+0.0467),-1.8339+dr*10*(-1.8249+1.8339),0.1449+dr*10*(0.2885-0.1449)])#rotate
            self.robot.movel([-0.4798+dx-dr,-0.0561+dy+dr,0.1850+dz,-0.0577+dr*10*(-0.2949+0.0467),-1.7996+dr*10*(-1.8249+1.8339),0.1619+dr*10*(0.2885-0.1449)])#lift





    def unwrist(self,n=0,dr=0):
        if dr>0.0125 or dr<-0.0125:
            input("large dr, are you sure?")
        self.robot.movej(wp.grasp2_wpj)
        if n==0:
            #sphere_1
            #start
            self.robot.movel([-0.475336000000000,	-0.127265000000000,	0.253073000000000,	-1.09256000000000,	-1.24834000000000,	0.994270000000000])

            #re-orient
            self.robot.movel([-0.464269000000000,	-0.135539000000000,	0.1096001000000000+2*dr,	0.836164000000000,	-1.50540000000000,	-0.834779000000000])
            
            self.robot.movel([-0.464269000000000,	-0.135539000000000,	0.0896001000000000+2*dr,	0.836164000000000,	-1.50540000000000,	-0.834779000000000])
            self.robot.movel([-0.463991000000000,	-0.144895000000000-2*dr*np.sin(pi/4),	0.0555249000000000+2*dr*np.cos(pi/4),	0.839806000000000,	-1.55382000000000,	-0.733549000000000])#45deg thumb contact
            self.robot.movel([-0.469873000000000,	-0.141023000000000,	0.0698323000000000+2*dr,	-0.646488000000000,	-1.60852000000000,	0.567921000000000])
            self.robot.movel([-0.470525000000000,	-0.133916000000000+2*dr*np.cos(pi/3),	0.0614267000000000+2*dr*np.sin(pi/3),	-0.931745000000000,	-1.51233000000000,	0.794648000000000])#60deg finger contact
            
            #end
            self.robot.movel([-0.475336000000000,	-0.127265000000000,	0.253073000000000,	-1.09256000000000,	-1.24834000000000,	0.994270000000000])
        elif n==1:
            #sphere_2
            #start
            self.robot.movel([-0.475290000000000,	-0.127222000000000,	0.252966000000000,	-1.08880000000000,	-1.25116000000000,	0.990201000000000])
            
            #re-orient
            self.robot.movel([-0.4642,              -0.1322,            0.1132+2*dr,             0.5979,             -1.5548,            -0.6313])
            
            self.robot.movel([-0.464493000000000,	-0.133169000000000,	0.0800803000000000+2*dr,	0.666141000000000,	-1.59008000000000,	-0.667927000000000])
            self.robot.movel([-0.464752000000000,	-0.137853000000000,	0.0539380000000000,	0.682107000000000,	-1.61892000000000,	-0.635169000000000])
            #self.robot.movel([-0.465065000000000,	-0.138804000000000,	0.0599959000000000,	0.245689000000000-10*dr*(0.245689-0.682107),	-1.69582000000000-10*dr*(-1.69582+1.61892),	-0.254333000000000-10*dr*(-0.254333+0.635169)])
            tic = time.time()
            new_rot = self.rot_adjust([0.682107000000000,	-1.61892000000000,	-0.635169000000000],[0.245689,-1.69582,-0.254333],dr,0.025)
            print(time.time()-tic)
            self.robot.movel([-0.465065000000000,	-0.138804000000000,	0.0599959000000000,new_rot[0],new_rot[1],new_rot[2]])

            #end
            self.robot.movel([-0.475290000000000,	-0.127222000000000,	0.252966000000000,	-1.08880000000000,	-1.25116000000000,	0.990201000000000])
        elif n==2:
            #sphere_3
            #start
            self.robot.movel([-0.475272000000000,	-0.127183000000000,	0.252938000000000,	-1.09243000000000,	-1.24870000000000,	0.994123000000000])
            
            self.robot.movel([-0.466183000000000,	-0.114057000000000,	0.0929575000000000+2*dr,	-0.516627000000000,	-1.41614000000000,	0.419263000000000])
            self.robot.movel([-0.459429000000000,	-0.108625000000000,	0.0528503000000000+2*dr,	-0.350401000000000,	-1.55180000000000,	0.225758000000000])
            self.robot.movel([-0.430991000000000,	-0.105603000000000,	0.111022000000000+2*dr,	-1.30041000000000,	-1.94761000000000,	0.551213000000000])
            self.robot.movel([-0.434280000000000,	-0.109978000000000,	0.0881898000000000+2*dr*np.sin(pi/4),	-1.32060000000000,	-2.00411000000000,	0.533541000000000])
            
            #self.robot.movel([-0.466183000000000,	-0.114057000000000,	0.0929575000000000-0.01,	-0.516627000000000,	-1.41614000000000,	0.419263000000000])
            #self.robot.movel([-0.459429000000000,	-0.108625000000000,	0.0528503000000000-0.01,	-0.350401000000000,	-1.55180000000000,	0.225758000000000])
            #self.robot.movel([-0.430991000000000,	-0.105603000000000,	0.111022000000000-0.01,	-1.30041000000000,	-1.94761000000000,	0.551213000000000])
            #self.robot.movel([-0.434280000000000,	-0.109978000000000,	0.0881898000000000-0.005*np.sin(pi/4),	-1.32060000000000,	-2.00411000000000,	0.533541000000000])

            #end
            self.robot.movel([-0.475272000000000,	-0.127183000000000,	0.252938000000000,	-1.09243000000000,	-1.24870000000000,	0.994123000000000])
        elif n==3:
            #sphere_4
            #start
            self.robot.movel([-0.475324000000000,	-0.127279000000000,	0.253031000000000,	-1.08806000000000,	-1.25150000000000,	0.989910000000000])
            
            self.robot.movel([-0.472289000000000,	-0.118017000000000,	0.0878093000000000+2*dr,	-0.586390000000000,	-1.45509000000000,	0.466814000000000])
            self.robot.movel([-0.452589000000000,	-0.097893000000000,	0.0466695000000000+2*dr,	-0.845408000000000,	-1.53770000000000,	0.592837000000000])
            #self.robot.movel([-0.452589000000000,	-0.097893000000000,	0.0466695000000000-0.01,	-0.845408000000000,	-1.53770000000000,	0.592837000000000])
            #self.robot.movel([-0.448573000000000,	-0.095725500000000,	0.0462534000000000+dr,	-0.906224000000000,	-1.70268000000000,	0.455787000000000])
            new_rot = self.rot_adjust([-0.845408000000000,	-1.53770000000000,	0.592837000000000],[-0.906224000000000,-1.70268000000000,0.455787000000000],-dr,0.025)
            self.robot.movel([-0.448573000000000,	-0.095725500000000,	0.0462534000000000+dr,	new_rot[0],new_rot[1],new_rot[2]])
            
            #end
            self.robot.movel([-0.475324000000000,	-0.127279000000000,	0.253031000000000,	-1.08806000000000,	-1.25150000000000,	0.989910000000000])


    def rot_adjust(self,start,end,dr,r):
        f = 1-dr/r
        base_rot = self.axis_angle_solver(start,end)
        new_rot = [f*base_rot[0],f*base_rot[1],f*base_rot[2]]
        return self.axis_angle_fwrd(start,new_rot)

    def axis_angle_solver(self,start,end):
        start = np.array(start)
        self.alpha = np.linalg.norm(start)
        self.L = start/self.alpha

        end = np.array(end)
        self.gamma = np.linalg.norm(end)
        self.N = end/self.gamma

        sol = optimize.least_squares(self.axis_angle_func,np.array([0,0,0,pi/2]),method='lm').x
        applied = [sol[0]*sol[1],sol[0]*sol[2],sol[0]*sol[3]]
        return applied

    def axis_angle_func(self,applied):
        beta = applied[0]
        M = np.array([applied[1],applied[2],applied[3]])
        eq1 = np.cos(self.alpha/2)*np.cos(beta/2)-np.sin(self.alpha/2)*np.sin(beta/2)*np.dot(self.L,M)-np.cos(self.gamma/2)
        eq2 = np.sin(self.alpha/2)*np.cos(beta/2)*self.L +np.cos(self.alpha/2)*np.sin(beta/2)*M + np.sin(self.alpha/2)*np.sin(beta/2)*np.cross(self.L,M) - np.sin(self.gamma/2)*self.N
        return [eq1, eq2[0], eq2[1], eq2[2]]

    def axis_angle_fwrd(self,start,applied):
        start = np.array(start)
        alpha = np.linalg.norm(start)
        L = start/alpha

        applied = np.array(applied)
        beta = np.linalg.norm(applied)
        M = applied/beta
        
        gamma = 2*np.arccos(np.cos(alpha/2)*np.cos(beta/2)-np.sin(alpha/2)*np.sin(beta/2)*np.dot(L,M))
        N = (1/np.sin(gamma/2))*(np.sin(alpha/2)*np.cos(beta/2)*L +np.cos(alpha/2)*np.sin(beta/2)*M + np.sin(alpha/2)*np.sin(beta/2)*np.cross(L,M))
        return [gamma*N[0],gamma*N[1],gamma*N[2]]
