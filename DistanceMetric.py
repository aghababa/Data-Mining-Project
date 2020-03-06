# -*- coding: utf-8 -*-
"""
Created on Wed Feb 26 03:17:42 2020

@author: Kanchana
"""
import numpy as np
import math

class Line:
    
    def __init__(self,st,en):
        #self.u = u
        self.st = st
        self.en = en
    
    def set_u(self, u):
        self.u = u
        
    def set_st(self, st):
        self.st = st
            
    def set_en(self, en):
        self.en = en
        
    def get_st(self):
        return self.st
    
    def get_en(self):
        return self.en
    
    def get_dist(self,q):
        t = (1.0/np.linalg.norm(self.en-self.st)**2)*np.dot(q-self.st,self.en-self.st) #projection of the point to the line segment
        t = min(max(t,0),1) #check if within the line segment
        pt = self.st+t*(self.en-self.st) #pick the closest point
        dst = np.linalg.norm(q-pt) #calculate distance
        return (dst,pt) #distance and point pair

class Trajectory:
    
    def __init__(self,lines):
        self.lines = lines
        
    def set_lines(self,lines):
        self.lines = lines
        
    def get_lines(self):
        return self.lines
    
class DistanceMetric:
    
    def __init__(self,Q):
        self.Q = Q
    
    def calc_landmarkdst(self,trajectory):
        D = []
        lines = trajectory.get_lines()
        for q in self.Q:
            min_dpt = None
            min_d = float('inf')
            for l in lines:
                (d,pt) = l.get_dist(q)  
                if(min_d>d):
                    min_d = d
                    min_dpt = pt
            D.append((min_d,min_dpt))
        return D
    
    def calc_trajectorydst(self,traj_a,traj_b):
        D_a = self.calc_landmarkdst(traj_a)
        D_b = self.calc_landmarkdst(traj_b)
        n =  len(self.Q)
        dist_Q = 0
        #calculating dQ
        for i in range(n):
            dist_Q = dist_Q + (D_a[i][0]-D_b[i][0])**2
        dist_Q = math.sqrt((1.0/n)*dist_Q)
        dist_Qpi = 0
        #calculating dQpi
        for i in range(n):
            dist_Qpi = dist_Qpi + np.linalg.norm(D_a[i][1]-D_b[i][1])
        dist_Qpi = (1.0/n)*dist_Qpi
        return (dist_Q,dist_Qpi)
    
    