''' Praticle Collision detection for autonomous driving.
Rectangle and circle.
'''
import numpy as np
import os, yaml, time
from math import cos, sin

path_cfg = os.path.join("configs", "control.yaml")
f_cfg = open(path_cfg)
car_cfg = yaml.load(f_cfg)


class Rectangle:
    def __init__(self, x, y, yaw, length=4, width=3, pos='center'):
        '''
        Params:
            pos: whether the point (x,y) represents the 'center' or 'rear_axle_center'
        '''
        self.x = x
        self.y = y
        self.yaw = yaw
        self.length = length
        self.width = width


    def calc_vertices(self):
        '''
        '''
        # vertices relative displacement to center
        v_d = np.array([[self.length, self.width],\
                    [-self.length, self.width],\
                    [-self.length, -self.width],\
                    [self.length, -self.width]])
        v_d = v_d/2
        
        yaw = self.yaw
        rotate_matrix = np.array([[cos(yaw), -sin(yaw)],\
            [sin(yaw), cos(yaw)]])
        
        v_d = v_d @ rotate_matrix.T

        # add the displacement
        v = v_d + np.array([self.x, self.y])
        self.vertice = v
        return v

class Circle:
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r

