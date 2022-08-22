import numpy as np
import os, math, yaml, time
from math import cos, sin, sqrt
from . import Rectangle, Circle
import matplotlib.pyplot as plt
from collision import is_collision


def plotobj(*objs):
    ''' plot all the given rectangles and circles.
    '''
    fig = plt.figure(1)
    plt.clf()
    ax = fig.gca()
    for obj in objs:
        plt.pause(0.001)
        if type(obj) == Rectangle:
            if not hasattr(obj, 'vertices'):
                vertices = obj.calc_vertices()
                # add the first point to the end.
                vertices = np.append(vertices, [vertices[0,:].tolist()], axis=0)
            plt.plot(vertices[:, 0], vertices[:, 1], 'blue')
        elif type(obj) == Circle:
            c = plt.Circle((obj.x, obj.y), obj.r, color='r', fill=False)
            ax.add_patch(c)
    plt.axis('equal')
    # print('debug')
    # plt.show()


def unitest():
    '''
    test method: 4 angles * 4 center * near crash and crash cases
    '''


    angles = [30, 110, 200, -30]
    angles = [np.deg2rad(angle) for angle in angles]

    center_theta = [55, 150, 210, -45]
    centers = np.ones([4, 2])
    for i in range(centers.shape[0]):
        centers[i, 0] = cos(center_theta[i])
        centers[i, 1] = sin(center_theta[i])


    dis_r2r = np.array([
        [
            [4,     3.8,      4.4,      3.7],
            [0,     0,      0,      0],
            [0,     0,      0,      0],
            [0,     0,      0,      0],
        ],
        [
            [4.1,     3.9,      4.5,      3.8],
            [0,     0,      0,      0],
            [0,     0,      0,      0],
            [0,     0,      0,      0],
        ]
    ])

    dis_r2c = np.array([
        [
            [3.9,    3.6,      4.1,      3.5],
            [4.2,     4.3,      0,      0],
            [0,     0,      0,      0],
            [0,     0,      0,      0],
        ],
        [
            [4,     3.7,      4.2,     3.6],
            [4.3,     4.4,      0,      0],
            [0,     0,      0,      0],
            [0,     0,      0,      0],
        ]
    ])

    # rect and rect detection
    # for i_ego in range(len(angles)):
    #     angle_rect = angles[i_ego]
    #     if i_ego<len(angles)-1:
    #         i_other = i_ego+1 
    #     else:
    #         i_other = 0
    #     angle_other = angles[i_other]    
    #     for k in range(np.size(centers, 0)):
    #         center = centers[k]
    #         r1 = Rectangle(0, 0, angle_rect, 4, 3)

    #         d1 = dis_r2r[0, i_ego,  k]
    #         r2 = Rectangle(center[0]*d1, center[1] * d1, angle_other, 4, 3)
    #         d2 = dis_r2r[1, i_ego,  k]
    #         r3 = Rectangle(center[0] * d2, center[1] * d2, angle_other, 4, 3)
    #         res = [is_collision(r1, r2), is_collision(r1, r3)]
    #         plotobj(r1, r2, r3)
    #         print("res: ", res)
    #         if k>=32 or dis_r2r.reshape(-1)[ k+1] == 0:
    #             print('continue')
            # assert res == [True, False]

    # rect and circle collision detection
    for i_ego in range(len(angles)):
        angle_rect = angles[i_ego]
        for k in range(np.size(centers, 0)):
            center = centers[k]
            r1 = Rectangle(0, 0, angle_rect, 4, 3)
            d1 = dis_r2c[0, i_ego,  k]
            d2 = dis_r2c[1, i_ego, k]
            c1 = Circle(center[0]*d1, center[1]*d1, r=2)
            c2 = Circle(center[0]*d2, center[1]*d2, r=2)
            res = [is_collision(r1, c1), is_collision(r1, c2)]
            plotobj(r1, c1, c2)
            print("res: ", res)
            ik = i_ego*4+k
            if ik>=16 or dis_r2c.reshape(-1)[ ik+1] == 0:
                print('continue')
            # assert res == [True, False]
   
if __name__=='__main__':
    t1x = time.time()

    r3 = Rectangle(0, 0, 2)
    t2x = time.time()
    print('use rect class time' ,t2x-t1x)
    t1 = time.time()
    for i in range(100):
        unitest()
    t2 = time.time()
    print('time', t2-t1)