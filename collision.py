''' Praticle and Fast Collision detection for autonomous driving.
objects: Rectangle and circle.
According to the implemetation difficulty. 3 method are used.
methods:  seperating axis therom; double circle method; particle model;

'''
import numpy as np
import os, math, yaml, time
from math import cos, sin, sqrt
from geometry import Rectangle, Circle
import matplotlib.pyplot as plt
from vehicleparams import VehicleParams

# path_cfg = os.path.join("configs", "control.yaml")
# f_cfg = open(path_cfg)
# car_cfg = yaml.load(f_cfg)


def collision_rect_and_rect(rect1: Rectangle, rect2: Rectangle):
    ''' Use seperating axis therom (Reference to apollo). \
        Judge collision between 2 rectangles.

    '''
    shift_x = rect2.x - rect1.x
    shift_y = rect2.y - rect1.y

 
    cos_v = math.cos(rect1.yaw)
    sin_v = math.sin(rect1.yaw)
    cos_o = math.cos(rect2.yaw)
    sin_o = math.sin(rect2.yaw)
    half_l_v = rect1.length/2
    half_w_v = rect1.width/2
    half_l_o = rect2.length/2
    half_w_o = rect2.width/2

    dx1 = cos_v * rect1.length/2
    dy1 = sin_v * rect1.length/2
    dx2 = sin_v * rect1.width/2
    dy2 = -cos_v * rect1.width/2

    dx3 = cos_o * rect2.length/2
    dy3 = sin_o * rect2.length/2
    dx4 = sin_o * rect2.width/2
    dy4 = -cos_o * rect2.width/2

    # use seperating axis therom
    return ((abs(shift_x * cos_v + shift_y * sin_v) <=
             abs(dx3 * cos_v + dy3 * sin_v) + abs(dx4 * cos_v + dy4 * sin_v) + half_l_v)
            and (abs(shift_x * sin_v - shift_y * cos_v) <=
                 abs(dx3 * sin_v - dy3 * cos_v) + abs(dx4 * sin_v - dy4 * cos_v) + half_w_v)
            and (abs(shift_x * cos_o + shift_y * sin_o) <=
                 abs(dx1 * cos_o + dy1 * sin_o) + abs(dx2 * cos_o + dy2 * sin_o) + half_l_o)
            and (abs(shift_x * sin_o - shift_y * cos_o) <=
                 abs(dx1 * sin_o - dy1 * cos_o) + abs(dx2 * sin_o - dy2 * cos_o) + half_w_o))


def collision_circle_and_rect(c:Circle, r:Rectangle):
    ''' dectect 2 + 1 axis. https://www.sevenson.com.au/programming/sat/
    '''
    # find the nearest vertices to circle
    if not hasattr(r, 'vertices'):
        vertices = r.calc_vertices()
    d_min = np.Inf
    ind = -1

    for i_v in range(vertices.shape[0]):
        d = ((vertices[i_v, 0] - c.x)**2 + (vertices[i_v, 1] - c.y)**2)**0.5 - c.r
        if d < d_min:
            d_min = d
            ind = i_v
    
    if d_min<0:
        return True
    
    circle_center = np.array([c.x, c.y])
    
    yaw = r.yaw
    axes = [[cos(yaw), sin(yaw)], [-sin(yaw), cos(yaw)]]

    axis_point = vertices[ind, :] - circle_center
    axis_point = normalize(axis_point)

    axes.append(axis_point)
    for axis in axes:
        projection_a = project(vertices, axis)
        projection_c = project([circle_center], axis)[0]
        projection_b = [projection_c-c.r, projection_c+c.r]

        overlapping = overlap(projection_a, projection_b)

        if not overlapping:
            return False

    return True
    

def normalize(vector):
    """
    :return: The vector scaled to a length of 1
    """
    norm = sqrt(vector[0] ** 2 + vector[1] ** 2)
    return vector[0] / norm, vector[1] / norm


def dot(vector1, vector2):
    """
    :return: The dot (or scalar) product of the two vectors
    """
    return vector1[0] * vector2[0] + vector1[1] * vector2[1]


def edge_direction(point0, point1):
    """
    :return: A vector going from point0 to point1
    """
    return point1[0] - point0[0], point1[1] - point0[1]


def orthogonal(vector):
    """
    :return: A new vector which is orthogonal to the given vector
    """
    return vector[1], -vector[0]


def vertices_to_edges(vertices):
    """
    :return: A list of the edges of the vertices as vectors
    """
    return [edge_direction(vertices[i], vertices[(i + 1) % len(vertices)])
            for i in range(len(vertices))]


def project(vertices, axis):
    """
    :return: A vector showing how much of the vertices lies along the axis
    """
    dots = [dot(vertex, axis) for vertex in vertices]
    return [min(dots), max(dots)]



def overlap(projection1, projection2):
    """
    :return: Boolean indicating if the two projections overlap
    """
    return min(projection1) <= max(projection2) and \
        min(projection2) <= max(projection1)


def separating_axis_theorem(vertices_a, vertices_b):
    ''' slow but unified function for convex polygon collision dectection.
    '''
    edges = vertices_to_edges(vertices_a) + vertices_to_edges(vertices_b)
    axes = [normalize(orthogonal(edge)) for edge in edges]

    for axis in axes:
        projection_a = project(vertices_a, axis)
        projection_b = project(vertices_b, axis)

        overlapping = overlap(projection_a, projection_b)

        if not overlapping:
            return False

    return True


def get_dist_circle_collision(c1: Circle, c2: Circle):
    # d_c1_to_c2 - (r1 + r2)
    return ((c1.x - c2.x)**2 + (c1.y - c2.y)**2)**0.5 - (c1.r + c2.r)

def calc_dis(x1, y1, x2, y2):
    return sqrt((x1-x2)**2 + (y1-y2)**2)

def is_collision(o1, o2):
    ''' use seperating axis theorem
    '''
    if type(o1) == Circle and type(o2) == Circle:
        return get_dist_circle_collision(o1, o2)<0
    elif type(o1) == Rectangle  and type(o2) == Rectangle:
        return collision_rect_and_rect(o1, o2)
        # if not hasattr(o1, 'vertices'):
        #     v1 = o1.calc_vertices()
        #     v2 = o2.calc_vertices()
        # return separating_axis_theorem(v1, v2)
    else:
        (c, r) = (o1, o2) if type(o1)==Circle else (o2, o1)
        return collision_circle_and_rect(c, r)


def is_collision_simple(obj1, obj2):
    ''' detect distance between 2 objects. 
    Use double circles to represent a rectangle.
    [abandoned]: Slower than the not simple version.
    '''
    if type(obj1) == Circle and type(obj2) == Circle:
        return get_dist_circle_collision(obj1, obj2)<0
    elif type(obj1) == Rectangle  and type(obj2) == Rectangle:
        xf1, yf1, xr1, yr1 = get_disc_positions(obj1.x, obj1.y, obj1.yaw)
        xf2, yf2, xr2, yr2 = get_disc_positions(obj2.x, obj2.y, obj2.yaw)
        r = VehicleParams.radius
        i1 =calc_dis(xf1, yf1, xf2, yf2) -2*r < 0
        i2 =calc_dis(xf1, yf1, xr2, yr2) -2*r < 0
        i3 =calc_dis(xr1, yr1, xf2, yf2) -2*r < 0
        i4 =calc_dis(xr1, yr1, xr2, yr2) -2*r < 0
        return i1 | i2 | i3 | i4

    else:
        (c, r) = (obj1, obj2) if type(obj1)==Circle else (obj2, obj1)

        return collision_circle_and_rect(c, r)
    pass


def get_disc_positions(x, y, theta):
    if type(x) is np.ndarray:
        cos, sin = np.cos, np.sin
    else:
        cos, sin = math.cos, math.sin

    
    Lw, Lf, Lr = car_cfg['Lw'], car_cfg['Lf'], car_cfg['Lr']
    f2x = 1/4 * (3*Lw + 3*Lf - Lr)
    r2x = 1/4 * (Lw + Lf - 3*Lr)

    xf = x + f2x * cos(theta)
    xr = x + r2x * cos(theta)
    yf = y + f2x * sin(theta)
    yr = y + r2x * sin(theta)
    return xf, yf, xr, yr


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
            [4,     3.8,      0,      0],
            [0,     0,      0,      0],
            [0,     0,      0,      0],
            [0,     0,      0,      0],
        ],
        [
            [4.1,     3.9,      0,      0],
            [0,     0,      0,      0],
            [0,     0,      0,      0],
            [0,     0,      0,      0],
        ]
    ])
    rs = np.array([
        [1.3, 1.3, 1.3],
        [2.2,2.2,2.2],
        [2.18,2.18,2.18]
    ])

    # rect and rect detection
    # 平移检测不出什么东西
    for i_ego in range(len(angles)):
        angle_ego = angles[i_ego]
        if i_ego<len(angles)-1:
            i_other = i_ego+1 
        else:
            i_other = 0
        angle_other = angles[i_other]    
        for k in range(np.size(centers, 0)):
            center = centers[k]
            r1 = Rectangle(0, 0, angle_ego, 4, 3)
            if dis_r2r[0, i_ego,  k+1] == 0:
                print('continue')
            d1 = dis_r2r[0, i_ego,  k]
            r2 = Rectangle(center[0]*d1, center[1] * d1, angle_other, 4, 3)
            d2 = dis_r2r[1, i_ego,  k]
            r3 = Rectangle(center[0] * d2, center[1] * d2, angle_other, 4, 3)
            res = [is_collision(r1, r2), is_collision(r1, r3)]
            plotobj(r1, r2, r3)
            print("res: ", res)
            # assert res == [True, False]

    # rect and circle collision detection
    # for i_ego in range(3):
    #     angle_ego = angles[i_ego]
    #     for j in range(3):
    #         center = centers[j]
    #         d = dis_r2r[i_ego, j]
    #         r = rs[i_ego,j]
    #         r1 = Rectangle(center[0], center[1], angle_ego)
    #         r2 = Rectangle(center[0]+d, center[1], angle_ego*2)
    #         r3 = Rectangle(center[0]+d+0.1, center[1], angle_ego*2)
    #         # c1 = Circle(center[0]+d, center[1], r+0.1)
    #         # c2 = Circle(center[0]+d, center[1], r)
    #         # res = [is_collision(r1, r2), is_collision(r1, r3), is_collision(r1, c1), is_collision(r1, c2)]
    #         # res = [is_collision_simple(r1, r2), is_collision_simple(r1, r3)]
    #         res = [is_collision(r1, r2), is_collision(r1, r3)]
    #         print("res: ", res)
    #         # assert res == [True, False, True, False]
    #         # assert res == [True, False]
    #         # plotobj(r1, r2, r3, c1, c2)
    #         plotobj(r1, r2, r3)


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