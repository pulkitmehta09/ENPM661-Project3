from queue import PriorityQueue
import cv2
from collections import deque
import numpy as np
from operator import itemgetter
import datetime
from cv2 import VideoWriter, VideoWriter_fourcc

def vectorize(pt1, pt2):
    return np.array([pt2[0]-pt1[0], pt2[1]-pt1[1]])
def normalize(vector):
    return vector/np.sqrt(vector[0]*vector[0] + vector[1]*vector[1])
def rotate90(vector):
    return np.array([[0, -1], [1, 0]]).dot(vector)
def pointfind(pt1, pt2, clearance):
    pt1_vec = np.array([pt1[0], pt1[1]])
    pt2_vec = np.array([pt2[0], pt2[1]])
    vec = vectorize(pt1, pt2)
    n_vec = normalize(vec)
    o_vec = rotate90(n_vec)
    new_pt1 = (o_vec*clearance + pt1_vec).astype(np.uint8)
    new_pt2 = (o_vec*clearance + pt2_vec).astype(np.uint8)
    return new_pt1, new_pt2
def line_solver(lpt1, lpt2, ept):
    '''
    lpt - line point
    ept - evaluation point
    '''
    x0, _ = ept
    x1, y1 = lpt1
    x2, y2 = lpt2
    return int(((((y2-y1)/(x2-x1))*(x0-x1))+y1))


def create_map(clearance=5):
    map = np.ones((250, 400))
    # clearance
    c = clearance

    # map generator
    for i in range(400):
        for j in range(250):
            # coordinate system conversion
            x = -j 
            y = i
            # arrow
            # line 1
            x1, y1 = 36, 185
            x2, y2 = 115, 210
            x1, y1 = -y1, x1
            x2, y2 = -y2, x2
            fy1 = int(((((y2-y1)/(x2-x1))*(x-x1))+y1))
            fy1c = int(((((y2-y1)/(x2-x1))*(x-x1))+y1-c))
            # if fy1 > 0 and fy1 < 250:
            #     map[x, fy1] = 0
            # if fy1a > 0 and fy1a < 250:
            #     map[x, fy1a] = 0
            # line 2
            x1, y1 = 115, 210
            x2, y2 = 80, 180
            x1, y1 = -y1, x1
            x2, y2 = -y2, x2
            fy2 = int(((((y2-y1)/(x2-x1))*(x-x1))+y1))
            fy2c = int(((((y2-y1)/(x2-x1))*(x-x1))+y1+c))
            # if fy2 > 0 and fy2 < 250:
            #     map[x, fy2] = 0
            # if fy2a > 0 and fy2a < 250:
            #     map[x, fy2a] = 0
            # line 3
            x1, y1 = 80, 180
            x2, y2 = 105, 100
            x1, y1 = -y1, x1
            x2, y2 = -y2, x2
            fy3 = int(((((y2-y1)/(x2-x1))*(x-x1))+y1))
            fy3c = int(((((y2-y1)/(x2-x1))*(x-x1))+y1+c))
            # if fy3 > 0 and fy3 < 250:
            #     map[x, fy3] = 0
            # if fy3a > 0 and fy3a < 250:
            #     map[x, fy3a] = 0
            # line 4
            x1, y1 = 105, 100
            x2, y2 = 36, 185
            x1, y1 = -y1, x1
            x2, y2 = -y2, x2
            fy4 = int(((((y2-y1)/(x2-x1))*(x-x1))+y1))
            fy4c = int(((((y2-y1)/(x2-x1))*(x-x1))+y1-c))
            # if fy4 > 0 and fy4 < 250:
            #     map[x, fy4] = 0
            # if fy4a > 0 and fy4a < 250:
            #     map[x, fy4a] = 0
            # obstacle cut
            if y > fy1:
                if y < fy2:
                    if y > fy4:
                        map[x, y] = 0
            if y >= fy2:
                if y < fy3:
                    if y > fy4:
                        map[x, y] = 0
            # # obstacle cut with clearance
            # if y > fy1c:
            #     if y < fy2c:
            #         if y > fy4c:
            #             map[x, y] = 0
            # if y >= fy2c:
            #     if y < fy3c:
            #         if y > fy4c:
            #             map[x, y] = 0
            
            # hexagon
            # line 5
            x1, y1 = 200, 60
            x2, y2 = 165, 80
            x1, y1 = -y1, x1
            x2, y2 = -y2, x2
            fy5 = int(((((y2-y1)/(x2-x1))*(x-x1))+y1))
            fy5c = int(((((y2-y1)/(x2-x1))*(x-x1))+y1-c))
            # if fy5 > 0 and fy5 < 250:
            #     map[x, fy5] = 0
            # if fy5a > 0 and fy5a < 250:
            #     map[x, fy5a] = 0
            # line 6
            x1, y1 = 165, 80
            x2, y2 = 165, 120
            x1, y1 = -y1, x1
            x2, y2 = -y2, x2
            fy6 = int(((((y2-y1)/(x2-x1))*(x-x1))+y1))
            fy6c = int(((((y2-y1)/(x2-x1))*(x-x1))+y1-c))
            # if fy6 > 0 and fy6 < 250:
            #     map[x, fy6] = 0
            # if fy6a > 0 and fy6a < 250:
            #     map[x, fy6a] = 0
            # line 7
            x1, y1 = 165, 120
            x2, y2 = 200, 140
            x1, y1 = -y1, x1
            x2, y2 = -y2, x2
            fy7 = int(((((y2-y1)/(x2-x1))*(x-x1))+y1))
            fy7c = int(((((y2-y1)/(x2-x1))*(x-x1))+y1-c))
            # if fy7 > 0 and fy7 < 250:
            #     map[x, fy7] = 0
            # if fy7a > 0 and fy7a < 250:
            #     map[x, fy7a] = 0
            # line 8
            x1, y1 = 200, 140
            x2, y2 = 235, 120
            x1, y1 = -y1, x1
            x2, y2 = -y2, x2
            fy8 = int(((((y2-y1)/(x2-x1))*(x-x1))+y1))
            fy8c = int(((((y2-y1)/(x2-x1))*(x-x1))+y1+c))
            # if fy8 > 0 and fy8 < 250:
            #     map[x, fy8] = 0
            # if fy8a > 0 and fy8a < 250:
            #     map[x, fy8a] = 0
            # line 9
            x1, y1 = 235, 120
            x2, y2 = 235, 80
            x1, y1 = -y1, x1
            x2, y2 = -y2, x2
            fy9 = int(((((y2-y1)/(x2-x1))*(x-x1))+y1))
            fy9c = int(((((y2-y1)/(x2-x1))*(x-x1))+y1+c))
            # if fy9 > 0 and fy9 < 250:
            #     map[x, fy9] = 0
            # if fy9a > 0 and fy9a < 250:
            #     map[x, fy9a] = 0
            # line 10
            x1, y1 = 235, 80
            x2, y2 = 200, 60
            x1, y1 = -y1, x1
            x2, y2 = -y2, x2
            fy10 = int(((((y2-y1)/(x2-x1))*(x-x1))+y1))
            fy10c = int(((((y2-y1)/(x2-x1))*(x-x1))+y1+c))
            # if fy10 > 0 and fy10 < 250:
            #     map[x, fy10] = 0
            # if fy10a > 0 and fy10a < 250:
            #     map[x, fy10a] = 0
            # obstacle cut
            if y > fy5:
                if y > fy6:
                    if y > fy7:
                        if y < fy8:
                            if y < fy9:
                                if y < fy10:
                                    map[x, y] = 0
            # # obstacle cut with clearance
            # if y > fy5c:
            #     if y > fy6c:
            #         if y > fy7c:
            #             if y < fy8c:
            #                 if y < fy9c:
            #                     if y < fy10c:
            #                         map[x, y] = 0
            
            # circle
            x1, y1 = 300, 185
            cx, cy = -y1, x1
            radius = 40
            distance = np.sqrt((x-cx)*(x-cx) + (y-cy)*(y-cy))
            # obstacle cut
            if distance < radius:
                map[x, y] = 0
            # # obstacle cut with clearance
            # if distance < radius + c:
            #     map[x, y] = 0                
    

            # borders
            if  (y < c or y > 399 - c):
            # if  (j < c):
                map[x, y] = 0
            if  (x > - c or x < - 249 + c):
                map[x, y] = 0

    return map

def make_map(clearance=5):
    # x, y = cols, rows
    rows, cols = 250, 400
    # Plotting Notes
    # --------------
    # x = 0 to 399
    # y = 249 to 0
    # y' = 249 - y
    # --------------
    map_array = np.ones((rows, cols))
    # constrains
    x1, y1 = 36, 185
    x2, y2 = 115, 210
    x3, y3 = 80, 180
    x4, y4 = 105, 100
    np1, np2 = pointfind((x1, y1), (x2, y2), clearance)
    # nx1, ny1, nx2, ny2 = np1, np2
    np3, np4 = pointfind((x2, y2), (x3, y3), clearance)
    # nx3, ny3, nx4, ny4 = np3, np4
    np5, np6 = pointfind((x3, y3), (x4, y4), clearance)
    # nx5, ny5, nx6, ny6 = np5, np6
    np7, np8 = pointfind((x4, y4), (x1, y1), clearance)
    # nx7, ny7, nx8, ny8 = np7, np8
    
    # Arrow
    for i in range(cols):
        for j in range(rows):
            fy1 = line_solver((np1[0], 249-np1[1]), (np2[0], 249-np1[1]), (i, 249-j))
            fy2 = line_solver((np2), (np3), (i, j))
            fy3 = line_solver((np3), (np4), (i, j))
            fy4 = line_solver((np4), (np1), (i, j))
            # if 249-j > 249-fy1:
            #     if 249-j < 249-fy2:
            #         if 249-j > 249-fy4:
            #             map_array[i, j] = 0
            # if 249-j >= 249-fy2:
            #     if 249-j < 249-fy3:
            #         if 249-j > 249-fy4:
            #             map_array[i, j] = 0
            if j == fy1:
               map_array[i, j] = 0


    if True:
        x1, y1 = 36, 249-185
        cv2.circle(map_array, (x1, y1), clearance, (0, 0, 0), -1)
        x1, y1 = 115, 249-210
        cv2.circle(map_array, (x1, y1), clearance, (0, 0, 0), -1)
        x1, y1 = 80, 249-180
        cv2.circle(map_array, (x1, y1), clearance, (0, 0, 0), -1)
        x1, y1 = 105, 249-100
        cv2.circle(map_array, (x1, y1), clearance, (0, 0, 0), -1)

        # Hexagon
        x1, y1 = 200, 249-60
        cv2.circle(map_array, (x1, y1), clearance, (0, 0, 0), -1)
        x1, y1 = 165, 249-80
        cv2.circle(map_array, (x1, y1), clearance, (0, 0, 0), -1)
        x1, y1 = 165, 249-120
        cv2.circle(map_array, (x1, y1), clearance, (0, 0, 0), -1)
        x1, y1 = 200, 249-140
        cv2.circle(map_array, (x1, y1), clearance, (0, 0, 0), -1)
        x1, y1 = 235, 249-120
        cv2.circle(map_array, (x1, y1), clearance, (0, 0, 0), -1)
        x1, y1 = 235, 249-80
        cv2.circle(map_array, (x1, y1), clearance, (0, 0, 0), -1)

        # circle
        x1, y1 = 300, 249-185
        radius = 40
        cv2.circle(map_array, (x1, y1), radius + clearance, (0, 0, 0), -1)

        # p1, p2 = pointfind((36, 185), (115, 210), clearance)
        # print(p1, p2)
        # x1, y1 = p1
        # x2, y2 = p2
        # print(x1, x2)
        # cv2.circle(map_array, (x1, 249-y1), 5, (0, 0, 0), -1)
        # cv2.circle(map_array, (x2, 249-y2), 5, (0, 0, 0), -1)

    return map_array

if __name__ == "__main__":
    print("mmap")