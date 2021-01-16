import tkinter as tk
import math
import numpy as np
import time
from tkinter import *


class Point():
    def __init__(self, x_init, y_init):
        self.x = x_init
        self.y = y_init

    def prnt_point(self):
        print("(", self.x, ", ", self.y, ")")

    def toList(self):
        l = [self.x, self.y]
        return l

    def canvas_to_planner(self):
        self.x = int (self.x / 600 * 128)
        self.y = int (self.y / 600 * 128)
        return self



class Configuration():
    def __init__(self, x_init, y_init, angle_init):
        self.x = x_init
        self.y = y_init
        self.angle = angle_init
        self.bdbox = []

    def prnt_con(self):
        print("(", self.x, ", ", self.y, ", ", self.angle, ")")


class Robot():
    def __init__(self):
        self.polynum = 0
        self.poly = []
        self.start = Configuration(0, 0, 0)
        self.goal = Configuration(0, 0, 0)
        self.cp = []
        self.bdbox = []




class Obstacle():
    def __init__(self):
        self.poly = []
        self.configuration = Configuration(0, 0, 0)
        self.bdbox = []
        self.bdbox.append(Point(0,0))
        self.bdbox.append(Point(0,0))
        self.bdbox.append(Point(0,0))
        self.bdbox.append(Point(0,0))
        self.GUIindex = []


def xconvert_canvas(x):
    t = (x)/128*600
    return t

def yconvert_canvas(y):
    t =600 - (y)/128*600
    return t

def xconvert_planner(x):
    t = (x)/600*128
    return t

def yconvert_planner(y):
    t =(600-y)/600*128
    return t

def pointoncanvas(point,conf):
    new = Point(0,0)
    tmp = np.array(point.toList())
    a = conf.angle
    theta = math.radians(a)
    cos = math.cos(theta)
    sin = math.sin(theta)
    rotate = np.array([[cos, sin], [-sin, cos]])
    tmp = tmp.dot(rotate)
    tmp[0] = tmp[0] + conf.x
    tmp[1] = tmp[1] + conf.y
    new.x = xconvert_canvas(tmp[0])
    new.y = yconvert_canvas(tmp[1])
    return new

def bd_update(obstacle):
    xlist = []
    ylist = []
    for poly_index, convex in enumerate(obstacle.poly):
        for point in convex:
            point_cv = pointoncanvas(point, obstacle.configuration)
            xlist.append(point_cv.x)
            ylist.append(point_cv.y)
    xmax = max(xlist)
    ymax = max(ylist)
    xmin = min(xlist)
    ymin = min(ylist)
    obstacle.bdbox[0] = Point(xmax, ymax)
    obstacle.bdbox[1] = Point(xmax, ymin)
    obstacle.bdbox[2] = Point(xmin, ymin)
    obstacle.bdbox[3] = Point(xmin, ymax)
    xlist.clear()
    ylist.clear()

def r_update(robot):
    xlist = []
    ylist = []
    robot.bdbox.clear()
    for index, convex in enumerate(robot.poly):
        for p in convex:
            point_cv = pointoncanvas(p, robot.start)
            xlist.append(point_cv.x)
            ylist.append(point_cv.y)
    xmax = max(xlist)
    ymax = max(ylist)
    xmin = min(xlist)
    ymin = min(ylist)
    robot.bdbox.append(Point(xmax, ymax))
    robot.bdbox.append(Point(xmax, ymin))
    robot.bdbox.append(Point(xmin, ymin))
    robot.bdbox.append(Point(xmin, ymax))
    xlist.clear()
    ylist.clear()