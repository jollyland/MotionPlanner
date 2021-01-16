import tkinter as tk
import math
import numpy as np
import structure
import getData
import bitmap
import BFS
from tkinter import *

#----------GUI-------------
class GUI():
    def __init__(self):
        self.obstacle = list()
        self.robot = structure.Robot()
        self.window = Tk()
        self.window.geometry("800x600")
        self.window.title("MOTION PlANNING")

        self.canvas = Canvas(self.window, width=600, height=600, bg="white")
        self.canvas.pack(side="left")
        self.canvas.tag_bind('o_bdbox', '<B1-Motion>', self.o_move)
        self.canvas.tag_bind('r_bdbox', '<B1-Motion>', self.r_move)

        self.lockGUI_bt = Button(self.window, text='Lock GUI',font=('Arial',8), bg="#DDD", width=20, height=2, command=self.lock)
        self.lockGUI_bt.place(x=650)
        self.lockGUI_bt.pack()
        self.bm_bt = Button(self.window, text='build bitmap/PF', font=('Arial', 8), bg="#DDD", width=20, height=2, command=self.buildmap)
        self.bm_bt.place(x = 650)
        self.bm_bt.pack()
        self.c_bt = Button(self.window, text='c check', font=('Arial', 8), bg="#DDD", width=20, height=2, command=self.collidecheck)
        self.c_bt.place(x=650)
        self.c_bt.pack()
        self.BFS_bt = Button(self.window, text='BFS', font=('Arial', 8), bg="#DDD", width=20, height=2, command=self.findpath)
        self.BFS_bt.place(x=650)
        self.BFS_bt.pack()


        # self.debug_bt = Button(self.window, text='show info', font=('Arial', 8), bg="#DDD", width=20, height=2,command=self.debugmessage)

        # self.debug_bt.pack(side="right")



        self.target = -1
        self.target_bd = -1


    def debugmessage(self):
        print("robot at")
        for convex in self.robot.poly:
            for point in convex:
                point.prnt_point()
        print("cp stay in")
        self.robot.cp[0].prnt_point()
        self.robot.cp[1].prnt_point()
        print("configuration now at:")
        print(str(self.robot.start.x)+", "+str(self.robot.start.y)+", "+str(self.robot.start.angle))


    def collidecheck(self):
        for o in self.obstacle:
            if BFS.collide(o, self.robot):
                print("collide!")
            else:
                print("nope!")

    def buildmap(self):
        bitmap.build_bmap()

    def unbind(self,e):
        print("alrdy lock")

    def lock(self):
        self.canvas.tag_bind('o_bdbox', '<B1-Motion>', self.unbind)
        self.canvas.tag_bind('r_bdbox', '<B1-Motion>', self.unbind)
        R = self.robot
        O_L = self.obstacle
        self.canvas.delete("all")
        self.drawCV()

        for convex in R.poly:
            # print("R point:")
            for index,point in enumerate(convex):
                point = structure.pointoncanvas(point, R.start)
                convex[index] = point
                # convex[index].prnt_point()
        for o in O_L:
            for convex in o.poly:
                # print("O point:")
                for index, point in enumerate(convex):
                    point = structure.pointoncanvas(point, o.configuration)
                    convex[index] = point
                    # convex[index].prnt_point()



        for poly in self.robot.poly:
            for i, p in enumerate(poly):
                poly[i] = p.canvas_to_planner()


    def drawCV(self):
        self.draw_obstacle()
        self.draw_robot()
        self.draw_goal()

    def draw_robot(self):
        temp = []
        for convex in self.robot.poly:
            for point in convex:
                t = structure.pointoncanvas(point, self.robot.start)
                temp.append(t.x)
                temp.append(t.y)
            self.canvas.create_polygon(temp, outline='black', fill="blue", width=1, tag='robot')
            temp.clear()
        bd = []
        for p in self.robot.bdbox:
            t = p.toList()
            bd.append(t[0])
            bd.append(t[1])

        self.canvas.create_polygon(bd, fill="", outline='red', width=1, tag='r_bdbox')
        bd.clear()

    def draw_goal(self):
        temp = []
        for convex in self.robot.poly:
            for point in convex:
                t = structure.pointoncanvas(point, self.robot.goal)
                temp.append(t.x)
                temp.append(t.y)
            self.canvas.create_polygon(temp, outline='black', fill="purple", width=1, tag='robot')
            temp.clear()
        bd = []

    def draw_obstacle(self):
        temp = []
        tmp = []
        for o in self.obstacle:
            o.GUIindex.clear()
            for convex in o.poly:
                for point in convex:
                    tmp_point = structure.pointoncanvas(point, o.configuration)
                    temp.append(tmp_point.x)
                    temp.append(tmp_point.y)
                o.GUIindex.append(self.canvas.create_polygon(temp, outline="black", fill="yellow", width=1, tags='obstacle'))
                temp.clear()

            for b in o.bdbox:
                tmp.append(b.x)
                tmp.append(b.y)
            self.canvas.create_polygon(tmp, fill="", outline='red', width=1, tag='o_bdbox')
            tmp.clear()


    def r_move(self, e):

        self.robot.start.x =structure.xconvert_planner(e.x)
        self.robot.start.y =structure.yconvert_planner(e.y)
        self.canvas.delete("robot")
        structure.r_update(self.robot)
        self.drawCV()


    def o_move(self, e):
        target = []
        for index, o in enumerate(self.obstacle):
            xmax = o.bdbox[0].x
            xmin = o.bdbox[2].x
            ymax = o.bdbox[0].y
            ymin = o.bdbox[1].y
            if e.x < xmax and e.x > xmin and e.y > ymin and e.y < ymax:
                target = o.GUIindex
                break
            else:
                target = -1
        #print(index, target)
        # print(target)
        if isinstance(target, int):
            self.canvas.delete(target)
        else :
            for t in target:
                #print("multi"+str(t))
                self.canvas.delete(t)
            target.clear()


        self.obstacle[index].configuration.x = structure.xconvert_planner(e.x)
        self.obstacle[index].configuration.y = structure.yconvert_planner(e.y)
        structure.bd_update(self.obstacle[index])

        self.drawCV()



    def findpath(self):
        node_list = []
        if( BFS.BFS(self.obstacle, self.robot, bitmap.PF1, bitmap.PF2, node_list)):
            print("success!")
            for node in node_list:
                # print("u="+str(node.U))
                conf = node.configuration
                temp = []
                for convex in getData.origin.poly:
                    for point in convex:
                        t = structure.pointoncanvas(point, conf)
                        temp.append(t.x)
                        temp.append(t.y)
                        self.canvas.create_polygon(temp, outline='black', fill="gray", width=1, tag='robot')
                    temp.clear()
        else:
            print("fail")
            if not node_list:
                print("you little shit")
            for node in node_list:
                print(str(node.u))