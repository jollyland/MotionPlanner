import structure
import numpy as np
import math
import getData
import copy
#---------collision---------
#bdbox可能沒有更新


def collide(o, robot):
    o_xmax = o.bdbox[0].x
    o_ymax = o.bdbox[0].y
    o_xmin = o.bdbox[2].x
    o_ymin = o.bdbox[2].y

    r_xmax = robot.bdbox[0].x
    r_ymax = robot.bdbox[0].y
    r_xmin = robot.bdbox[2].x
    r_ymin = robot.bdbox[2].y

    flag = 0

    if r_xmax >= o_xmin  and r_xmax <= o_xmax:
        flag+=1
    if r_xmin >= o_xmin  and r_xmin <= o_xmax:
        flag+=1
    if r_ymax >= o_ymin  and r_ymax <= o_xmax:
        flag+=1
    if r_ymin >= o_ymin  and r_ymin <= o_xmax:
        flag+=1
    r_edge = []
    o_edge = []
    if flag>=2:
        r_edge.clear()
        o_edge.clear()

        # return 1

        for poly in robot.poly:
            for i, point in enumerate(poly):
                if i < len(poly)-1:
                    r_edge.append(point)
                    r_edge.append(poly[i + 1])
                else:
                    r_edge.append(point)
                    r_edge.append(poly[0])



        for convex in o.poly:
            for i, point in enumerate(convex):
                if i < len(convex)-1:
                    o_edge.append(point)
                    o_edge.append(convex[i + 1])
                else:
                    o_edge.append(point)
                    o_edge.append(convex[0])

        for p in r_edge:
            p.prnt_point()
        for p in o_edge:
            p.prnt_point()

        for index in range(len(r_edge)):
            if index%2 == 0 :
                for i_o in range(len(o_edge)):
                    if i_o%2  == 0 :
                        if v_collide(r_edge[index],r_edge[index+1],o_edge[i_o],o_edge[i_o+1]):
                            return 1

    else:
        return 0;

def innerproduct(a,b):
    return a[0]*b[0]+a[1]*b[1]

def neg(a):
    return [-a[0],-a[1]]

def v_collide(p0,p1,p2,p3):
    v01 = [p1.x-p0.x, p1.y-p0.y]
    v02 = [p2.x-p0.x, p2.y-p0.y]
    v03 = [p3.x-p0.x, p3.y-p0.y]
    v12 = [p2.x-p1.x, p2.y-p2.y]
    v23 = [p3.x-p2.x, p3.y-p2.y]
    n_v01 = [v01[1], -v01[0]]
    n_v23 = [v23[1], -v23[0]]

    if innerproduct(n_v01, v02) * innerproduct(n_v01, v03) < 0 and innerproduct(n_v23,neg(v02)) * innerproduct(n_v23,neg(v12)) < 0:
        # print("碰到ㄌ")
        p0.prnt_point()
        p1.prnt_point()
        p2.prnt_point()
        p3.prnt_point()
        return 1
    else:
        # print("沒碰到ㄌ")
        return 0

#=====BFS==========

class ListNode():
    def __init__(self,c,u):
        self.configuration = c
        self.U = u
        self.previous = None

def Arbitration(PF1, PF2, cp1, cp2, conf):
    a = structure.pointoncanvas(cp1,conf)
    a = a.canvas_to_planner()
    b = structure.pointoncanvas(cp2, conf)
    b = b.canvas_to_planner()
    u = int(0.7*PF1[a.x][a.y] + 0.3*PF2[b.x][b.y])
    return u

def Empty(Tree):  # if empty return 1
    for i in range(256):
        if len(Tree[i]) > 0:
            return 0
    return 1

def First(Tree):
    min = -1
    for i in range(256):
        if len(Tree[i]) != 0:
            min = i
            break
    # print(min)
    return Tree[min][0]


def delfirst(Tree):
    min = -1
    for i in range(256):
        if len(Tree[i]) != 0:
            min = i
            break
    del Tree[min][0]

    #-----------------------
def BFS(obstaclelist, robot, PtField1, PtField2, all_node):
    Tree = list()
    for i in range(256):
        nodes = list()
        Tree.append(nodes)

    robot_cp0 = robot.cp[0]
    robot_cp1 = robot.cp[1]



    mark = list()  # dx=1 dy=1 dd=3
    for i in range(128):
        y = list()
        for j in range(128):
            d = list()
            for k in range(360):
                d.append(0)
            y.append(d)
        mark.append(y)

    # initial position and U
    U = Arbitration(PtField1, PtField2, robot_cp0, robot_cp1, getData.main.robot.start)

    robot.start.x = int (robot.start.x)
    robot.start.y = int (robot.start.y)

    init_node = ListNode(robot.start, U)

    Tree[int(U)].append(init_node)

    x = int(robot.start.x)
    y = int(robot.start.y)
    d = int(robot.start.angle)
    mark[x][y][d] = 1
    success = 0
    time = 0
    # start loop
    while (Empty(Tree) == 0 and success == 0):
        # print("loop" + str(time))
        time += 1
        c_node = First(Tree)
        delfirst(Tree)
        c_con = c_node.configuration
        dx = [+1, -1, 0, 0, 0, 0]
        dy = [0, 0, +1, -1, 0, 0]
        dd = [0, 0, 0, 0, +10, -10]

        for i in range(6):
            robot2 = copy.deepcopy(robot)
            n_x = int(c_con.x) + dx[i]
            n_y = int(c_con.y) + dy[i]
            n_d = int(c_con.angle) + dd[i]
            if (n_d >= 360):
                n_d -= 360
            if (n_d < 0):
                n_d += 360
            if (n_x >= 128):
                n_x -= 128
            if (n_x < 0):
                n_x += 128
            if (n_y >= 128):
                n_y -= 128
            if (n_y < 0):
                n_y += 128
            # print("next is", n_x, n_y, n_d)
            if (mark[n_x][n_y][n_d] == 0):
                # print("沒來過ㄉ")
                n_con = structure.Configuration(n_x, n_y, n_d)
                n_c0 = structure.pointoncanvas(robot_cp0,n_con).canvas_to_planner()
                n_c1 = structure.pointoncanvas(robot_cp1,n_con).canvas_to_planner()
                # n_c0.prnt_point()
                # n_c1.prnt_point()
                if (n_c0.x >= 127 or n_c0.x < 0 or n_c0.y >= 127 or n_c0.y < 0 or n_c1.x >= 127 or n_c1.x < 0 or n_c1.y >= 127 or n_c1.y < 0):
                    print("撞牆")
                    continue
                else:

                    robot2.start = n_con
                    for o in obstaclelist:
                        if (collide(o, robot2)):

                            continue
                        else:
                            U = Arbitration(PtField1, PtField2, robot_cp0, robot_cp1, n_con)
                            # print("U is",U)
                            if (U == 0):
                                goal_node = ListNode(n_con, U)
                                goal_node.previous = c_node
                                all_node.append(goal_node)
                                success += 1
                            else:
                                next_node = ListNode(n_con, U)
                                next_node.previous = c_node
                                # print("u is",int(U))
                                Tree[int(U)].insert(0, next_node)
                        mark[n_x][n_y][n_d] = 1
            else:
                pass
                # print("it has been marked",n_x,n_y,n_d)
    if success:
        while (1):
            # print(all_node[0].configuration)
            if (all_node[0].previous == None):
                break
            all_node.insert(0, all_node[0].previous)
        return 1
    else:
        return 0