from structure import *
import getData

mark = [[]]

def canvas_to_planner(point):
    point.x = (int)point.x/600*128
    point.y = (int)point.y/600*128
    return point

ob_list = getData.main.obstacle
ob_vertice = []
for obstacle in ob_list:
    for convex in obstacle:
        for point in convex:
            tmp = canvas_to_planner(point).toList
            ob_vertice.append(tmp)
        print(ob_vertice)

    for
     ob_vertice.clear()


PF = [[]]

for i in range(128):
    for j in range(128):
        PF[i][j] = 254