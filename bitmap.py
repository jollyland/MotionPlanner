from structure import *
import getData
import structure

PF1 = [[254 for i in range(128)] for j in range(128)]
PF2 = [[254 for i in range(128)] for j in range(128)]

def build_bmap():
    mark = [[0 for i in range(128)]for j in range(128)]
    # for i in range(128):
    #     for j in range(128):
    #         print(mark[i][j],end="")
    #     print()
    ob_list = getData.main.obstacle
    ob_vertice = []
    edge = []
    for obstacle in ob_list:
        for convex in obstacle.poly:
            for point in convex:
                tmp = (point.canvas_to_planner()).toList()
                ob_vertice.append(tmp)
            ob_vertice.append(ob_vertice[0])
            #print(ob_vertice)
            for index, v in enumerate(ob_vertice):
                if index == len(ob_vertice)-1:
                    break
                d_x = ob_vertice[index+1][0] - v[0]
                d_y = ob_vertice[index+1][1] - v[1]
                d = (max( abs(d_x),abs(d_y) ))
                if abs(d_x)>abs(d_y):
                    d_y = d_y/d
                    d_x = d_x/d
                else:
                    d_x = d_x/d
                    d_y = d_y/d
                for n in range(0,int(d)):
                    xi = int(v[0] + d_x * n)
                    yi = int(v[1] + d_y * n)
                    if mark[xi][yi]==0:
                        mark[xi][yi]=1

        ob_vertice.clear()


    file = open("./bitmap.txt", mode='w')
    for i in range(128):
        for j in range(128):
            file.write(str(mark[j][i])+"  ")
        file.write('\n')
    file.close()

    #-----Potential Field----------

    cp1 = (structure.pointoncanvas(getData.main.robot.cp[0],getData.main.robot.goal).canvas_to_planner()).toList()
    cp1[0] = int(cp1[0])
    cp1[1] = int(cp1[1])
    cp2 = (structure.pointoncanvas(getData.main.robot.cp[1],getData.main.robot.goal).canvas_to_planner()).toList()
    cp2[0] = int(cp2[0])
    cp2[1] = int(cp2[1])

    #print(cp1,cp2)


    for i in range(128):
        for j in range(128):
            if mark[j][i]==1:
                PF1[j][i] = 255;
                PF2[j][i] = 255;
            else:
                PF1[j][i] = -1;
                PF2[j][i] = -1;
    PF1[cp1[0]][cp1[1]] = 0
    PF2[cp2[0]][cp2[1]] = 0

    def Propagate(list, field):
        next = []

        for point in list:
            value = field[point[0]][point[1]]
            if point[0] - 1 >= 0 and field[point[0] - 1][point[1]] == -1:
                field[point[0] - 1][point[1]] = value + 1
                next.append([point[0] - 1, point[1]])
            if point[0] + 1 < 128 and field[point[0] + 1][point[1]] == -1:
                field[point[0] + 1][point[1]] = value + 1
                next.append([point[0] + 1, point[1]])
            if point[1] - 1 >= 0 and field[point[0]][point[1] - 1] == -1:
                field[point[0]][point[1]-1] = value + 1
                next.append([point[0], point[1]-1])
            if point[1] + 1 < 128 and field[point[0]][point[1] + 1] == -1:
                field[point[0]][point[1]+1] = value + 1
                next.append([point[0], point[1]+1])
        if next:
            Propagate(next, field)

    Propagate([cp1],PF1)
    Propagate([cp2],PF2)


    file = open("./PF1.txt", mode='w')
    for i in range(128):
        for j in range(128):
            if PF1[j][i] == -1:
                PF1[j][i] = 255
            file.write(str(PF1[j][i])+"\t")
        file.write('\n')
        file.write('\n')
    file.close()

    file = open("./PF2.txt", mode='w')
    for i in range(128):
        for j in range(128):
            if PF2[j][i] == -1:
                PF2[j][i] = 255
            file.write(str(PF2[j][i])+"\t")
        file.write('\n')
        file.write('\n')
    file.close()
