from structure import *
import GUI
import copy

fp = open( 'D:/1082/project/0626/robot.dat.txt', 'r')
new = open('D:/1082/project/0626/r.txt', 'w')

line = fp.readline()
while line:
    if line[0]!='#':
        new.write(line)
    line = fp.readline()
fp.close()
new.close()

#把註解拿掉

data = Robot()
r_data = open('D:/1082/project/0626/r.txt', 'r')
data.polynum = r_data.readline()
rxlist = []
rylist = []
for i in range(0, int(data.polynum)):
    #讀有幾個面 面的點數
    v = r_data.readline()
    data.poly.append([])
    for j in range(0, int(v)):
    #讀每個面ㄉ點
        line = r_data.readline()
        p = line.split()
        #print(float(p[0]), float(p[1]))
        temp = Point(float(p[0]), float(p[1]))
        data.poly[i].append(temp)
line = r_data.readline()
t = line.split()
data.start = Configuration(float(t[0]),float(t[1]),float(t[2]))
line = r_data.readline()
t = line.split()
data.goal = Configuration(float(t[0]),float(t[1]),float(t[2]))
#起點終點的conf
cp = r_data.readline()
for i in range(0, int(cp)):
    line = r_data.readline()
    p = line.split()
    #print(float(p[0]), float(p[1]))
    temp = Point(float(p[0]), float(p[1]))
    data.cp.append(temp)
r_data.close()
#讀完robot
origin = copy.deepcopy(data)
fp = open( 'D:/1082/project/0626/obstacle.dat.txt', 'r')
new = open('D:/1082/project/0626/o.txt', 'w')
line = fp.readline()
while line:
    if line[0]!='#':
        new.write(line)
    line = fp.readline()
fp.close()
new.close()

o_list = []
o_data = open('D:/1082/project/0626/o.txt', 'r')
n = o_data.readline()
#o個數
for i in range(0,int(n)):
    temp = Obstacle()
    poly = o_data.readline()
    for j in range(0,int(poly)):
        v = o_data.readline()
        temp.poly.append([])
        for k in range(0,int(v)):
            line = o_data.readline()
            p = line.split()
            pnt = Point(float(p[0]), float(p[1]))
            temp.poly[j].append(pnt)
    line = o_data.readline()
    c = line.split()
    temp.configuration = Configuration(float(c[0]), float(c[1]), float(c[2]))
    #temp.getbdbox()
    o_list.append(temp)
o_data.close()


#--------讀完數據---------
main = GUI.GUI()
main.obstacle = o_list
main.robot = data

#canvas to GUI
xlist=[]
ylist=[]
for index, o in enumerate(main.obstacle):
    for poly_index, convex in enumerate(o.poly):
        for point in convex:
            point_cv = pointoncanvas(point, o.configuration)
            xlist.append(point_cv.x)
            ylist.append(point_cv.y)
    xmax = max(xlist)
    ymax = max(ylist)
    xmin = min(xlist)
    ymin = min(ylist)
    o.bdbox[0] = Point(xmax, ymax)
    o.bdbox[1] = Point(xmax, ymin)
    o.bdbox[2] = Point(xmin, ymin)
    o.bdbox[3] = Point(xmin, ymax)
    xlist.clear()
    ylist.clear()

#robot to GUI

for index, convex in enumerate(main.robot.poly):
    for p in convex:
        point_cv = pointoncanvas(p, main.robot.start)
        xlist.append(point_cv.x)
        ylist.append(point_cv.y)
xmax = max(xlist)
ymax = max(ylist)
xmin = min(xlist)
ymin = min(ylist)
main.robot.bdbox.append(Point(xmax, ymax))
main.robot.bdbox.append(Point(xmax, ymin))
main.robot.bdbox.append(Point(xmin, ymin))
main.robot.bdbox.append(Point(xmin, ymax))
