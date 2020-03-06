import math
import numpy as np
import matplotlib.pyplot as plt
import time
import heapq
import argparse
show_animation=True

#Define Node
class Node:
    def __init__(self,x,y,c2c,parentId):
        self.x = x
        self.y = y
        self.c2c = c2c
        self.parentId = parentId
    def __str__(self):
        return str(self.x)+","+str(self.y)+","+str(self.c2c)+","+str(self.parentId)

    def __lt__(self,other):
        return self.c2c<other.c2c
    def __eq__(self,other):
        return self.c2c == other.c2c
    def __ne__(self,other):
        return self.c2c!=other.c2c

#Define costs for each movement
def motion_model():
    # dx, dy, cost
    #Moving in anticlockwise direction
    motion = [[1,0,1],                #Move Right
              [1,1,math.sqrt(2)],     #Move Right Up
              [0,1,1],                #Move Up
              [-1,1,math.sqrt(2)],    #Move Left Up
              [-1,0,1],               #Move Left
              [-1,-1,math.sqrt(2)],   #Move Left Down
              [0,-1,1],               #Move Down
              [1,-1,math.sqrt(2)]]    #Move Left Down
    return motion

def gen_index(node):
    return node.x*250+node.y


def collision(node, grid,res):
    if grid[int(node.y)][int(node.x)] ==False:
        return False
    elif node.y>150/res:
        return False
    elif node.x < 0:
        return False
    elif node.x >250/res:
        return False
    elif node.y< 0:
        return False
    else:
        return True

def path_planning(nStart,nGoal,robot_radius,grid,ox,oy,res,show_animation):

    motion = motion_model()
    cost_heap = []
    active, dead = dict(),dict()

    active[gen_index(nStart)] = nStart

    heapq.heappush(cost_heap,[nStart.c2c,nStart])

    #cache the background for plotting
    fig,ax = plt.subplots(figsize = (int(25//res),int(15//res)))
    canvas = ax.figure.canvas
    ax.grid(True)
    plt.xlim(-1,(250//res) +1)
    plt.ylim(-1,(150//res) +1)

    x = np.linspace(1,150//res,150//res)
    y = np.linspace(1,250//res,250//res)

    for i in x:
        plt.axhline(y=i,color='y')
    for j in y:
        plt.axvline(x=j,color='y')
    p1 = ax.plot(ox,oy,".k")
    p2 = ax.plot(nStart.x,nStart.y,"or")
    p3 = ax.plot(nGoal.x,nGoal.y,"ob")
    ax.grid(True)
    plt.show(False)
    canvas.draw()


    background = fig.canvas.copy_from_bbox(ax.bbox)
    pt1 = ax.plot(nStart.x,nStart.y,".c")[0]
    pt2 = ax.plot(nStart.x,nStart.y,".m")[0]
    j=0
    k=len(x)*len(y)/100
    pts_x = []
    pts_y = []

    while 1:
        if len(cost_heap)==0:
            print("Goal Can't be reached")
            break
        current = heapq.heappop(cost_heap)[1]
        # print("test",current)
        curr_id = gen_index(current)
        # if current ID is already in Dead nodes pass
        if curr_id in dead:
            continue
        pts_x.append(current.x)
        pts_y.append(current.y)
        #Draw Graph to show node exploration
        if show_animation:
            # if current != nStart:
            if (j%k ==0) | (current.x == nGoal.x and current.y == nGoal.y):
                pt1.set_data(pts_x,pts_y)
                ax.draw_artist(pt1)
                fig.canvas.blit(ax.bbox)

        if current.x == nGoal.x and current.y == nGoal.y:
            print("Target Found \n")
            nGoal.parentId = current.parentId
            nGoal.c2c = nGoal.c2c
            break


        #Pop the visited nodes from the active set
        del active[curr_id]
        #Add the visited node to the dead set
        dead[curr_id] = current

        #Search the adjacent nodes
        new_x=[]
        new_y=[]
        for i,_ in enumerate(motion):
            new_node = Node(current.x + motion[i][0],current.y + motion[i][1],current.c2c + motion[i][2],curr_id)

            #Calculate new ID for each new nodes
            new_id = gen_index(new_node)

            #Checks for unique and feasible node
            if collision(new_node,grid,res)==False:
                continue

            if new_id in dead:
                continue

            if new_id in active:
                if active[new_id].c2c > new_node.c2c:
                    active[new_id].c2c = new_node.c2c
                    active[new_id].parentId = new_node.parentId

                    heapq.heappush(cost_heap,[active[new_id].c2c,active[new_id]])

            else:
                active[new_id] = new_node

                heapq.heappush(cost_heap,[active[new_id].c2c,active[new_id]])
                new_x.append(active[new_id].x)
                new_y.append(active[new_id].y)

        if show_animation:
            if (j%k ==0) | (current.x == nGoal.x and current.y == nGoal.y):
                pt2.set_data(new_x,new_y)
                ax.draw_artist(pt2)
                fig.canvas.blit(ax.bbox)

        j=j+1
    path_x,path_y = findPath(nGoal,dead,ax,canvas,background,fig)

    return path_x,path_y, dead

#Trace Back the final path
def findPath(nGoal,dead,ax,canvas,background,fig):
    rx,ry = [nGoal.x],[nGoal.y]
    parentId = nGoal.parentId

    while parentId !=-1:
        visited = dead[parentId]
        rx.append(visited.x)
        ry.append(visited.y)
        parentId = visited.parentId
    rx.reverse()
    ry.reverse()
    pt = ax.plot(rx[0],ry[0],".r")[0]
    for i in range(len(rx)):
        pt.set_data(rx[i],ry[i])
        ax.draw_artist(pt)
        fig.canvas.blit(ax.bbox)
        # plt.draw()
    # plt.pause(20)
    canvas.draw()
    # print(len(rx))
    return rx,ry



#Define required Functions
def generate_map(robot_radius,res,clearence):
    ox = [] #list of x coordinates based on if there is an obstacle or not
    oy = [] #list of y coordinates based on if there is an obstacle or not
    xx,yy = np.mgrid[:int(150/res)+2,:int(250/res)+2]
    xx = res*xx
    yy = res*yy
    robot_radius = (clearence + robot_radius)
    #Grid Boundary
    o1 = xx
    o2 = yy
    o3 = xx
    o4 = yy

    #cirle and ellipse
    circle = (xx-130)**2+(yy-190)**2
    ellipse = (6*(yy-140))**2 + (15*(xx-120))**2

    #polygon
    l4 = 38*yy+23*xx-(8530+robot_radius*(math.sqrt(38**2 + 23**2)))
    l3 = -20*xx+37*yy-(6101+robot_radius*(math.sqrt(20**2 + 37**2)))
    l2 = -xx+15-robot_radius
    l1 = 41*yy+25*xx-(6525-robot_radius*(math.sqrt(41**2 + 25**2)))
    l6 = 4*yy+38*xx-(2628+robot_radius*(math.sqrt(4**2 + 38**2)))
    l5 = 38*yy-7*xx-(5830-robot_radius*(math.sqrt(38**2 + 7**2)))

    #rect
    s1 = xx
    s2 = yy
    s3 = xx
    s4 = yy

    grid = np.full((int((150//res) +2), int((250//res) +2)), True, dtype=bool)
    grid[o1<0+robot_radius]=False
    grid[o2<0+robot_radius]=False
    grid[o3>150-robot_radius]=False
    grid[o4>250-robot_radius]=False
    grid[(circle<=(15+robot_radius)**2) | (ellipse<=(90+robot_radius)**2) |((s1<=112.5+robot_radius)&(s2<=100+robot_radius)&(s3>=67.5-robot_radius)&(s4>=50-robot_radius)) ]=False  #(l5>0) | (l6<0)))
    grid[(l1>=0)&(l2<=0)&(l3<=0)&(l4<=0)&((l5>=0)|(l6<=0))] = False
    for i in range(int((150//res) +2)):
        for j in range(int((250//res) +2)):
            if grid[i][j]==False:
                ox.append(j)
                oy.append(i)
    for i in range(int((150//res)+2)):
        ox.append(-1)
        oy.append(i)

    for j in range(int((250//res)+2)):
        ox.append(j)
        oy.append(-1)
    return grid,ox,oy



def main():
    Parser = argparse.ArgumentParser()
    # Parser.add_argument('--mode', default="dijk", help='Provide the planning algorithm to be used: dijk or astar')
    Parser.add_argument('--gui', default=True, help='Check for animation')


    Args = Parser.parse_args()

    # mode = Args.mode
    animate = Args.gui
    # hur = Args.hur
    # flag = str(hur)
    # res = Args.res
    res = int(input("Define resolution(Min alowed = 1): \n"))
    # print(res)
    # print(res)

    show = str(animate)
    if show == 'True':
        show_animation = True
    if show == 'False':
        show_animation = False
    robot_radius = int(input("Define Robot Radius(Min allowed = 0): \n"))
    clearence = int(input("Define Clearence(Min allowed = 0): \n"))
    grid,ox,oy = generate_map(robot_radius,res,clearence)

    sx = input("Enter x-y coordinates of start node: ")
    sy = input("Enter x-y coordinates of start node: ")

    nStart=Node(int(sx)//res,int(sy)//res,0.0,-1)

    start = False
    goal=False
    #collision check for start node
    if collision(nStart,grid,res):
        start=True
        print("Start node valid \n")
    else:
        print("Start node INVALID, Please enter valid start node \n")
        return -1

    gx = input("Enter x-y coordinates of goal node: ")
    gy = input("Enter x-y coordinates of goal node: ")

    nGoal = Node(int(gx)//res,int(gy)//res,0.0,-1)

    #Check for collision of goal node
    if collision(nGoal,grid,res):
        goal=True
        print("Goal Node valid \n")
    else:
        print("Goal node INVALID, Please enter valid goal node \n")
        return -1
    if start==True and goal==True:
        start = time.time()
        rx,ry,dead = path_planning(nStart,nGoal,robot_radius=robot_radius,grid=grid,ox=ox,oy=oy,res=res,show_animation=show_animation)
        end = time.time()
        print("Time to completion:",(end-start))
        # if show_animation:
        list = dead.values()
        # print(len(list))
        x=[]
        y=[]
        for i in range(len(list)):
            x.append(list[i].x)
            y.append(list[i].y)

        # print(list)
        plt.rcParams["figure.figsize"] = (250,150)
        plt.plot(x,y,".c")
        plt.plot(rx,ry,".r")
        plt.plot(rx,ry,"-r")
        # plt.pause(5)
        plt.plot(nStart.x,nStart.y,"og")
        plt.plot(nGoal.x,nGoal.y,"ob")
        plt.show()
        # plt.pause(5)
        # plt.close()

if __name__ == '__main__':
    main()