#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from sawyers_travels.srv import ctr_pos
import numpy as np
import matplotlib.pyplot as mp
from matplotlib.path import Path
import matplotlib.patches as patches
from scipy.misc import imread
from math import cos,sin
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time
# import time

class maze_node:
    def __init__(self):

        # self.ballpnt = []
        # self.vect_pub = []
        # self.image_pub = []
        # self.img_out = Image()
        self.maze = []
        self.ax = []
        self.verts_h = []
        self.glist = []
        self.geom = []
        self.num_grid = []
        self.row = []
        self.patchlist=[]
        self.globalpatch=[]
        self.localpatch = []
        self.dist_out = Float64MultiArray()

        self.ballpnt = rospy.Subscriber('ball_pos',Point,self.update_cb,queue_size=1)
        self.vect_pub = rospy.Publisher('testing_error',Float64MultiArray,queue_size=10)
        # self.image_pub = rospy.Publisher('path_pic',Image,queue_size=10)

    def update_cb(self,data):
        maze = self.maze
        ax = self.ax
        verts_h = self.verts_h
        ballpnt = [int(data.x),int(data.y)]
        glist = self.glist
        geom = self.geom
        num_grid = self.num_grid
        r = self.row
        patchlist = self.patchlist
        localpatch = self.localpatch
        globalpatch = self.globalpatch

        lclp,m,dist = update_path(maze,ax,verts_h,ballpnt,glist,geom,num_grid,r,globalpatch,patchlist=patchlist,localpatch=localpatch)

        self.ax = m[0]
        self.patchlist = m[3]
        self.localpatch = lclp

        self.dist_out.data = dist
        # mp.draw()
        # mp.pause(0.0000001)
        # mp.savefig('maze.png')
        # self.img_out = imread('maze.png')
        # self.image_pub.publish(self.img_out)
        self.vect_pub.publish(self.dist_out)


def nearest_vert(qrand,G,gtable,world,p_init,geom):
    qnear = False
    dx = [0,-1,0,1]
    dy = [1,0,-1,0]

    if geom == 'square':
        if np.shape(G)[0] == 1:
            for i in range(np.shape(gtable)[0]):
                if p_init in gtable[i]:
                    x,y = i,gtable[i].index(p_init)
                    break

            for j in range(4):
                if (x+dx[j]) >= 0 and (x+dx[j]) < np.shape(gtable)[0] and (y+dy[j]) >= 0 and (y+dy[j]) < np.shape(gtable)[1]:

                    if qrand == gtable[x+dx[j]][y+dy[j]]:
                        if check_edge_n(world,G[0],qrand) == False:
                            qnear = G[0]
                            break

        else:

            for i in range(np.shape(gtable)[0]):
                if qrand in gtable[i]:
                    x,y = i,gtable[i].index(qrand)
                    break

            for j in range(4):
                if (x+dx[j]) >= 0 and (x+dx[j]) < np.shape(gtable)[0] and (y+dy[j]) >= 0 and (y+dy[j]) < np.shape(gtable)[1]:
                    if gtable[x+dx[j]][y+dy[j]] in G:
                        if check_edge_n(world,gtable[x+dx[j]][y+dy[j]],qrand) == False:
                            qnear = gtable[x+dx[j]][y+dy[j]]
                            if qnear == p_init:
                                qnear = G[0]
                            break
    else:
        if np.shape(G)[0] == 1:
            for i in range(np.shape(gtable)[0]):
                if p_init in gtable[i]:
                    theta,rad = i,gtable[i].index(p_init)
                    break
                    #print theta,rad

            for j in range(4):
                if (theta+dx[j]) >= 0 and (theta+dx[j]) < np.shape(gtable)[0] and (rad+dy[j]) >= 0 and (rad+dy[j]) < np.shape(gtable)[1]:

                    if qrand == gtable[theta+dx[j]][rad+dy[j]]:

                        if check_edge_n(world,G[0],qrand) == False:
                            qnear = G[0]
                            break

        else:
            for i in range(np.shape(gtable)[0]):
                if qrand in gtable[i]:
                    theta,rad = i,gtable[i].index(qrand)
                    break

            for j in range(4):
                offset = 0
                if (rad+dy[j]) >= 0 and (rad+dy[j]) < np.shape(gtable)[1]:
                    if (theta + dx[j]) == np.shape(gtable)[0]:
                        offset = np.shape(gtable)[0]
                    #print [theta,rad],[theta+dx[j]-offset,rad+dy[j]]
                    if gtable[theta+dx[j]-offset][rad+dy[j]] in G:
                        #print 'yes'
                        #print [theta,rad],[theta+dx[j],rad+dy[j]]
                        if check_edge_n(world,gtable[theta+dx[j]-offset][rad+dy[j]],qrand) == False:
                                qnear = gtable[theta+dx[j]-offset][rad+dy[j]]
                                if qnear == p_init:
                                    qnear = G[0]
                                break

    return qnear

def closest_node(node, nodes):
    nodes = np.asarray(nodes)
    dist_2 = np.sum((nodes - node)**2, axis=1)
    return np.argmin(dist_2)

def new_config(qnear,qrand,dq):
    xa = qrand[0]
    ya = qrand[1]
    xb = qnear[0]
    yb = qnear[1]
    qnew = [xb + (xa-xb)*dq,yb + (ya-yb)*dq]
    return qnew

def add_vertex(G,qnew):
    G.append(qnew)
    return G

def add_edge(E,qnear,qnew):
    E.append([qnear,qnew])
    return E

def points (p0, p1):
    x0, y0 = p0
    x1, y1 = p1

    dx = abs(x1-x0)
    dy = abs(y1-y0)
    if x0 < x1:
        sx = 1
    else:
        sx = -1


    if y0 < y1:
        sy = 1
    else:
        sy = -1
    err = dx-dy

    point_list = []
    while True:
        point_list.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break

        e2 = 2*err
        if e2 > -dy:
            # overshot in the y direction
            err = err - dy
            x0 = x0 + sx
        if e2 < dx:
            # overshot in the x direction
            err = err + dx
            y0 = y0 + sy

    return point_list

def check_edge_n(c,qnear, qnew):
    check = False
    line_pnts = points(qnear,qnew)

    for i in range(len(line_pnts)):
        x,y = line_pnts[i]
        if c[y,x] > 0:
            check = True
            break

    return check

def maze2graph(maze,num_grid,geom):
    gtable = []
    mlist = []
    row,col = maze.shape

    if geom == 'square':

        xgrid = col/num_grid
        ygrid = row/num_grid

        for i in range(xgrid/2,col,xgrid):
            glist = []
            for j in range(ygrid/2,row,ygrid):
                glist.append([i,j])
                mlist.append([i,j])
            gtable.append(glist)
    else:
        theta = np.deg2rad(np.arange(11.25, 360,22.5))
        rgridx = col/num_grid
        rgridy = row/num_grid
        rad_lx = rgridx*1
        rad_ly = rgridy*1
        # mid = [col/2,row/2]
        # mlist.append(mid)

        for i in range(len(theta)):
            glist = []
            # glist.append(mid)
            for j in range(num_grid/2):
                x = col/2 + int(round((rad_lx + rgridx*j)*cos(theta[i])))
                y = row/2 + int(round((rad_ly + rgridy*j)*sin(theta[i])))
                glist.append([x,y])
                mlist.append([x,y])
            gtable.append(glist)

    return mlist,gtable

def n_rrt_gen(img,numpath,geom,qinit,qgoal,ax=None):
    patchlist = None
    # FNAME = img
    world = img
    #imread(FNAME,mode='L')

    # for i in range(world.shape[0]):
    #     for j in range(world.shape[1]):
    #         if world[i,j]<255:
    #             world[i,j]=0

    # nonz = np.nonzero(world)
    # wc = (np.ones(np.shape(world),dtype=np.uint8))*255
    # wc[nonz] = 0
    # world = wc
    # world = np.flipud(world)
    # worldo = world.copy()
    # kernel = np.ones((3,3), np.uint8)
    # world = cv2.dilate(world,kernel,iterations=1)
    worldo = world.copy()

    glist,gtable = maze2graph(world,numpath,geom)
    garr = np.copy(glist)

    G = []
    E = []

    p_init = glist[closest_node(qinit,glist)]
    p_goal = glist[closest_node(qgoal,glist)]

    # if ax == None:
    #     Xmax = world.shape[1]
    #     Ymax = world.shape[0]
    #     fig = mp.figure()
    #     ax = fig.add_subplot(111)
    #     ax.imshow(world,cmap=mp.cm.binary,interpolation='nearest', origin='lower',extent=[0,Xmax,0,Ymax])
    #     mp.plot([qinit[0]],[qinit[1]],marker='o',markersize=10,color='red')
    #     mp.plot([qgoal[0]],[qgoal[1]],marker='o',markersize=10,color='blue')
    #     mp.plot(garr[:,0],garr[:,1],marker='o',markersize=10,color='green')
    #     ax.set_xlim([0,world.shape[1]])
    #     ax.set_ylim([0,world.shape[0]])
    #     mp.show()
    #     ax = None
    #     fig=None

    G.append(qinit)

    if check_edge_n(world,qinit,qgoal) == False:
        G = add_vertex(G,qgoal)
        E = add_edge(E,qinit,qgoal)
    else:
        while True:
            qnear = False
            cnt = 0
            while qnear == False:
                qrand = glist[cnt]
                cnt+=1
                qnear = nearest_vert(qrand,G,gtable,world,p_init,geom)

            qnew = qrand

            if qrand == p_goal:
                qnew = qgoal
                # if check_edge_n(world,qrand,qgoal) == False:
                #     G = add_vertex(G,qrand)
                #     E = add_edge(E,qnear,qrand)
                #     qnew = qgoal
                #     qnear = qrand
                #     p_goal = qrand

            G = add_vertex(G,qnew)
            E = add_edge(E,qnear,qnew)

            if qrand == p_goal:
                break
            glist.remove(qnew)

    verts = []
    codes = []

    for i in range(np.shape(E)[0]):
        verts.append(E[i][0])
        verts.append(E[i][1])
        codes.append(Path.MOVETO)
        codes.append(Path.LINETO)

    verts_h = []
    codes_h = []
    verts_h.append(verts[-1])
    verts_h.append(verts[-2])
    codes_h.append(Path.MOVETO)
    codes_h.append(Path.LINETO)

    check = 0
    while check == 0:
        for sublist in E:
            if sublist[1] == verts_h[-1]:
                verts_h.append(sublist[1])
                verts_h.append(sublist[0])
                codes_h.append(Path.MOVETO)
                codes_h.append(Path.LINETO)
            if verts_h[-1] == qinit:
                check = 1
                break

    #print verts_h[::-1]

    Xmax = worldo.shape[1]
    Ymax = worldo.shape[0]
    if ax==None:
        fig = mp.figure()
    # path = Path(verts, codes)
    # patch = patches.PathPatch(path)
    path_h = Path(verts_h,codes_h)
    if ax == None:
        lstyle = '-'
        clr = 'blue'
    else:
        lstyle = '--'
        clr = 'red'
    patch_h = patches.PathPatch(path_h,color=clr,lw=2,ls=lstyle)
    if ax==None:
        ax = fig.add_subplot(111)
        ax.imshow(worldo,cmap=mp.cm.binary,interpolation='nearest', origin='lower',extent=[0,Xmax,0,Ymax])
        ax.set_xlim([0,worldo.shape[1]])
        ax.set_ylim([0,worldo.shape[0]])

    startp = patches.Circle((qinit[0],qinit[1]),4,fc='r')
    endp = patches.Circle((qgoal[0],qgoal[1]),4,fc='b')

    ax.add_patch(patch_h)
    ax.add_patch(startp)
    ax.add_patch(endp)
    patchlist = [startp,endp,patch_h]

    verts_h = verts_h[::-1]


    return ax,garr.tolist(),verts_h,patchlist

def update_path(maze,ax,verts_h,ballpnt,glist,geom,num_grid,r,globalpatch,patchlist=None,localpatch=None):
    ballpnt = flippnt(ballpnt,r)

    if localpatch != None:
        for i in range(len(localpatch)):
            localpatch[i].remove()
        localpatch = None

    if patchlist != None:
        for i in range(len(patchlist)):
            patchlist[i].remove()
        patchlist = None

    m = ax,glist,verts_h,patchlist
    locpatch = localpatch

    # glist.append(verts_h[0])
    # glist.append(verts_h[-1])
    p_init = glist[closest_node(ballpnt,glist)]
    p_goal = verts_h[closest_node(ballpnt,verts_h)]

    if p_goal == verts_h[0]:
        p_goal = verts_h[1]

    # print p_init,p_goal

    vg = []
    for i in range(0,len(verts_h),2):
        vg.append(verts_h[i])
    vg.append(verts_h[-1])

    if p_init != p_goal:
        m=n_rrt_gen(maze,num_grid,geom,p_init,p_goal,ax)
        vl = []

        for i in range(0,len(m[2]),2):
            vl.append(m[2][i])
        vl.append(m[2][-1])

        if geom == 'square':
            xl,yl = vl[0]
            xl2,yl2 =vl[1]

            if xl==xl2:
                indx = 0
            elif yl==yl2:
                indx = 1
            else:
                indx = 2

            if indx != 2:
                checkg = False
                i = 0
                while 1:
                    if (vl[i] in vg) == True:
                        checkg = True
                        j = vg.index(vl[i])
                        break

                    if vl[i][indx] != vl[i+1][indx]:
                        p_end = vl[i]
                        break
                    i+=1

                if checkg == True:
                    i = 0
                    while 1:
                        if vg[j+i][indx] != vg[j+i+1][indx]:
                            p_end = vg[j+i]
                            break
                        i+=1
            else:
                p_end = vl[-1]

        else:
            k = vl.index(p_init)
            p_end = vl[k+1]

    else:
        j = vg.index(p_init)

        if geom == 'square':

            if j == len(vg)-1:
                p_end=vg[-1]
            elif j == 0:
                p_end = vg[j+1]
            else:
                xl,yl = vg[j]
                xl2,yl2 =vg[j+1]

                if xl==xl2:
                    indx = 0
                elif yl==yl2:
                    indx = 1
                else:
                    indx = 2

                if indx != 2:
                    i = 0
                    while 1:
                        if vg[j+i][indx] != vg[j+i+1][indx]:
                            p_end = vg[j+i]
                            break
                        i+=1
                else:
                    p_end = vg[-1]

        else:
            if j == len(vg)-1:
                p_end = vg[-1]
            else:
                p_end = vg[j+1]

    pdist = [p_end[0]-ballpnt[0],p_end[1]-ballpnt[1]]
    pdist = [pdist[0]/10.0,pdist[1]/10.0]
    if p_end == vg[-1] and abs(pdist[0])<1 and abs(pdist[1])<1:
        pdist = [0,0]

    startp = patches.Circle((ballpnt[0],ballpnt[1]),4,fc='y')
    endp = patches.Circle((p_end[0],p_end[1]),4,fc='g')
    ax.add_patch(startp)
    ax.add_patch(endp)

    verts_g = []
    codes_g = []
    verts_g.append(ballpnt)
    verts_g.append(p_end)
    codes_g.append(Path.MOVETO)
    codes_g.append(Path.LINETO)
    path_g = Path(verts_g,codes_g)
    patch_g = patches.PathPatch(path_g,color='green',lw=2,ls='--')
    ax.add_patch(patch_g)

    for i in range(len(globalpatch)):
        ax.add_patch(globalpatch[i])

    locpatch = [startp,endp,patch_g]

    # print ballpnt,p_end
    # print pdist
    return locpatch,m,pdist

def flippnt(pnt,r):
    pnt = [pnt[0],r-pnt[1]-1]
    return pnt

def main():

    rospy.init_node('maze_node')

    # time.sleep(2)
    mn = maze_node()
    mn.maze = '/home/williamshwang/workspace/catkin_ws/src/Sawyers-Travels/maze_bin.png'
    mn.geom = 'square'
    mn.num_grid = 7

    # bridge = CvBridge()

    # rospy.wait_for_service('maze_bin')
    # try:
    #     mazepic = rospy.ServiceProxy('maze_bin',maze_bin)

    # except: rospy.ServiceException, e:
    #     print "Service call failed: %s"%e
    # getEndpoint = rospy.ServiceProxy('maze_dest',ctr_pos)
    # rospy.wait_for_service('maze_dest')

    # try:
    #     endpoint = getEndpoint()
    #     pass
    # except rospy.ServiceException, e:
    #     print "Service call failed: %s"%e

    # try:
    #     mn.maze = bridge.imgmsg_to_cv2(mazepic,'mono8')
    # except CvBridgeError as e:
    #     print(e)

    getStartpoint = rospy.ServiceProxy('maze_start',ctr_pos)
    rospy.wait_for_service('maze_start')
    try:
        startpoint = getStartpoint()
        pass
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


    getEndpoint = rospy.ServiceProxy('maze_dest',ctr_pos)
    rospy.wait_for_service('maze_dest')
    try:
        endpoint = getEndpoint()
        pass
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


    worlds = imread(mn.maze,mode='L')
    mn.maze =worlds
    r = worlds.shape[0]
    mn.row = r

    for i in range(worlds.shape[0]):
        for j in range(worlds.shape[1]):
            if worlds[i,j]<255:
                worlds[i,j]=0

    # nonz = np.nonzero(world)
    # wc = (np.ones(np.shape(world),dtype=np.uint8))*255
    # wc[nonz] = 0
    # world = wc
    worlds = np.flipud(worlds)
    kernel = np.ones((3,3), np.uint8)
    worlds = cv2.dilate(worlds,kernel,iterations=1)
    mn.maze = worlds
    # endpoint = flippnt(endpoint,r)

    # ballpnt = [endpoint.Pos.x,endpoint.Pos.y]
    # qgoal = flippnt(ballpnt,r)

    # qinit_s = [(np.shape(worlds)[1])/2+5,20]
    # qgoal_s = [(np.shape(worlds)[1])/2-7,np.shape(worlds)[0]-20]



    # worlds = imread(mn.maze,mode='L')
    # mn.maze =worlds
    # r = worlds.shape[0]
    # mn.row = r


    startpnt = [int(startpoint.Pos.x),int(startpoint.Pos.y)]
    qinit = flippnt(startpnt,r)

    endpnt = [int(endpoint.Pos.x),int(endpoint.Pos.y)]
    qgoal = flippnt(endpnt,r)

    qinit_s = qinit
    #[(np.shape(worlds)[1])/2+5,20]
    qgoal_s = qgoal
    #[(np.shape(worlds)[1])/2-7,np.shape(worlds)[0]-20]

    ms2=n_rrt_gen(mn.maze, mn.num_grid,mn.geom,qinit_s,qgoal_s)
    mn.ax = ms2[0]
    mn.glist = ms2[1]
    mn.verts_h = ms2[2]
    mn.patchlist = ms2[3]
    mn.localpatch = None
    mn.globalpatch = ms2[3]

    try:
        # mp.ion()
        mp.show(block=False)
        rospy.sleep(1)
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()



if __name__ == '__main__':
    main()
