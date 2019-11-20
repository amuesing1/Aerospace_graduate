import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import argparse
import sys
import copy
from tqdm import tqdm
import time

class A_star():
    def construct_path(self,G,start,end,h,debug=False):
        V=G[0]
        E=G[1]
        W=G[2]
        if len(E)!=len(W):
            print("Not a valid graph")
            sys.exit()
        if len(V)!=len(h):
            print("Not a valid graph")
            sys.exit()

        open_set=[start]
        came_from={}
        g_score=[np.inf]*len(V)
        f_score=copy.copy(g_score)

        g_score[V.index(start)]=0
        f_score[V.index(start)]=h[V.index(start)]

        iterations=0
        while len(open_set)>0:
            f_min=np.inf
            for node in open_set:
                #  print(f_score[V.index(node)],node)
                if f_score[V.index(node)]<f_min:
                    current=node
                    f_min=f_score[V.index(node)]
            if current==end:
                #reconstruct path
                total_path=[current]
                #  print(current,came_from.keys())
                current=tuple(current)
                while current in came_from.keys():
                    current=tuple(came_from[current])
                    total_path.insert(0,list(current))
                if debug:
                    print("We did it",total_path)
                    print(iterations)
                return total_path

            iterations+=1
            #  print(current,f_score,g_score,came_from,open_set)
            open_set.remove(current)
            neighbors=[]
            connections=[]
            for edge in E:
                if edge[0]==current:
                    neighbors.append(edge[1])
                    connections.append(E.index(edge))
                elif edge[1]==current:
                    neighbors.append(edge[0])
                    connections.append(E.index(edge))
            count=0
            for neighbor in neighbors:
                trial_g=g_score[V.index(current)]+W[connections[count]]
                if trial_g<g_score[V.index(neighbor)]:
                    came_from[tuple(neighbor)]=current
                    g_score[V.index(neighbor)]=trial_g
                    f_score[V.index(neighbor)]=trial_g+h[V.index(neighbor)]

                    if neighbor not in open_set:
                        open_set.append(neighbor)
                count+=1
        if debug:
            print("Not possible")
        return 1

class TrajectoryGridSearch():
    def solve(self,start,end,x_dim,y_dim,v_dim,theta_dim,phi_dim,a_dim,alpha_dim,obs):
        #start is [x,y,theta,v,phi]
        #end is [[x_min,x_max],[y_min,y_max],[theta_min,theta_max],[v_min,v_max]]
        end_mid=[np.mean(end[0]),np.mean(end[1]),np.mean(end[2]),np.mean(end[3])]
        self.V=[start]
        self.edges=[]
        self.W=[]
        h=[]
        cells=copy.copy(self.V)
        stop=False
        while stop==False:
            next_round=[]
            for cell in cells:
                #find all surrounding cells
                possible_a=self.accel(cell[3])
                new_cells=[]
                for a in possible_a:
                    new_point=self.dynamics(cell,a)
                    if not self.check_valid(new_point,x_dim,y_dim,theta_dim,v_dim,phi_dim,obs):
                        new_cells.append(new_point)
                        self.V.append(new_point)
                        self.edges.append(list(cell),list(new_point))
                        self.W.append(self.d(cell,new_point))
                        h.append(self.d(new_point,end_mid))
                if len(new_cells)==0:
                    print("No solution")
                    return 0
                for new_cell in new_cells:
                    #stop if we reach the beginning
                    #  if self.grid[new_cell[0],new_cell[1]]==-1:
                    if self.check_end(new_cell,end):
                        stop=True
                        final=copy.copy(new_cell)
                        break
                    # add all the new cells to our next round to go out from
                    next_round.append(new_cell)
                if stop==True:
                    break
            cells=copy.copy(next_round)
        star=A_star()
        self.path=star.construct_path([self.V,self.edges,self.W],start,final,h)

    #  def dynamics(self,x,y,v,theta,phi,a,alpha):
    def dynamics(self,point,a):
        v_new=point[3]+a[0]*self.dt
        phi_new=point[4]+a[1]*self.dt
        theta_new=point[2]+(v_new/self.L)*np.tan(phi_new)*self.dt
        x_new=point[0]+v_new*np.cos(theta_new)*self.dt
        y_new=point[1]+v_new*np.sin(theta_new)*self.dt
        return [x_new,y_new,theta_new,v_new,phi_new]

    def d(self,point1,point2):
        return np.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2+ \
                (point1[2]-point2[2])**2+(point1[3]-point2[3])**2)

    def check_valid(self,point,x_dim,y_dim,theta_dim,v_dim,phi_dim,obs):
        if (point[0]<x_dim[0]) or (point[0]>x_dim[1]) or \
                (point[1]<y_dim) or (point[1]>y_dim[1]) or \
                (point[2]<theta_dim[0]) or (point[2]>theta_dim[1]) or \
                (point[3]<v_dim[0]) or (point[3]>v_dim[1]) or \
                (point[4]<phi_dim[0]) or (point[4]>phi_dim[1]):
                    return 1
        for obstacle in obs:
            x_min=np.inf
            x_max=-np.inf
            y_min=np.inf
            y_max=-np.inf
            for point in obstacle:
                if point[0]<x_min:
                    x_min=point[0]
                if point[0]>x_max:
                    x_max=point[0]
                if point[1]<y_min:
                    y_min=point[1]
                if point[1]>y_max:
                    y_max=point[1]
            if (point[0]<x_max) and (point[0]>x_min) and (point[1]<y_max) and (point[1]>y_min):
                return 1
        return 0

    def accel(self,v):
        # return all 9 accelration attempts
        def make_accels(u1,u2):
            accels=[0]*9
            for i in range(3):
                for j in range(3):
                    accels[i*3+j]=[u1[i],u2[j]]
            return accels
        u2=[-np.pi/6,np/pi/6]
        if v<1/6:
            #  self.g=1
            u1=[-1/6,0,1/6]
            make_accels(u1,u2)
        elif (v>1/6) and (v<2/6):
            #  self.g=2
            u1=[-1/6,0,1/3]
            make_accels(u1,u2)
        elif v>2/6:
            #  self.g=3
            u1=[-1/6,0,1/2]
            make_accels(u1,u2)

    def check_end(self,point,end):
        #end is [[x_min,x_max],[y_min,y_max],[theta_min,theta_max],[v_min,v_max]]
        if (point[0]>end[0][0]) and (point[0]<end[0][1]) and \
                (point[1]>end[1][0]) and (point[1]<end[1][1]) and \
                (point[2]>end[2][0]) and (point[2]<end[2][1]) and \
                (point[3]>end[3][0]) and (point[3]<end[3][1]):
                    return 1
        else:
            return 0


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument("problem",help="which problem do you want to run? 1 or 2?",type=int)
    args=parser.parse_args()
    if args.problem==1:
        problem1()
    elif args.problem==2:
        problem2()
