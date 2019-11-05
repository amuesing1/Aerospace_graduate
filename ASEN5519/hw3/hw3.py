import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import argparse
import sys
import copy
from tqdm import tqdm

class A_star():
    def construct_path(self,G,start,end,h):
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
                print("We did it",total_path)
                return total_path

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
        print("Not possible")
        return 1

    def plot_path(self):
        pass

class PRM():
    def solve(self,n,r,obs,x_dim,y_dim,start,end):
        V=np.array([start,end])
        #  print(V)
        V=np.append(V,self.sample_space(n,obs,x_dim,y_dim),axis=0)
        #  print(V)
        edges=[]
        W=[]
        for point in V:
            for other in V:
                if list(point)==list(other):
                    continue
                if (self.d(point,other)<r) and not (self.edge_exists(point,other,edges)):
                    edges.append([list(point),list(other)])
                    W.append(self.d(point,other))
        #  print(edges)
        a=A_star()
        h=[0]*len(V)
        V=[list(x) for x in V]
        #  print(V)
        #  sys.exit()
        path=a.construct_path([V,edges,W],start,end,h)
        self.plot_path(obs,x_dim,y_dim,start,end,V,edges,path)

    def sample_space(self,n,obs,x_dim,y_dim):
        x_sam=np.random.uniform(x_dim[0],x_dim[1],n) 
        y_sam=np.random.uniform(y_dim[0],y_dim[1],n) 
        points=np.transpose(np.array([x_sam,y_sam]))
        #  print(points)
        to_delete=[]
        count=0
        for point in points:
            if self.check_collision(point,obs):
                to_delete.append(count)
            count+=1
        points=np.delete(points,to_delete,0)
        return(points)

    def check_collision(self,sample,obs):
        # we are assuming all obstacles are rectangular
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
            if (sample[0]<x_max) and (sample[0]>x_min) and (sample[1]<y_max) and (sample[1]>y_min):
                return 1
        return 0

    def d(self,point1,point2):
        return np.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)

    def edge_exists(self,point1,point2,edges):
        #  print(edges)
        for edge in edges:
            #  print(edge[0],point1)
            if (list(point1)==edge[0]) and (list(point2)==edge[1]):
                return 1
            if (list(point1)==edge[1]) and (list(point2)==edge[0]):
                return 1
        return 0

    def local_path(self,r,obs):
        pass

    def plot_path(self,obs,x_dim,y_dim,start,end,V,edges,path):
        fig,ax=plt.subplots()
        for obstacle in obs:
            ob=plt.Polygon(obstacle,color='k')
            ax.add_patch(ob)
        ax.set_xlim(x_dim)
        ax.set_ylim(y_dim)
        x_scatter=[x[0] for x in V]
        y_scatter=[y[1] for y in V]
        ax.scatter(x_scatter,y_scatter)
        ax.scatter(start[0],start[1],marker='*',color='green')
        ax.scatter(end[0],end[1],marker='*',color='red')
        for edge in edges:
            #  print(edge[0],edge[1])
            ax.plot([edge[0][0],edge[1][0]],[edge[0][1],edge[1][1]],color='C0')
        for i in range(1,len(path)):
            ax.plot([path[i-1][0],path[i][0]],[path[i-1][1],path[i][1]],linewidth=3,color='C1')
        ax.axis('equal')
        plt.show()


def problem1():
    # part a
    V=['s','A','B','C','D','E','F','G','H','I','J','K','L','e']
    E=[['s','A'],['s','B'],['s','C'],['A','D'],['A','E'],['A','F'],
            ['C','L'],['C','K'],['C','J'],['B','G'],['B','H'],
            ['B','I'],['E','e'],['K','e'],['G','e'],['I','e']]
    W=[1,1,1,1,1,3,1,1,1,4,1,2,3,2,3,3]
    h=[0,3,2,3,3,1,3,2,1,2,3,2,3,0]
    start='s'
    end='e'
    a=A_star()
    a.construct_path([V,E,W],start,end,h)
    h=[0 for x in h]
    a.construct_path([V,E,W],start,end,h)
    #  a.plot_path()

def problem2():
    p=PRM()
    obs1=[[[3.5,0.5],[3.5,1.5],[4.5,1.5],[4.5,0.5]],[[6.5,-1.5],[6.5,-0.5],[7.5,-0.5],[7.5,-1.5]]]
    p.solve(200,1,obs1,[-1,11],[-3,3],[0,0],[10,0])

if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument("problem",help="which problem do you want to run? 1 or 2?",type=int)
    args=parser.parse_args()
    if args.problem==1:
        problem1()
    elif args.problem==2:
        problem2()
