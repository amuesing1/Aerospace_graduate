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
        if debug:
            print("Not possible")
        return 1

class PRM():
    def solve(self,n,r,obs,x_dim,y_dim,start,end,smoothing=False):
        self.V=np.array([start,end])
        #  print(V)
        self.V=np.append(self.V,self.sample_space(n,obs,x_dim,y_dim),axis=0)
        #  print(V)
        self.edges=[]
        self.W=[]
        count1=0
        for point in self.V:
            count2=0
            for other in self.V:
                if list(point)==list(other):
                    continue
                if count2<count1:
                    count2+=1
                    continue
                #  if self.V.index(other)<self.V.index(point):
                #      continue
                #  if (self.d(point,other)<r) and not (self.edge_exists(point,other,self.edges)):
                if (self.d(point,other)<r):
                    if not self.edge_collision(point,other,obs):
                        self.edges.append([list(point),list(other)])
                        self.W.append(self.d(point,other))
                count2+=1
            count1+=1
        #  print(edges)
        a=A_star()
        h=[0]*len(self.V)
        self.V=[list(x) for x in self.V]
        #  print(V)
        #  sys.exit()
        self.path=a.construct_path([self.V,self.edges,self.W],start,end,h)
        if self.path==1:
            return 0
        if smoothing:
            self.path=self.smooth_path(path,obs)
        return 1

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

    #  def edge_exists(self,point1,point2,edges):
    #      #  print(edges)
    #      for edge in edges:
    #          #  print(edge[0],point1)
    #          if (list(point1)==edge[0]) and (list(point2)==edge[1]):
    #              return 1
    #              print('yes')
    #          if (list(point1)==edge[1]) and (list(point2)==edge[0]):
    #              return 1
    #      return 0

    def edge_collision(self,point1,point2,obs):
        edge_points_x=np.linspace(point1[0],point2[0])
        edge_points_y=np.linspace(point1[1],point2[1])
        for i in range(len(edge_points_x)):
            if self.check_collision([edge_points_x[i],edge_points_y[i]],obs):
                return 1
        return 0

    def smooth_path(self,path,obs):
        for n in range(100):
            sam=[0,0]
            while sam[0]==sam[1]:
                sam=np.random.randint(len(path),size=2)
            if self.edge_collision(path[sam[0]],path[sam[1]],obs):
                continue
            else:
                max_ind=max(sam)
                min_ind=min(sam)
                del path[min_ind+1:max_ind]
        return path

    def plot_path(self,obs,x_dim,y_dim,start,end):
        fig,ax=plt.subplots()
        for obstacle in obs:
            ob=plt.Polygon(obstacle,color='k')
            ax.add_patch(ob)
        ax.set_xlim(x_dim)
        ax.set_ylim(y_dim)
        x_scatter=[x[0] for x in self.V]
        y_scatter=[y[1] for y in self.V]
        ax.scatter(x_scatter,y_scatter)
        ax.scatter(start[0],start[1],marker='*',color='green')
        ax.scatter(end[0],end[1],marker='*',color='red')
        for edge in self.edges:
            #  print(edge[0],edge[1])
            ax.plot([edge[0][0],edge[1][0]],[edge[0][1],edge[1][1]],color='C0')
        path_length=0
        for i in range(1,len(self.path)):
            ax.plot([self.path[i-1][0],self.path[i][0]],[self.path[i-1][1],self.path[i][1]],linewidth=3,color='C1')
            path_length+=self.d(self.path[i-1],self.path[i])
        ax.axis('equal')
        plt.title('Path Length=%.2f'%path_length)
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
    #part a
    #part i
    p=PRM()
    x_dim=[-1,11]
    y_dim=[-3,3]
    start=[0,0]
    end=[10,0]
    obs1=[[[3.5,0.5],[3.5,1.5],[4.5,1.5],[4.5,0.5]],[[6.5,-1.5],[6.5,-0.5],[7.5,-0.5],[7.5,-1.5]]]
    p.solve(200,1,obs1,x_dim,y_dim,start,end)
    #  p.plot_path(obs1,x_dim,y_dim,start,end)
    #  sys.exit()

    #part ii
    n=[200,200,200,200,500,500,500,500]
    r=[.5,1,1.5,2,.5,1,1.5,2]
    #  n=[200]
    #  r=[2]
    all_sol=[]
    all_paths=[]
    all_times=[]
    num_sam=5
    for i in range(len(n)):
        path_lengths=[np.nan]*num_sam
        times=[np.nan]*num_sam
        for j in tqdm(range(num_sam),ncols=100):
            start_time=time.time()
            result=p.solve(n[i],r[i],obs1,x_dim,y_dim,start,end)
            times[j]=time.time()-start_time
            if result==1:
                path_length=0
                for k in range(1,len(p.path)):
                    path_length+=p.d(p.path[k-1],p.path[k])
                path_lengths[j]=path_length
        if np.isnan(np.nanmean(path_lengths)):
            path_lengths_clean=[0]*num_sam
        print(np.nanmean(path_lengths))
        print(np.nanmean(times))
        path_lengths_clean=[x for x in path_lengths if not np.isnan(x)]
        all_paths.append(path_lengths_clean)
        all_times.append(times)

    plt.figure()
    plt.boxplot(all_paths)

    plt.figure()
    plt.boxplot(all_times)
    #  plt.show()

if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument("problem",help="which problem do you want to run? 1 or 2?",type=int)
    args=parser.parse_args()
    if args.problem==1:
        problem1()
    elif args.problem==2:
        problem2()
