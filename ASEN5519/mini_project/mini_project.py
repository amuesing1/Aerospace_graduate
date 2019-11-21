import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d import Axes3D
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
    def solve(self,start,end,x_dim,y_dim,v_dim,theta_dim,phi_dim,obs):
        #start is [x,y,theta,v,phi]
        #end is [[x_min,x_max],[y_min,y_max],[theta_min,theta_max],[v_min,v_max]]
        self.L=2
        end_mid=[np.mean(end[0]),np.mean(end[1]),np.mean(end[2]),np.mean(end[3])]
        self.c_space_obs(obs)
        self.V=[start]
        self.edges=[]
        self.W=[]
        h=[self.d(start,end_mid)]
        cells=copy.copy(self.V)
        stop=False
        k=0
        #  while stop==False:
        for k in tqdm(range(7),ncols=100):
            next_round=[]
            #  print(cells)
            if len(cells)==0:
                print("No solution")
                return 0
            #  cells=np.array(cells)
            #  options=np.r_[np.random.choice(range(len(cells)),4)]
            #  if (k%4==0) and (k!=0):
            #      location=h.index(min(h))
            #      cells=[self.V[location]]
            #  cells=list(cells[options])
            for cell in cells:
                #find all surrounding cells
                possible_a=list(self.accel(cell[3]))
                #  options=np.r_[np.random.choice(range(9),2)]
                #  possible_a=list(possible_a[options])
                #  print(np.r_[np.random.choice(range(len(possible_a)),3)])
                #  possible_a=possible_a[np.r_[np.random.choice(range(len(possible_a)),3)]]
                #  print(possible_a)
                #  print(len(self.V))
                #  print(possible_a,cell[3])
                new_cells=[]
                for a in possible_a:
                    new_point=self.dynamics(cell,a)
                    #  print(new_point)
                    #  sys.exit()
                    if not self.check_valid(new_point,x_dim,y_dim,theta_dim,v_dim,phi_dim,obs):
                        #  print(new_point)
                        new_cells.append(new_point)
                        self.V.append(new_point)
                        self.edges.append([list(cell),list(new_point)])
                        self.W.append(self.d(cell,new_point))
                        h.append(self.d(new_point,end_mid))
                #  if len(new_cells)==0:
                #      print("No solution")
                #      return 0
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
            #  k+=1
        star=A_star()
        #  self.path=star.construct_path([self.V,self.edges,self.W],start,final,h)

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
        #  print(point,x_dim,y_dim,theta_dim,v_dim,phi_dim)
        #  if (point[1]<y_dim[0]) or (point[1]>y_dim[1]):
        #      print(point[1])
        #  if point[1]>0:
        #      print(point,y_dim)
        if point in self.V:
            return 1
        if (point[0]<x_dim[0]) or (point[0]>x_dim[1]) or \
                (point[1]<y_dim[0]) or (point[1]>y_dim[1]) or \
                (point[2]<theta_dim[0]) or (point[2]>theta_dim[1]) or \
                (point[3]<v_dim[0]) or (point[3]>v_dim[1]) or \
                (point[4]<phi_dim[0]) or (point[4]>phi_dim[1]):
                    #  print(point)
                    return 1
        # the c_obs needs to be indexed by theta values
        for obstacle in self.c_obs:
            if (point[0]>obstacle[0]) and (point[0]<obstacle[1]) \
                    and (point[1]>obstacle[2]) and (point[1]<obstacle[3]):
                return 1
        return 0

    def accel(self,v):
        # return all 9 accelration attempts
        def make_accels(u1,u2):
            #  accels=[0]*9
            accels=np.zeros((9,2))
            for i in range(3):
                for j in range(3):
                    #  accels[i*3+j]=[u1[i],u2[j]]
                    accels[i*3+j,:]=[u1[i],u2[j]]
            return accels
        u2=[-np.pi/20,0,np.pi/20]
        if v<1/6:
            #  self.g=1
            u1=[-1/6,0,1/6]
            return make_accels(u1,u2)
        elif (v>=1/6) and (v<=2/6):
            #  self.g=2
            u1=[-1/6,0,1/3]
            return make_accels(u1,u2)
        elif v>2/6:
            #  self.g=3
            u1=[-1/6,0,1/2]
            return make_accels(u1,u2)

    def check_end(self,point,end):
        #end is [[x_min,x_max],[y_min,y_max],[theta_min,theta_max],[v_min,v_max]]
        if (point[0]>end[0][0]) and (point[0]<end[0][1]) and \
                (point[1]>end[1][0]) and (point[1]<end[1][1]) and \
                (point[2]>end[2][0]) and (point[2]<end[2][1]) and \
                (point[3]>end[3][0]) and (point[3]<end[3][1]):
                    return 1
        else:
            return 0

    def c_space_obs(self,obs):
        self.c_obs=[]
        for obstacle in obs:
            new_obs=np.array(obstacle)
            # adding two for the wrost positioning of the car in each direction
            #  x_min=np.min(new_obs[:,0])-2
            #  x_max=np.max(new_obs[:,0])+2
            #  y_min=np.min(new_obs[:,1])-2
            #  y_max=np.max(new_obs[:,1])+2
            x_min=np.min(new_obs[:,0])
            x_max=np.max(new_obs[:,0])
            y_min=np.min(new_obs[:,1])
            y_max=np.max(new_obs[:,1])
            self.c_obs.append([x_min,x_max,y_min,y_max])

    def plot_path(self,obs,x_dim,y_dim,v_dim,phi_dim,start,end):
        V=np.array(self.V)
        end_mid=[np.mean(end[0]),np.mean(end[1]),np.mean(end[2]),np.mean(end[3])]
        fig,ax=plt.subplots()
        for obstacle in obs:
            ob=plt.Polygon(obstacle,color='k')
            ax.add_patch(ob)
        ax.set_xlim(x_dim)
        ax.set_ylim(y_dim)
        ax.scatter(V[:,0],V[:,1])
        ax.scatter(start[0],start[1],marker='*',color='green')
        ax.scatter(end_mid[0],end_mid[1],marker='*',color='red')
        #  for i in range(1,len(self.path)):
        #      ax.plot([self.path[i-1][0],self.path[i][0]],[self.path[i-1][1],self.path[i][1]],linewidth=3,color='C1')
        ax.axis('equal')
        #  plt.show()

        fig=plt.figure()
        ax=fig.gca(projection='3d')
        ax.set_aspect('equal')
        #voxels are y,z,x for some reason
        x,y,z=np.mgrid[x_dim[0]:x_dim[1]:12j,y_dim[0]:y_dim[1]:12j,v_dim[0]:v_dim[1]:11j]
        obs_space=np.zeros((11,11,10))
        for obstacle in obs:
            new_obs=np.array(obstacle)
            x_min=np.min(new_obs[:,0])
            x_max=np.max(new_obs[:,0])
            y_min=np.min(new_obs[:,1])
            y_max=np.max(new_obs[:,1])
            #  if x_max+2>10:
            #      x_max=10
            #  else:
            #      x_max+=2
            #  if y_max+2>10:
            #      y_max=10
            #  else:
            #      y_max+=2
            #  if x_min-2<0:
            #      x_min=0
            #  else:
            #      x_min-=2
            #  if y_min-2<0:
            #      y_min=0
            #  else:
            #      y_min-=2
            #  print(x_min,x_max,y_min,y_max)
            #  print(obs_space[y_min:y_max,:,x_min:x_max].shape)
            obs_space[x_min:x_max,y_min:y_max,:]=np.ones((x_max-x_min,y_max-y_min,10))
        ax.voxels(x,y,z,obs_space,color='k')
        ax.scatter(V[:,0],V[:,1],V[:,3])
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('v')
        plt.show()

        #  fig=plt.figure()
        #  ax=fig.gca(projection='3d')
        #  ax.set_aspect('equal')
        #  #voxels are y,z,x for some reason
        #  x,y,z=np.mgrid[x_dim[0]:x_dim[1]:12j,y_dim[0]:y_dim[1]:12j,theta_dim[0]:theta_dim[1]:11j]
        #  obs_space=np.zeros((11,11,10))
        #  for obstacle in obs:
        #      new_obs=np.array(obstacle)
        #      x_min=np.min(new_obs[:,0])
        #      x_max=np.max(new_obs[:,0])
        #      y_min=np.min(new_obs[:,1])
        #      y_max=np.max(new_obs[:,1])
        #      #  if x_max+2>10:
        #      #      x_max=10
        #      #  else:
        #      #      x_max+=2
        #      #  if y_max+2>10:
        #      #      y_max=10
        #      #  else:
        #      #      y_max+=2
        #      #  if x_min-2<0:
        #      #      x_min=0
        #      #  else:
        #      #      x_min-=2
        #      #  if y_min-2<0:
        #      #      y_min=0
        #      #  else:
        #      #      y_min-=2
        #      #  print(x_min,x_max,y_min,y_max)
        #      #  print(obs_space[y_min:y_max,:,x_min:x_max].shape)
        #      obs_space[x_min:x_max,y_min:y_max,:]=np.ones((x_max-x_min,y_max-y_min,10))
        #  ax.voxels(x,y,z,obs_space,color='k')
        #  ax.set_xlabel('x')
        #  ax.set_ylabel('y')
        #  ax.set_zlabel('theta')

        #  fig,ax=plt.subplots(nrows=5,ncols=1)
        #  ax[0].scatter(self.time,V[:,0])
        #  ax[0].set_ylabel('x')
        #  ax[1].scatter(self.time,V[:,1])
        #  ax[1].set_ylabel('y')
        #  ax[2].scatter(self.time,V[:,2])
        #  ax[2].set_ylabel('theta')
        #  ax[3].scatter(self.time,V[:,3])
        #  ax[3].set_ylabel('v')
        #  ax[4].scatter(self.time,V[:,4])
        #  ax[4].set_ylabel('phi')
        #  ax[4].set_xlabel('Time')


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument("problem",help="which problem do you want to run? 1 or 2?",type=int)
    args=parser.parse_args()
    if args.problem==1:
        trajectory=TrajectoryGridSearch()
        #start is [x,y,theta,v,phi]
        #end is [[x_min,x_max],[y_min,y_max],[theta_min,theta_max],[v_min,v_max]]
        start=[0,0,0,0,0]
        end=[[8,9],[8,9],[(4*np.pi)/9,(5*np.pi)/9],[-1/20,1/20]]
        #  end=[[1,3],[-1,1],[-np.pi,np.pi],[-1/20,1/20]]
        x_dim=[-1,11]
        y_dim=[-1,11]
        v_dim=[-1/6,1/2]
        theta_dim=[-np.pi,np.pi]
        phi_dim=[-np.pi/6,np.pi/6]
        obs=[[[3,0],[5,0],[5,2],[3,2]],
                [[7,3],[9,3],[9,5],[7,5]],
                [[1,4],[4,4],[4,6],[1,6]],
                [[5,7],[6,7],[6,10],[5,10]]]
        #  obs=[[[7,3],[9,3],[9,5],[7,5]],
        #          [[1,4],[4,4],[4,6],[1,6]],
        #          [[5,7],[6,7],[6,10],[5,10]]]
        trajectory.dt=2
        trajectory.solve(start,end,x_dim,y_dim,v_dim,theta_dim,phi_dim,obs)
        trajectory.plot_path(obs,x_dim,y_dim,v_dim,phi_dim,start,end)
    elif args.problem==2:
        problem2()
