import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import sys
import copy
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
        final=None
        self.path=None
        #  k=0
        while stop==False:
            next_round=[]
            if len(cells)==0:
                print("No solution")
                return 0
            for cell in cells:
                possible_a=list(self.accel(cell[3],cell[4]))
                new_cells=[]
                for a in possible_a:
                    new_point=self.dynamics(cell,a)
                    if not self.check_valid(new_point,x_dim,y_dim,theta_dim,v_dim,phi_dim,obs):
                        new_cells.append(new_point)
                        self.V.append(new_point)
                        self.edges.append([list(cell),list(new_point)])
                        self.W.append(self.d(cell,new_point))
                        h.append(self.d(new_point,end_mid))
                for new_cell in new_cells:
                    #stop if we reach the goal
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
        if final:
            print('finding path')
            star=A_star()
            self.path=star.construct_path([self.V,self.edges,self.W],start,final,h)

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
        idx=(np.abs(self.thetas-point[2])).argmin()
        obstacles=self.c_obs[idx,:,:]
        for obstacle in obstacles:
            if (point[0]>obstacle[0]) and (point[0]<obstacle[1]) \
                    and (point[1]>obstacle[2]) and (point[1]<obstacle[3]):
                return 1
        return 0

    def accel(self,v,phi):
        # return all 9 accelration attempts
        def make_accels(u1,u2):
            accels=np.zeros((9,2))
            for i in range(3):
                for j in range(3):
                    accels[i*3+j,:]=[u1[i],u2[j]]
            return accels
        u2=[-np.pi/20,0,np.pi/20]
        if v<1/6:
            u1=[-1/6,0,1/6]
            return make_accels(u1,u2)
        elif (v>=1/6) and (v<=2/6):
            u1=[-1/6,0,1/3]
            return make_accels(u1,u2)
        elif v>2/6:
            u1=[-1/6,0,1/2]
            return make_accels(u1,u2)

    def check_end(self,point,end):
        #end is [[x_min,x_max],[y_min,y_max],[theta_min,theta_max],[v_min,v_max]]
        #  if (point[0]>end[0][0]) and (point[0]<end[0][1]) and \
        #          (point[1]>end[1][0]) and (point[1]<end[1][1]) and \
        #          (point[2]>end[2][0]) and (point[2]<end[2][1]) and \
        #          (point[3]>end[3][0]) and (point[3]<end[3][1]):
        #              return 1
        #had to remove velocity constriant
        if (point[0]>end[0][0]) and (point[0]<end[0][1]) and \
                (point[2]>end[2][0]) and (point[2]<end[2][1]) and \
                (point[1]>end[1][0]) and (point[1]<end[1][1]):
                    return 1
        else:
            return 0

    def c_space_obs(self,obs):
        self.thetas=np.linspace(0,np.pi,500)
        self.c_obs=np.zeros((len(self.thetas),len(obs),4))
        i=0
        angs1=[0,np.pi/2,np.pi,(3*np.pi)/2,2*np.pi]
        for theta in self.thetas:
            # rotate robot
            new_robot=[0]*4
            new_robot[0]=[0,0]
            new_robot[1]=[2*np.cos(theta),2*np.sin(theta)]
            new_robot[2]=[np.sqrt(5)*np.cos(theta+np.arctan(.5)),np.sqrt(5)*np.sin(theta+np.arctan(.5))]
            new_robot[3]=[np.cos(theta+np.pi/2),np.sin(theta+np.pi/2)]
            #  print(theta,new_robot)
            min_point=None
            min_y=None
            count=0
            for k in new_robot:
                if min_y is not None:
                    if k[1]<min_y:
                        min_y=k[1]
                        min_point=copy.copy(count)
                    elif k[1]==min_y:
                        if k[0]<new_robot[min_point][0]:
                            min_point=copy.copy(count)
                else:
                    min_y=k[1]
                    min_point=copy.copy(count)
                count+=1
            if min_point==1:
                new_robot=np.array([new_robot[1],new_robot[2],new_robot[3],new_robot[0],new_robot[1]])
            elif min_point==2:
                new_robot=np.array([new_robot[2],new_robot[3],new_robot[0],new_robot[1],new_robot[2]])
            elif min_point==3:
                new_robot=np.array([new_robot[3],new_robot[0],new_robot[1],new_robot[2],new_robot[3]])
            else:
                new_robot=np.array([new_robot[0],new_robot[1],new_robot[2],new_robot[3],new_robot[0]])

            W=new_robot
            angs2=[theta+x for x in angs1]
            j=0
            for obstacle in obs:
                obstacle.append(obstacle[0])
                V=np.array(obstacle)
                k=0
                l=0
                points=[]
                x_points=[]
                y_points=[]

                #run the minkowski sum
                while (k!=5) and (l!=5):
                    x=V[k][0]+W[l][0]
                    y=V[k][1]+W[l][1]
                    points.append([x,y])
                    x_points.append(x)
                    y_points.append(y)
                    if angs1[k]<angs2[l]:
                        k+=1
                    elif angs1[k]>angs2[l]:
                        l+=1
                    else:
                        k+=1
                        l+=1
                x_min=min(x_points)
                x_max=max(x_points)
                y_min=min(y_points)
                y_max=max(y_points)
                self.c_obs[i,j,:]=[x_min,x_max,y_min,y_max]
                #graphing to make sure it works
                #  fig,ax=plt.subplots()
                #  ob=plt.Polygon(points)
                #  print(points)
                #  print(list(zip(x_points,y_points)))
                #  print(ob)
                #  ax.add_patch(ob)
                #  ax.axis('equal')
                #  plt.show()
                #  sys.exit()
                j+=1
            i+=1

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
        for edge in self.edges:
            #  print(edge[0],edge[1])
            ax.plot([edge[0][0],edge[1][0]],[edge[0][1],edge[1][1]],color='C0')
        if self.path:
            for i in range(1,len(self.path)):
                ax.plot([self.path[i-1][0],self.path[i][0]],[self.path[i-1][1],self.path[i][1]],linewidth=3,color='C1')
        car=plt.Polygon([[0,0],[0,-1],[-2,-1],[-2,0]],color='C2')
        ax.add_patch(car)
        ax.axis('equal')
        plt.show()

        if self.path:
            path=np.array(self.path)
            fig,ax=plt.subplots(nrows=5,ncols=1)
            ax[0].plot(np.linspace(0,self.dt*len(self.path),len(self.path)),path[:,0],color='C1')
            ax[0].set_ylabel('x')
            ax[1].plot(np.linspace(0,self.dt*len(self.path),len(self.path)),path[:,1],color='C1')
            ax[1].set_ylabel('y')
            ax[2].plot(np.linspace(0,self.dt*len(self.path),len(self.path)),path[:,2],color='C1')
            ax[2].set_ylabel('theta')
            ax[3].plot(np.linspace(0,self.dt*len(self.path),len(self.path)),path[:,3],color='C1')
            ax[3].set_ylabel('v')
            ax[4].plot(np.linspace(0,self.dt*len(self.path),len(self.path)),path[:,4],color='C1')
            ax[4].set_ylabel('phi')
            ax[4].set_xlabel('Time')
            plt.suptitle('States over Time for Solution Path')
            plt.show()


if __name__ == '__main__':
    trajectory=TrajectoryGridSearch()
    #start is [x,y,theta,v,phi]
    #end is [[x_min,x_max],[y_min,y_max],[theta_min,theta_max],[v_min,v_max]]
    start=[0,0,0,0,0]
    end=[[8,9],[8,9],[(4*np.pi)/9,(5*np.pi)/9],[-1/20,1/20]]
    x_dim=[-1,11]
    y_dim=[-1,11]
    v_dim=[-1/6,1/2]
    theta_dim=[-np.pi,np.pi]
    phi_dim=[-np.pi/6,np.pi/6]
    obs=[[[3,0],[5,0],[5,2],[3,2]],
            [[7,3],[9,3],[9,5],[7,5]],
            [[1,4],[4,4],[4,6],[1,6]],
            [[5,7],[6,7],[6,10],[5,10]]]
    trajectory.dt=2.6
    trajectory.solve(start,end,x_dim,y_dim,v_dim,theta_dim,phi_dim,obs)
    trajectory.plot_path(obs,x_dim,y_dim,v_dim,phi_dim,start,end)
