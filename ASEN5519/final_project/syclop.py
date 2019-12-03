import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import sys
import copy
import time
import pdb
from tqdm import tqdm

from a_star import A_star
from hybrid_automaton import Automaton
from controller import Controller

class SyCLoP():
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

    def solve2(self,start,end,x_dim,y_dim,v_dim,theta_dim,phi_dim,obs):
        self.T=[]
        self.T_edges=[]
        self.W_low=[]
        self.came_from={}
        self.L=1
        end_mid=[np.mean(end[0]),np.mean(end[1]),np.mean(end[2]),np.mean(end[3])]
        self.c_space_obs(obs)
        self.discritize(x_dim,y_dim)
        #  start_cell=self.locate_region(start)
        #  end_cell=self.locate_region(end_mid)
        #  self.cov[start_cell]=1
        self.update_converage(start)
        self.calc_freevol()

        self.high_plan(start,end_mid)
        #update high level selections
        for i in range(len(self.high_path)-1):
            connection_index=self.get_connection_index(self.high_path[i],self.high_path[i+1])
            if connection_index:
                self.high_selection[connection_index]+=1

        self.available=[]
        for R in reversed(self.high_path):
            if self.cov[R]!=0:
                self.available.append(R)
        self.stop=False

        #  for high_run in tqdm(range(100),ncols=100):
        while not self.stop:
            continue_high=np.random.choice([0,1],p=[0.125,0.875])
            if not continue_high:
                self.high_plan(start,end_mid)
                #update high level selections
                for i in range(len(self.high_path)-1):
                    connection_index=self.get_connection_index(self.high_path[i],self.high_path[i+1])
                    if connection_index:
                        self.high_selection[connection_index]+=1

                self.available=[]
                for R in reversed(self.high_path):
                    if self.cov[R]!=0:
                        self.available.append(R)

            probs=[]
            for R in self.available:
                probs.append((self.freevol[R]**4)/((1+self.cov[R])*(1+self.nsel[R]**2)))

            suma=sum(probs)
            probs=[x/suma for x in probs]
            R_select=np.random.choice(self.available,p=probs)
            self.nsel[R_select]+=1
            self.explore(R_select,end)
        self.path=None
        if self.stop:
            h=[0]*len(self.T)
            astar=A_star()
            print('finding path')
            #  self.path=astar.construct_path([self.T,self.T_edges,self.W_low],start,end_mid,h)
            total_path=[self.final]
            #  print(current,came_from.keys())
            current=tuple(self.final)
            while current!=tuple(start):
                current=tuple(self.came_from[current])
                total_path.insert(0,list(current))
            self.path=total_path

    def high_plan(self,start,end):
        start=self.locate_region(start)
        end=self.locate_region(end)
        count=0
        for i in range(len(self.x)-1):
            for j in range(len(self.y)-1):
                if not count%32==31:
                    index_count=self.get_connection_index(count,count+1)
                    if index_count:
                        self.W[index_count]=self.cost(count,count+1,index_count)
                if not count>991:
                    index_count=self.get_connection_index(count,count+32)
                    if index_count:
                        self.W[index_count]=self.cost(count,count+32,index_count)
                count+=1
        astar=A_star()
        path_type=np.random.choice([0,1],p=[0.05,0.95])
        if path_type:
            h=[0]*len(self.R)
        else:
            h=np.random.uniform(0,max(self.W),len(self.R))
        self.high_path=astar.construct_path_fast([self.R,self.edges,self.W],start,end,h)

    def explore(self,R,end):
        #  for low_run in range(5):
        self.new_cells=1
        continue_low=1
        while continue_low:
            if self.new_cells:
                continue_low=1
            else:
                continue_low=np.random.choice([0,1],p=[0.25,0.75])
            if continue_low:
                self.new_cells=0
                # select the cell inside R
                cell_options=self.R_cells[R]
                total_select=0
                for cell in cell_options:
                    total_select+=1/(1+self.R_cells_select[(R,cell)])
                probs=[0]*len(cell_options)
                count=0
                for cell in cell_options:
                    probs[count]=(1/(1+self.R_cells_select[(R,cell)]))/total_select
                    count+=1
                chosen_cell=np.random.choice(cell_options,p=probs)
                # select the point inside the cell
                point_options=self.cells_verts[R][chosen_cell]
                total_select=0
                for point in point_options:
                    total_select+=1/(1+self.verts_select[tuple(point)])
                probs=[0]*len(point_options)
                count=0
                for point in point_options:
                    probs[count]=(1/(1+self.verts_select[tuple(point)]))/total_select
                    count+=1
                chosen_point=point_options[np.random.choice(range(len(point_options)),p=probs)]
                # update the selecion total (make variable for that)
                self.R_cells_select[(R,chosen_cell)]+=1
                self.verts_select[tuple(chosen_point)]+=1
                # extend the point using the dynamics
                bounds=self.R_bounds[R]
                possible_a=list(self.accel(chosen_point[3],chosen_point[4]))
                for a in possible_a:
                    new_point=self.dynamics(chosen_point,a)
                    if not self.check_valid(new_point,x_dim,y_dim,theta_dim,v_dim,phi_dim,obs):
                        # find out if the point left the cell
                        if (new_point[0]<bounds[0]) or (new_point[0]>bounds[1]) or \
                                (new_point[1]<bounds[1]) or (new_point[1]>bounds[3]):
                            new_R=self.locate_region(new_point)
                            if new_R not in self.available:
                                self.available.append(new_R)
                            self.update_connections(R,new_R)
                            connection_index=self.get_connection_index(R,new_R)
                            if connection_index:
                                self.low_selection[connection_index]+=1
                        self.T.append(new_point)
                        self.T_edges.append([list(chosen_point),list(new_point)])
                        self.W_low.append(self.d(chosen_point,new_point))
                        self.came_from[tuple(new_point)]=chosen_point
                        # update everything
                        self.update_converage(new_point)
                        if self.check_end(new_point,end):
                            self.stop=True
                            self.final=new_point
        
    def discritize(self,x_dim,y_dim,z_dim=None):
        self.x=np.linspace(x_dim[0],x_dim[1],33)
        self.y=np.linspace(y_dim[0],y_dim[1],33)
        #  xx,yy=np.meshgrid(x,y)
        self.R=[]
        self.R_bounds=[]
        self.edges=[]
        self.edges_cells=[]
        self.R_cells=[]
        self.nsel=[]
        self.cov=[]
        self.R_cells_select={}
        # dict shall be {(R_cell,cell in R cell):selctions}
        self.cells_verts=[]
        self.verts_select={}
        # dict shall be {(point):selections)
        self.high_selection=[]
        self.low_selection=[]
        # W is the cost
        self.W=[]
        count=0
        for i in range(len(self.x)-1):
            for j in range(len(self.y)-1):
                #  print(count,x[i],x[i+1],y[j],y[j+1])
                self.R.append(count)
                self.R_bounds.append([self.x[i],self.x[i+1],self.y[j],self.y[j+1]])
                self.nsel.append(0)
                self.cov.append(0)
                self.R_cells.append([])
                cells_list=[]
                for k in range(16**2):
                    cells_list.append([])
                self.cells_verts.append(cells_list)
                #  self.R_cells_select.append(copy.copy(cells_list))
                if not count%32==31:
                    #right is connected
                    self.edges.append([count,count+1])
                    self.edges_cells.append(0)
                    self.high_selection.append(0)
                    self.low_selection.append(0)
                    self.W.append(0)
                if not count>991:
                    #bottom is connected
                    self.edges.append([count,count+32])
                    self.edges_cells.append(0)
                    self.high_selection.append(0)
                    self.low_selection.append(0)
                    self.W.append(0)
                count+=1
        

    def cost(self,Ri,Rj,connection_index):
        c=((1+self.selections(Ri,Rj,connection_index)**2)/(1+self.edges_cells[connection_index]**2))* \
                (1/((1+self.cov[Ri])*self.freevol[Ri]))* \
                (1/((1+self.cov[Rj])*self.freevol[Rj]))
        return c

    def locate_region(self,point):
        #TODO: this can be more effcient
        #optional variable of R so it can look in surrounding cells first
        for i in range(len(self.x)-1):
            if (point[0]>self.x[i]) and (point[0]<self.x[i+1]):
                x_bin=copy.copy(i)
                break
        for j in range(len(self.y)-1):
            if (point[1]>self.y[j]) and (point[1]<self.y[j+1]):
                y_bin=copy.copy(j)
                break
        if len(point)==3:
            # stuff for the 3D problem
            pass
        return x_bin*32+y_bin

    def cell_coords(self,R,point):
        bounds=self.R_bounds[R]
        x=np.linspace(bounds[0],bounds[1],17)
        y=np.linspace(bounds[2],bounds[3],17)
        for i in range(len(x)-1):
            if (point[0]>x[i]) and (point[0]<x[i+1]):
                x_bin=copy.copy(i)
                break
        for j in range(len(y)-1):
            if (point[1]>y[j]) and (point[1]<y[j+1]):
                y_bin=copy.copy(j)
                break
        if len(point)==3:
            # stuff for the 3D problem
            pass
        return x_bin,y_bin

    def update_converage(self,point):
        R=self.locate_region(point)
        x_bin,y_bin=self.cell_coords(R,point)
        if x_bin*16+y_bin not in self.R_cells[R]:
            self.new_cells=1
            self.R_cells[R].append(x_bin*16+y_bin)
            self.R_cells_select[(R,x_bin*16+y_bin)]=0
            self.cells_verts[R][x_bin*16+y_bin].append(point)
            self.verts_select[tuple(point)]=0
            self.cov[R]=len(self.R_cells[R])


    def selections(self,Ri,Rj,connection_index):
        if (self.cov[Ri]==0) and (self.cov[Rj]==0):
            return self.high_selection[connection_index]
        else:
            return self.low_selection[connection_index]

    def get_connection_index(self,Ri,Rj):
        if [Ri,Rj] in self.edges:
            return self.edges.index([Ri,Rj])
        elif [Rj,Ri] in self.edges:
            return self.edges.index([Rj,Ri])
        else:
            return 0

    def update_connections(self,Ri,Rj):
        connection_index=self.get_connection_index(Ri,Rj)
        if connection_index:
            self.edges_cells[connection_index]+=1

    def calc_freevol(self):
        self.freevol=[0]*len(self.R)
        valid=[0]*len(self.R)
        invalid=[0]*len(self.R)
        # this will change with the space size
        samples=np.random.rand(5000,5)
        samples[:,0:2]=samples[:,0:2]*12-1
        samples[:,2]=samples[:,2]*2*np.pi-np.pi
        samples[:,3]=samples[:,3]*(2/3)-(1/6)
        samples[:,4]=samples[:,4]*(np.pi/3)-(np.pi/6)
        for sample in samples:
            R=self.locate_region(sample)
            if self.check_valid(sample,x_dim,y_dim,theta_dim,v_dim,phi_dim,obs):
                invalid[R]+=1
            else:
                valid[R]+=1
        for i in range(len(self.R)):
            bounds=self.R_bounds[R]
            vol=(bounds[1]-bounds[0])*(bounds[3]-bounds[2])
            self.freevol[i]=vol*(sys.float_info.epsilon+valid[i])/(sys.float_info.epsilon+valid[i]+invalid[i])

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
        #  if point in self.V:
        #      return 1
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
        u2=[-np.pi/6,0,np.pi/6]
        #  u2=[-np.pi/20,0,np.pi/20]
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
        #  #had to remove velocity constriant
        if (point[0]>end[0][0]) and (point[0]<end[0][1]) and \
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
            #  new_robot[1]=[2*np.cos(theta),2*np.sin(theta)]
            #  new_robot[2]=[np.sqrt(5)*np.cos(theta+np.arctan(.5)),np.sqrt(5)*np.sin(theta+np.arctan(.5))]
            #  new_robot[3]=[np.cos(theta+np.pi/2),np.sin(theta+np.pi/2)]
            new_robot[1]=[(self.L)*np.cos(theta),(self.L)*np.sin(theta)]
            new_robot[2]=[np.sqrt(self.L**2+(self.L/2)**2)*np.cos(theta+np.arctan(.5)),np.sqrt(self.L**2+(self.L/2)**2)*np.sin(theta+np.arctan(.5))]
            new_robot[3]=[(self.L/2)*np.cos(theta+np.pi/2),(self.L/2)*np.sin(theta+np.pi/2)]
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
        V=np.array(self.T)
        print(len(V))
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
        #  for edge in self.T_edges:
        #      #  print(edge[0],edge[1])
        #      ax.plot([edge[0][0],edge[1][0]],[edge[0][1],edge[1][1]],color='C0')
        if self.path:
            for i in range(1,len(self.path)):
                ax.plot([self.path[i-1][0],self.path[i][0]],[self.path[i-1][1],self.path[i][1]],linewidth=3,color='C1')
        #  car=plt.Polygon([[0,0],[0,-1],[-2,-1],[-2,0]],color='C2')
        car=plt.Polygon([[0,0],[0,-self.L/2],[-self.L,-self.L/2],[-self.L,0]],color='C2')
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
    plan=SyCLoP()
    #start is [x,y,theta,v,phi]
    #end is [[x_min,x_max],[y_min,y_max],[theta_min,theta_max],[v_min,v_max]]
    start=[0,0,0,0,0]
    end=[[8,9],[8,9],[(4*np.pi)/9,(5*np.pi)/9],[-1/20,1/20]]
    x_dim=[-1,11]
    y_dim=[-1,11]
    v_dim=[-1/6,1/2]
    theta_dim=[-np.pi,np.pi]
    phi_dim=[-np.pi/6,np.pi/6]
    #  obs=[[[7,3],[9,3],[9,5],[7,5]],
    #          [[1,4],[4,4],[4,6],[1,6]],
    #          [[5,7],[6,7],[6,10],[5,10]]]
    obs=[[[3,0],[5,0],[5,2],[3,2]],
            [[7,3],[9,3],[9,5],[7,5]],
            [[1,4],[4,4],[4,6],[1,6]],
            [[5,7],[6,7],[6,10],[5,10]]]
    plan.dt=0.5
    plan.discritize(x_dim,y_dim)
    plan.solve2(start,end,x_dim,y_dim,v_dim,theta_dim,phi_dim,obs)
    #  print(plan.stop)
    plan.plot_path(obs,x_dim,y_dim,v_dim,phi_dim,start,end)
