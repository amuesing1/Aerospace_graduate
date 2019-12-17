import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import sys
import copy
import time
import pdb
#  from tqdm import tqdm

from a_star import A_star
from hybrid_automaton import Automaton
from controller import Controller
from estimator import UKF

class SyCLoP():
    def __init__(self):
        self.UKF=UKF()
        self.hybrid=Automaton()

    def solve(self,start,end,x_dim,y_dim,z_dim,obs,place=None,task=False):
        """Solve RoboSub Motion Planning"""

        self.dis_size=11
        self.dis_size_big=self.dis_size**2
        self.cov_size=int(self.dis_size/2)
        self.cov_size_big=self.cov_size**2
        self.T=[]
        self.T_edges=[]
        self.came_from={}
        self.L=.5
        # used to calculate the final high level region
        end_mid=[np.mean(end[0]),np.mean(end[1]),np.mean(end[2]),np.mean(end[3]),
                np.mean(end[4]),np.mean(end[5]),np.mean(end[6]),np.mean(end[7]),
                np.mean(end[8]),np.mean(end[9]),np.mean(end[10]),np.mean(end[11])]
        # construct the cspace for the car
        self.c_space_obs(obs)
        # create the high level discritization
        if task:
            self.discritize_task(x_dim,y_dim,z_dim,start,end_mid)
        else:
            self.discritize(x_dim,y_dim,z_dim)

        self.update_converage(start)
        self.calc_freevol()

        if task:
            self.place=place
            self.high_plan_task(start,end_mid,place)
        else:
            self.place=0
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
                if task:
                    self.high_plan_task(start,end_mid,place)
                else:
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
            # select which R to propagate samples
            R_select=np.random.choice(self.available,p=probs)
            self.nsel[R_select]+=1
            self.explore(R_select,end)

        self.path=None
        if self.stop:
            h=[0]*len(self.T)
            astar=A_star()
            print('finding path')
            # backpropagate from the final point
            total_path=[self.final]
            current=tuple(self.final)
            while current!=tuple(start):
                current=tuple(self.came_from[current])
                total_path.insert(0,list(current))
            self.path=total_path

    def high_plan(self,start,end):
        """
        Create a high level plan to bais sampling

        Inputs:
        start = start position of the sub
        end = goal position of the sub
        """

        start=self.locate_region(start)
        end=self.locate_region(end)
        count=0
        for k in range(len(self.z)-1):
            for i in range(len(self.x)-1):
                for j in range(len(self.y)-1):
                    # go to the right
                    if not count%self.dis_size==self.dis_size-1:
                        index_count=self.get_connection_index(count,count+1)
                        if index_count:
                            # compute the cost of transition for each region
                            self.W[index_count]=self.cost(count,count+1,index_count)
                    # go to the bottom
                    if count%self.dis_size_big not in range(self.dis_size_big-self.dis_size,self.dis_size_big):
                        index_count=self.get_connection_index(count,count+self.dis_size)
                        if index_count:
                            self.W[index_count]=self.cost(count,count+self.dis_size,index_count)
                    #go to the back
                    if not count>(self.dis_size**3-self.dis_size_big-1):
                        index_count=self.get_connection_index(count,count+self.dis_size_big)
                        if index_count:
                            self.W[index_count]=self.cost(count,count+self.dis_size_big,index_count)
                    count+=1
        astar=A_star()
        # occationally choose a random path that will go to goal but not optimal
        path_type=np.random.choice([0,1],p=[0.05,0.95])
        if path_type:
            h=[0]*len(self.R)
        else:
            # make random costs
            h=np.random.uniform(0,max(self.W),len(self.R))
        # use A* to compute the high level path
        self.high_path=astar.construct_path_fast([self.R,self.edges,self.W],start,end,h)
        #  print('path',self.high_path)

    def high_plan_task(self,start,end,place):
        """
        Create a high level plan to bais sampling

        Inputs:
        start = start position of the sub
        end = goal position of the sub
        """

        closest_start=self.locate_region(start)
        self.closest_start='x'+str(closest_start)+'q'+str(place)
        closest_end=self.locate_region(end)
        self.closest_end='x'+str(closest_end)+'q'+str(place+1)
        count=0
        for k in range(len(self.z)-1):
            for i in range(len(self.x)-1):
                for j in range(len(self.y)-1):
                    # go to the right
                    if not count%self.dis_size==self.dis_size-1:
                        index_count=self.get_connection_index(count,count+1)
                        if index_count:
                            # compute the cost of transition for each region
                            if index_count in self.edges_task_index.keys():
                                indicies=self.edges_task_index[index_count]
                                current_cost=self.cost(count,count+1,index_count)
                                for ind in indicies:
                                    self.W_task[ind]=current_cost
                    # go to the bottom
                    if count%self.dis_size_big not in range(self.dis_size_big-self.dis_size,self.dis_size_big):
                        index_count=self.get_connection_index(count,count+self.dis_size)
                        if index_count:
                            if index_count in self.edges_task_index.keys():
                                indicies=self.edges_task_index[index_count]
                                current_cost=self.cost(count,count+self.dis_size,index_count)
                                for ind in indicies:
                                    self.W_task[ind]=current_cost
                    #go to the back
                    if not count>(self.dis_size**3-self.dis_size_big-1):
                        index_count=self.get_connection_index(count,count+self.dis_size_big)
                        if index_count:
                            if index_count in self.edges_task_index.keys():
                                indicies=self.edges_task_index[index_count]
                                current_cost=self.cost(count,count+self.dis_size_big,index_count)
                                for ind in indicies:
                                    self.W_task[ind]=current_cost
                    count+=1
        astar=A_star()
        # occationally choose a random path that will go to goal but not optimal
        path_type=np.random.choice([0,1],p=[0.05,0.95])
        if path_type:
            h=[0]*len(self.R_task)
        else:
            # make random costs
            h=np.random.uniform(0,max(self.W_task),len(self.R_task))
        # use A* to compute the high level path
        self.high_path=astar.construct_path_fast([self.R_task,self.edges_task,self.W_task],self.closest_start,self.closest_end,h)
        #  print('path',self.high_path)
        self.high_path=[int(x[1:-2]) for x in self.high_path]

    def explore(self,R,end):
        """Take the region and explore using
        low level sampling techniques

        Inputs:
        R = region to sample from
        end = end criteria
        """

        self.new_cells=1
        continue_low=1
        while continue_low:
            # if new cells were found, we are making progress
            # of not, determine if we need to replan
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

                # update the selecion total
                self.R_cells_select[(R,chosen_cell)]+=1
                self.verts_select[tuple(chosen_point)]+=1

                # extend the point using the dynamics
                bounds=self.R_bounds[R]
                possible_a=self.accel()
                for a in possible_a:
                    new_point=self.UKF.kinematics(chosen_point,a,self.dt)
                    if abs(new_point[0])>100:
                        pdb.set_trace()
                    #  print(chosen_point,new_point,a)
                    if self.check_valid(new_point,x_dim,y_dim,z_dim):
                        # generate 5 points between the sampled point and the previous point
                        # we get these points for free and allows for a larger dt
                        prev_point=copy.copy(chosen_point)
                        for t in np.linspace(0,self.dt,6)[1:]:
                            new_point=self.UKF.kinematics(chosen_point,a,t)
                            v=self.update_converage(new_point)
                            # this checks if the point skips over an obstacle
                            # or briefly goes out of bounds
                            if v:
                                # add to the low level points
                                self.T.append(new_point)
                                self.T_edges.append([list(prev_point),list(new_point)])
                                self.came_from[tuple(new_point)]=prev_point
                                prev_point=copy.copy(new_point)
                            else:
                                new_point=prev_point
                                break

                        # find out if the point left the cell
                        if (new_point[0]<bounds[0]) or (new_point[0]>bounds[1]) or \
                                (new_point[2]<bounds[2]) or (new_point[2]>bounds[3]) or \
                                (new_point[4]<bounds[4]) or  (new_point[4]>bounds[5]):
                            new_R=self.locate_region(new_point)
                            # update the variables between regions
                            if new_R not in self.available:
                                self.available.append(new_R)
                            self.update_connections(R,new_R)
                            connection_index=self.get_connection_index(R,new_R)
                            if connection_index:
                                self.low_selection[connection_index]+=1

                        # check if this point meets the ending criteria
                        if self.check_end(new_point,end):
                            self.stop=True
                            self.final=new_point
        
    def discritize(self,x_dim,y_dim,z_dim):
        """
        Creates the high level discritization of the
        space and creates varriables for later use

        Inputs:
        x_dim = [min x, max x]
        y_dim = [min y, max y]
        z_dim = [min z, max z]
        """

        self.x=np.linspace(x_dim[0],x_dim[1],self.dis_size+1)
        self.y=np.linspace(y_dim[0],y_dim[1],self.dis_size+1)
        self.z=np.linspace(z_dim[0],z_dim[1],self.dis_size+1)
        self.R=range(self.dis_size**3)
        self.R_bounds=[[0]]*(self.dis_size**3)
        self.edges=[]
        # dictionaries are much faster to index
        self.edges_dict={}
        self.R_cells=[[]]*(self.dis_size**3)
        self.R_cells=[[] for x in self.R_cells]
        #  self.R_cells[0].append(1)
        self.nsel=[0]*(self.dis_size**3)
        self.cov=[0]*(self.dis_size**3)
        self.R_cells_select={}
        cells_list=[[]]*(self.cov_size**3)
        cells_list=[[] for x in cells_list]
        self.cells_verts=[[]]*(self.dis_size**3)
        self.cells_verts=[copy.deepcopy(cells_list) for x in self.cells_verts]
        #  self.cells_verts=[cells_list]*(self.dis_size**3)
        self.verts_select={}
        # W is the cost
        count=0
        edge_count=0
        for k in range(len(self.z)-1):
            for i in range(len(self.x)-1):
                for j in range(len(self.y)-1):
                    # create the grided regions
                    self.R_bounds[count]=[self.x[i],self.x[i+1],self.y[j],self.y[j+1],self.z[k],self.z[k+1]]

                    # create the edges between regions
                    # right is connected
                    if not count%self.dis_size==self.dis_size-1:
                        self.edges.append([count,count+1])
                        self.edges_dict[tuple([count,count+1])]=edge_count
                        edge_count+=1
                    # bottom is connected
                    if count%self.dis_size_big not in range(self.dis_size_big-self.dis_size,self.dis_size_big):
                        self.edges.append([count,count+self.dis_size])
                        self.edges_dict[tuple([count,count+self.dis_size])]=edge_count
                        edge_count+=1
                    #back is connected
                    if not count>(self.dis_size**3-self.dis_size_big-1):
                        self.edges.append([count,count+self.dis_size_big])
                        self.edges_dict[tuple([count,count+self.dis_size_big])]=edge_count
                        edge_count+=1
                    count+=1
        self.edges_cells=[0]*len(self.edges)
        self.W=[0]*len(self.edges)
        self.high_selection=[0]*len(self.edges)
        self.low_selection=[0]*len(self.edges)

    def discritize_task(self,x_dim,y_dim,z_dim,start,end):
        """
        Creates the high level discritization of the
        space and creates varriables for later use
        specifically while using LTL task specification

        Inputs:
        x_dim = [min x, max x]
        y_dim = [min y, max y]
        z_dim = [min z, max z]
        """

        self.x=np.linspace(x_dim[0],x_dim[1],self.dis_size+1)
        self.y=np.linspace(y_dim[0],y_dim[1],self.dis_size+1)
        self.z=np.linspace(z_dim[0],z_dim[1],self.dis_size+1)
        self.R=range(self.dis_size**3)
        self.R_bounds=[[0]]*(self.dis_size**3)
        self.edges=[]
        phy_sys=[]
        # dictionaries are much faster to index
        self.edges_dict={}
        self.R_cells=[[]]*(self.dis_size**3)
        self.R_cells=[[] for x in self.R_cells]
        #  self.R_cells[0].append(1)
        self.nsel=[0]*(self.dis_size**3)
        self.cov=[0]*(self.dis_size**3)
        self.R_cells_select={}
        cells_list=[[]]*(self.cov_size**3)
        cells_list=[[] for x in cells_list]
        self.cells_verts=[[]]*(self.dis_size**3)
        self.cells_verts=[copy.deepcopy(cells_list) for x in self.cells_verts]
        #  self.cells_verts=[cells_list]*(self.dis_size**3)
        self.verts_select={}
        # W is the cost
        count=0
        edge_count=0
        gate=False
        dice=False
        for k in range(len(self.z)-1):
            for i in range(len(self.x)-1):
                for j in range(len(self.y)-1):
                    state={'name':'x'+str(count),'location':[self.x[i],self.y[j],self.z[k]],
                            'trans':[{'state':'x'+str(count),'sigma':'s'}]}
                    # create the grided regions
                    self.R_bounds[count]=[self.x[i],self.x[i+1],self.y[j],self.y[j+1],self.z[k],self.z[k+1]]

                    if self.z[k]<0:
                        #surface
                        state['output']='s'
                    elif (self.x[i]<4.5) and (self.x[i]>3) and (self.y[j]>3) and (self.y[j]<7) and (self.z[k]>2) and (self.z[k]<7):
                        #gate
                        state['output']='g'
                        gate=True
                    elif (self.x[i]<15.5) and (self.x[i]>14.5) and (self.y[j]>4.5) and (self.y[j]<5.5) and (self.z[k]>4.5) and (self.z[k]<5.5):
                        #gate
                        state['output']='d'
                        dice=True
                    elif self.x[i]<3.5:
                        #left side
                        state['output']='l'
                    else:
                        #right side
                        state['output']='r'
                    # create the edges between regions
                    # right is connected
                    if not count%self.dis_size==self.dis_size-1:
                        self.edges.append([count,count+1])
                        self.edges_dict[tuple([count,count+1])]=edge_count
                        state['trans'].append({'state':'x'+str(count+1),'sigma':'r'})
                        edge_count+=1
                    # bottom is connected
                    if count%self.dis_size_big not in range(self.dis_size_big-self.dis_size,self.dis_size_big):
                        self.edges.append([count,count+self.dis_size])
                        self.edges_dict[tuple([count,count+self.dis_size])]=edge_count
                        state['trans'].append({'state':'x'+str(count+self.dis_size),'sigma':'f'})
                        edge_count+=1
                    #back is connected
                    if not count>(self.dis_size**3-self.dis_size_big-1):
                        self.edges.append([count,count+self.dis_size_big])
                        self.edges_dict[tuple([count,count+self.dis_size_big])]=edge_count
                        state['trans'].append({'state':'x'+str(count+self.dis_size_big),'sigma':'d'})
                        edge_count+=1
                    #these are just for the automata code
                    #left is connected
                    if not count%self.dis_size==0:
                        state['trans'].append({'state':'x'+str(count-1),'sigma':'l'})
                    #top is connected
                    if count%self.dis_size_big not in range(0,self.dis_size):
                        state['trans'].append({'state':'x'+str(count-self.dis_size),'sigma':'b'})
                    #front is connected
                    if count>=self.dis_size_big:
                        state['trans'].append({'state':'x'+str(count-self.dis_size_big),'sigma':'u'})


                    phy_sys.append(state)
                    count+=1
        if not gate:
            closest=1000
            for state in phy_sys:
                point=state['location']
                dis=np.sqrt((point[0]-3.5)**2+(point[1]-5)**2+ \
                        (point[2]-5)**2)
                if dis<closest:
                    closest=dis
                    loc=copy.copy(point)
            for state in phy_sys:
                if state['location']==loc:
                    state['output']='g'
        if not dice:
            closest=1000
            for state in phy_sys:
                point=state['location']
                dis=np.sqrt((point[0]-15)**2+(point[1]-5)**2+ \
                        (point[2]-5)**2)
                if dis<closest:
                    closest=dis
                    loc=copy.copy(point)
            for state in phy_sys:
                if state['location']==loc:
                    state['output']='d'

        self.edges_cells=[0]*len(self.edges)
        self.W=[0]*len(self.edges)
        self.high_selection=[0]*len(self.edges)
        self.low_selection=[0]*len(self.edges)

        buchi=[{'name':'q0','type':'init',
                'trans':[{'state':'q0','output':'l'},
                        {'state':'q1','output':'g'}]},
                {'name':'q1','type':'regular',
                'trans':[{'state':'q1','output':'grl'},
                        {'state':'q2','output':'d'}]},
                {'name':'q2','type':'regular',
                'trans':[{'state':'q2','output':'grld'},
                        {'state':'q3','output':'s'}]},
                {'name':'q3','type':'accept',
                'trans':[{'state':'q3','output':'s'}]}]
        automata=self.hybrid.product(phy_sys,buchi)

        self.R_task=[0]*len(automata)
        self.edges_task=[]
        self.edges_task_dict={}
        self.edges_task_index={}
        edges_dup=[]
        closest_end=None
        closest_point_end=1000
        closest_start=None
        closest_point_start=1000
        count=0
        for item in automata:
            item['name']=item['name1']+item['name2']
            point=item['location']
            self.R_task[count]=item['name']
            for trans in item['trans']:
                if trans['sigma']!='s':
                    if [trans['name1']+trans['name2'],item['name']] not in self.edges_task:
                        self.edges_task.append([item['name'],trans['name1']+trans['name2']])
            count+=1
        locations=[]
        count=0
        locations=[]
        for trans in self.edges_task:
            if (trans[0] not in self.R_task):
                locations.append(self.edges_task.index(trans))
            if (trans[1] not in self.R_task):
                locations.append(self.edges_task.index(trans))
        locations.sort(reverse=True)
        for loc in locations:
            del self.edges_task[loc]
        for trans in self.edges_task:
            self.edges_task_dict[tuple(trans)]=count
            count+=1
        for trans in self.edges_task:
            ind=self.get_connection_index(int(trans[0][1:-2]),int(trans[1][1:-2]))
            if ind in self.edges_task_index.keys():
                self.edges_task_index[ind].append(self.edges_task_dict[tuple(trans)])
            else:
                self.edges_task_index[ind]=[self.edges_task_dict[tuple(trans)]]

        self.W_task=[0]*len(self.edges_task)
        
    def cost(self,Ri,Rj,connection_index):
        """
        Calculates the cost between regions to be used by A*

        Inputs:
        Ri = starting region
        Rj = ending region
        connection_index=index of the edge between Ri and Rj

        Outputs:
        c = cost of moving between regions
        """

        c=((1+self.selections(Ri,Rj,connection_index)**2)/(1+self.edges_cells[connection_index]**2))* \
                (1/((1+self.cov[Ri])*self.freevol[Ri]))* \
                (1/((1+self.cov[Rj])*self.freevol[Rj]))
        return c

    def locate_region(self,point):
        """
        Finds the high level region a point belongs to

        Inputs:
        point = the sampled point [x,xdot,y,ydot,z,zdot,
                                theta,theatdot,phi,phidot,
                                psi,psidot]

        Outputs:
        x_bin*dis_size+y_bin+(z_bin*dis_size**2) = index of region or
        None if the point isn't valid
        """

        x_bin=None
        y_bin=None
        z_bin=None
        for i in range(len(self.x)-1):
            if (point[0]>self.x[i]) and (point[0]<self.x[i+1]):
                x_bin=copy.copy(i)
                break
        for j in range(len(self.y)-1):
            if (point[2]>self.y[j]) and (point[2]<self.y[j+1]):
                y_bin=copy.copy(j)
                break
        for k in range(len(self.z)-1):
            if (point[4]>self.z[k]) and (point[4]<self.z[k+1]):
                z_bin=copy.copy(k)
                break
        if (x_bin is None) or (y_bin is None) or (z_bin is None):
            return None

        return x_bin*self.dis_size+y_bin+(z_bin*self.dis_size_big)

    def cell_coords(self,R_point,point):
        """
        Finds the sub-cell a point belongs to

        Inputs:
        R = the region to search over for the point
        point = the sampled point [x,y,theta,v,phi]

        Outputs:
        x_bin*dis_size+y_bin+(z_bin*dis_size**2)= index of bin inside of R
        """

        bounds=self.R_bounds[R_point]
        x=np.linspace(bounds[0],bounds[1],self.cov_size+1)
        y=np.linspace(bounds[2],bounds[3],self.cov_size+1)
        z=np.linspace(bounds[4],bounds[5],self.cov_size+1)
        for i in range(len(x)-1):
            if (point[0]>x[i]) and (point[0]<x[i+1]):
                x_bin=copy.copy(i)
                break
        for j in range(len(y)-1):
            if (point[2]>=y[j]) and (point[2]<y[j+1]):
                y_bin=copy.copy(j)
                break
        for k in range(len(z)-1):
            if (point[4]>z[k]) and (point[4]<z[k+1]):
                z_bin=copy.copy(k)
                break

        return x_bin*self.cov_size+y_bin+(self.cov_size_big*z_bin)

    def update_converage(self,point):
        """
        Updates total coverage and places
        sampled point in separated list 
        for future propagation

        Inputs:
        point = the sampled point [x,xdot,y,ydot,z,zdot,
                                theta,theatdot,phi,phidot,
                                psi,psidot]

        Outputs:
        success if point is valid
        """

        R_now=self.locate_region(point)
        if R_now is None:
            return 0
        sub_cell=self.cell_coords(R_now,point)
        if sub_cell not in self.R_cells[R_now]:
            # indicates progress
            self.new_cells=1
            self.R_cells[R_now].append(sub_cell)
            self.R_cells_select[(R_now,sub_cell)]=0
            # point is started in verticies
            self.cells_verts[R_now][sub_cell].append(point)
            self.verts_select[tuple(point)]=0
            # used for cost
            self.cov[R_now]=len(self.R_cells[R_now])
        return 1

    def selections(self,Ri,Rj,connection_index):
        """
        Computes the number of selections for cost

        Inputs:
        Ri = region coming from
        Rj = region going to
        connection_index = index of the edge between regions

        Outputs:
        number of selections from high plan if no points
        are currently in the region, otherwise the number
        of selections by the low level sampler
        """

        if (self.cov[Ri]==0) and (self.cov[Rj]==0):
            return self.high_selection[connection_index]
        else:
            return self.low_selection[connection_index]

    def get_connection_index(self,Ri,Rj):
        """
        The index of the edge between regions

        Inputs:
        Ri = the region coming from
        Rj = the region going to

        Outputs:
        the index of the edge between regions
        """

        # disctionary indexing is significantly faster
        attempt1=tuple([Ri,Rj])
        attempt2=tuple([Ri,Rj])
        if attempt1 in self.edges_dict:
            return self.edges_dict[attempt1]
        elif attempt2 in self.edges_dict:
            return self.edges_dict[attempt2]
        else:
            return 0

    def update_connections(self,Ri,Rj):
        """
        Update the number of connections between regions

        Inputs:
        Ri = the region coming from
        Rj = the region going to
        """

        connection_index=self.get_connection_index(Ri,Rj)
        if connection_index:
            self.edges_cells[connection_index]+=1

    def calc_freevol(self):
        """
        Estimates the free volume in all high level cells
        """

        self.freevol=[0]*len(self.R)
        valid=[0]*len(self.R)
        invalid=[0]*len(self.R)
        # generate samples to check
        # this will change with the space size
        samples=np.random.rand(5000,6)
        # bias the samples for the dimension
        samples[:,0]=samples[:,0]*20-2
        samples[:,2]=samples[:,2]*14-2
        samples[:,4]=samples[:,4]*10-2
        for sample in samples:
            R=self.locate_region(sample)
            if not self.check_valid(sample,x_dim,y_dim,z_dim):
                invalid[R]+=1
            else:
                valid[R]+=1
        for i in range(len(self.R)):
            bounds=self.R_bounds[R]
            # set the free volume for the cost function
            vol=(bounds[1]-bounds[0])*(bounds[3]-bounds[2])
            self.freevol[i]=vol*(sys.float_info.epsilon+valid[i])/(sys.float_info.epsilon+valid[i]+invalid[i])

    def accel(self):
        """
        Generate small set of possible accelerations

        Inputs:
        v = current velocity
        
        Outputs:
        list of accelerations [[u1,u2],..]
        """

        u = np.zeros((5,8))
        u_for = np.random.uniform(-1,1,size=(5,2))*self.UKF.u_scale[0]
        if self.place==2:
            u_down = np.random.normal(0.2262,size=5)
        else:
            #  u_down = np.random.normal(0.2262,.1,size=5)
            u_down = np.random.normal(0.24,.1,size=5)
        u_down = np.clip(u_down,-1,1)
        u_down = [self.UKF.u_scale[4]*x for x in u_down]

        for i in range(5):
            u[i,0:2]=max(-.5,u_for[i,0])
            u[i,2:4]=u_for[i,1]*.5
            u[i,4:]=u_down[i]
        return u

    def check_valid(self,point,x_dim,y_dim,z_dim):
        """
        Check if the sampled point is valid

        Inputs:
        point = the sampled point [x,xdot,y,ydot,z,zdot,
                                theta,theatdot,phi,phidot,
                                psi,psidot]
        x_dim = [min x, max x]
        y_dim = [min y, max y]
        theta_dim = [min theta, max theta]
        v_dim = [min v, max v]
        phi_dim = [min phi, max phi]

        Outputs:
        0 if it isn't valid, 1 if it is
        """

        # checking the bounds
        if (point[0]<x_dim[0]) or (point[0]>x_dim[1]) or \
                (point[2]<y_dim[0]) or (point[2]>y_dim[1]) or \
                (point[4]<0) or (point[4]>z_dim[1]):
                #  (point[4]<z_dim[0]) or (point[4]>z_dim[1]):
                    return 0
        #  # the c_obs needs to be indexed by theta values
        #  idx=(np.abs(self.thetas-point[2])).argmin()
        #  obstacles=self.c_obs[idx,:,:]
        #  # checking obstacle collision
        #  for obstacle in obstacles:
        #      if (point[0]>obstacle[0]) and (point[0]<obstacle[1]) \
        #              and (point[1]>obstacle[2]) and (point[1]<obstacle[3]):
        #          return 0
        return 1

    def check_end(self,point,end):
        """
        Check if the point satisfies the end

        Inputs:
        point = the sampled point [x,xdot,y,ydot,z,zdot,
                                theta,theatdot,phi,phidot,
                                psi,psidot]
        end = ending area [[x_min,x_max],[xdot_min,xdot_max],[y_min,y_max]...]

        Outputs:
        1 if it satisfies, 0 otherwise
        """

        if (point[0]>end[0][0]) and (point[0]<end[0][1]) and \
                (point[2]>end[2][0]) and (point[2]<end[2][1]) and \
                (point[4]>end[4][0]) and (point[4]<end[4][1]):
                    return 1
        else:
            return 0

    def c_space_obs(self,obs):
        """
        Creates the c-space for the car problem

        Inputs:
        obs = list of obstacles in rectangles [[[x1,y1],[x2,y2]...],[obs2]]
        """

        # all angles the car could be at
        self.thetas=np.linspace(0,np.pi,500)
        self.c_obs=np.zeros((len(self.thetas),len(obs),4))
        i=0
        # angles of the obstacles
        angs1=[0,np.pi/2,np.pi,(3*np.pi)/2,2*np.pi]
        for theta in self.thetas:
            # rotate robot
            new_robot=[0]*4
            new_robot[0]=[0,0]
            new_robot[1]=[(self.L)*np.cos(theta),(self.L)*np.sin(theta)]
            new_robot[2]=[np.sqrt(self.L**2+(self.L/2)**2)*np.cos(theta+np.arctan(.5)),np.sqrt(self.L**2+(self.L/2)**2)*np.sin(theta+np.arctan(.5))]
            new_robot[3]=[(self.L/2)*np.cos(theta+np.pi/2),(self.L/2)*np.sin(theta+np.pi/2)]
            min_point=None
            min_y=None
            count=0
            # re-orient the robot points
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
            # angles for the robot
            angs2=[theta+x for x in angs1]
            j=0
            # compute for all obstacles in the space
            for obstacle in obs:
                obstacle.append(obstacle[0])
                V=np.array(obstacle)
                k=0
                l=0
                points=[]
                x_points=[]
                y_points=[]

                # run the minkowski sum
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
                j+=1
            i+=1

    def plot_path(self,obs,x_dim,y_dim,z_dim,start,end,demo=False,points=False,path=False,high=False):
        V=np.array(self.T)
        #  print(len(V))
        end_mid=[np.mean(end[0]),np.mean(end[1]),np.mean(end[2]),np.mean(end[3]),
                np.mean(end[4]),np.mean(end[5]),np.mean(end[6]),np.mean(end[7]),
                np.mean(end[8]),np.mean(end[9]),np.mean(end[10]),np.mean(end[11])]
        fig=plt.figure()
        ax=fig.add_subplot(111,projection='3d')
        if points:
            ax.scatter(V[:,0],V[:,2],V[:,4])
        ax.scatter(start[0],start[2],start[4],marker='*',color='green')
        ax.scatter(end_mid[0],end_mid[2],end_mid[4],marker='*',color='red')
        if high:
            for cell in self.high_path:
                bounds=self.R_bounds[cell]
                to_graph=[np.mean(bounds[0:2]),np.mean(bounds[2:4]),np.mean(bounds[4:6])]
                ax.scatter(to_graph[0],to_graph[1],to_graph[2],color='C1')

        if not demo:
            y_top=np.linspace(3,7,100)
            z_side=np.linspace(2,7,100)
            x=3.5*np.ones(100)
            y_side1=3*np.ones(100)
            y_side2=7*np.ones(100)
            z_top=2*np.ones(100)
            ax.plot(x,y_top,z_top,label='Gate',c='k')
            ax.plot(x,y_side1,z_side,c='k')
            ax.plot(x,y_side2,z_side,c='k')
            ax.scatter(15,5,5,s=75,marker='s',c='w')

        x=np.linspace(x_dim[0],x_dim[1])
        y=np.linspace(y_dim[0],y_dim[1])
        X,Y=np.meshgrid(x,y)
        Z=0.5*np.ones((len(X),len(Y)))
        ax.plot_surface(X,Y,Z,color='lightskyblue',alpha=0.25)

        ax.set_xlim(x_dim)
        ax.set_ylim(y_dim)
        ax.set_zlim(z_dim)
        ax.invert_zaxis()
        ax.set_facecolor('lightskyblue')
        if (self.path) and (path):
            for i in range(1,len(self.path)):
                ax.plot([self.path[i-1][0],self.path[i][0]],[self.path[i-1][2],self.path[i][2]],[self.path[i-1][4],self.path[i][4]],linewidth=3,color='C1')

        plt.show()

if __name__ == '__main__':
    plan=SyCLoP()
    demo=False
    if demo:
        start=[0.1,0,5,0,5,0,0,0,0,0,0,0]
        end=[[14,18],[-100,100],[1,8],[-100,100],[4,5],[-100,100],[-100,100],[-100,100],[-100,100],[-100,100],[-100,100],[-100,100]]
        x_dim=[-2,18]
        y_dim=[-2,12]
        z_dim=[-2,8]
        obs=[[]]
        plan.dt=1.5
        plan.solve(start,end,x_dim,y_dim,z_dim,obs)
        plan.plot_path(obs,x_dim,y_dim,z_dim,start,end,demo=True,points=True)
        plan.plot_path(obs,x_dim,y_dim,z_dim,start,end,demo=True,path=True)
    else:
        x_dim=[-2,18]
        y_dim=[-2,12]
        z_dim=[-2,8]
        obs=[[[3,0],[5,0],[5,2],[3,2]],
                [[7,3],[9,3],[9,5],[7,5]],
                [[1,4],[4,4],[4,6],[1,6]],
                [[5,7],[6,7],[6,10],[5,10]]]
        plan.dt=1.5
        start=[0.1,0,5,0,5,0,0,0,0,0,0,0]
        # add key points to hit
        keys=[[[4,5],[3,7],[2,7]],[[14,15],[4,6],[4,6]],[[4,18],[-2,8],[-1,1]]]
        total=[]
        all_points=[]
        place=0
        for key in keys:
            end=[key[0],[-100,100],key[1],[-100,100],key[2],[-100,100],[-100,100],[-100,100],[-100,100],[-100,100],[-100,100],[-100,100]]
            plan.solve(start,end,x_dim,y_dim,z_dim,obs,place,task=True)
            #  plan.plot_path(obs,x_dim,y_dim,z_dim,start,end,points=True)
            start=plan.path[-1]
            start[6:]=[0]*6
            total.extend(plan.path)
            all_points.extend(plan.T)
            place+=1
        plan.path=total
        plan.T=all_points
        plan.plot_path(obs,x_dim,y_dim,z_dim,start,end,points=True)
        plan.plot_path(obs,x_dim,y_dim,z_dim,start,end,path=True)
