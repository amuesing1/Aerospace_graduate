import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import argparse
import sys
import copy
from tqdm import tqdm

np.set_printoptions(threshold=np.inf)

class WaveFront():
    def __init__ (self,grid):
        self.grid=grid

    def surround_cells(self,point):
        x_len=self.grid.shape[0]
        y_len=self.grid.shape[1]
        x=point[0]
        y=point[1]
        cells=[]
        if x-1 < 0:
            pass
        elif self.grid[x-1,y]>0:
            pass
        else:
            cells.append([x-1,y])
        if x+1==x_len:
            pass
        elif self.grid[x+1,y]>0:
            pass
        else:
            cells.append([x+1,y])
        if y-1 < 0:
            pass
        elif self.grid[x,y-1]>0:
            pass
        else:
            cells.append([x,y-1])
        if y+1==y_len:
            pass
        elif self.grid[x,y+1]>0:
            pass
        else:
            cells.append([x,y+1])
        return cells

    def surround_cells_move(self,point):
        x_len=self.grid.shape[0]
        y_len=self.grid.shape[1]
        x=point[0]
        y=point[1]
        max_point=self.grid.shape[0]*self.grid.shape[1]
        move=None
        if x-1 < 0:
            pass
        elif (self.grid[x-1,y]<max_point) and (self.grid[x-1,y] not in [0,1,-1]):
            max_point=self.grid[x-1,y]
            move=[x-1,y]
        if x+1==x_len:
            pass
        elif (self.grid[x+1,y]<max_point) and (self.grid[x+1,y] not in [0,1,-1]):
            max_point=self.grid[x+1,y]
            move=[x+1,y]
        if y-1 < 0:
            pass
        elif (self.grid[x,y-1]<max_point) and (self.grid[x,y-1] not in [0,1,-1]):
            max_point=self.grid[x,y-1]
            move=[x,y-1]
        if y+1==y_len:
            pass
        elif (self.grid[x,y+1]<max_point) and (self.grid[x,y+1] not in [0,1,-1]):
            max_point=self.grid[x,y+1]
            move=[x,y+1]
        return move

    def construct_path(self):
        goal=np.where(self.grid==2)
        goal=np.array([int(goal[0]),int(goal[1])])
        cells=[copy.copy(goal)]
        count=3
        stop=False
        while stop==False:
            next_round=[]
            for cell in cells:
                new_cells=self.surround_cells(cell)
                for new_cell in new_cells:
                    if self.grid[new_cell[0],new_cell[1]]==-1:
                        stop=True
                        break
                    self.grid[new_cell[0],new_cell[1]]=count
                    next_round.append(new_cell)
                if stop==True:
                    break
            cells=copy.copy(next_round)
            count+=1

    def move_to_path(self):
        self.loc=np.where(self.grid==-1)
        self.path=np.zeros(self.grid.shape)
        self.path_graph=[]
        self.loc=np.array([int(self.loc[0]),int(self.loc[1])])
        self.path[self.loc[0],self.loc[1]]+=1
        self.path_graph.append(copy.copy(self.loc))
        while self.grid[self.loc[0],self.loc[1]]!=2:
            self.loc=self.surround_cells_move(self.loc)
            self.path[self.loc[0],self.loc[1]]+=1
            self.path_graph.append(copy.copy(self.loc))

    def graph_path(self,sizing,problem_num):
        fig,ax=plt.subplots()
        #graph the goal
        for x in range(self.grid.shape[0]):
            for y in range(self.grid.shape[1]):
                if self.grid[x,y]==2:
                    ax.scatter(x,y,marker='*')
        #graph the obstacles
        if problem_num==1:
            ob1=Rectangle((1*sizing,1*sizing),1*sizing,4*sizing)
            ob2=Rectangle((3*sizing,4*sizing),1*sizing,8*sizing)
            ob3=Rectangle((3*sizing,12*sizing),9*sizing,1*sizing)
            ob4=Rectangle((12*sizing,5*sizing),1*sizing,8*sizing)
            ob5=Rectangle((6*sizing,5*sizing),6*sizing,1*sizing)
            ax.add_patch(ob1)
            ax.add_patch(ob2)
            ax.add_patch(ob3)
            ax.add_patch(ob4)
            ax.add_patch(ob5)
        elif problem_num==2:
            shift=7
            ob1=Rectangle(((-6+shift)*sizing,(-6+shift)*sizing),31*sizing,1*sizing)
            ob2=Rectangle(((-6+shift)*sizing,(5+shift)*sizing),36*sizing,1*sizing)
            ob3=Rectangle(((-6+shift)*sizing,(-5+shift)*sizing),1*sizing,10*sizing)
            ob4=Rectangle(((4+shift)*sizing,(-5+shift)*sizing),1*sizing,6*sizing)
            ob5=Rectangle(((9+shift)*sizing,(0+shift)*sizing),1*sizing,5*sizing)
            ob6=Rectangle(((14+shift)*sizing,(-5+shift)*sizing),1*sizing,6*sizing)
            ob7=Rectangle(((19+shift)*sizing,(0+shift)*sizing),1*sizing,5*sizing)
            ob8=Rectangle(((24+shift)*sizing,(-5+shift)*sizing),1*sizing,6*sizing)
            ob9=Rectangle(((29+shift)*sizing,(0+shift)*sizing),1*sizing,5*sizing)
            ax.add_patch(ob1)
            ax.add_patch(ob2)
            ax.add_patch(ob3)
            ax.add_patch(ob4)
            ax.add_patch(ob5)
            ax.add_patch(ob6)
            ax.add_patch(ob7)
            ax.add_patch(ob8)
            ax.add_patch(ob9)
        #graph the path taken
        path=np.array(self.path_graph)
        ax.plot(path[:,0],path[:,1],color='black')
        ax.axis('equal')
        ax.axis('off')
        plt.show()

class Potential():
    def __init__(self,grid):
        self.grid=grid
        self.obstacle_grid=copy.copy(grid)
        goal=np.where(self.grid==2)
        start=np.where(self.grid==-1)
        self.obstacle_grid[goal[0],goal[1]]=0
        self.obstacle_grid[start[0],start[1]]=0

    def surround_cells(self,point):
        x_len=self.obstacle_grid.shape[0]
        y_len=self.obstacle_grid.shape[1]
        x=point[0]
        y=point[1]
        cells=[]
        if x-1 < 0:
            pass
        elif self.obstacle_grid[x-1,y]>0:
            pass
        else:
            cells.append([x-1,y])
        if x+1==x_len:
            pass
        elif self.obstacle_grid[x+1,y]>0:
            pass
        else:
            cells.append([x+1,y])
        if y-1 < 0:
            pass
        elif self.obstacle_grid[x,y-1]>0:
            pass
        else:
            cells.append([x,y-1])
        if y+1==y_len:
            pass
        elif self.obstacle_grid[x,y+1]>0:
            pass
        else:
            cells.append([x,y+1])
        return cells

    def surround_cells_move(self,point):
        x_len=self.grid.shape[0]
        y_len=self.grid.shape[1]
        x=point[0]
        y=point[1]
        max_point=self.grid.shape[0]*self.grid.shape[1]
        move=None
        #  print(self.obstacle_grid[x-1,y],self.obstacle_grid[x+1,y],self.obstacle_grid[x,y-1],self.obstacle_grid[x,y+1])
        if x-1 < 0:
            pass
        elif (self.obstacle_grid[x-1,y]<max_point) and (self.obstacle_grid[x-1,y] not in [0,-1]):
            max_point=self.obstacle_grid[x-1,y]
            move=[x-1,y]
        if x+1==x_len:
            pass
        elif (self.obstacle_grid[x+1,y]<max_point) and (self.obstacle_grid[x+1,y] not in [0,-1]):
            max_point=self.obstacle_grid[x+1,y]
            move=[x+1,y]
        if y-1 < 0:
            pass
        elif (self.obstacle_grid[x,y-1]<max_point) and (self.obstacle_grid[x,y-1] not in [0,-1]):
            max_point=self.obstacle_grid[x,y-1]
            move=[x,y-1]
        if y+1==y_len:
            pass
        elif (self.obstacle_grid[x,y+1]<max_point) and (self.obstacle_grid[x,y+1] not in [0,-1]):
            max_point=self.obstacle_grid[x,y+1]
            move=[x,y+1]
        return move

    def brushfire(self):
        cells=np.where(self.obstacle_grid==1)
        cells=list(zip(cells[0],cells[1]))
        count=2
        stop=False
        while stop==False:
            next_round=[]
            for cell in cells:
                new_cells=self.surround_cells(cell)
                for new_cell in new_cells:
                    self.obstacle_grid[new_cell[0],new_cell[1]]=count
                    next_round.append(new_cell)
            cells=copy.copy(next_round)
            if not cells:
                stop=True
            count+=1
        #  print(self.obstacle_grid)

    def follow_brush(self,point):
        while self.obstacle_grid[point[0],point[1]]!=1:
            point=self.surround_cells_move(point)
            #  print(point)
            #  print(self.obstacle_grid[point[0],point[1]])
        return point

    def gradient(self,point,goal,xi,d_star,eta,Q):
        d=np.sqrt((goal[0]-point[0])**2+(goal[1]-point[1])**2)
        goal=[float(goal[0]),float(goal[1])]
        x=int(round(point[0]))
        y=int(round(point[1]))
        D=self.obstacle_grid[x,y]
        goal_vec=[point[0]-goal[0],point[1]-goal[1]]

        if d<=d_star:
            attract=[-xi*i for i in goal_vec]
        else:
            attract=[(-d_star*xi*i)/d for i in goal_vec]
        if D<=Q:
            d_point=self.follow_brush([x,y])
            d_vec=[point[0]-d_point[0],point[1]-d_point[1]]
            repel=[-eta*(1/Q-1/D)*(i*D) for i in d_vec]
        else:
            repel=[0,0]
        U=[attract[0]+repel[0],attract[1]+repel[1]]
        return U

    def move(self,a,e,xi,d_star,eta,Q):
        self.loc=np.where(self.grid==-1)
        self.path=np.zeros(self.grid.shape)
        self.path_graph=[]
        self.loc=np.array([float(self.loc[0]),float(self.loc[1])])
        #  self.path[self.loc[0],self.loc[1]]+=1
        self.path_graph.append(copy.copy(self.loc))
        goal=np.where(self.grid==2)
        U=[10,10]
        while np.sqrt(U[0]**2+U[1]**2)>e:
        #  for i in range(70000):
            U=self.gradient(self.loc,goal,1,10,20,5)
            #  print(U)
            self.loc[0]+=a*U[0]
            self.loc[1]+=a*U[1]
            #  print(self.loc)
            #  self.path[self.loc[0],self.loc[1]]+=1
            self.path_graph.append(copy.copy(self.loc))

    def vector_field(self):
        fig,ax=plt.subplots()
        self.field=np.zeros((self.grid.shape[0],self.grid.shape[1],2))
        x_mag=np.zeros(self.grid.shape)
        y_mag=np.zeros(self.grid.shape)

        goal=np.where(self.grid==2)
        xx, yy = np.meshgrid(range(self.grid.shape[0]), range(self.grid.shape[1]))
        for x in range(self.grid.shape[0]):
            for y in range(self.grid.shape[1]):
                self.field[x,y,:]=self.gradient([x,y],goal,1,10,50,5)
        plt.quiver(xx.transpose(),yy.transpose(),self.field[:,:,0],self.field[:,:,1])
        sizing=10
        shift=2
        ob1=Rectangle((int(3.5*sizing),int((0.5+shift)*sizing)),sizing,sizing)
        ob2=Rectangle((int(6.5*sizing),int((-1.5+shift)*sizing)),sizing,sizing)
        #  ax.add_patch(ob1)
        #  ax.add_patch(ob2)
        ax.axis('equal')
        plt.show()

    def graph_path(self,sizing,problem_num):
        fig,ax=plt.subplots()
        #graph the goal
        for x in range(self.grid.shape[0]):
            for y in range(self.grid.shape[1]):
                if self.grid[x,y]==2:
                    ax.scatter(x,y,marker='*')
        #graph the obstacles
        if problem_num==1:
            ob1=Rectangle((1*sizing,1*sizing),1*sizing,4*sizing)
            ob2=Rectangle((3*sizing,4*sizing),1*sizing,8*sizing)
            ob3=Rectangle((3*sizing,12*sizing),9*sizing,1*sizing)
            ob4=Rectangle((12*sizing,5*sizing),1*sizing,8*sizing)
            ob5=Rectangle((6*sizing,5*sizing),6*sizing,1*sizing)
            ax.add_patch(ob1)
            ax.add_patch(ob2)
            ax.add_patch(ob3)
            ax.add_patch(ob4)
            ax.add_patch(ob5)
        elif problem_num==2:
            shift=7
            ob1=Rectangle(((-6+shift)*sizing,(-6+shift)*sizing),31*sizing,1*sizing)
            ob2=Rectangle(((-6+shift)*sizing,(5+shift)*sizing),36*sizing,1*sizing)
            ob3=Rectangle(((-6+shift)*sizing,(-5+shift)*sizing),1*sizing,10*sizing)
            ob4=Rectangle(((4+shift)*sizing,(-5+shift)*sizing),1*sizing,6*sizing)
            ob5=Rectangle(((9+shift)*sizing,(0+shift)*sizing),1*sizing,5*sizing)
            ob6=Rectangle(((14+shift)*sizing,(-5+shift)*sizing),1*sizing,6*sizing)
            ob7=Rectangle(((19+shift)*sizing,(0+shift)*sizing),1*sizing,5*sizing)
            ob8=Rectangle(((24+shift)*sizing,(-5+shift)*sizing),1*sizing,6*sizing)
            ob9=Rectangle(((29+shift)*sizing,(0+shift)*sizing),1*sizing,5*sizing)
            ax.add_patch(ob1)
            ax.add_patch(ob2)
            ax.add_patch(ob3)
            ax.add_patch(ob4)
            ax.add_patch(ob5)
            ax.add_patch(ob6)
            ax.add_patch(ob7)
            ax.add_patch(ob8)
            ax.add_patch(ob9)
        elif problem_num==3:
            shift=2
            ob1=Rectangle((int(3.5*sizing),int((0.5+shift)*sizing)),sizing,sizing)
            ob2=Rectangle((int(6.5*sizing),int((-1.5+shift)*sizing)),sizing,sizing)
            ax.add_patch(ob1)
            ax.add_patch(ob2)
        #graph the path taken
        path=np.array(self.path_graph)
        ax.plot(path[:,0],path[:,1],color='black')
        ax.axis('equal')
        ax.axis('off')
        plt.show()

class Arm():
    def kinematics(self,len1,len2,ang1,ang2):
        #rotation and tranlation matricies
        T1=np.array([[np.cos(ang1),-np.sin(ang1),0],
            [np.sin(ang1),np.cos(ang1),0],
            [0,0,1]])
        T2=np.array([[np.cos(ang2),-np.sin(ang2),len1],
            [np.sin(ang2),np.cos(ang2),0],
            [0,0,1]])
        T3=np.array([[1,0,len2],
            [0,1,0],
            [0,0,1]])
        #getting the positions of each part of the links for graphing
        link1_base=np.array([[0],[0],[1]])
        link1_top=np.matmul(T1,np.matmul(T2,link1_base))
        link2_base=copy.copy(link1_top)
        link2_top=np.matmul(T1,np.matmul(T2,np.matmul(T3,link1_base)))
        link1_top=link1_top.tolist()
        link2_top=link2_top.tolist()
        return [link1_top[0][0],link1_top[1][0],link2_top[0][0],link2_top[1][0]]

    def c_space(self,len1,len2,sizing):
        self.ang1_values=np.linspace(-2*np.pi,2*np.pi,sizing)
        self.ang2_values=np.linspace(-2*np.pi,2*np.pi,sizing)
        self.grid=np.zeros((sizing,sizing))
        self.reference=np.zeros((sizing,sizing,4))
        for i in tqdm(range(sizing),ncols=100):
            for j in range(sizing):
                self.reference[i,j,:]=self.kinematics(len1,len2,self.ang1_values[i],self.ang2_values[j])

    def set_goals(self,start,finish):
        idx1 = (np.abs(self.ang1_values - start[0])).argmin()
        idy1 = (np.abs(self.ang2_values - start[1])).argmin()
        idx2 = (np.abs(self.ang1_values - finish[0])).argmin()
        idy2 = (np.abs(self.ang2_values - finish[2])).argmin()
        self.grid[idx1,idy1]=-1
        self.grid[idx2,idy2]=2
        print(self.ang1_values[idx1],self.ang2_values[idy1])
        print(self.ang1_values[idx2],self.ang2_values[idy2])
        #  print(idx,idy)

    def inverse(self,len1,len2,xpos,ypos):
        if np.sqrt(xpos**2+ypos**2)>(len1+len2):
            print("Can't get there")
            return 0
        start=(1/(2*len1*len2))*((xpos**2+ypos**2)-(len1**2-len2**2))
        if start>1:
            while start>1:
                start-=1
        elif start<-1:
            while start<-1:
                start+=1
        theta2=np.arccos(start)
        theta1_1=np.arccos((1/(xpos**2+ypos**2))*(xpos*(len1+len2*np.cos(theta2))+ypos*len2*np.sqrt(1-np.cos(theta2)**2)))
        theta1_2=np.arccos((1/(xpos**2+ypos**2))*(xpos*(len1+len2*np.cos(theta2))-ypos*len2*np.sqrt(1-np.cos(theta2)**2)))
        return [theta2,theta1_1,theta1_2]

    def graph_arm(self,path):
        #take path_graph, convert to link points, graph most of them
        link1=np.zeros((len(path),2))
        link2=np.zeros((len(path),2))
        i=0
        for point in path:
            link1[i,:]=self.reference[point[0],point[1],0:2]
            link2[i,:]=self.reference[point[0],point[1],2:]
            i+=1

        #  print(link1)
        fig,ax=plt.subplots()
        for i in range(len(path)):
            ax.plot([0,link1[i,0]],[0,link1[i,1]],color='C0')
            ax.plot([link1[i,0],link2[i,0]],[link1[i,1],link2[i,1]],color='C1')
        ax.axis('equal')
        plt.show()

def make_grid_world(problem,sizing):
    #making the grid with our obstacles
    if problem==1:
        grid=np.zeros((15*sizing,15*sizing))
        grid[10*sizing,10*sizing]=2
        grid[1*sizing,0*sizing]=-1
        grid[1*sizing:2*sizing,1*sizing:5*sizing]=1
        grid[3*sizing:4*sizing,4*sizing:12*sizing]=1
        grid[3*sizing:12*sizing,12*sizing:13*sizing]=1
        grid[12*sizing:13*sizing,5*sizing:13*sizing]=1
        grid[6*sizing:12*sizing,5*sizing:6*sizing]=1
    elif problem==2:
        shift=7
        grid=np.zeros((43*sizing,15*sizing))
        grid[(35+shift)*sizing,(shift+0)*sizing]=2
        grid[(shift+0)*sizing,(shift+0)*sizing]=-1
        grid[(-6+shift)*sizing:(25+shift)*sizing,(-6+shift)*sizing:(-5+shift)*sizing]=1
        grid[(-6+shift)*sizing:(30+shift)*sizing,(5+shift)*sizing:(6+shift)*sizing]=1
        grid[(-6+shift)*sizing:(-5+shift)*sizing,(-5+shift)*sizing:(5+shift)*sizing]=1
        grid[(4+shift)*sizing:(5+shift)*sizing,(-5+shift)*sizing:(1+shift)*sizing]=1
        grid[(9+shift)*sizing:(10+shift)*sizing,(0+shift)*sizing:(5+shift)*sizing]=1
        grid[(14+shift)*sizing:(15+shift)*sizing,(-5+shift)*sizing:(1+shift)*sizing]=1
        grid[(19+shift)*sizing:(20+shift)*sizing,(0+shift)*sizing:(5+shift)*sizing]=1
        grid[(24+shift)*sizing:(25+shift)*sizing,(-5+shift)*sizing:(1+shift)*sizing]=1
        grid[(29+shift)*sizing:(30+shift)*sizing,(0+shift)*sizing:(5+shift)*sizing]=1
    elif problem==3:
        shift=2
        grid=np.zeros((12*sizing,5*sizing))
        grid[10*sizing,(shift+0)*sizing]=2
        grid[0*sizing,(shift+0)*sizing]=-1
        grid[int(3.5*sizing):int(4.5*sizing),int((0.5+shift)*sizing):int((1.5+shift)*sizing)]=1
        grid[int(6.5*sizing):int(7.5*sizing),int((-1.5+shift)*sizing):int((-0.5+shift)*sizing)]=1
    return grid

if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument("problem",help="which problem do you want to run? 5, 6 or 7?",type=int)
    args=parser.parse_args()
    if args.problem==5:
        sizing=10
        a=0.1
        e=0.25
        xi=100
        d_star=10
        eta=30
        Q=5
        grid3=make_grid_world(1,sizing)
        pot=Potential(grid3)
        pot.brushfire()
        pot.vector_field()
        pot.move(a,e,xi,d_star,eta,Q)
        pot.graph_path(sizing,1)
        #  pot.graph_path(2,sizing)
    elif args.problem==6:
        sizing=4
        grid1=make_grid_world(1,sizing)
        grid2=make_grid_world(2,sizing)
        planner=WaveFront(grid1)
        planner.construct_path()
        planner.move_to_path()
        print('Path W1 Length:',np.sum(planner.path)/sizing)
        planner.graph_path(sizing,1)
        planner=WaveFront(grid2)
        planner.construct_path()
        planner.move_to_path()
        print('Path W2 Length:',np.sum(planner.path)/sizing)
        planner.graph_path(sizing,2)
    elif args.problem==7:
        sizing=750
        a=Arm()
        config_1=a.inverse(1,1,1.4,-0.3)
        config_2=a.inverse(1,1,1,0.5)
        print(config_1,config_2)
        a.c_space(1,1,sizing)
        a.set_goals(config_1,config_2)
        planner=WaveFront(a.grid)
        planner.construct_path()
        planner.move_to_path()
        #  planner.graph_path(sizing,3)
        a.graph_arm(planner.path_graph)
