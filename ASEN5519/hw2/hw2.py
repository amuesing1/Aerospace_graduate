import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import argparse
import sys
import copy

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

    def bushfire(self):
        cells=np.where(self.obstacle_grid==1)
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

    def gradient(self,point,goal,xi,d_star,eta,Q):
        d=np.sqrt((goal[0]-point[0])**2+(goal[1]-point[1])**2)
        D=self.obstacle_grid[point[0],point[1]]
        x=point[0]
        y=point[1]
        if d<=d_star:
            attract=xi*(self.grid[point[0],point[1]]-self.grid[goal[0],goal[1]])
        else:
            attract=(d_star*xi*(self.grid[point[0],point[1]]-self.grid[goal[0],goal[1]]))/d
        if D<=Q:
            repel=eta*(1/Q-1/D)
        else:
            repel=0
        #  print(attract,repel)
        U=attract+repel
        return U

    def move(self,a,e,goal_value):
        self.loc=np.where(self.grid==-1)
        self.path=np.zeros(self.grid.shape)
        self.path_graph=[]
        self.loc=np.array([int(self.loc[0]),int(self.loc[1])])
        self.path[self.loc[0],self.loc[1]]+=1
        self.path_graph.append(copy.copy(self.loc))
        goal=np.where(self.grid==goal_value)
        U=10
        #  while abs(U)>e:
        for i in range(80):
            U=self.gradient(self.loc,goal,1,goal_value,5,3)
            x_dist=goal[0]-self.loc[0]
            y_dist=goal[1]-self.loc[1]
            x_mag=(a*(-U/np.sqrt(x_dist**2+y_dist**2))*x_dist)
            y_mag=(a*(-U/np.sqrt(x_dist**2+y_dist**2))*y_dist)
            print(x_mag,y_mag,U)
            x_mag=round(float(x_mag))
            y_mag=round(float(y_mag))
            self.loc[0]+=x_mag
            self.loc[1]+=y_mag
            self.path[self.loc[0],self.loc[1]]+=1
            self.path_graph.append(copy.copy(self.loc))
            print(self.loc)
            #  sys.exit()


    def vector_field(self,goal_value):
        self.field=np.zeros(self.grid.shape)
        x_mag=np.zeros(self.grid.shape)
        y_mag=np.zeros(self.grid.shape)

        goal=np.where(self.grid==goal_value)
        xx, yy = np.meshgrid(range(self.grid.shape[0]), range(self.grid.shape[1]))
        for x in range(self.grid.shape[0]):
            for y in range(self.grid.shape[1]):
                self.field[x,y]=self.gradient([x,y],goal,1,10,5,3)
                x_dist=goal[0]-x
                y_dist=goal[1]-y
                x_mag[x,y]=(-self.field[x,y]/np.sqrt(x_dist**2+y_dist**2))*x_dist
                y_mag[x,y]=(-self.field[x,y]/np.sqrt(x_dist**2+y_dist**2))*y_dist
        plt.figure()
        #  plt.contourf(xx.transpose(),yy.transpose(),self.field,100,cmap='viridis_r')
        plt.quiver(xx.transpose(),yy.transpose(),x_mag,y_mag)
        plt.axis('equal')
        plt.show()
                
        

def make_grid_world(problem,sizing,goal_value=None):
    #making the grid with our obstacles
    if problem==1:
        grid=np.zeros((15*sizing,15*sizing))
        grid[10*sizing,10*sizing]=2
        grid[0*sizing,0*sizing]=-1
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
        grid[10*sizing,(shift+0)*sizing]=goal_value
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
        goal_value=8
        grid3=make_grid_world(3,sizing,goal_value=goal_value)
        pot=Potential(grid3)
        pot.bushfire()
        #  pot.vector_field(goal_value)
        pot.move(1,0.25,goal_value)
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
