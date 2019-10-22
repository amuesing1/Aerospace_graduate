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

    def graph_path(self,sizing,path=True):
        fig,ax=plt.subplots()
        #graph the goal
        for x in range(self.grid.shape[0]):
            for y in range(self.grid.shape[1]):
                if self.grid[x,y]==2:
                    ax.scatter(x,y,marker='*')
        points=np.where(self.grid==1)
        ax.scatter(points[0],points[1],marker='s',color='C0')
        #graph the path taken
        if path:
            path=np.array(self.path_graph)
            ax.plot(path[:,0],path[:,1],color='black')
        ax.axis('equal')
        #  ax.axis('off')
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
        self.ang1_values=np.linspace(-np.pi,np.pi,sizing)
        self.ang2_values=np.linspace(-np.pi,np.pi,sizing)
        points=np.linspace(-(len1+len2),len1+len2,sizing)
        self.grid=np.zeros((sizing,sizing))
        self.reference=np.zeros((sizing,sizing,4))
        for i in tqdm(range(sizing),ncols=100):
            for j in range(sizing):
                position=self.kinematics(len1,len2,self.ang1_values[i],self.ang2_values[j])
                #make ten more points on the arm to check
                arm1_points_x=np.linspace(0,position[0],5)
                arm1_points_y=np.linspace(0,position[1],5)
                arm2_points_x=np.linspace(position[0],position[2],5)
                arm2_points_y=np.linspace(position[1],position[3],5)
                idx1=[]
                idy1=[]
                idx2=[]
                idy2=[]
                for k in range(len(arm1_points_x)):
                    idx1.append((np.abs(points - arm1_points_x[k])).argmin())
                    idy1.append((np.abs(points - arm1_points_y[k])).argmin())
                    idx2.append((np.abs(points - arm2_points_x[k])).argmin())
                    idy2.append((np.abs(points - arm2_points_y[k])).argmin())
                for k in range(len(idx1)):
                    if self.grid[i,j]==1:
                        break
                    if (self.grid_pos[idx1[k],idy1[k]]==1) or (self.grid_pos[idx2[k],idy2[k]]==1):
                        self.grid[i,j]=1
                self.reference[i,j,:]=position

    def set_goals(self,start,finish):
        idx1 = (np.abs(self.ang1_values - start[0])).argmin()
        idy1 = (np.abs(self.ang2_values - start[1])).argmin()
        idx2 = (np.abs(self.ang1_values - finish[0])).argmin()
        idy2 = (np.abs(self.ang2_values - finish[1])).argmin()
        self.grid[idx1,idy1]=-1
        self.grid[idx2,idy2]=2

    def inverse(self,len1,len2,xpos,ypos):
        if np.sqrt(xpos**2+ypos**2)>(len1+len2):
            print("Can't get there")
            return 0
        x=(xpos**2+ypos**2-len1**2-len2**2)/(2*len1*len2)
        y=np.sqrt(1-x**2)
        theta2=np.arctan2([y,-y],[x,x])
        theta1=np.arctan2(ypos,xpos)-np.arctan2(len2*np.sin(theta2),len1+len2*np.cos(theta2))
        return [theta1[0],theta2[0],theta1[1],theta2[1]]

    def make_obstacles(self,sizing,len1,len2,obstacles):
        #obs is a list of obs
        #check the length of each obs (3 or 4)
        #construct obs like the make grid world
        self.grid_pos=np.zeros((sizing,sizing))
        points=np.linspace(-(len1+len2),len1+len2,sizing)
        for obs in obstacles:
            idx=[]
            idy=[]
            for vert in obs:
                #  print(np.abs(points-vert[0]))
                idx.append(np.abs(points - vert[0]).argmin())
                idy.append(np.abs(points - vert[1]).argmin())
            if len(obs)==4:
                self.grid_pos[idx[0]:idx[2],idy[0]:idy[2]]=1
            elif len(obs)==3:
                x_to_search=range(min(idx),max(idx))
                y_to_search=range(min(idy),max(idy))
                slope1=(idy[1]-idy[0])/(idx[1]-idx[0])
                intercept1=idy[0]-slope1*idx[0]
                slope2=(idy[2]-idy[1])/(idx[2]-idx[1])
                intercept2=idy[1]-slope2*idx[1]
                slope3=(idy[0]-idy[2])/(idx[0]-idx[2])
                intercept3=idy[2]-slope3*idx[2]
                for x in x_to_search:
                    for y in y_to_search:
                        if (y-slope1*x-intercept1<=0) and (y-slope2*x-intercept2<=0) and (y-slope3*x-intercept3>=0):
                            self.grid_pos[x,y]=1
        #  sys.exit()


    def graph_workspace(self,obstacles,path=None):
        #take path_graph, convert to link points, graph most of them
        fig,ax=plt.subplots()
        if path:
            link1=np.zeros((len(path),2))
            link2=np.zeros((len(path),2))
            i=0
            for point in path:
                link1[i,:]=self.reference[point[0],point[1],0:2]
                link2[i,:]=self.reference[point[0],point[1],2:]
                i+=1

            #  print(link1)
            for i in range(len(path))[::5]:
                ax.set_xlim(-3,3)
                ax.set_ylim(-2,2.5)
                ax.plot([0,link1[i,0]],[0,link1[i,1]],color='C0')
                ax.plot([link1[i,0],link2[i,0]],[link1[i,1],link2[i,1]],color='C1')
                for obs in obstacles:
                    ob=plt.Polygon(obs)
                    ax.add_patch(ob)
                ax.axis('equal')
                plt.pause(0.05)
                plt.cla()


            for i in range(len(path))[::5]:
                ax.set_xlim(-3,3)
                ax.set_ylim(-2,2.5)
                ax.plot([0,link1[i,0]],[0,link1[i,1]],color='C0')
                ax.plot([link1[i,0],link2[i,0]],[link1[i,1],link2[i,1]],color='C1')

        for obs in obstacles:
            ob=plt.Polygon(obs)
            #  ob=Rectangle((obs[0][0],obs[0][1]),obs[2][0]-obs[0][0],obs[2][1]-obs[0][1])
            ax.add_patch(ob)
        ax.axis('equal')
        ax.set_xlim(-3,3)
        ax.set_ylim(-2,2.5)
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

def problem2():
    fig,ax=plt.subplots()
    V=np.array([[0.0,0.0],[1.0,2.0],[0.0,2.0]])
    thetas=np.linspace(0,2*np.pi,1000)
    #  thetas=[0]
    for theta in thetas:
        W=copy.copy(V)
        ang1=np.arctan(2)+theta
        ang2=np.deg2rad(90)+theta
        if ang1>2*np.pi:
            ang1-=2*np.pi
        if ang2>2*np.pi:
            ang2-=2*np.pi
        W[1]=[round(np.sqrt(5)*np.cos(ang1),5),round(np.sqrt(5)*np.sin(ang1),5)]
        W[2]=[round(2*np.cos(ang2),5),round(2*np.sin(ang2),5)]
        #  print(W)
        #  W=-V
        #find the lowest point,lower left if needed
        min_point=None
        min_y=None
        count=0
        for i in W:
            if min_y is not None:
                if i[1]<min_y:
                    min_y=i[1]
                    min_point=copy.copy(count)
                elif i[1]==min_y:
                    if i[0]<W[min_point][0]:
                        min_point=copy.copy(count)
            else:
                min_y=i[1]
                min_point=copy.copy(count)
            count+=1
        #  print(min_point)
        if min_point==1:
            W=np.array([W[1],W[2],W[0],W[1]])
        elif min_point==2:
            W=np.array([W[2],W[0],W[1],W[2]])
        else:
            W=np.array([W[0],W[1],W[2],W[0]])

        V=np.array([V[0],V[1],V[2],V[0]])
        #  print(W)

        #find the first side length
        angs1=[np.deg2rad(60),np.deg2rad(180),np.deg2rad(270),2*np.pi]
        side2=np.sqrt((W[0][0]-W[1][0])**2+(W[0][1]-W[1][1])**2)
        if W[0][1]==W[1][1]:
            angs2=[0]
        else:
            angs2=[np.arccos((W[1][0]-W[0][0])/(W[1][1]-W[0][1]))]
        if (side2-np.sqrt(5))<0.001:
            angs2.extend([angs2[0]+np.deg2rad(120),angs2[0]+np.deg2rad(120)+np.deg2rad(90),2*np.pi])
        elif side2==2:
            angs2.extend([angs2[0]+np.deg2rad(150),angs2[0]+np.deg2rad(150)+np.deg2rad(120),2*np.pi])
        elif side2==1:
            angs2.extend([angs2[0]+np.deg2rad(90),angs2[0]+np.deg2rad(90)+np.deg2rad(150),2*np.pi])
        #  print(theta,side2)
        #  print(angs1,angs2)
        i=0
        j=0
        points=[]
        while (i!=4) and (j!=4):
            x=V[i][0]+W[j][0]
            y=V[i][1]+W[j][1]
            #  print(x,y,i,j)
            points.append([x,y])
            if angs1[i]<angs2[j]:
                i+=1
            elif angs1[i]>angs2[j]:
                j+=1
            else:
                i+=1
                j+=1
        ob=plt.Polygon(points)
        ax.add_patch(ob)
    ob=plt.Polygon(V,color='C1')
    ax.add_patch(ob)
    ax.axis('equal')
    plt.show()


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument("problem",help="which problem do you want to run? 2, 5, 6 or 7?",type=int)
    args=parser.parse_args()
    if args.problem==2:
        problem2()
    elif args.problem==5:
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
        planner.graph_path(sizing)
        planner=WaveFront(grid2)
        planner.construct_path()
        planner.move_to_path()
        print('Path W2 Length:',np.sum(planner.path)/sizing)
        planner.graph_path(sizing)
    elif args.problem==7:
        method=input('Part a, b, or do you want to type a ton of points? ')
        sizing=500
        a=Arm()
        if method=='a':
            len1=1
            len2=1

            obstacles=[[[-.25,.25],[0,.75],[.25,.25]]]
            a.make_obstacles(sizing,len1,len2,obstacles)
            a.c_space(len1,len2,sizing)
            planner=WaveFront(a.grid)
            #  planner.construct_path()
            planner.graph_path(sizing,path=False)
            a.graph_workspace(obstacles)

            obstacles=[[[-.25,1.1],[-.25,2],[.25,2],[.25,1.1]],
                    [[-2,-2],[-2,-1.8],[2,-1.8],[2,-2]]]
            a.make_obstacles(sizing,len1,len2,obstacles)
            a.c_space(len1,len2,sizing)
            planner=WaveFront(a.grid)
            #  planner.construct_path()
            planner.graph_path(sizing,path=False)
            a.graph_workspace(obstacles)

            a=Arm()
            obstacles=[[[-.25,1.1],[-.25,2],[.25,2],[.25,1.1]],
                    [[-2,-.5],[-2,-.3],[2,-.3],[2,-.5]]]
            a.make_obstacles(sizing,len1,len2,obstacles)
            a.c_space(len1,len2,sizing)
            planner=WaveFront(a.grid)
            #  planner.construct_path()
            planner.graph_path(sizing,path=False)
            a.graph_workspace(obstacles)
        elif method=='b':
            len1=1
            len2=1
            config_1=a.inverse(len1,len2,2,0)
            config_2=a.inverse(len1,len2,-2,0)
            obstacles=[[[-.25,1.1],[-.25,2],[.25,2],[.25,1.1]],
                    [[-2,-.5],[-2,-.3],[2,-.3],[2,-.5]]]
            a.make_obstacles(sizing,len1,len2,obstacles)
            a.c_space(len1,len2,sizing)
            a.set_goals(config_1,config_2)
            planner=WaveFront(a.grid)
            planner.construct_path()
            planner.move_to_path()
            planner.graph_path(sizing)
            a.graph_workspace(obstacles,planner.path_graph)
        else:
            len1=float(input('Length of link 1: '))
            len2=float(input('Length of link 2: '))
            obstacles=[]
            num_obs=int(input('How many obstacles do you want? '))
            for i in range(num_obs):
                points=[]
                num_points=int(input('How many points on this polygon? '))
                if num_points>4:
                    print("Don't make this too complicated")
                    sys.exit()
                elif num_points<3:
                    print("Can't make a shape")
                    sys.exit()
                print("Enter points starting from the left most point and going clockwise. \nIf there are two most left points, chose the lower one.")
                for point in range(num_points):
                    x=float(input('X: '))
                    y=float(input('Y: '))
                    points.append([x,y])
                obstacles.append(points)
            print(obstacles)

