import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys
import copy

class Bug():
    def __init__ (self,grid):
        self.grid=grid

    def check_boundary(self,x_move,y_move):
        end_spot=[self.loc[0]+x_move,self.loc[1]+y_move]
        cell_contents=self.grid[end_spot[0],end_spot[1]]
        if cell_contents==1:
            return 1
        else:
            return 0
        #check each neighbor cell if it's filled

    def get_distance(self):
        q_goal=np.where(self.grid==10)
        d=np.sqrt((q_goal[0]-self.loc[0])**2+(q_goal[1]-self.loc[1])**2)
        return d,q_goal[0]-self.loc[0],q_goal[1]-self.loc[1]
        #check absolute distance to target from current postion

    def move(self):
        pass
        #check_boundary
        #if no, get distance
        #move in direction of target
        #if boundary
        #check where you came from
        #move in the direction of the boundary area and opposite where you came

    def boundary_follow(self,x_move,y_move):
        def check_sides():
            attempt=np.array([[0,1],[1,0],[-1,0],[0,-1]])
            sides=0
            for row in attempt:
                place=row+self.loc
                #  print(place,row[:2],self.loc)
                if self.grid[place[0],place[1]]==1:
                    sides=1
            return sides
        def check_corners():
            attempt=np.array([[1,1,0,1],[1,-1,1,0],[-1,1,-1,0],[-1,-1,0,-1]])
            #  print(self.loc)
            for row in attempt:
                place=row[:2]+self.loc
                #  print(place,row[:2],self.loc)
                if self.grid[place[0],place[1]]==1:
                    return row[2:]
        def turning(x_try,y_try):
            next_move=np.array([[1,0,0,1],[-1,0,0,-1],[0,1,-1,0],[0,-1,1,0]])
            for row in next_move:
                if (x_try==row[0]) and (y_try==row[1]):
                    return row[2:]
        new_move=turning(x_move,y_move)
        starting_point=copy.copy(self.loc)
        self.loc+=new_move
        while not np.array_equal(self.loc,starting_point):
            stop=False
            self.bug_path[self.loc[0],self.loc[1]]=1
            still_there=check_sides()
            while (still_there) and (not stop):
                self.loc+=new_move
                #  print(self.loc)
                self.bug_path[self.loc[0],self.loc[1]]=1
                still_there=check_sides()
                if np.array_equal(self.loc,starting_point):
                    stop=True

            if not stop:
                new_move=check_corners()
                self.loc+=new_move

        sys.exit()

    def bug1(self):
        self.loc=np.where(self.grid==5)
        self.bug_path=np.zeros(self.grid.shape)
        self.loc=np.array([int(self.loc[0]),int(self.loc[1])])
        self.bug_path[self.loc[0],self.loc[1]]=1
        #  print(self.loc)
        d,xd,yd=self.get_distance()
        #  print(d,xd,yd)
        x_move=int(xd/yd)
        y_move=int(yd/xd)
        while (self.grid[self.loc[0],self.loc[1]]!=10):
            boundary=self.check_boundary(x_move,0)
            if boundary:
                self.loc=self.boundary_follow(x_move,0)
            else:
                self.loc[0]+=x_move
                self.bug_path[self.loc[0],self.loc[1]]=1
            boundary=self.check_boundary(0,y_move)
            if boundary:
                self.loc=self.boundary_follow(0,y_move)
            else:
                self.loc[1]+=y_move
                self.bug_path[self.loc[0],self.loc[1]]=1
        print(self.loc)
        #implement bug 1 algorithm
        #return the path of the bug through the array

    def bug2(self):
        pass
        #implement bug1 algorithm
        #return the path of the bug through the grid

def make_grid_world(problem):
    if problem==1:
        sizing=10
        grid=np.zeros((15*sizing,15*sizing))
        grid[10*sizing,10*sizing]=10
        grid[0*sizing,0*sizing]=5
        grid[1*sizing:2*sizing,1*sizing:5*sizing]=1
        grid[3*sizing:4*sizing,4*sizing:12*sizing]=1
        grid[3*sizing:12*sizing,12*sizing:13*sizing]=1
        grid[12*sizing:5*sizing,13*sizing:13*sizing]=1
        grid[6*sizing:5*sizing,12*sizing:6*sizing]=1
    elif problem==2:
        sizing=100
        shift=6
        grid=np.zeros((42*sizing,42*sizing))
        grid[(35+shift)*sizing,(shift+0)*sizing]=10
        grid[(shift+0)*sizing,(shift+0)*sizing]=5
        grid[(-6+shift)*sizing:(-6+shift)*sizing,(25+shift)*sizing:(-5+shift)*sizing]=1
        grid[(-6+shift)*sizing:(5+shift)*sizing,(30+shift)*sizing:(6+shift)*sizing]=1
        grid[(-6+shift)*sizing:(-5+shift)*sizing,(-5+shift)*sizing:(5+shift)*sizing]=1
        grid[(4+shift)*sizing:(-5+shift)*sizing,(5+shift)*sizing:(1+shift)*sizing]=1
        grid[(9+shift)*sizing:(0+shift)*sizing,(10+shift)*sizing:(5+shift)*sizing]=1
        grid[(14+shift)*sizing:(-5+shift)*sizing,(15+shift)*sizing:(1+shift)*sizing]=1
        grid[(19+shift)*sizing:(0+shift)*sizing,(20+shift)*sizing:(5+shift)*sizing]=1
        grid[(24+shift)*sizing:(-5+shift)*sizing,(25+shift)*sizing:(1+shift)*sizing]=1
        grid[(29+shift)*sizing:(0+shift)*sizing,(30+shift)*sizing:(5+shift)*sizing]=1
    return grid
        #  print(np.where(grid==10))
        #  x=np.linspace(0,15,10000)
        #  y=np.linspace(0,15,10000)
        #  xx,yy=np.meshgrid(x,y)
        #  print(xx)

if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument("problem",help="which problem do you want to run? 9 or 10?",type=int)
    args=parser.parse_args()
    if args.problem==9:
        grid1=make_grid_world(1)
        #  grid2=make_grid_world(2)
        bug1=Bug(grid1)
        bug1.bug1()
    elif args.problem==10:
        print("not ready yet")
