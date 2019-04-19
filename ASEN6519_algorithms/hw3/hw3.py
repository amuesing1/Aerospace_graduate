from __future__ import division

import numpy as np
import matplotlib.pyplot as plt
import copy
import sys

class MDP():
    def __init__(self):
        self.rows=15
        self.cols=23
        self.pi=np.zeros((self.rows,self.cols))
        self.R=np.ones((self.rows,self.cols))
        self.R*=-1
        obst=-200
        obstXY = np.array([[ 2, 3 ],[
                   3, 2 ],[
                   5, 1],[
                   5, 13],[
                   4, 4],[
                   7, 7],[
                   10, 10],[
                   11, 12],[
                   14, 20],[
                   10, 15 ],[
                   13, 5],[
                   13, 12],[
                   10, 5],[
                   8, 12],[
                   15, 12],[
                   3, 16],[
                   4, 17],[
                   5, 18],[
                   6, 19],[
                   2,  18]])
        obstXY-=1
        self.terminate_points=obstXY
        for point in obstXY:
            self.R[point[0],point[1]]=obst
        self.hazzards=obstXY
        self.R[6,14]=100
        #extra points
        self.R[11,8]=100
        self.R[5,3]=100
        self.R[11,20]=100
        self.goals=np.array([[6,14],[11,8],[5,3],[11,20]])
        #  self.terminate_points=self.goals.tolist()
        self.terminat_points=np.append(self.terminate_points,[[6,14],[11,8],[5,3],[11,20]],axis=0)
        self.terminate_points=self.terminate_points.tolist()
        self.r=np.zeros((self.rows,self.cols,4))
        for i in range(4):
            self.r[:,:,i]=self.R
        for x in range(self.rows):
            for y in range(self.cols):
                #up
                if x>0:
                    self.r[x,y,0]=self.R[x-1,y]
                else:
                    self.r[x,y,0]=-1
                #right
                if (y+1)<self.cols:
                    self.r[x,y,1]=self.R[x,y+1]
                else:
                    self.r[x,y,1]=-1
                #down
                if (x+1)<self.rows:
                    self.r[x,y,2]=self.R[x+1,y]
                else:
                    self.r[x,y,2]=-1
                #left
                if y>0:
                    self.r[x,y,3]=self.R[x,y-1]
                else:
                    self.r[x,y,3]=-1
        for point in obstXY:
            self.r[point[0],point[1],:]=obst
        intend=0.75
        side=0.125
        self.gamma=0.95
        self.T=np.zeros((self.rows*self.cols,self.rows*self.cols,4))
        for i in range(self.T.shape[0]):
            #UP
            if (i+1)>self.cols:
                self.T[i,i-self.cols,0]=intend
                self.T[i,i-self.cols,1]=side
                self.T[i,i-self.cols,3]=side
            #LEFT
            if not i%self.cols==0:
                self.T[i,i-1,0]=side
                self.T[i,i-1,2]=side
                self.T[i,i-1,3]=intend
            #RIGHT
            if not (i+1)%self.cols==0:
                self.T[i,i+1,0]=side
                self.T[i,i+1,1]=intend
                self.T[i,i+1,2]=side
            #DOWN
            if i<(self.rows*self.cols-self.cols):
                self.T[i,i+self.cols,1]=side
                self.T[i,i+self.cols,2]=intend
                self.T[i,i+self.cols,3]=side

        for i in range(self.T.shape[0]):
            for a in range(4):
                self.T[i,:,a]=self.T[i,:,a]/sum(self.T[i,:,a])

    def value_iteration(self):
        self.U=np.ones((self.rows,self.cols))
        self.U*=np.amin(self.R)
        self.U=self.U.reshape((1,self.U.shape[0]*self.U.shape[1]))
        self.U_final=copy.copy(self.U)
        self.U_final[0,0]-=1
        count=0
        while np.max(np.abs(self.U_final-self.U))>1e-5:
            count+=1
            self.U=copy.copy(self.U_final)
            for x in range(self.rows):
                for y in range(self.cols):
                    value=np.zeros(4)
                    for a in range(4):
                        value[a]=self.r[x,y,a]+np.sum(self.U*self.T[x*self.cols+y,:,a])
                    self.U_final[0,x*self.cols+y]=self.gamma*np.max(value)
            #  if count in [5,10,25,55,95,110]:
            #      self.graph_value()
        #  print count
        #  print self.U_final

    def find_policy(self,point):
        value=np.zeros(4)
        i=point[0]*self.cols+point[1]
        for a in range(4):
            value[a]=self.r[point[0],point[1],a]+np.sum(self.U_final*self.T[point[0]*self.cols+point[1],:,a])
        #fix corners
        #UP
        if not (i+1)>self.cols:
            value[0]-=1000
        #LEFT
        if i%self.cols==0:
            value[3]-=1000
        #RIGHT
        if (i+1)%self.cols==0:
            value[1]-=1000
        #DOWN
        if not i<(self.rows*self.cols-self.cols):
            value[2]-=1000
        u=np.argmax(value)
        return u

    def graph_value(self):
        plt.Figure()
        U_graph=self.U_final.reshape((self.rows,self.cols))
        for point in self.hazzards:
            U_graph[point[0],point[1]]=100
        plt.imshow(U_graph,origin='lower')
        width=0.2
        for x in range(self.rows):
            for y in range(self.cols):
                if self.pi[x,y]==0:
                    plt.arrow(y,x,0,-1,head_width=width)
                elif self.pi[x,y]==1:
                    plt.arrow(y,x,1,0,head_width=width)
                elif self.pi[x,y]==2:
                    plt.arrow(y,x,0,1,head_width=width)
                elif self.pi[x,y]==3:
                    plt.arrow(y,x,-1,0,head_width=width)

        #  init_points=[[2,14],[14,1],[13,15],[1,0],[2,21]]
        #  for point in init_points:
        #      plt.scatter(point[1],point[0],300,marker="x",color='C1')

        #  rewards=[[6,14],[11,8],[5,3],[11,20]]
        #  for point in rewards:
        #      plt.scatter(point[1],point[0],400,marker="X",color='C3')

        plt.gca().invert_yaxis()
        plt.tight_layout()
        plt.show()

    def MC_sim(self,init_pos):
        intend=0.75
        side=0.125
        reward=self.R[init_pos[0],init_pos[1]]
        current=copy.copy(init_pos)
        while (current not in self.terminate_points):
            allowed=[0,1,2,3]
            if current[0]==0:
                allowed.remove(0)
            if current[0]==(self.rows-1):
                allowed.remove(2)
            if current[1]==0:
                allowed.remove(3)
            if current[1]==(self.cols-1):
                allowed.remove(1)
            action=4
            while action not in allowed:
                if self.pi[current[0],current[1]]==0:
                    action=np.random.choice([0,1,3],p=[intend,side,side])
                elif self.pi[current[0],current[1]]==1:
                    action=np.random.choice([1,2,0],p=[intend,side,side])
                elif self.pi[current[0],current[1]]==2:
                    action=np.random.choice([2,1,3],p=[intend,side,side])
                elif self.pi[current[0],current[1]]==3:
                    action=np.random.choice([3,2,0],p=[intend,side,side])
            if action==0:
                current[0]=current[0]-1
            elif action==1:
                current[1]=current[1]+1
            elif action==2:
                current[0]=current[0]+1
            elif action==3:
                current[1]=current[1]-1

            reward+=self.R[current[0],current[1]]

        return current,reward

if __name__=="__main__":
    a=MDP()
    a.value_iteration()
    for x in range(a.rows):
        for y in range(a.cols):
            a.pi[x,y]=a.find_policy([x,y])
    init_points=[[2,14],[14,1],[13,15],[1,0],[2,21]]
    num_sims=10000
    for start in init_points:
        endings=[]
        rewards=[]
        for i in range(num_sims):
            end,reward=a.MC_sim(start)
            endings.append(end)
            rewards.append(reward)
        for j in a.goals.tolist():
            print endings.count(j)
        print start,np.mean(rewards)
    a.graph_value()
