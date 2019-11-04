import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import argparse
import sys
import copy
from tqdm import tqdm

class A_star():
    def construct_path(self,G,start,end,h):
        V=G[0]
        E=G[1]
        W=G[2]
        if len(E) is not len(W):
            print("Not a valid graph")
            sys.exit()
        if len(V) is not len(h):
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
                if f_score[V.index(node)]<f_min:
                    current=node
            if current==end:
                #reconstruct path
                total_path=[current]
                while current in came_from.keys():
                    current=came_from[current]
                    total_path.insert(0,current)
                print("We did it",total_path)
                return 0

            #  print(current,f_score,g_score,came_from)
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
                    came_from[neighbor]=current
                    g_score[V.index(neighbor)]=trial_g
                    f_score[V.index(neighbor)]=trial_g+h[V.index(neighbor)]

                    if neighbor not in open_set:
                        open_set.append(neighbor)
                count+=1
        print("Not possible")
        return 1


    def plot_path(self):
        pass

class PRM():
    def solve(self,n,r,obs,x_dim,y_dim,start,end):
        V=[start,end]
        V.append(self.sample_space(n,obs,x_dim,y_dim))

    def sample_space(self,n,obs,x_dim,y_dim):
        pass

    def local_path(self,r,obs):
        pass

    def plot_path(self,obs,x_dim,y_dim,start,end):
        pass


def problem1():
    # part a
    V=['start','A','B','C','D','E','F','G','H','I','J','K','L','end']
    E=[['start','A'],['start','B'],['start','C'],['A','D'],['A','E'],['A','F'],
            ['C','L'],['C','K'],['C','J'],['B','G'],['B','H'],
            ['B','I'],['E','end'],['K','end'],['G','end'],['I','end']]
    W=[1,1,1,1,1,3,1,1,1,4,1,2,3,2,3,3]
    h=[0,3,2,3,3,1,3,2,1,2,3,2,3,0]
    start='start'
    end='end'
    a=A_star()
    a.construct_path([V,E,W],start,end,h)
    h=[0 for x in h]
    a.construct_path([V,E,W],start,end,h)
    #  a.plot_path()

def problem2():
    pass

if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument("problem",help="which problem do you want to run? 1 or 2?",type=int)
    args=parser.parse_args()
    if args.problem==1:
        problem1()
    elif args.problem==2:
        problem2()
