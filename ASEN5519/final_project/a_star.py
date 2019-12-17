import sys
import copy
import numpy as np
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
                if not isinstance(current,int):
                    current=tuple(current)
                while current in came_from.keys():
                    if isinstance(current,int):
                        current=came_from[current]
                        total_path.insert(0,current)
                    else:
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
                    if isinstance(neighbor,int):
                        came_from[neighbor]=current
                    else:
                        came_from[tuple(neighbor)]=current
                    g_score[V.index(neighbor)]=trial_g
                    f_score[V.index(neighbor)]=trial_g+h[V.index(neighbor)]

                    if neighbor not in open_set:
                        open_set.append(neighbor)
                count+=1
        if debug:
            print("Not possible")
        return 1

    def construct_path_fast(self,G,start,end,h,debug=False):
        V=G[0]
        E=G[1]
        W=G[2]
        if len(E)!=len(W):
            print("Not a valid graph")
            sys.exit()
        if len(V)!=len(h):
            print("Not a valid graph")
            sys.exit()

        count=0
        V_dict={}
        for v in V:
            if isinstance(v,str):
                V_dict[v]=count
            elif not isinstance(v,int):
                if len(v)>1:
                    V_dict[tuple(v)]=count
                else:
                    V_dict[v]=count
            else:
                V_dict[v]=count
            count+=1
        count=0
        E_dict={}
        for e in E:
            E_dict[tuple(e)]=count
            count+=1
        E=[tuple(e) for e in E]
        open_set=[start]
        came_from={}
        g_score=[np.inf]*len(V)
        f_score=copy.copy(g_score)

        g_score[V.index(start)]=0
        f_score[V.index(start)]=h[V.index(start)]

        iterations=0
        while len(open_set)>0:
            #  print(open_set)
            f_min=np.inf
            for node in open_set:
                #  print(f_score[V.index(node)],node)
                if f_score[V_dict[node]]<f_min:
                    current=node
                    f_min=f_score[V_dict[node]]
            if current==end:
                #reconstruct path
                total_path=[current]
                #  print(current,came_from.keys())
                if isinstance(current,str):
                    pass
                elif not isinstance(current,int):
                    current=tuple(current)
                while current in came_from.keys():
                    #  pdb.set_trace()
                    if isinstance(current,int):
                        current=came_from[current]
                        total_path.insert(0,current)
                    elif isinstance(current,str):
                        current=came_from[current]
                        total_path.insert(0,current)
                    else:
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
                    connections.append(E_dict[edge])
                elif edge[1]==current:
                    neighbors.append(edge[0])
                    connections.append(E_dict[edge])
            count=0
            #  pdb.set_trace()
            for neighbor in neighbors:
                trial_g=g_score[V_dict[current]]+W[connections[count]]
                if trial_g<g_score[V_dict[neighbor]]:
                    if isinstance(neighbor,int):
                        came_from[neighbor]=current
                    elif isinstance(neighbor,str):
                        came_from[neighbor]=current
                    else:
                        came_from[tuple(neighbor)]=current
                    g_score[V_dict[neighbor]]=trial_g
                    f_score[V_dict[neighbor]]=trial_g+h[V_dict[neighbor]]

                    if neighbor not in open_set:
                        open_set.append(neighbor)
                count+=1
        if debug:
            print("Not possible")
        return 1

if __name__ == '__main__':
    V=['s','A','B','C','D','E','F','G','H','I','J','K','L','e']
    E=[['s','A'],['s','B'],['s','C'],['A','D'],['A','E'],['A','F'],
            ['C','L'],['C','K'],['C','J'],['B','G'],['B','H'],
            ['B','I'],['E','e'],['K','e'],['G','e'],['I','e']]
    W=[1,1,1,1,1,3,1,1,1,4,1,2,3,2,3,3]
    h=[0,3,2,3,3,1,3,2,1,2,3,2,3,0]
    start='s'
    end='e'
    a=A_star()
    # A star version
    start_t=time.time()
    a.construct_path([V,E,W],start,end,h,debug=True)
    print(time.time()-start_t)
    start_t=time.time()
    a.construct_path_fast([V,E,W],start,end,h,debug=True)
    print(time.time()-start_t)
    # dijkstras
    #  h=[0 for x in h]
    #  a.construct_path([V,E,W],start,end,h,debug=True)
