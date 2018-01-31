'''
***************************************************
File: assignment1.py

CSCI 5822 assignment 1 code

***************************************************
'''

from __future__ import division
import math
import matplotlib.pyplot as plt
import numpy as np

__author__="Jeremy Muesing"
__email__="jeremy.muesing@colorado.edu"
__version__="1.0.0"

def calc_prior(s,sigma):
    P=math.exp(-((s/sigma)+(s/sigma)))
    return P

def uninformed_prior(s):
    if s==0:
        s=1e-4
    P=1/(math.pow(s,2))
    return P

def calc_likelihood(s,observations):
    h=math.pow(s,2)
    n=0
    for observation in observations:
        if (-s/2<observation[0]<s/2) and (-s/2<observation[1]<s/2):
            n=n+1 
    if n>0:
        likelihood=1/(math.pow(h,n))
    else:
        likelihood=0
    return likelihood

def bayes(prior,likelihood):
    post=likelihood*prior
    return post

def bar_plot(x,P,sigma,task):
    plt.bar(x,P)
    plt.ylabel('p(s)')
    plt.xlabel('s')
    if task==2:
        title_string='P(H|X) $\sigma$ = %d' % sigma
    elif task==1:
        title_string='Expected Size Prior $\sigma$ = %d' % sigma
    plt.title(title_string)
    plt.show()

def contour_plot(P):
    Z=np.ones([21,21])*1e-4
    for i in range(len(P)):
        if P[i]>0:
            Z[10-i:10+i,10-i]=P[i]
            Z[10-i,10-i:10+i]=P[i]
            Z[10+i,10-i:10+i+1]=P[i]
            Z[10-i:10+i,10+i]=P[i]
    Z=np.log(Z)
    x=range(-10,11)
    plt.contourf(x,x,Z,corner_mask=False)
    plt.title('Contour plot log(P(y $\epsilon$ concept|X))')
    plt.show()
   
if __name__ == "__main__":
    # task 1
    P=[]
    x=[]
    sigma=6
    for i in range(0,21,2):
        x.append(i)
        P.append(calc_prior(i,sigma))
    P=[i/sum(P) for i in P]
    bar_plot(x,P,sigma,1)

    P=[]
    x=[]
    sigma=12
    for i in range(0,21,2):
        x.append(i)
        P.append(calc_prior(i,sigma))
    P=[i/sum(P) for i in P]
    bar_plot(x,P,sigma,1)

    # task 2
    P=[]
    x=[]
    sigma=12
    observations=[[1.5,0.5]]
    for i in range(0,21,2):
        x.append(i)
        prior=calc_prior(i,sigma)
        likelihood=calc_likelihood(i,observations)
        P.append(bayes(prior,likelihood))
    P=[i/sum(P) for i in P]
    bar_plot(x,P,sigma,2)

    # task 3
    P=[]
    sigma=10
    observations=[[1.5,0.5]]
    for i in range(0,21,2):
        prior=calc_prior(i,sigma)
        likelihood=calc_likelihood(i,observations)
        P.append(bayes(prior,likelihood))
    P=[i/sum(P) for i in P]
    contour_plot(P)

    # task 4
    P=[]
    sigma=10
    observations=[[4.5,2.5]]
    for i in range(0,21,2):
        prior=calc_prior(i,sigma)
        likelihood=calc_likelihood(i,observations)
        P.append(bayes(prior,likelihood))
    P=[i/sum(P) for i in P]
    contour_plot(P)

    # task 5
    P=[]
    sigma=30
    observations=[[2.2,-.2]]
    for i in range(0,21,2):
        prior=calc_prior(i,sigma)
        likelihood=calc_likelihood(i,observations)
        P.append(bayes(prior,likelihood))
    P=[i/sum(P) for i in P]
    contour_plot(P)

    P=[]
    observations=[[2.2,-.2],[.5,.5]]
    for i in range(0,21,2):
        prior=calc_prior(i,sigma)
        likelihood=calc_likelihood(i,observations)
        P.append(bayes(prior,likelihood))
    P=[i/sum(P) for i in P]
    contour_plot(P)

    P=[]
    observations=[[2.2,-.2],[.5,.5],[1.5,1]]
    for i in range(0,21,2):
        prior=calc_prior(i,sigma)
        likelihood=calc_likelihood(i,observations)
        P.append(bayes(prior,likelihood))
    P=[i/sum(P) for i in P]
    contour_plot(P)

    # task 6
    P=[]
    observations=[[2.2,-.2]]
    for i in range(0,21,2):
        prior=uninformed_prior(i)
        likelihood=calc_likelihood(i,observations)
        P.append(bayes(prior,likelihood))
    P=[i/sum(P) for i in P]
    contour_plot(P)

    P=[]
    observations=[[2.2,-.2],[.5,.5]]
    for i in range(0,21,2):
        prior=uninformed_prior(i)
        likelihood=calc_likelihood(i,observations)
        P.append(bayes(prior,likelihood))
    P=[i/sum(P) for i in P]
    contour_plot(P)

    P=[]
    observations=[[2.2,-.2],[.5,.5],[1.5,1]]
    for i in range(0,21,2):
        prior=uninformed_prior(i)
        likelihood=calc_likelihood(i,observations)
        P.append(bayes(prior,likelihood))
    P=[i/sum(P) for i in P]
    contour_plot(P)
