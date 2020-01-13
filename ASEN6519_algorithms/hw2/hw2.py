from __future__ import division

import numpy as np
import matplotlib.pyplot as plt
import time
import sys

def forward(priors,trans_probs,obs_like,obs,normalized=False):
    #  obs=np.transpose(obs)
    alphas=np.zeros((len(priors),len(obs)+1))
    alphas[:,0]=priors
    #  print obs.shape
    for i in range(1,len(obs)+1):
        for j in range(len(priors)):
            alphas[j,i]=obs_like[j][obs[i-1]]*sum(
                   alphas[state][i-1]*trans_probs[state][j] for state in range(len(priors)))
        if normalized:
            suma=sum([alphas[state][i] for state in range(len(priors))])
            for state in range(len(priors)):
                alphas[state][i]/=suma
    return alphas[:,1:]

def backward(priors,trans_probs,obs_like,obs):
    betas=np.zeros((len(priors),len(obs)))
    betas[:,-1]=1
    for i in range(len(obs)-2,-1,-1):
        for j in range(len(priors)):
            betas[j,i]=sum(betas[state][i+1]*trans_probs[j][state]
                    *obs_like[state][obs[i+1]] for state in range(len(priors)))
    return betas

def log_likelihood(priors,trans_probs,obs_like,obs):
    alphas=forward(priors,trans_probs,obs_like,obs)
    return np.log(sum(alphas[:,-1]))

def HMMbaumwelch(priors, obs, tol=1e-4, maxIt = 1000):
    '''
    compute maximum likehood estimate using Expectation-Maximization
    iterations
    in:   obs = vector of observations [time,features,num_obs]
        priors = initial distribution of the hidden chain
        tol = tolerance for the stopping criterion
        maxIt = maximal number of iterations
    out:  a = estimate of the transition matrix of the hidden markov process
        b = estimated probabilities of transition: b(x,y) = estimate of P(Y=y | X=x) for 0<=x<k
        log_like = log-likelihood of y for parameters a and b  
    '''  
    states = len(priors)
    r = 1+np.amax(obs)
    # TODO only works for same number of observations each time
    n = obs.shape[0]
    seq = obs.shape[2]
    xi=np.zeros([states,states,seq,n-1])
    gamma=np.zeros((len(priors),n))
    # random initialization
    a = np.random.rand(states, states)
    b = np.random.rand(states, int(r))
    for i in range(states):
        a[i,:] = a[i,:] / sum(a[i,:])
        b[i,:] = b[i,:] / sum(b[i,:])

    it = 0
    olda = a
    oldb = b + tol +1
  
    while (sum(sum(abs(olda[:]-a[:]))) + sum(sum(abs(oldb-b))) > tol) & (it<maxIt):
        P=[]
        log_like=0
        # assume multiple observations
        for k in range(seq):
            alpha = forward(priors,a,b,obs[:,:,k])
            beta = backward(priors,a,b,obs[:,:,k])
            P.append(sum(alpha[:,-1]))
            log_like+=np.log(P[-1])
            # getting multipliers
            c=[]
            for i in range(len(alpha)):
                c.append(sum(alpha[:,i]))
            # scaling values
            for i in range(len(alpha)):
                alpha[i]*=c[i]
                beta[i]*=c[i]
            xi_dom=np.zeros([n-1,seq])
            for i in range(states):
                for j in range(states):
                        xi_dom[:,k]=np.add(xi_dom[:,k],alpha[i,0:-1]*a[i,j]*b[i,np.int_(obs[1:,:,k])].flatten()*beta[j,1:])
            #  print xi_dom[:,k].shape
            for i in range(states):
                for j in range(states):
                        xi[i,j,k,:]=alpha[i,0:-1]*a[i,j]*b[i,np.int_(obs[1:,:,k])].flatten()*beta[j,1:]/xi_dom[:,k]
            for t in range(len(obs)):
                for i in range(len(priors)):
                        gamma[i,t]=(alpha[i,t]*beta[i,t])/sum(alpha[st,t]*beta[st,t] for st in range(len(priors)))
            it+=1

        olda = a.copy()
        oldb = b.copy()
        #  print xi
        #  print gamma
        #  print P
        #  sys.exit()
        for i in range(states):
            for o in range(r):
                #TODO won't work for multiple dimensions
                b_num=0
                b_dom=0
                for k in range(seq):
                    #  b_num+=(1/P[k])*(obs[:,:,k]==o).sum()
                    #  b_dom+=(1/P[k])*sum(gamma[i,:])
                    b_num+=(obs[:,:,k]==o).sum()
                    b_dom+=sum(gamma[i,:])
                b[i,o]=b_num/b_dom
            a_num=0
            for j in range(states):
                for k in range(seq):
                    #  a_num+=(1/P[k])*sum(xi[i,j,k,:])
                    a_num+=sum(xi[i,j,k,:])
                a[i,j]=a_num/b_dom
    return (a, b, log_like)

if __name__=="__main__":
    trans_probs=np.transpose(np.array([[0.02,0.019,0,0.666],[0,0.025,0.517,0],
        [0.163,0.769,0.466,0],[0.817,0.187,0.017,0.334]]))
    obs_like=np.transpose(np.array([[0.0338,0,0,0.3273],[0.0934,0,0,0.0949],
        [0.1356,0,0,0.0311],[0.1031,0,0,0.0125],[0.1350,0,0,0.0113],
        [0.0289,0,0,0.3354],[0.0968,0,0,0.1094],[0.1409,0,0,0.0488],
        [0.1117,0,0,0.0149],[0.1208,0,0,0.0144],[0,0.0842,0.7353,0],
        [0,0.2048,0.1869,0],[0,0.3774,0.0195,0],[0,0.3336,0.0267,0],
        [0,0,0.0316,0]]))
    priors=[0,0,1,0]
    obs=[14,11,11,12,14,11,12,2,6,5,6,8,1,5,2]
    data = np.loadtxt('nominal_hmm_multi_logs.csv', delimiter=",")
    data=data[:,:,np.newaxis]
    data=np.swapaxes(data,1,2)
    data=np.int_(data)
    obs=[x-1 for x in obs]
    HMMbaumwelch(priors,data)
