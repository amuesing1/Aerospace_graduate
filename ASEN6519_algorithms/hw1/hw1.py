from __future__ import division
import numpy as np
import matplotlib.pyplot as plt

def forward(priors,trans_probs,obs_like,obs):
    alphas=np.zeros((len(priors),len(obs)+1))
    alphas[:,0]=priors
    for i in range(1,len(obs)+1):
        for j in range(len(priors)):
            alphas[j,i]=obs_like[j][obs[i-1]]*sum(
                   alphas[state][i-1]*trans_probs[state][j] for state in range(len(priors)))
    #      suma=sum([alphas[state][i] for state in range(len(priors))])
    #      for state in range(len(priors)):
    #          alphas[state][i]/=suma
    return alphas[:,1:]

def backward(priors,trans_probs,obs_like,obs):
    betas=np.zeros((len(priors),len(obs)))
    betas[:,-1]=1
    for i in range(len(obs)-2,-1,-1):
        for j in range(len(priors)):
            betas[j,i]=sum(betas[state][i+1]*trans_probs[j][state]
                    *obs_like[state][obs[i+1]] for state in range(len(priors)))
    return betas

def forward_backward(priors,trans_probs,obs_like,obs):
    probs=np.zeros((len(priors),len(obs)))
    alphas=forward(priors,trans_probs,obs_like,obs)
    betas=backward(priors,trans_probs,obs_like,obs)
    for i in range(len(obs)):
        for j in range(len(priors)):
            probs[j,i]=(alphas[j,i]*betas[j,i])/sum(alphas[st,i]*betas[st,i] for st in range(len(priors)))
    #  print probs
    return probs

def log_likelihood(priors,trans_probs,obs_like,obs):
    alphas=forward(priors,trans_probs,obs_like,obs)
    return np.log(sum(alphas[:,-1]))

def viterbi(priors,trans_probs,obs_like,obs):
    T1=np.zeros((len(priors),len(obs)))
    T2=np.zeros((len(priors),len(obs)))
    for i in range(len(priors)):
        # if given an initial observation, use that
        T1[i,0]=priors[i]#*obs_like[i,obs[0]]

    for o in range(1,len(obs)):
        for j in range(len(priors)):
            max_tr_prob=max(T1[pre][o-1]*trans_probs[pre][j] for pre in range(len(priors)))
            for pre in range(len(priors)):
                if (T1[pre][o-1]*trans_probs[pre][j]==max_tr_prob):
                    #  print max_tr_prob,pre
                    T1[j,o]=max_tr_prob*obs_like[j][obs[o]]
                    T2[j,o]=pre
        suma=sum(T1[:,o])
        T1[:,o]=[x/suma for x in T1[:,o]]

    Z=[np.argmax(T1[:,-1])]
    X=np.argmax(T1[:,-1])
    for o in range(len(obs)-2,-1,-1):
        Z.insert(0,T2[X,o+1])
        X=int(T2[X,o+1])
    return Z

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
    obs=[x-1 for x in obs]
    probs=forward_backward(priors,trans_probs,obs_like,obs)
    sequence=viterbi(priors,trans_probs,obs_like,obs)
    print np.transpose(probs)
    print log_likelihood(priors,trans_probs,obs_like,obs)
    plt.figure()
    plt.plot(range(len(obs)),np.argmax(probs,0),'o-',label='forward-backward')
    plt.plot(range(len(obs)),sequence,'o-',label='viterbi')
    plt.xlabel('Time step k')
    plt.ylabel('Most Likely State')
    plt.legend()
    print sequence
    obs_long=[11,11,13,6,10,3,1,7,1,6,6,2,7,8,7,6,3,12,14,11,14,11,14,12,
            14,11,13,1,4,1,3,1,7,3,6,5,12,14,11,11,11,14,11,12,14]
    obs_long=[x-1 for x in obs_long]
    sequence=viterbi(priors,trans_probs,obs_like,obs_long)
    print sequence
    plt.show()
