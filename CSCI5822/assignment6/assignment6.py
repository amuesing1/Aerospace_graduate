from __future__ import division
import matplotlib.pyplot as plt
import pymc3 as pm
import theano
import numpy as np
import pandas as pd


with pm.Model() as model:
    G1=pm.Bernoulli('G1',0.5)
    G2_p=pm.Deterministic('G2_p',pm.math.switch(G1,0.9,0.1))
    G2=pm.Bernoulli('G2',G2_p)
    G3_p=pm.Deterministic('G3_p',pm.math.switch(G1,0.9,0.1))
    G3=pm.Bernoulli('G3',G3_p)
    X2=pm.Normal('X2',mu=pm.math.switch(G2,60,50),sd=np.sqrt(10))
    X3=pm.Normal('X3',mu=pm.math.switch(G3,60,50),sd=np.sqrt(10))
    trace=pm.sample(100000)

dictionary = {
            'G1': [2 if x==1 else 1 for x in trace['G1'].tolist() ],
            'X3': [int(round(x)) for x in trace['X3'].tolist()],
            'X2': [int(round(x)) for x in trace['X2'].tolist()],
            }
df=pd.DataFrame(dictionary)
set_X2_50=df[df['X2']==50]
problem_1=len(set_X2_50[set_X2_50['G1']==2])/len(set_X2_50)
problem_2=len(set_X2_50[set_X2_50['X3']==50])/len(set_X2_50)
print problem_1
print problem_2
pm.traceplot(trace)
plt.show()
