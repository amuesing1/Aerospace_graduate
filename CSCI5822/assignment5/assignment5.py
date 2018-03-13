from __future__ import division

import numpy as np
from scipy import stats
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
from pandas import DataFrame

# part I
mu=100
sigma=15
samples=100000
table=np.zeros([samples,3])
for i in range(samples):
    I=np.random.normal(mu,sigma)
    P_CU=1/(1+np.exp(-(I-100)/5))
    P_CS=1/(1+np.exp(-(I-110)/5))
    school=int(np.random.uniform()<P_CU)
    major=int(np.random.uniform()<P_CS)
    salary=round(stats.gamma.rvs(a=(0.1*I+major+3*school),scale=5))
    table[i,:]=[school,major,salary]
df=DataFrame(table,columns=['School','Major','Salary'])
CUCS=df[(df['School']==1) & (df['Major']==1)]
CUB=df[(df['School']==1) & (df['Major']==0)]
MetroCS=df[(df['School']==0) & (df['Major']==1)]
MetroB=df[(df['School']==0) & (df['Major']==0)]
PCUCS=[]
PCUB=[]
PMetroCS=[]
PMetroB=[]
for sal in [120,60,20]:
    total=df[df['Salary']==sal].shape[0]
    PCUCS.append(CUCS[CUCS['Salary']==sal].shape[0]/total)
    PCUB.append(CUB[CUB['Salary']==sal].shape[0]/total)
    PMetroCS.append(MetroCS[MetroCS['Salary']==sal].shape[0]/total)
    PMetroB.append(MetroB[MetroB['Salary']==sal].shape[0]/total)
print PCUCS
print PCUB
print PMetroCS
print PMetroB
fig,ax=plt.subplots()
x=np.arange(3)
width=0.15
CUCS=plt.bar(x,PCUCS,width,color='y',label='PCUCS')
CUB=plt.bar(x+width,PCUB,width,color='k',label='PCUB')
MetroCS=plt.bar(x+2*width,PMetroCS,width,color='b',label='PMetroCS')
MetroB=plt.bar(x+3*width,PMetroB,width,color='r',label='PMetroB')
plt.xlabel('Salary')
plt.ylabel('Pr')
plt.xticks(x+width,('120','60','20'))
plt.legend()
plt.show()

# Part II
burnin=500
samples=100000+burnin
means=np.array([1,0])
covs=np.array([[1,-0.5],[-0.5,3]])
total=np.zeros([samples,2])
x1=np.random.randn()*4
x2=np.random.randn()*4
for i in range(samples):
    mux1=means[0]+covs[0,1]/covs[1,1]*(x2-means[1])
    sigmax1=covs[0,0]-covs[0,1]/covs[1,1]*covs[1,0]
    x1=np.random.normal(mux1,sigmax1)
    mux2=means[1]+covs[1,0]/covs[0,0]*(x1-means[0])
    sigmax2=covs[1,1]-covs[1,0]/covs[0,0]*covs[0,1]
    x2=np.random.normal(mux2,sigmax2)
    if i<burnin:
        continue
    else:
        total[i-burnin,:]=[x1,x2]
#  plt.clf()
fig, (ax1,ax2) = plt.subplots(1,2)
num_bins=50
n, bins, patches = ax1.hist(total[:,0], num_bins, normed=1)
y = mlab.normpdf(bins, means[0], covs[0,0])
ax1.plot(bins, y)
ax1.set_title('$X_1$')
n, bins, patches = ax2.hist(total[:,1], num_bins, normed=1)
y = mlab.normpdf(bins, means[1], covs[1,1])
ax2.plot(bins, y)
ax2.set_title('$X_2$')
plt.show()

# Task III
def P_GF(g,f):
    if (g<0) or (g>1) or (f<0) or (f>1):
        return 0
    else:
        return (1-abs(g-f))*(f**3)

n = 1000000
s = 2
xf = 0.5
xg = 0.5
F = []
G= []
F.append(xf)
G.append(xg)
for i in range(n):
    canf = xf + np.random.uniform(-s,s) #candidate
    cang = xg + np.random.uniform(-s,s) #candidate
    a = P_GF(cang,canf)/P_GF(xg,xf) #acceptance probability
    if a>=1:
        xf=canf
        xg=cang
        G.append(xg)
        F.append(xf)
    elif a>np.random.uniform(0,1):
        xf=canf
        xg=cang
        G.append(xg)
        F.append(xf)
plt.hist2d(G,F,bins=20)
plt.xlabel('G')
plt.ylabel('F')
plt.show()
print np.mean(np.array(F)*np.array(G))
