from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
from pandas import DataFrame

# task 1
alpha=0.5
x=np.linspace(1,500,500)
y=alpha/(x-1+alpha)
plt.plot(x,y)
plt.xlabel('Number of customers')
plt.ylabel('Probability of new tabel')
plt.title('P(New Table|Customer #)')
plt.show()

# task 2
plt.clf()
clusters=[]
cluster_centers=[]
for i in range(1,500):
    probs=[]
    for cluster in clusters:
        probs.append(len(cluster)/(i-1+alpha))
    probs.append(alpha/(i-1+alpha))
    cluster_num=np.random.choice(range(len(clusters)+1),p=probs)
    if cluster_num<=len(clusters)-1:
        cluster_center=cluster_centers[cluster_num]
        new=False
    else:
        cluster_center=[np.random.uniform(),np.random.uniform()]
        new=True
    sample=np.random.multivariate_normal(cluster_center,[[0.01,0],[0,0.01]])
    if new:
        clusters.append(np.array([sample]))
        cluster_centers.append(sample)
    else:
        clusters[cluster_num]=np.vstack((clusters[cluster_num],sample))
for cluster in clusters:
    plt.scatter(cluster[:,0],cluster[:,1])
plt.title('%s Clusters' % (len(clusters)))
plt.show()

# extra street cred
data=np.array([])
with open('assignment8_data.txt') as inputfile:
    for line in inputfile:
        strings=filter(None,line.strip().split(' '))
        clean_line=[float(i) for i in strings]
        if len(data)!=0:
            data=np.vstack((data,clean_line))
        else:
            data=np.array(clean_line)

data[:,-1]=0
data=DataFrame(data,columns=('X','Y','cluster'))
#  plt.clf()
#  for cluster in data.cluster.unique():
#      cluster_points=data[data['cluster']==cluster].as_matrix()[:,:-1]
#      plt.scatter(cluster_points[:,0],cluster_points[:,1])
#  plt.title('%s Clusters' % (len(data.cluster.unique())))
#  plt.show()
N=len(data)
for i in range(25):
    convered=True
    for idx, point in data.iterrows():
        clean_point=point.tolist()[:-1]
        new_data=data.drop(data.index[idx])
        probs=[]
        for cluster in data.cluster.unique():
            cluster_data=data[data['cluster']==cluster].as_matrix()[:,:-1]
            P1=len(cluster_data)/(N+alpha-1)
            cluster_mean=np.mean(cluster_data,axis=0)
            P2=(len(cluster_data)/N)*(stats.multivariate_normal(cluster_mean,[[0.01,0],[0,0.01]]).pdf(clean_point))
            probs.append(P1*P2)
        P1=alpha/(N+alpha-1)
        P2=stats.multivariate_normal(clean_point,[[0.01,0],[0,0.01]]).pdf(clean_point)/N
        probs.append(P1*P2)
        probs=probs/sum(probs)
        cluster_num=np.random.choice(len(data.cluster.unique())+1,p=probs)
        if cluster_num!=point['cluster']:
            converged=False
        data.set_value(idx,'cluster',cluster_num)
plt.clf()
for cluster in data.cluster.unique():
    cluster_points=data[data['cluster']==cluster].as_matrix()[:,:-1]
    plt.scatter(cluster_points[:,0],cluster_points[:,1])
plt.title('%s Clusters' % (len(data.cluster.unique())))
plt.show()
