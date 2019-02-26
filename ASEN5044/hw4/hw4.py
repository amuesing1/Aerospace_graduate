# Jeremy Muesing
# ASEN 5044
#HW4
from __future__ import division

import numpy as np
import matplotlib.pyplot as plt

# 2.15
values=np.random.uniform(size=50)
print np.mean(values),np.std(values)
plt.figure(1)
plt.hist(values,bins=10)
plt.title('50 Uniform Samples')

values=np.random.uniform(size=500)
print np.mean(values),np.std(values)
plt.figure(2)
plt.hist(values,bins=10)
plt.title('500 Uniform Samples')

values=np.random.uniform(size=5000)
print np.mean(values),np.std(values)
plt.figure(3)
plt.hist(values,bins=10)
plt.title('5000 Uniform Samples')

# 2.16
x1=np.random.uniform(-.5,.5,size=10000)
x2=np.random.uniform(-.5,.5,size=10000)
values=(x1+x2)/2
plt.figure(4)
plt.hist(values,bins=50)
plt.title('10,000 Uniform (x1+x2)/2  Samples')


x1=np.random.uniform(-.5,.5,size=10000)
x2=np.random.uniform(-.5,.5,size=10000)
x3=np.random.uniform(-.5,.5,size=10000)
x4=np.random.uniform(-.5,.5,size=10000)
values=(x1+x2+x3+x4)/4
plt.figure(5)
plt.hist(values,bins=50)
plt.title('10,000 Uniform (x1+x2+x3+x4)/4  Samples')

# AQ2
p_all=[None]*int(14/0.001)
mixands=[[0.1859,-4,-2],[0.0961,-2,-1],[0.1055,-.75,0],[.2104,0,1],[.0678,3,5.5],[.195,4,6],[.1393,-.9,7]]
x=np.linspace(-6,8,14/0.001)

mean=0
moment=0
for mixand in mixands:
    mean+=(mixand[0]*(mixand[1]+mixand[2]))/2
    moment+=mixand[0]*(mixand[1]**2+mixand[1]*mixand[2]+mixand[2]**2)/3
print "mean",mean
print "var",moment-mean**2
count=0
for xi in x:
    p=0
    for mixand in mixands:
        if mixand[1]<=xi<=mixand[2]:
            p+=mixand[0]/(mixand[2]-mixand[1])
    p_all[count]=p
    count+=1

plt.figure(6)
plt.plot(x,p_all)

# c.1
Ex=sum(x*p_all*0.001)
print Ex
# c.2
Ex2=sum(x**2*p_all*0.001)
print Ex2-Ex**2
# c.3
Elogx=0
for px in p_all:
    if px!=0:
        Elogx+=-np.log(px)*px*0.001
print Elogx
# c.4
KLD=0
for px in p_all:
    if px!=0:
        KLD+=np.log(px*11)*px*0.001
print KLD
p_all=np.asarray(p_all)*0.001

values=np.random.choice(x,size=100,p=p_all)
plt.figure(7)
plt.hist(values,bins=1000)
plt.title('100 p(x) samples')
# d E[x]
print sum(values)/len(values)
# d E[x^2]
print sum(values**2)/len(values)-sum(values)/len(values)
p_again=[]
q_again=[]
for xi in values:
    p=0
    q=0
    if -4<=xi<=7:
        q=1/11
    for mixand in mixands:
        if mixand[1]<=xi<=mixand[2]:
            p+=mixand[0]/(mixand[2]-mixand[1])
    p_again.append(p)
    q_again.append(q)
p_again=np.asarray(p_again)
q_again=np.asarray(q_again)
print sum(-np.log(p_again))/len(values)
print sum(np.log(p_again/q_again))/len(values)

values=np.random.choice(x,size=1000,p=p_all)
plt.figure(8)
plt.hist(values,bins=1000)
plt.title('1,000 p(x) samples')
# d E[x]
print sum(values)/len(values)
# d E[x^2]
print sum(values**2)/len(values)-sum(values)/len(values)
p_again=[]
q_again=[]
for xi in values:
    p=0
    q=0
    if -4<=xi<=7:
        q=1/11
    for mixand in mixands:
        if mixand[1]<=xi<=mixand[2]:
            p+=mixand[0]/(mixand[2]-mixand[1])
    p_again.append(p)
    q_again.append(q)
p_again=np.asarray(p_again)
q_again=np.asarray(q_again)
print sum(-np.log(p_again))/len(values)
print sum(np.log(p_again/q_again))/len(values)

values=np.random.choice(x,size=50000,p=p_all)
plt.figure(9)
plt.hist(values,bins=1000)
plt.title('50,000 p(x) samples')
# d E[x]
print sum(values)/len(values)
# d E[x^2]
print sum(values**2)/len(values)-sum(values)/len(values)
p_again=[]
q_again=[]
for xi in values:
    if -4<=xi<=7:
        p=0
        q=1/11
        for mixand in mixands:
            if mixand[1]<=xi<=mixand[2]:
                p+=mixand[0]/(mixand[2]-mixand[1])
    p_again.append(p)
    q_again.append(q)
p_again=np.asarray(p_again)
q_again=np.asarray(q_again)
print sum(-np.log(p_again))/len(values)
print sum(np.log(p_again/q_again))/len(values)

#  plt.show()
