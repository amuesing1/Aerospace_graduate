#  Jeremy Muesing
#  ASEN 5044
#  Exam 1
from __future__ import division

import numpy as np
import matplotlib.pyplot as plt


tm=0.61
te=0.78
tr=1.4

a=-3
b=3
y=np.linspace(a,b,1e4)
likelihood=(1-np.exp(-10**y*(te-tm)))/(1-np.exp(-10**y*(tr-tm)))
case1=(1/(b-a))*likelihood
case1/=sum(case1*((b-a)/1e4))
plt.figure(1)
plt.plot(y,case1)
plt.title('Case 1')
plt.xlabel('y')
plt.ylabel('p(y)')

prior=np.abs(np.log(10)*10**y)/(10**3-10**(-3))
case2=prior*likelihood
case2/=sum(case2*((b-a)/1e4))
plt.figure(2)
plt.plot(y,case2)
plt.title('Case 2')
plt.xlabel('y')
plt.ylabel('p(y)')

prior=np.abs(-np.log(10)*10**(-y))/(10**3-10**(-3))
case3=prior*likelihood
case3/=sum(case3*((b-a)/1e4))
plt.figure(3)
plt.plot(y,case3)
plt.title('Case 3')
plt.xlabel('y')
plt.ylabel('p(y)')

plt.show()
