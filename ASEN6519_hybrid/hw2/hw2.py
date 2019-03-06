from __future__ import division

import numpy as np
import matplotlib.pyplot as plt

x_points=np.arange(-2,2,.05)
y_points=np.arange(-2,2,.05)
A1=np.array([[-1,10],[-100,-1]])
A2=np.array([[-1,100],[-10,-1]])

X, Y = np.meshgrid(x_points,y_points)
U=np.zeros(shape=(X.shape[0],X.shape[1]))
V=np.zeros(shape=(Y.shape[0],Y.shape[1]))

four_points=np.array([[1.5,1.5],[-1.5,1.5],[1.5,-1.5],[-1.5,-1.5]])
time_scale=500
four_lines=np.zeros((4,2,time_scale))
four_lines[:,:,0]=four_points

for x in range(len(x_points)):
    for y in range(len(y_points)):
        if X[x,y]*Y[x,y]>=0:
            x_dot=np.matmul(A1,np.array([X[x,y],Y[x,y]]))
        elif X[x,y]*Y[x,y]<=0:
            x_dot=np.matmul(A2,np.array([X[x,y],Y[x,y]]))
        U[x,y]=x_dot[0]
        V[x,y]=x_dot[1]

for t in range(0,time_scale-1):
    for i in range(4):
        if four_lines[i,0,t]*four_lines[i,1,t]>=0:
            four_lines[i,:,t+1]=four_lines[i,:,t]+0.01*np.matmul(A1,four_lines[i,:,t])
        elif four_lines[i,0,t]*four_lines[i,1,t]<=0:
            four_lines[i,:,t+1]=four_lines[i,:,t]+0.01*np.matmul(A2,four_lines[i,:,t])
    
fig,ax=plt.subplots()
ax.quiver(X[::3],Y[::3],U[::3],V[::3])
for i in range(4):
    ax.plot(four_lines[i,0,:],four_lines[i,1,:],color='b')
ax.axis('equal')
plt.show()
