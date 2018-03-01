from __future__ import division
import numpy as np

sigma=10
X2=0
i=0
for G1 in [0.5,0.5]:
    if i==0:
        G2_mat=[0.9,0.1]
    elif i==1:
        G2_mat=[0.1,0.9]
    i=i+1
    j=0
    for G2 in G2_mat:
        if j==0:
            m=50
        elif j==1:
            m=60
        X2=X2+(G2*G1*(1/(np.sqrt(2*np.pi*sigma))*np.exp(-(float(50-m)**2)/(2*sigma))))
        j=j+1
print X2

joint1=0
G2_mat=[0.1,0.9]
i=0
for G2 in G2_mat:
    if i==0:
        m=50
    elif i==1:
        m=60
    joint1=joint1+(G2*(1/(np.sqrt(2*np.pi*sigma))*np.exp(-(float(50-m)**2)/(2*sigma))))
    i=i+1
joint1=joint1*0.5
print joint1
print joint1/X2

joint2=0
i=0
for G1 in [0.5,0.5]:
    if i==0:
        G2_mat=[0.9,0.1]
    elif i==1:
        G2_mat=[0.1,0.9]
    i=i+1
    j=0
    for G2 in G2_mat:
        if j==0:
            m2=50
        elif j==1:
            m2=60
        j=j+1
        k=0
        for G3 in G2_mat:
            if k==0:
                m3=50
            elif k==1:
                m3=60
            joint2=joint2+(G2*G3*G1*(1/(np.sqrt(2*np.pi*sigma))*np.exp(-(float(50-m2)**2)/(2*sigma)))*(1/(np.sqrt(2*np.pi*sigma))*np.exp(-(float(50-m3)**2)/(2*sigma))))
            k=k+1
print joint2
print joint2/X2
