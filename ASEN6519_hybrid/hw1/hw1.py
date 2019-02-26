from __future__ import division
import numpy as np
import matplotlib.pyplot as plt

def problem1(init):
    # constants
    dt=0.01
    e=0.1
    def state(theta,omega,u):
        theta_new=omega*dt+theta
        omega_new=dt*(np.sin(theta)-u*np.cos(theta))+omega
        return theta_new,omega_new

    def pump(theta,omega):
        u=(-omega*np.cos(theta))/(1+np.abs(omega))+0.001
        theta,omega=state(theta,omega,u)
        if u>1:
            u=1
        elif u<-1:
            u=-1
        return theta,omega

    def remove(theta,omega):
        u=(omega*np.cos(theta))/(1+np.abs(omega))
        theta,omega=state(theta,omega,u)
        if u>1:
            u=1
        elif u<-1:
            u=-1
        return theta,omega

    def stabilize(theta,omega):
        if (2*np.pi-theta)<theta:
            u=(2*omega-(2*np.pi-theta)+np.sin(theta))/np.cos(theta)
        else:
            u=(2*omega+theta+np.sin(theta))/np.cos(theta)
        if u>1:
            u=1
        elif u<-1:
            u=-1
        theta,omega=state(theta,omega,u)
        return theta,omega

    theta=init[0]
    omega=init[1]
    all_theta=[]
    all_omega=[]
    all_states=[]
    all_energy=[]
    
    for i in range(3500):
        E=0.5*omega*omega+(np.cos(theta)-1)

        if E<-e:
            all_states.append(1)
            theta,omega=pump(theta,omega)
        elif E>e:
            all_states.append(-1)
            theta,omega=remove(theta,omega)
        else:
            all_states.append(0)
            theta,omega=stabilize(theta,omega)

        all_theta.append(theta)
        all_omega.append(omega)
        all_energy.append(E)

    fig=plt.figure()
    ax=plt.subplot(3,1,1)
    color='tab:blue'
    ax.plot(np.linspace(0,len(all_theta)*dt,len(all_theta)),all_theta,label='theta',color=color)
    ax.set_ylabel('Rad',color=color)
    ax.set_ylim(-2.5, np.pi*2+.5)
    ax.tick_params(axis='y',labelcolor=color)

    ax0=ax.twinx()
    color='tab:orange'
    ax0.plot(np.linspace(0,len(all_theta)*dt,len(all_theta)),all_omega,label='omega',color=color)
    ax0.set_ylabel('Rad/s',color=color)
    ax0.set_ylim(-2.5, np.pi*2+.5)
    ax0.tick_params(axis='y',labelcolor=color)
    ax.set_title('States over time')
    ax.set_xlabel('Time (s)')

    ax1=plt.subplot(3,1,2)
    ax1.plot(np.linspace(0,len(all_theta)*dt,len(all_theta)),all_states)
    ax1.set_title('Discrete states over time')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('States')

    ax2=plt.subplot(3,1,3)
    ax2.plot(np.linspace(0,len(all_theta)*dt,len(all_theta)),all_energy)
    ax2.set_title('Energy over time')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Energy')

    fig.tight_layout()

    plt.show()

def problem3():
    x1=[0]
    x2=[1]
    m1=.5
    m2=.5
    lamda=.75
    h1=0
    h2=0
    states=[1]
    state=states[-1]
    dt=0.01
    
    def tank1(x1,fill=False):
        new=x1[-1]-m1*dt
        if fill:
            new+=lamda*dt
        x1.append(new)
        return x1

    def tank2(x2i,fill=False):
        new=x2[-1]-m2*dt
        if fill:
            new+=lamda*dt
        x2.append(new)
        return x2

    for t in range(int(3.5/dt)):
        if (x1[-1]<=h1 and states[-1]==2):
            state=1
        elif (x2[-1]<=h2 and states[-1]==1):
            state=2
        if state==1:
            x1=tank1(x1,fill=True)
        else:
            x1=tank1(x1)
        if state==2:
            x2=tank1(x2,fill=True)
        else:
            x2=tank2(x2)
        states.append(state)

    fig=plt.figure()
    ax=plt.subplot(2,1,1)
    ax.plot(np.linspace(0,3.5,len(x1)),x1,label='x1')
    ax.plot(np.linspace(0,3.5,len(x2)),x2,label='x2')
    ax.legend()
    ax.set_title('States over time')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Water level')

    ax1=plt.subplot(2,1,2)
    ax1.plot(np.linspace(0,3.5,len(x1)),states)
    ax1.set_title('Discrete states over time')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('States')

    fig.tight_layout()
    plt.show()


# problem1
#  problem1([np.pi/2,0])
#  problem1([np.pi/12,3/5])
#  problem1([np.pi,0])
problem3()
