import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import control
from estimator import UKF
from math import *
import scipy.linalg
import sys
import time
from hybrid_automaton import Automaton

np.set_printoptions(precision=4,suppress=True)

class StateSpace():
    def __init__(self,A,B,C,D,dt):
        n=A.shape[0]
        r=B.shape[1]
        Ahat=np.zeros([n+r,n+r])
        Ahat[0:n,0:n]=A
        Ahat[0:n,n:]=B
        FG0I=scipy.linalg.expm(Ahat*dt)
        F=FG0I[0:n,0:n]
        G=FG0I[0:n,n:]
        self.A=F
        self.B=G
        self.C=C
        self.D=D

    def __str__(self):
        """String representation of the state space."""
            
        str = "A = " + self.A.__str__() + "\n\n"
        str += "B = " + self.B.__str__() + "\n\n"
        str += "C = " + self.C.__str__() + "\n\n"
        str += "D = " + self.D.__str__() + "\n"
        return str


class Controller(UKF):
    def __init__(self,files_to_load=[]):
        UKF.__init__(self)

        if 'position_hold' in files_to_load:
            controller_hold=np.load('position_hold.npy').item()
            self.K_hold=controller_hold['K']
            self.F_hold=controller_hold['F']
        if 'moving' in files_to_load:
            controller_move=np.load('moving.npy').item()
            self.K_move=controller_move['K']
            self.F_move=controller_move['F']

    def create_controller(self,lin_x,lin_u,dt,Q_max,R_max,name):
        """Creates and saves the K and F matricies for a controller

        Inputs: 
            lin_x (array) [x,xdot,y,ydot,z,zdot,phi,phidot,theta,thetadot,psi,psidot]: linearization point-states 
            lin_u (array) [L,R,F,B,FL,FR,BL,BR]: linearization point-inputs 
            dt (float): time step size
            Q_max (array [1x12]): max distance away from state allowed
            R_max (float or array [1x8]): max input allowed
            name (str): name of the controller
        """
        state_space=self.linearize_matricies(lin_x,lin_u,dt)

        A=state_space.A
        B=state_space.B
        C=state_space.C
        D=state_space.D

        Q=(1/Q_max**2)*np.eye(12)
        R=(1/R_max**2)*np.eye(8)
        # get K matrix
        K,S,vals=self.LQR(A,B,Q,R)
        # get feed forward matrix
        F=self.feed_forward(A,B,C,D,K)

        save_controller={}
        save_controller['A']=A
        save_controller['B']=B
        save_controller['C']=C
        save_controller['D']=D
        save_controller['K']=K
        save_controller['F']=F
        np.save(name+'.npy',save_controller)
        
    def linearize_matricies(self,x,u,dt):
        """creates discritized linearized F,G,H & M matricies

        Inputs:
            x (array_like): linearization point-states
            u (array_like): linearization point-inputs
            dt (float): discrete interval
        """

        dxdotdot_dxdot=-(self.A_x*self.Cd_x*self.rho*x[1])/self.m
        dxdotdot_dtheta=-self.g*cos(x[10])*cos(x[8])+((self.Fb*cos(x[10])*cos(x[8]))/self.m)
        dxdotdot_dpsi=self.g*sin(x[10])*sin(x[8])-((self.Fb*sin(x[10])*sin(x[8]))/self.m)
        dxdotdot_dTL=1/self.m
        dxdotdot_dTR=1/self.m

        dydotdot_dydot=-(self.A_y*self.Cd_y*self.rho*x[3])/self.m
        dydotdot_dphi=self.g*cos(x[6])*cos(x[10])-((self.Fb*cos(x[10])*cos(x[6]))/self.m)
        dydotdot_dpsi=-self.g*sin(x[6])*sin(x[10])+((self.Fb*sin(x[10])*sin(x[6]))/self.m)
        dydotdot_dTF=1/self.m
        dydotdot_dTB=1/self.m

        dzdotdot_dzdot=-(self.A_z*self.Cd_z*self.rho*x[5])/self.m
        dzdotdot_dphi=-self.g*cos(x[8])*sin(x[6])+((self.Fb*cos(x[8])*sin(x[6]))/self.m)
        dzdotdot_dtheta=-self.g*cos(x[6])*sin(x[8])+((self.Fb*cos(x[6])*sin(x[8]))/self.m)
        dzdotdot_dTFL=1/self.m
        dzdotdot_dTFR=1/self.m
        dzdotdot_dTBL=1/self.m
        dzdotdot_dTBR=1/self.m

        dphidotdot_dydot=-.5*self.Ixx*self.rho*(2*self.A_yB*self.Cd_yB*self.dz_yB* \
                (self.dz_yB*x[7]+x[3])+2*self.A_yT*self.Cd_yT*self.dz_yT*(self.dz_yT*x[7]+x[3]))
        dphidotdot_dzdot=-.5*self.Ixx*self.rho*(2*self.A_zL*self.Cd_zL*self.dy_zL* \
                (self.dy_zL*x[7]+x[5])+2*self.A_zR*self.Cd_zR*self.dy_zR*(self.dy_zR*x[7]+x[5]))
        dphidotdot_dphi=(self.Fb/self.Ixx)*(self.phi_bz*cos(x[6])*cos(x[8])+self.phi_by*cos(x[8])*sin(x[6]))
        dphidotdot_dphidot=-.5*self.Ixx*self.rho*(2*self.A_yB*self.Cd_yB*self.dz_yB**2* \
                (self.dz_yB*x[7]+x[3])+2*self.A_yT*self.Cd_yT*self.dz_yT**2*(self.dz_yT*x[7]+x[3])+ \
                2*self.A_zL*self.Cd_zL*self.dy_zL**2*(self.dy_zL*x[7]+x[5])+2*self.A_zR*self.Cd_zR* \
                self.dy_zR**2*(self.dy_zR*x[7]+x[5]))
        dphidotdot_dtheta=(self.Fb/self.Ixx)*(self.phi_by*cos(x[6])*sin(x[8])-self.phi_bz*sin(x[6])*sin(x[8]))
        dphidotdot_dTFL=-self.phi_L/self.Ixx
        dphidotdot_dTFR=self.phi_R/self.Ixx
        dphidotdot_dTBL=-self.phi_L/self.Ixx
        dphidotdot_dTBR=self.phi_R/self.Ixx

        dthetadotdot_dxdot=-.5*self.Iyy*self.rho*(2*self.A_xB*self.Cd_xB*self.dz_xB* \
                (self.dz_xB*x[9]+x[1])+2*self.A_xT*self.Cd_xT*self.dz_xT*(self.dz_xT*x[9]+x[1]))
        dthetadotdot_dzdot=-.5*self.Iyy*self.rho*(2*self.A_zB*self.Cd_zB*self.dx_zB* \
                (self.dx_zB*x[9]+x[5])+2*self.A_zF*self.Cd_zF*self.dx_zF*(self.dx_zF*x[9]+x[5]))
        dthetadotdot_dphi=(-self.Fb/self.Iyy)*(self.theta_bx*cos(x[8])*sin(x[6]))
        dthetadotdot_dtheta=(self.Fb/self.Iyy)*(self.theta_bz*cos(x[8])-self.theta_bx*cos(x[6])*sin(x[8]))
        dthetadotdot_dthetadot=-.5*self.Iyy*self.rho*(2*self.A_xB*self.Cd_xB*self.dz_xB**2* \
                (self.dz_xB*x[9]+x[1])+2*self.A_xT*self.Cd_xT*self.dz_xT**2*(self.dz_xT*x[9]+x[1])+ \
                2*self.A_zB*self.Cd_zB*self.dx_zB**2*(self.dx_zB*x[9]+x[5])+2*self.A_zF*self.Cd_zF* \
                self.dx_zF**2*(self.dx_zF*x[9]+x[5]))
        dthetadotdot_dTFL=self.theta_F/self.Iyy
        dthetadotdot_dTFR=self.theta_F/self.Iyy
        dthetadotdot_dTBL=-self.theta_B/self.Iyy
        dthetadotdot_dTBR=-self.theta_B/self.Iyy
        
        dpsidotdot_dxdot=-.5*self.Izz*self.rho*(2*self.A_xL*self.Cd_xL*self.dy_xL* \
                (self.dy_xL*x[11]+x[1])+2*self.A_xR*self.Cd_xR*self.dy_xR*(self.dy_xR*x[11]+x[1]))
        dpsidotdot_dydot=-.5*self.Izz*self.rho*(2*self.A_yBa*self.Cd_yBa*self.dx_yBa* \
                (self.dx_yBa*x[11]+x[3])+2*self.A_yF*self.Cd_yF*self.dx_yF*(self.dx_yF*x[11]+x[3]))
        dpsidotdot_dphi=(self.Fb/self.Izz)*(self.psi_bx*cos(x[8])*sin(x[6]))
        dpsidotdot_dtheta=(self.Fb/self.Izz)*(-self.psi_by*cos(x[8])-self.psi_bx*sin(x[6])*sin(x[8]))
        dpsidotdot_dpsidot=-.5*self.Izz*self.rho*(2*self.A_xL*self.Cd_xL*self.dy_xL**2* \
                (self.dy_xL*x[11]+x[1])+2*self.A_xR*self.Cd_xR*self.dy_xR**2*(self.dy_xR*x[11]+x[1])+ \
                2*self.A_yBa*self.Cd_yBa*self.dx_yBa**2*(self.dx_yBa*x[11]+x[3])+2*self.A_yF*self.Cd_yF* \
                self.dx_yF**2*(self.dx_yF*x[11]+x[3]))
        dpsidotdot_dTL=self.psi_L/self.Izz
        dpsidotdot_dTR=-self.psi_R/self.Izz
        dpsidotdot_dTF=self.psi_F/self.Izz
        dpsidotdot_dTB=-self.psi_B/self.Izz

        # A should be a 12x12
        # B should be a 12x8
        # C should be a 10x12 (12x12 with DVL)
        # D should be a 10x8 (12x8 with DVL)
        A=np.array([[0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, dxdotdot_dxdot, 0, 0, 0, 0, 0, 0, dxdotdot_dtheta, 0, dxdotdot_dpsi, 0],
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, dydotdot_dydot, 0, 0, dydotdot_dphi, 0, 0, 0, dydotdot_dpsi, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, dzdotdot_dzdot, dzdotdot_dphi, 0, dzdotdot_dtheta, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, dphidotdot_dydot, 0, dphidotdot_dzdot, dphidotdot_dphi, dphidotdot_dphidot, dphidotdot_dtheta, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, dthetadotdot_dxdot, 0, 0, 0, dthetadotdot_dzdot, dthetadotdot_dphi, 0, dthetadotdot_dtheta, dthetadotdot_dthetadot, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
            [0, dpsidotdot_dxdot, 0, dpsidotdot_dydot, 0, 0, dpsidotdot_dphi, 0, dpsidotdot_dtheta, 0, dxdotdot_dpsi, dpsidotdot_dpsidot]])

        B=np.array([[0, 0, 0, 0, 0, 0, 0, 0],
            [dxdotdot_dTL, dxdotdot_dTR, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, dydotdot_dTF, dydotdot_dTB, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, dzdotdot_dTFL, dzdotdot_dTFR, dzdotdot_dTBL, dzdotdot_dTBR],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, dphidotdot_dTFL, dphidotdot_dTFR, dphidotdot_dTBL, dphidotdot_dTBR],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, dthetadotdot_dTFL, dthetadotdot_dTFR, dthetadotdot_dTBL, dthetadotdot_dTBR],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [dpsidotdot_dTL, dpsidotdot_dTR, dpsidotdot_dTF, dpsidotdot_dTB, 0, 0, 0, 0]])

        C=np.array([[0, dxdotdot_dxdot, 0, 0, 0, 0, 0, 0, dxdotdot_dtheta, 0, dxdotdot_dpsi, 0],
            [0, 0, 0, dydotdot_dydot, 0, 0, dydotdot_dphi, 0, 0, 0, dydotdot_dpsi, 0],
            [0, 0, 0, 0, 0, dzdotdot_dzdot, dzdotdot_dphi, 0, dzdotdot_dtheta, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]])

        D=np.array([[dxdotdot_dTL, dxdotdot_dTR, 0, 0, 0, 0, 0, 0],
            [0, 0, dydotdot_dTF, dydotdot_dTB, 0, 0, 0, 0],
            [0, 0, 0, 0, dzdotdot_dTFL, dzdotdot_dTFR, dzdotdot_dTBL, dzdotdot_dTBR],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0]])

        if np.linalg.matrix_rank(control.ctrb(A,B))<A.shape[0]:
            print("System is not fully controllable")
        else:
            print("System is fully controllable")
        #  print np.sum(control.obsv(A,C),axis=1)
        if np.linalg.matrix_rank(control.obsv(A,C))<A.shape[0]:
            print("System is not fully observable")
        else:
            print("System is fully observable")
        state_space=StateSpace(A,B,C,D,dt)
        return state_space

    def LQR(self,A,B,Q,R):
        """Solve the discrete time lqr controller.
        x[k+1] = A x[k] + B u[k]
        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
        """
        #  http://www.mwm.im/lqr-controllers-with-python/
        #ref Bertsekas, p.151

        #first, try to solve the ricatti equation
        S = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))
        #compute the LQR gain
        K = np.matrix(scipy.linalg.inv(B.T*S*B+R)*(B.T*S*A))
        eigVals, eigVecs = scipy.linalg.eig(A-B*K)
        return K, S, eigVals

    def feed_forward(self,A,B,C,D,K):
        """creates the feed forward matrix F to compute
        control inputs using a desired state"""

        F=np.linalg.pinv(-C*np.linalg.inv(A-B*K)*B)

        if np.linalg.matrix_rank(control.ctrb(A-B*K,B*F))<A.shape[0]:
            print("System is not fully controllable")
        else:
            print("System is fully controllable")
        if np.linalg.matrix_rank(control.obsv(A-B*K,C-D*K))<A.shape[0]:
            print("System is not fully observable")
        else:
            print("System is fully observable")

        return F

    def sub_planner(self, point):
        """given a set of points to hit on a trajectory,
        determine the desired state w/velocity to send
        to the controller"""
        #  for point in points:
        point=np.array(point)
        #  print np.max(np.abs(self.x_est-point))
        #  sub=ax.scatter(self.position[-1,0],self.position[-1,1],self.position[-1,2],c='r')
        count=0
        while np.max(np.abs(self.x_est-point))>0.5:
            u=self.control_output(self.x_est,point)
            #  self.x_est=np.array(self.kinematics(self.x_est,u))
            self.x_est=np.array(self.kinematics(self.x_est,u*self.u_scale))
            position=[self.x_est[0],self.x_est[2],self.x_est[4]]
            if count%50==0:
                #  self.position=np.append(self.position,[[self.x_est[0],self.x_est[2],self.x_est[4]]],axis=0)
                ax.scatter(position[0],position[1],position[2],c='r',marker='.')
                sub=ax.scatter(self.x_est[0],self.x_est[2],self.x_est[4],c='dimgrey',s=100)
                plt.pause(0.001)
                sub.remove()
                position=[self.x_est[0],self.x_est[2],self.x_est[4]]
            count+=1
            #  print self.x_est[0],self.x_est[2],self.x_est[4]
            #  print self.Fb,self.m*self.g
            #  sys.exit()

    def control_output(self,x,x_desired,con_type='position_hold'):
        """uses a previously computed gain K and F to 
        create a control input for the motors
        Inputs:
            x (array [1x12]): current state
            x_desired (array [1x12]): desired state
            type (str): which controller to use
        """
        #  r=np.array([0,0,0,x_desired[6],x_desired[7],x_desired[8],x_desired[9],x_desired[10],x_desired[11],x_desired[4]])
        if con_type=='position_hold':
            #  print np.dot(-self.K_hold,np.transpose(x))
            #  print np.dot(self.F_hold,np.transpose(x_desired)).shape
            u=np.array(np.dot(-self.K_hold,np.transpose(x-x_desired)))#+\
                    #  np.array(np.dot(self.F_hold,np.transpose(r)))
        elif con_type=='moving':
            u=-self.K_move*x+self.F_move*x_desired

        #  print u[0]
        return u[0]
    
if __name__ == '__main__':
    start=time.time()
    sub_control=Controller(['position_hold'])
    sub_control.position=np.zeros((1,3))
    a=Automaton()
    current=np.random.choice(['x13q0','x14q0','x17q0','x21q0','x22q0'])
    for state in a.automata:
        if len(current)==5:
            name1=current[0:3]
            name2=current[3:]
        else:
            name1=current[0:2]
            name2=current[2:]
        if (state['name1']==name1) and (state['name2']==name2):
            start_point=state['location']
            start_point=list(np.random.normal(start_point,2))
            sub_control.position[0,:]=np.array(start_point)
            sub_control.x_est=np.array([start_point[0],0,start_point[1],0,start_point[2],0,0,0,0,0,0,0])
            print(start_point)

    fig=plt.figure()
    ax=fig.add_subplot(111,projection='3d')
    ax.scatter(15,5,5,s=75,marker='s',c='w')

    y_top=np.linspace(3,7,100)
    z_side=np.linspace(2,7,100)
    x=3.5*np.ones(100)
    y_side1=3*np.ones(100)
    y_side2=7*np.ones(100)
    z_top=2*np.ones(100)
    ax.plot(x,y_top,z_top,label='Gate',c='k')
    ax.plot(x,y_side1,z_side,c='k')
    ax.plot(x,y_side2,z_side,c='k')

    x=np.linspace(-2,18,10)
    y=np.linspace(-2,12,10)
    X,Y=np.meshgrid(x,y)
    Z=0.5*np.ones((10,10))
    ax.plot_surface(X,Y,Z,color='lightskyblue',alpha=0.25)

    ax.set_xlim(-2,18)
    ax.set_ylim(-2,12)
    ax.set_zlim(-2,8)
    ax.invert_zaxis()
    ax.set_facecolor('lightskyblue')
    while sub_control.x_est[4]>0.5:
        current,point=a.planner(current,a.automata)
        sub_control.sub_planner([point[0],0,point[1],0,point[2],0,0,0,0,0,0,0])
    sub=ax.scatter(sub_control.x_est[0],sub_control.x_est[2],sub_control.x_est[4],c='dimgrey',s=100)
    #  print time.time()-start
    plt.show()
    #  points=[[0,0,0,0,20,0,0,0,0,0,0,0],[0,0,0,0,5,0,0,0,0,0,0,0]]
    #  dt=1/25
    #position hold controller

    #  sub_control=Controller()
    #  #linearization point
    #  lin_x=[0,0,0,0,0,0,0,0,0,0,0,0]
    #  lin_u=[0,0,0,0,0,0,0,0]
    #  #max R
    #  R_max=0.75
    #  #max Q
    #  x_max=5
    #  x_dot_max=0.1
    #  y_max=5
    #  y_dot_max=0.1
    #  z_max=0.5
    #  z_dot_max=1
    #  phi_max=5
    #  phi_dot_max=0.1
    #  theta_max=5
    #  theta_dot_max=0.1
    #  psi_max=5
    #  psi_dot_max=0.1
    #  Q_max=np.array([x_max,x_dot_max,y_max,y_dot_max,z_max,z_dot_max,
    #      phi_max,phi_dot_max,theta_max,theta_dot_max,psi_max,psi_dot_max])

    #  sub_control.create_controller(lin_x,lin_u,dt,Q_max,R_max,'position_hold')
