import numpy as np
import math
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

class Airplane_Env:

    def __init__(self):
        self.W = 9200.0
        self.Surface_area =27.87
        self.g = 9.81
        self.rho_air = 1.225
        self.tau = 2.0  # seconds between state updates

        self.goal = [5000,0]
        self.traj = np.array([])
        self.controlls  = np.array([])

        #x,y,h,V,gamma,chi
        self.init_pos = np.array([0.0,0.0,1000.0,100.0,0.26,np.pi], dtype=np.float32)
        self.reset()
        self.old_alpha = 0
        self.old_mu = 0
        #self.observation_space = spaces.Box(-high, high, dtype=np.float32)


    def step(self, action):

        alpha,mu = self.action_to_control(action)
        #self.old_alpha = alpha
        #self.old_mu = mu

        self.old_state = self.state

        self.state = self.state + self.tau * self.dynamics(self.state,alpha,mu)
        self.traj = np.append(self.traj,[self.state],axis=0)
        self.controlls = np.append(self.controlls,np.array([alpha,mu])*90.0,axis=0)
        
        done = (self.state[2] <= 0)
        done = bool(done)

        if np.linalg.norm(self.state[0:2]-self.goal) < 6000 and self.state[2] > 0:
            if self.state[2] <= 100 and np.linalg.norm(self.state[0:2]-self.goal) < 100:
                reward = 1000
                done = true
            else:
               reward = 100 * np.linalg.norm(self.state[0:2]-self.goal) / np.linalg.norm(self.init_pos[0:2]-self.goal)
        elif not done:
            reward = -1
        else:
            reward = -100

        return np.array(self.state), reward, done, {}

    def action_to_control(self,action):
        assert(action >= 0 and action < 26*5)

        alpha_list = np.array(range(26)) / 180.0 * np.pi
        mu_list = np.array([-90,-45,0,45,90]) / 180.0 * np.pi

        #alpha_idx * mu_idx
        alpha_idx = action / 5
        mu_idx = action % 5

        alpha = alpha_list[alpha_idx]
        mu = mu_list[mu_idx]

        return alpha,mu

    def reset(self):
        self.state = self.init_pos
        self.Vw = 10.0
        self.chiw = 0.0
        self.traj = np.array([self.state])
        self.controlls  = np.array([])
        return np.array(self.state, dtype=np.float32)

    def dynamics(self,state,alpha,mu):
        x,y,h,V,gamma,chi = state
        
        qS = 0.5 * self.rho_air * V*V * self.Surface_area
        CL, CD = self.find_coeff(alpha)
        L = CL * qS
        D = CD * qS
        
        xdot = V*np.cos(gamma)*np.cos(chi) + self.Vw*np.cos(self.chiw)
        ydot = V*np.cos(gamma)*np.sin(chi) + self.Vw*np.sin(self.chiw)
        hdot = V*np.sin(gamma)
        Vdot = -self.g/self.W*(D + self.W*np.sin(gamma))
        gammadot = self.g/V*(L/self.W*np.cos(mu) - np.cos(gamma))
        chidot = self.g*L*np.sin(mu) / (V*np.cos(gamma)*self.W)

        return np.array([xdot,ydot,hdot,Vdot,gammadot,chidot], dtype=np.float32)

    def find_coeff(self,alpha):

        alpha = alpha * 180.0 / np.pi
        
        table_alpha = [0.0,5.0,10.0,15.0,20.0,25.0]
        table_CD = [0.049,0.039,0.082,0.184,0.365,0.583]
        table_CL = [0.025,0.365,0.747,1.102,1.376,1.596]
        
        CD = interp1d(table_alpha, table_CD,kind='cubic',fill_value="extrapolate")
        CL = interp1d(table_alpha, table_CL,kind='cubic',fill_value="extrapolate")
            
        return CL(alpha),CD(alpha)

    def get_traj(self):
        return self.traj

    def get_controlls(self):
        return self.controlls

    def plot_traj(self):
        fig = plt.figure()
        ax = plt.axes(projection='3d')

        traj = self.get_traj()
        zline = traj[:,2]
        xline = traj[:,0]
        yline = traj[:,1]
        ax.plot3D(xline, yline, zline, 'gray')

        plt.xlabel('x')
        plt.ylabel('y')
        plt.show()

if __name__ == "__main__":
    a = Airplane_Env()
    for i in range(26*5):
        print(a.action_to_control(i))