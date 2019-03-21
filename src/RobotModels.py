import numpy as np
import matplotlib.pyplot as pl
from mpl_toolkits.mplot3d.axes3d import Axes3D

class Link():

    def __init__(self, d, a, alpha, *args):
        self.d = d
        self.a = a
        self.alpha = alpha
            
class SerialLink():

    def __init__(self, Links, origin = [0,0,0], initY = [], name = 'robot', *args):
        
        self.origin = np.array(origin).reshape(1,3)
        self.world_frame = Link(0,0,0)
        Links.insert(0,self.world_frame)
        self.Links = Links
        self.num_links = len(self.Links)
        self.dh_table = self._create_dh_table()
        self.name = name
        self.dim = 3

        if initY == []:
            self.currentY = np.zeros([1, self.num_links-1])
        else:
            self.currentY = initY
            
        self.currentX, self.currentR = self.fkin(self.currentY)
 
    def get_num_links(self):
        return self.num_links-1
       
    def _create_dh_table(self):
        dh_table = np.empty([self.num_links,3])
        for i in range(0, self.num_links):
            dh = np.hstack((self.Links[i].alpha, self.Links[i].a, self.Links[i].d))
            dh_table[i, :] = dh
        return dh_table
    
    def add_link(self, Link):      
        self.Links.append(Link)
        self.num_links = len(self.Links)
        self.dh_table = self._create_dh_table()
    
    def set_Y(self, Q):
        self.currentY = Q
        self.currentX, self.currentR = self.fkin(self.currentY)
        
    def T(self, frame, theta):
        alpha = self.Links[frame].alpha
        a = self.Links[frame].a
        d = self.Links[frame].d
        
        H = np.array([[np.cos(theta), -np.sin(theta), 0, a],
           [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -d*np.sin(alpha)],
           [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha),  np.cos(alpha),  d*np.cos(alpha)],
           [0, 0, 0, 1]
           ])
    
        return H
    
    def fkin(self, q, to_frame = -1, from_frame = 0):
        if to_frame < 0:
            to_frame = self.num_links
        H = np.eye(4)
        q = np.column_stack([q, 0])
        for i in range(from_frame, to_frame):
            H = np.matmul(H, self.T(i, q[0,i]))        
        P = H[:self.dim,-1].reshape(1,self.dim) + self.origin
        R = H[:self.dim,:self.dim]
        return P, R
    
    def fkin_all(self,Q, include_orientations = False):
    
        Positions = []
        Orientations = []
        
        for j in range(0,self.num_links):
            P,R = self.fkin(Q,j+1)
            Positions.append(P)
            Orientations.append(R)
            
        arm_positions = np.vstack((Positions[:]))
        arm_orientations = Orientations
        if include_orientations:
            return arm_positions, arm_orientations
        else:
            return arm_positions

    
    def plot3D(self, Q = [], elev = 30, azim = 30, floating = False, block = False):
        ax = pl.axes(projection='3d')
        if Q == []:
            Q = self.currentY
            
        r = np.sum(self.dh_table[:,1])
        xmin = -r + self.origin[0,0]
        xmax = r + self.origin[0,0]
        ymin = -r + self.origin[0,1]
        ymax = r + self.origin[0,1]
        zmin = -r + self.origin[0,2]
        zmax = r + self.origin[0,2]
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)
        ax.set_zlim(zmin, zmax)
        
        "Plot robot arm"
        arm = self.fkin_all(Q)  
        if not floating:
            ax.plot3D([self.origin[0,0],self.origin[0,0]],[self.origin[0,1],self.origin[0,1]],[zmin,self.origin[0,2]],'k-',linewidth=2)        

        ax.plot3D(arm[:,0],arm[:,1],arm[:,2],'b-',linewidth=5)
        ax.plot3D(arm[:,0],arm[:,1],arm[:,2],'ko',linewidth=5)

        ax.view_init(elev, azim)
        pl.tight_layout()
        pl.show(block=block)
        return ax
    
        

