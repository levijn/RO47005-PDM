import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
import time
class VelocityObstacle:

    def __init__(self, position, velocity, search_radius, preferred_velocity, max_speed, radius):
        
        self.position = position
        self.velocity = velocity
        self.radius = radius
        self.search_radius = search_radius
        self.preferred_velocity = preferred_velocity
        self.max_speed = max_speed
        self.min_speed = np.array([0,0])

    def compute_VO(self):
        VO = []
        for i in range(len(self.neighbors)):
            #t_collison = self.compute_TTC(self.neighbors[i])
            #if t_collison == False:
            #    break
            
            pos_rel = self.neighbors[i].position - self.position
            vel_rel = self.neighbors[i].velocity - self.velocity
            pos_rel_mag = np.sqrt(np.sum(pos_rel**2))
            #pos_rel_norm = pos_rel/pos_rel_mag
            #pos_rel_norm_perp = np.array([pos_rel_norm[1],-pos_rel_norm[0]])
            theta = np.arctan((self.neighbors[i].radius + self.radius)/ pos_rel_mag)
            
            rotation_matrix_1 = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
            rotation_matrix_2 = np.array([[np.cos(-theta), -np.sin(-theta)], [np.sin(-theta), np.cos(-theta)]])
            edge1 = np.dot(rotation_matrix_1, pos_rel) *10
            edge2 = np.dot(rotation_matrix_2, pos_rel) * 10

            
            
           # edge1 = pos_rel_norm * np.cos(theta) - pos_rel_norm_perp * np.sin(theta)
            #edge2 = pos_rel_norm * np.cos(theta) + pos_rel_norm_perp * np.sin(theta)
            VO.append(np.array([edge1,edge2,vel_rel,theta]))
        return np.array(VO)
    
    def compute_TTC(self,neighbor): ###Not really used now, only to check for possible collisions. Can be used later for prioritisation or something
        pos_rel = neighbor.position - self.position
        vel_rel = neighbor.velocity - self.velocity
        R = self.radius + neighbor.radius
        a = np.dot(vel_rel, vel_rel)
        b = 2 * np.dot(pos_rel, vel_rel)
        c = np.dot(pos_rel, pos_rel) - R**2
        
        discriminant = b**2 - 4*a*c
    
        if discriminant < 0:
            return False  # No real roots, no collision
    
        t1 = np.max((-b + np.sqrt(discriminant)) / (2*a),0)
        t2 = np.max((-b - np.sqrt(discriminant)) / (2*a),0)
        
        ttc = t1 #To be updated if necessary
        return ttc   
   
    def choose_speed(self,ref,VOs): ###To be updated, now just brakes or speeds up
        
        for i in range(100):
            v = ref + (ref-self.max_speed)*i/99
            col = self.velocity_check(v, VOs)
            if col == False :
                break
            v = ref - (ref-self.min_speed)*i/99
            col = self.velocity_check(v, VOs)
            if col == False:
                break
        return v   
    
    def velocity_check(self,v,VOs):
        is_in = False
        for VO in VOs:
            #edge1,edge2, vel_rel,theta
            edge1, edge2, apex, theta = VO
            v_trans = v - apex
            T1 = np.arccos(np.dot(v_trans,edge1)/(np.linalg.norm(v_trans)*np.linalg.norm(edge1)))
            T2 = np.arccos(np.dot(v_trans,edge2)/(np.linalg.norm(v_trans)*np.linalg.norm(edge2)))
            
            if T1 <= 2*theta and T2 <= 2*theta:
                is_in = True
                break
            
        return is_in
                   
    def update_position(self,agents,dt):
        self.detect_neighbors(agents)
        self.VOs = self.compute_VO()
        print(len(self.neighbors), len(self.VOs))
        v = self.choose_speed(self.preferred_velocity, self.VOs)
        self.velocity = v
        self.position = self.position + v * dt
        print(f"Agent v:{v}, p:{self.position}")
        
    def detect_neighbors(self, agents):
        self.neighbors = []
        for n in agents:
            pos_rel = n.position - self.position
            d = np.sqrt(np.sum(pos_rel**2))
            if d <= self.search_radius:
                self.neighbors.append(n)
                
    def plot_vo_cones(self,ax):
        for VO in self.VOs:
            edge1, edge2, vel_rel, theta = VO

            # Calculate the apex of the VO cone
            apex = self.position + vel_rel
            
            ax.plot([self.position[0], apex[0]], [self.position[1], apex[1]], 'b--')
            # Plotting the edges of the VO cone
            ax.plot([apex[0], apex[0] + edge1[0]], [apex[1], apex[1] + edge1[1]], 'g--')
            ax.plot([apex[0], apex[0] + edge2[0]], [apex[1], apex[1] + edge2[1]], 'g--')

            # Optionally, fill the VO cone (you can remove this part if you only want the edges)
            cone_x = [apex[0], apex[0] + edge1[0], apex[0] + edge2[0]]
            cone_y = [apex[1], apex[1] + edge1[1], apex[1] + edge2[1]]
            ax.fill(cone_x, cone_y, 'g', alpha=0.3)

class Obstacle:
    def __init__(self, pos, vel, radius):
        self.position = pos
        self.velocity = vel
        self.radius = radius
    def update_position(self,dt):
        self.position = self.position + self.velocity * dt
        
agent = VelocityObstacle(np.array([2,0]), np.array([0,0]), 2, np.array([0,1]),np.array([0,2]), 0.1)
obs1 = Obstacle(np.array([0,2]), np.array([1,0]), 0.1)
obs = [obs1]
for n in range(5):
    obs.append(Obstacle(np.array([4/5*n,0.2+3.8/5*n]), np.array([-0.3,0]), 0.1))
 
def run(i):
    ax.clear()
    ax.set_xlim([0,4])
    ax.set_ylim([0,4])
    agent.update_position(obs, 0.05)
    agent_circle = Circle((agent.position[0], agent.position[1]), agent.radius, color='blue')
    a = ax.add_patch(agent_circle)
    b = []
    for ob in obs:
        ob.update_position(0.05)
        ob_circle = Circle((ob.position[0], ob.position[1]), ob.radius, color='blue')
        b.append(ax.add_patch(ob_circle))
    #time.sleep(0.1)
    agent.plot_vo_cones(ax)
    return a,b
fig, ax = plt.subplots(1,1)

ani = animation.FuncAnimation(fig, run, frames=500, repeat = False, blit = False)
