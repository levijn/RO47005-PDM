import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
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
            t_collison = self.compute_TTC(self.neighbors[i])
            if t_collison == False:
                break
            pos_rel = self.neighbors[i].position - self.position
            vel_rel = self.neighbors[i].velocity - self.velocity
            pos_rel_mag = np.sqrt(np.sum(pos_rel**2))
            pos_rel_norm = pos_rel/pos_rel_mag
            pos_rel_norm_perp = np.array([-pos_rel_norm[1],pos_rel_norm[0]])
            theta = np.arctan((self.neighbors[i].radius + self.radius)/ pos_rel_mag)
            edge1 = pos_rel_norm * np.cos(theta) - pos_rel_norm_perp * np.sin(theta)
            edge2 = pos_rel_norm * np.cos(theta) + pos_rel_norm_perp * np.sin(theta)
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
            v = ref + (ref-self.max_speed)*i/100
            col = self.velocity_check(v, VOs)
            if col == False :
                break
            v = ref - (ref-self.min_speed)*i/100
            col = self.velocity_check(v, VOs)
            if col == False:
                break
        return v   
    
    def velocity_check(self,v,VOs):
        is_in = False
        for VO in VOs:
            edge1, edge2, apex, theta = VO
            v_trans = v - apex
            T1 = np.arccos(np.dot(v_trans,edge1)/(np.linalg.norm(v_trans)*np.linalg.norm(edge1)))
            T2 = np.arccos(np.dot(v_trans,edge2)/(np.linalg.norm(v_trans)*np.linalg.norm(edge2)))
            if T1 <= theta and T2 <= theta:
                is_in = True
                break
        return is_in
                   
    def update_position(self,agents,dt):
        self.detect_neighbors(agents)
        VOs = self.compute_VO()
        v = self.choose_speed(self.preferred_velocity, VOs)
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

class Obstacle:
    def __init__(self, pos, vel, radius):
        self.position = pos
        self.velocity = vel
        self.radius = radius
    def update_position(self,dt):
        self.position = self.position + self.velocity * dt
        
agent = VelocityObstacle(np.array([0,0]), np.array([0,0]), 20, np.array([1,1]),np.array([2,2]), 2)
obs1 = Obstacle(np.array([2,2]), np.array([0,0]), 1)

def run(i):
    ax.clear()
    ax.set_xlim([0,3])
    ax.set_ylim([0,3])
    agent.update_position(np.array([obs1]), 0.05)
    obs1.update_position(0.05)
    a = ax.plot(agent.position[0],agent.position[1], 'o', color ='blue')
    b = ax.plot(obs1.position[0],obs1.position[1] , 'o', color = 'red')
    #time.sleep(0.1)
    return a,b
fig, ax = plt.subplots(1,1)

ani = animation.FuncAnimation(fig, run, frames=100, repeat = False, blit = False)
