import matplotlib.pyplot as plt
import numpy as np


def normalize_angle(angle):
    """Normalize an angle to [-pi, pi]"""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


class Car:
    def __init__(self, x=0.0, y=0.0, theta=0.0, velocity=0.0, steering_angle=0, length=1.0):
        self.x = x         # x-coordinate of the car
        self.y = y         # y-coordinate of the car
        self.theta = theta # orientation angle of the car (in radians)
        self.velocity = velocity   # linear velocity of the car
        self.steering = steering_angle       # steering angle of the car (in radians)
        self.length = length       # length of the car


    def update(self, action, dt):
        """
        Update the state of the car using the action and time step
        Args:
            action (list): action to be taken by the car [acceleration, steering]
            dt (float): time step
        """
        self.x += self.velocity * np.cos(self.theta) * dt
        self.y += self.velocity * np.sin(self.theta) * dt
        self.theta += self.velocity * np.tan(self.steering) / self.length * dt
        self.theta = normalize_angle(self.theta)
        self.velocity += action[0] * dt
        self.steering += action[1] * dt
    

if __name__ == "__main__":
    
    car = Car(velocity=1.0, steering_angle=0.1)
    path_x = []
    path_y = []
    
    for i in range(10000):
        dt = 0.01
        car.update([0, 0.1], dt)
        
        path_x.append(car.x)
        path_y.append(car.y)
        print(car.x, car.y, car.theta, car.velocity, car.steering)
    
    # plot the path
    plt.plot(path_x, path_y)
    
    plt.show()
    