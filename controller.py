from car import Car
import numpy as np
import matplotlib.pyplot as plt


if __name__ == '__main__':
    car = Car(velocity=0, steering_angle=0)
    
    dt = 0.01
    simulation_time = 10
    time = np.linspace(0, simulation_time, int(simulation_time/dt)+1)

    reference_path_x = 2 * np.cos(0.1*time)
    reference_path_y = 3 * np.sin(0.2*time)
    
    car_path_x = []
    car_path_y = []
    
    
    for i in range(len(time)):
        #calculate the error between the reference path and the car path
        error_x = reference_path_x[i] - car.x
        error_y = reference_path_y[i] - car.y
        
        error = np.sqrt(error_x**2 + error_y**2)
        # p controller
        
        kp = 0.1
        acceleration = kp * error
        
        # calculate the steering angle
        kp_steering = 1.0
        
        car.update([acceleration, 0], dt)
        
        car_path_x.append(car.x)
        car_path_y.append(car.y)
        
        
    
    
    # plt.plot(reference_path_x, reference_path_y, label='reference path')
    # plt.plot(car_path_x, car_path_y, label='car path')
    # plt.legend()
    # plt.show()
