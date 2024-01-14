# pylint: skip-file
from dataclasses import dataclass
from math import radians
import numpy as np

from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

from kinematic_model import KinematicBicycleModel
from car_description import CarDescription
from stanley_controller import StanleyController

from dubins import Dubins
from create_environment import load_environment
from RRT_Star_Dubins import RRT_Star_Dubins
from VelocityObstacle import VelocityObstacle, Obstacle


class Simulation:
    def __init__(self):
        fps = 50.0
        self.dt = 1/fps
        self.map_size_x = 20 // 2
        self.map_size_y = 20 // 2
        self.frames = 1200
        self.loop = False
        self.save = True
        self.cam_tracks_car = True


class Path:
    def __init__(self, px, py, yaw):
        self.px = px
        self.py = py
        self.pyaw = yaw
    

class Car:

    def __init__(self, init_x, init_y, init_yaw, px, py, pyaw, delta_time):

        # Model parameters
        self.x = init_x
        self.y = init_y
        self.yaw = init_yaw
        self.delta_time = delta_time
        self.time = 0.0
        self.velocity = 0.0
        self.wheel_angle = 0.0
        self.angular_velocity = 0.0
        max_steer = radians(30)
        wheelbase = 2.0

        # Acceleration parameters
        self.preferred_velocity = 5.0
        self.target_velocity = self.preferred_velocity
        self.max_acceleration = 2.0
        self.time_to_reach_target_velocity = 5.0
        self.required_acceleration = self.target_velocity / self.time_to_reach_target_velocity

        # Tracker parameters
        self.px = px
        self.py = py
        self.pyaw = pyaw
        self.k = 8.0
        self.ksoft = 1.0
        self.kyaw = 0.01
        self.ksteer = 0.0
        self.crosstrack_error = None
        self.target_id = None

        # Description parameters
        self.colour = 'black'
        overall_length = 3.0
        overall_width = 2.0
        tyre_diameter = 0.5
        tyre_width = 0.25
        axle_track = 1.4
        rear_overhang = 0.5 * (overall_length - wheelbase)
        self.tracker = StanleyController(self.k, self.ksoft, self.kyaw, self.ksteer, max_steer, wheelbase, self.px, self.py, self.pyaw)
        self.kinematic_bicycle_model = KinematicBicycleModel(wheelbase, max_steer, self.delta_time)
        self.description = CarDescription(overall_length, overall_width, rear_overhang, tyre_diameter, tyre_width, axle_track, wheelbase)
        
        self.testindex = 0

    
    def get_required_acceleration(self):
        margin = 0.1
        if self.velocity < self.target_velocity - margin:
            self.required_acceleration = self.max_acceleration
        elif self.velocity > self.target_velocity + margin:
            self.required_acceleration = -self.max_acceleration
        else:
            self.required_acceleration = 0.0
            
        self.time += self.delta_time
        return self.required_acceleration
    

    def plot_car(self):
        return self.description.plot_car(self.x, self.y, self.yaw, self.wheel_angle)


    def drive(self):
        acceleration = self.get_required_acceleration()
        
        # calculates the steering angle for a given position and yaw
        self.wheel_angle, self.target_id, self.crosstrack_error = self.tracker.stanley_control(self.x, self.y, self.yaw, self.velocity, self.wheel_angle)
        
        self.x, self.y, self.yaw, self.velocity, _, _ = self.kinematic_bicycle_model.update(self.x, self.y, self.yaw, self.velocity, acceleration, self.wheel_angle)


@dataclass
class Fargs:
    ax: plt.Axes
    sim: Simulation
    path: Path
    car: Car
    car_outline: plt.Line2D
    front_right_wheel: plt.Line2D
    front_left_wheel: plt.Line2D
    rear_right_wheel: plt.Line2D
    rear_left_wheel: plt.Line2D
    rear_axle: plt.Line2D
    car_velocity: plt.Line2D
    annotation: plt.Annotation
    target: plt.Line2D
    path_plot: plt.plot
    moving_obstacles: list
    moving_obstacles_plot: list
    agent: VelocityObstacle
   

def animate(frame, fargs):

    ax                = fargs.ax
    sim               = fargs.sim
    path              = fargs.path
    car               = fargs.car
    car_outline       = fargs.car_outline
    front_right_wheel = fargs.front_right_wheel
    front_left_wheel  = fargs.front_left_wheel
    rear_right_wheel  = fargs.rear_right_wheel
    rear_left_wheel   = fargs.rear_left_wheel
    rear_axle         = fargs.rear_axle
    car_velocity_plot = fargs.car_velocity
    annotation        = fargs.annotation
    target            = fargs.target
    path_plot         = fargs.path_plot
    moving_obstacles  = fargs.moving_obstacles
    moving_obstacles_plot = fargs.moving_obstacles_plot
    agent             = fargs.agent
    
    # Update moving obstacles
    for i, obstacle in enumerate(moving_obstacles):
        obstacle.update_position(sim.dt)
        moving_obstacles_plot[i][0].set_data(obstacle.position[0], obstacle.position[1])
    
    # calculate new target velocity for the car if it is close to a moving obstacle
    # get velocity of car
    current_car_vel = car.velocity
    direction = np.array([np.cos(car.yaw), np.sin(car.yaw)]) # create unit vector of direction of car
    velocity_array_current = current_car_vel * direction
    velocity_array_preffered = car.preferred_velocity * direction
    
    # set velocity of agent to current velocity of car
    # offset the position to front of car
    offset = direction * 2
    agent.position = np.array([car.x, car.y]) + offset
    agent.velocity = velocity_array_current
    
    agent.detect_neighbors(moving_obstacles)
    if len(agent.neighbors) > 0:
        agent.VOs = agent.compute_VO()
        new_desired_speed = agent.choose_speed(velocity_array_preffered, agent.VOs, sim.dt)
    else:
        new_desired_speed = velocity_array_preffered
        
    # calculate the distance to the goal and decalerate if close to goal
    distance_to_goal = np.sqrt((car.x - path.px[-1])**2 + (car.y - path.py[-1])**2)
    if distance_to_goal < 8:
        new_desired_speed = np.array([0,0])
        car.target_velocity = 0
        car.preferred_velocity = 0

    #set speed of car to new desired speed
    car.target_velocity = np.sqrt(new_desired_speed[0]**2 + new_desired_speed[1]**2)
    
    # plot the car velocity as a line in the direction of the velocity
    car_velocity_plot[0].set_data([car.x + offset[0], car.x + offset[0] + velocity_array_current[0]], [car.y + offset[1], car.y + offset[1] + velocity_array_current[1]])

    # Camera tracks car
    if sim.cam_tracks_car:
        ax.set_xlim(car.x - sim.map_size_x, car.x + sim.map_size_x)
        ax.set_ylim(car.y - sim.map_size_y, car.y + sim.map_size_y)

    # Drive and draw car
    car.drive()
    outline_plot, fr_plot, rr_plot, fl_plot, rl_plot = car.plot_car()
    car_outline.set_data(*outline_plot)
    front_right_wheel.set_data(*fr_plot)
    rear_right_wheel.set_data(*rr_plot)
    front_left_wheel.set_data(*fl_plot)
    rear_left_wheel.set_data(*rl_plot)
    rear_axle.set_data(car.x, car.y)
    
    
    path_plot.set_data(car.px, car.py)
    # Annotate car's coordinate above car
    annotation.set_text(f'{car.x:.1f}, {car.y:.1f}')
    annotation.set_position((car.x, car.y + 4))

    plt.title(f'{sim.dt*frame:.2f}s', loc='right')
    plt.xlabel(f'Speed: {car.velocity:.2f} m/s', loc='left')

    return car_outline, front_right_wheel, rear_right_wheel, front_left_wheel, rear_left_wheel, rear_axle, target,


def main(show_simulation):
    sim  = Simulation()
    sim.save = not show_simulation
    interval = sim.dt * 10**3

    fig = plt.figure(figsize=(10, 10))
    ax = plt.axes()
    ax.set_aspect('equal')
    
    # Load and Draw map
    map_name = 'scenario3'
    map1 = load_environment(map_name)
    patches = map1.get_patches()
    for patch in patches:
        ax.add_patch(patch)
    # plot the goal and start position
    ax.plot(map1.start.x, map1.start.y, 'o', color='green', markersize=10)
    ax.plot(map1.goal.x, map1.goal.y, 'o', color='red', markersize=10)
    
    # set the limits of the plot to map size
    ax.set_xlim(0, map1.size[0])
    ax.set_ylim(0, map1.size[1])

    # Set some parameters for the RRT* algorithm
    n_max = 500
    gamma = 1500
    r_goal = 0.5
    min_dist_nodes = 0
    goal_sample_rate = 50
    dubins = Dubins(4, 0.3)
    
    # set the name for saving the plot
    save_name = f"sim_rrt_star_dubins_{map_name}_g_{gamma}_n_{n_max}"
    
    # run the rrt* algorithm and save plots every quarter of the iterations
    rrtstar = RRT_Star_Dubins(map1, gamma, n_max=n_max, min_dist_nodes=min_dist_nodes, goal_sample_rate=goal_sample_rate, dubins=dubins)
    if sim.save:
        rrtstar.create_intermediate_plots(show=False, save=True, save_path="plots/"+save_name+".png", plot_score=True)
    else:
        rrtstar.run()
    
    # set the path for the car to follow
    rrtpath = rrtstar.get_path()
    car  = Car(rrtpath[0, 0], rrtpath[0, 1], rrtpath[0, 2], rrtpath[:, 0], rrtpath[:, 1], rrtpath[:, 2], sim.dt)
    path = Path(rrtpath[:, 0], rrtpath[:, 1], rrtpath[:, 2])
    
    # create the moving obstacles
    moving_obstacles = [Obstacle(np.array([7,16]), np.array([1,-1.5]), 2), 
                        Obstacle(np.array([4,23]), np.array([1.2,-0.2]), 2),
                        Obstacle(np.array([40,30]), np.array([-1,1.2]), 2),
                        Obstacle(np.array([40,20]), np.array([1.5,0.3]), 2)]
    
    # create the agent for the car (used for velocity obstacles)
    agent = VelocityObstacle(position=np.array([path.px[0], path.py[0]]), 
                             velocity=np.array([0,0]), 
                             search_radius=8, 
                             preferred_velocity=np.array([0,5]), 
                             radius=2.2,
                             accel=[10,10])

    # create axis for plotting the simulation
    empty              = ([], [])
    target,            = ax.plot(*empty, '+r')
    car_outline,       = ax.plot(*empty, color=car.colour)
    front_right_wheel, = ax.plot(*empty, color=car.colour)
    rear_right_wheel,  = ax.plot(*empty, color=car.colour)
    front_left_wheel,  = ax.plot(*empty, color=car.colour)
    rear_left_wheel,   = ax.plot(*empty, color=car.colour)
    rear_axle,         = ax.plot(car.x, car.y, '+', color=car.colour, markersize=2)
    path_plot,         = ax.plot(*empty, color='gold')
    car_velocity       = ax.plot(*empty, color='blue', linewidth=3)
    annotation         = ax.annotate(f'{car.x:.1f}, {car.y:.1f}', xy=(car.x, car.y + 5), color='black', annotation_clip=False)
    moving_obstacles_plot = []
    for obstacle in moving_obstacles:
        moving_obstacles_plot.append(ax.plot(obstacle.position[0], obstacle.position[1], 'o', markersize=obstacle.radius*25, color = 'red'))

    fargs = [Fargs(
        ax=ax,
        sim=sim,
        path=path,
        car=car,
        car_outline=car_outline,
        front_right_wheel=front_right_wheel,
        front_left_wheel=front_left_wheel,
        rear_right_wheel=rear_right_wheel,
        rear_left_wheel=rear_left_wheel,
        rear_axle=rear_axle,
        car_velocity=car_velocity,
        annotation=annotation,
        target=target,
        path_plot=path_plot,
        moving_obstacles=moving_obstacles,
        moving_obstacles_plot=moving_obstacles_plot,
        agent=agent
    )]

    plt.grid()
    anim = FuncAnimation(fig, animate, frames=sim.frames, init_func=lambda: None, fargs=fargs, interval=interval, repeat=sim.loop)

    if sim.save:
        anim.save("results/simulation.gif", writer='imagemagick', fps=50)
    else:
        plt.show()
        
    
if __name__ == '__main__':
    show_simulation = True # set to true to show the simulation, false to save the animation
    
    main(show_simulation)
