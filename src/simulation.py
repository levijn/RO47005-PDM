# pylint: skip-file
from csv import reader
from dataclasses import dataclass
from math import radians
import numpy as np

from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

from kinematic_model import KinematicBicycleModel
from cubic_spline_interpolator import generate_cubic_spline
from car_description import CarDescription
from stanley_controller import StanleyController

from map import Map, get_simple_map, get_simple_map_large, get_random_map, create_grid_map
from RRTs import RRTstar
from dubins import Dubins
from create_environment import load_environment
from RRT_Star_Dubins import RRT_Star_Dubins


class Simulation:

    def __init__(self):

        fps = 50.0

        self.dt = 1/fps
        self.map_size_x = 20 // 2
        self.map_size_y = 20 // 2
        self.frames = 600
        self.loop = False
        self.save = True


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
        target_velocity = 5.0
        self.time_to_reach_target_velocity = 5.0
        self.required_acceleration = target_velocity / self.time_to_reach_target_velocity

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

        self.time += self.delta_time
        return self.required_acceleration
    

    def plot_car(self):
        
        return self.description.plot_car(self.x, self.y, self.yaw, self.wheel_angle)


    def drive(self):
        
        
        # self.testindex += 1
        # if self.testindex/500 == 1:
        #     #shift the path to the right
        #     self.px += 5
        #TODO: here we can add changes in path by directly changing the self.px and self.py arrays
        
        acceleration = 0 if self.time > self.time_to_reach_target_velocity else self.get_required_acceleration()
        
        # calculates the steering angle for a given position and yaw
        self.wheel_angle, self.target_id, self.crosstrack_error = self.tracker.stanley_control(self.x, self.y, self.yaw, self.velocity, self.wheel_angle)
        
        self.x, self.y, self.yaw, self.velocity, _, _ = self.kinematic_bicycle_model.update(self.x, self.y, self.yaw, self.velocity, acceleration, self.wheel_angle)

        print(f"Cross-track term: {self.crosstrack_error}{' '*10}", end="\r")


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
    annotation: plt.Annotation
    target: plt.Line2D
    path_plot: plt.plot
    moving_obstacles: list
    moving_obstacles_plot: list
   

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
    annotation        = fargs.annotation
    target            = fargs.target
    path_plot         = fargs.path_plot
    moving_obstacles  = fargs.moving_obstacles
    moving_obstacles_plot = fargs.moving_obstacles_plot
    
    # Update moving obstacles
    for i, obstacle in enumerate(moving_obstacles):
        obstacle.update_position(sim.dt)
        moving_obstacles_plot[i][0].set_data(obstacle.position[0], obstacle.position[1])

    # Camera tracks car
    # ax.set_xlim(car.x - sim.map_size_x, car.x + sim.map_size_x)
    # ax.set_ylim(car.y - sim.map_size_y, car.y + sim.map_size_y)

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
    annotation.set_position((car.x, car.y + 5))

    plt.title(f'{sim.dt*frame:.2f}s', loc='right')
    plt.xlabel(f'Speed: {car.velocity:.2f} m/s', loc='left')
    # plt.savefig(f'image/visualisation_{frame:03}.png', dpi=300)

    return car_outline, front_right_wheel, rear_right_wheel, front_left_wheel, rear_left_wheel, rear_axle, target,


class MovingObstacle:
    def __init__(self, pos, vel, radius):
        self.position = pos
        self.velocity = vel
        self.radius = radius
        
    def update_position(self, dt):
        self.position = self.position + self.velocity * dt


def main():
    
    sim  = Simulation()
    
    #car  = Car(path.px[0], path.py[0], path.pyaw[0], path.px, path.py, path.pyaw, sim.dt)

    interval = sim.dt * 10**3

    fig = plt.figure()
    ax = plt.axes()
    ax.set_aspect('equal')
    
   

    # Draw map
    map_name = 'map_grid'
    # map1 = load_environment(map_name)
    map1 = create_grid_map()
    patches = map1.get_patches()
    for patch in patches:
        ax.add_patch(patch)
    
     #set the limits of the plot to map size
    ax.set_xlim(0, map1.size[0])
    ax.set_ylim(0, map1.size[1])

    # My code
    n_max = 600
    gamma = 600
    r_goal = 0.5
    min_dist_nodes = 0
    goal_sample_rate = 30
    dubins = Dubins(4, 0.3)
    
    save_name = f"sim_rrt_star_dubins_{map_name}_g_{gamma}_n_{n_max}"
    
    rrtstar = RRT_Star_Dubins(map1, gamma, n_max=n_max, min_dist_nodes=min_dist_nodes, goal_sample_rate=goal_sample_rate, dubins=dubins)
    rrtstar.create_intermediate_plots(show=False, save=True, save_path="plots/"+save_name+".png", plot_score=True)
    rrtpath = rrtstar.get_path()

    car  = Car(rrtpath[0, 0], rrtpath[0, 1], rrtpath[0, 2], rrtpath[:, 0], rrtpath[:, 1], rrtpath[:, 2], sim.dt)
    
    path = Path(rrtpath[:, 0], rrtpath[:, 1], rrtpath[:, 2])
    # ax.plot(rrtpath[:, 0], rrtpath[:, 1], 'r--')
    #ax.plot(path.px, path.py, '--', color='gold')
    
    moving_obstacles = [MovingObstacle(np.array([30,10]), np.array([2,-2]), 1)]
    moving_obstacles = []

    empty              = ([], [])
    target,            = ax.plot(*empty, '+r')
    car_outline,       = ax.plot(*empty, color=car.colour)
    front_right_wheel, = ax.plot(*empty, color=car.colour)
    rear_right_wheel,  = ax.plot(*empty, color=car.colour)
    front_left_wheel,  = ax.plot(*empty, color=car.colour)
    rear_left_wheel,   = ax.plot(*empty, color=car.colour)
    rear_axle,         = ax.plot(car.x, car.y, '+', color=car.colour, markersize=2)
    path_plot,         = ax.plot(*empty, color='gold')
    annotation         = ax.annotate(f'{car.x:.1f}, {car.y:.1f}', xy=(car.x, car.y + 5), color='black', annotation_clip=False)
    moving_obstacles_plot = []
    for obstacle in moving_obstacles:
        moving_obstacles_plot.append(ax.plot(obstacle.position[0], obstacle.position[1], 'o', color = 'red'))

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
        annotation=annotation,
        target=target,
        path_plot=path_plot,
        moving_obstacles=moving_obstacles,
        moving_obstacles_plot=moving_obstacles_plot
    )]

    plt.grid()
    anim = FuncAnimation(fig, animate, frames=sim.frames, init_func=lambda: None, fargs=fargs, interval=interval, repeat=sim.loop)

    if sim.save:
        anim.save("plots/"+save_name+".gif", writer='imagemagick', fps=50)
    else:
        plt.show()
        
    


if __name__ == '__main__':
    main()
