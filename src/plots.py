import matplotlib.pyplot as plt
import time

from create_environment import load_environment, plot_map
from RRT_Star_Dubins import RRT_Star_Dubins
from map import create_grid_map


def main():
    folder = "plots/"
    
    # first make a plot of the map that is used
    map_name = "map_grid"
    # env_map = load_environment(map_name)
    env_map = create_grid_map()
    plot_map(env_map, show=False, save=True, save_path=folder+map_name+".png")
    
    # settings for the RRT* algorithm
    gamma = 300
    n_max = 1200
    min_dist_nodes = 0
    goal_sample_rate = 30
    
    save_name = f"rrt_star_dubins_{map_name}_g_{gamma}_n_{n_max}"
    
    rrt = RRT_Star_Dubins(env_map, gamma, n_max=n_max, min_dist_nodes=min_dist_nodes, goal_sample_rate=goal_sample_rate)
    rrt.create_intermediate_plots(save=True, save_path=folder+save_name+".png", plot_score=True)
    
    # gamma optimization
    # gamma_list = [500, 750, 1000, 1250, 1500]
    # computation_time = []
    
    # for g in gamma_list:
    #     t_start = time.time()
    #     rrt = RRT_Star_Dubins(env_map, g, n_max=n_max, min_dist_nodes=min_dist_nodes, goal_sample_rate=goal_sample_rate)
    #     rrt.create_intermediate_plots(show=False, save=True, save_path=folder+"rrt_star_dubins_gamma_"+str(g)+".png")
    #     t_end = time.time()
    #     computation_time.append(t_end-t_start)
    
    # create computation time plot dependent on gamma
    # plt.figure()
    # plt.plot(gamma_list, computation_time)
    # plt.xlabel("gamma")
    # plt.ylabel("computation time [s]")
    # plt.title("computation time dependent on gamma")
    # plt.savefig(folder+"computation_time_gamma.png")
    # plt.show()
    
    
    # iterations time complexity
    # n_max_list = list(range(100, 1100, 200))
    # computation_time = []
    
    # for n in n_max_list:
    #     t_start = time.time()
    #     rrt = RRT_Star_Dubins(env_map, gamma, n_max=n, min_dist_nodes=min_dist_nodes, goal_sample_rate=goal_sample_rate)
    #     rrt.run()
    #     t_end = time.time()
    #     computation_time.append(t_end-t_start)
    
    # # create computation time plot dependent on n_max
    # plt.figure()
    # plt.plot(n_max_list, computation_time)
    # plt.xlabel("n_max")
    # plt.ylabel("computation time [s]")
    # plt.title("computation time dependent on n_max")
    # plt.savefig(folder+"computation_time_n_max.png")
    # plt.show()
    
    
    
        
    
    

if __name__ == '__main__':
    main()
