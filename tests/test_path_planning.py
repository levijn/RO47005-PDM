import test
import time

from src.RRT import RRT
from src.map import get_simple_map
from src.RRT_Star import RRT_Star
from src.RRT_Dubins import RRT_Dubins
from src.RRT_Star_Dubins import RRT_Star_Dubins
from src.dubins import Dubins
from src.create_environment import load_environment

def test_RRT(map, n_max, r_goal, min_dist_nodes, goal_sample_rate):
    planner = RRT(map, n_max, r_goal, min_dist_nodes, goal_sample_rate)
    start_time = time.time()
    planner.run()
    print("RRT")
    print("--- %s seconds ---" % (time.time() - start_time))
    planner.plot()

def test_RRT_Star(map, gamma, n_max, min_dist_nodes, goal_sample_rate):
    planner = RRT_Star(map, gamma, n_max, min_dist_nodes, goal_sample_rate)
    start_time = time.time()
    planner.run()
    print("RRT_Star")
    print("--- %s seconds ---" % (time.time() - start_time))
    planner.plot()

def test_RRT_Dubins(map, n_max, r_goal, min_dist_nodes, goal_sample_rate, dubins):
    planner = RRT_Dubins(map, n_max, r_goal, min_dist_nodes, goal_sample_rate, dubins)
    start_time = time.time()
    planner.run()
    print("RRT_Dubins")
    print("--- %s seconds ---" % (time.time() - start_time))
    planner.plot()

def test_RRT_Star_Dubins(map, gamma, n_max, min_dist_nodes, goal_sample_rate, dubins):
    planner = RRT_Star_Dubins(map, gamma, n_max, min_dist_nodes, goal_sample_rate, dubins)
    start_time = time.time()
    planner.run()
    print("RRT_Star_Dubins")
    print("--- %s seconds ---" % (time.time() - start_time))
    planner.plot()

if __name__ == '__main__':
    
    # map = get_simple_map()
    map = load_environment("map")
    
    # RRT, RRT_Star, RRT_Dubins, RRT_Star_Dubins
    n_max = 50 # Max Iterations
    r_goal = 0.5
    min_dist_nodes = 0
    goal_sample_rate = 50

    # RRT_Star, RRT_Star_Dubins
    gamma = 100

    # RRT_Dubins, RRT_Star_Dubins
    dubins = Dubins(3, 0.25)

    test_RRT(map, n_max, r_goal, min_dist_nodes, goal_sample_rate)
    test_RRT_Star(map, gamma, n_max, min_dist_nodes, goal_sample_rate)
    test_RRT_Dubins(map, n_max, r_goal, min_dist_nodes, goal_sample_rate, dubins)
    test_RRT_Star_Dubins(map, gamma, n_max, min_dist_nodes, goal_sample_rate, dubins)