# RO47005 PDM Project - Group 5

## Introduction
This repository contains the source code used for the PDM project of group 5. 

## Contribution of others
- The creation of the [Dubins](https://github.com/FelicienC/RRT-Dubins/blob/master/code/dubins.py) paths (`src/dubins.py`)
- The [kinematic bicycle model](https://github.com/winstxnhdw/KinematicBicycleModel/blob/main/kinematic_model.py) (`src/kinematic_model.py`)
- The [stanley controller](https://github.com/winstxnhdw/FullStanleyController/blob/master/stanley_controller.py) for path tracking of the car (`src/stanley_controller.py`)
- The [simulation](https://github.com/winstxnhdw/KinematicBicycleModel/blob/main/animate.py) of the car (`src/simulation.py`)

## Our contribution
- Implementation of various RRT planners:
   - RRT planner with standard connector function (`src/RRT.py`)
   - RRT* planner with standard connector function (`src/RRT_Star.py`)
   - RRT planner with Dubins connector function (`src/RRT_Dubins.py`)
   - RRT* planner with Dubins connector function (`src/RRT_Star_Dubins.py`)
- Created a basic GUI using tkinter for creating 2D-environments with polygonal obstacles (`src/create_environment.py` & `src/create_polygons.py`)
- Collision detection of polygonal obstacles based on [Polygon/Point](https://www.jeffreythompson.org/collision-detection/poly-point.php) (`src/collision_detection.py`)
- Implementation of velocity obstacle (`src/VelocityObstacle.py`)

## How to use
1) Clone the repository in a directory of choice:

```terminal
git clone https://github.com/levijn/RO47005-PDM
```

2) Create a virtual environment (optional but recommended):
```terminal

```
3) Activate virtual environment:
```terminal

```

4) Install the required dependencies:

```terminal
pip install -r requirements.txt
```

## Run instructions

### Environment creator

```terminal
python src/create_environment.py
```

### Global planners

```terminal
python src/RRT_Star_Dubins.py
```

### Local planner

```terminal
python src/VelocityObstacle.py
```

### Simulation

```terminal
python src/simulation.py
```

## Future work

## Contact
