# Navigation Server

## Purpose
This node plans a path for the Lunabotics robot using A*.

## Inputs
- Start pose
- Goal pose
- Occupancy grid

## Outputs
- nav_msgs/Path
- waypoints
- bool if its making a path or not

## A* Behavior
- 8-connected movement
- Diagonal movement costs sqrt(2)
- Euclidean heuristic
- Turning penalty
- Obstacle proximity cost

## future
- publishes whether or not its making a new path
- when recieving a new map, consider whether or not it is different enough along the path to warrant a reroute
- recieve an input of where the bot currently is, so if we make a new path it starts where the robot is.
- factor in passability, because the obstacles wont be black and white they will be 0-256 (add threshold for not caring)