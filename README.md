# README #

ROS implementation of a global 2D path planner based on state lattices.

# Dependencies #

The package depends on [agv_control_msgs](https://bitbucket.org/larics/agv_control_msgs). Download agv_control_msgs into the same catkin workspace before building this package. 

## The pathPlanner node ##

The pathPlanner package provides path planning functionality, based on the state lattice approach, as described in Draganjac et. al. 2016, Decentralized Control of Multi-AGV Systems in Autonomous Warehousing Applications.

Subscribed topics: /tf (robot localization information), /goal (goal point)

Published topics: /plan (the path planned by the robot)

**WARNING:** Due to a limitation in the current planner implementation, it **requires the map origin to be at [0,0,0]**, otherwise the planner will fail to find feasible paths.

## Testing ##

Before using the node, it is necessary to download the lattice parameter files by running the `download_params.sh` script from the `params` folder.

To test the node use the provided `test.launch` file. You can set the pose of a dummy robot in RViz with the `2D Pose Estimate` tool and you can provide navigation goals with the `2D Nav Goal` tool. For distances greater than 10 meters, the planning can take quite a long time.

## Citing ##

If you find this package useful, please consider citing our work:
```
@ARTICLE{7571170,
author={I. Draganjac and D. Miklić and Z. Kovačić and G. Vasiljević and S. Bogdan},
journal={IEEE Transactions on Automation Science and Engineering},
title={Decentralized Control of Multi-AGV Systems in Autonomous Warehousing Applications},
year={2016},
volume={13},
number={4},
pages={1433-1447},
keywords={},
doi={10.1109/TASE.2016.2603781},
ISSN={1545-5955},
month={Oct},}
```