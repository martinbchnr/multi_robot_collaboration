
# Node: prob_grid_updater

This node creates and updates the three probability grids that are needed to learn the probability distribution and construct the cumulative probability grid which is needed for the path planning.

prob_grids is an float64MultiArray which contains [Probability Grid, Cumulative Probability Grid, Expected num. dirts Grid] in that order. Each is of the dimension of the modified occupancy grid = dirt occ grid dimension.

It is important to note that the probabilty values are updated only when they are within the vision of either of the two robots. The CP and Exp grids are updated everywhere.

## Subscriptions:
- /prob_grids
- /dirt_occ_grid
- /tb3_0/current_vision
- /tb3_1/current_vision

## Publications:
- /prob_grids

## Initialisation:
All of this is conatained within the prob_grid_updater_node.cpp. One can alter the initialisation constant of the uniform probability distribution by changing the "prob_init_const" value in the first line of the spin() function. The other two grids are initialised as zero.
