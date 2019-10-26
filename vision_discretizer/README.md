
# Node: vision_discretizer

This node discretizes the vision of a robot into a grid. Ones indicate where the robot can see and zeros where it cannot. The discretization is simply determined by testing weather the distance of a grid cell centre is below the sensing range of the robot. The size of the array is as small as possible such that it contains the whole circular sensing range of the robot. The physical resolution of the grid cells is equal to that of the modified_occ_grid.

## Subscriptions:
- /tb3_0/scan --  assumed to be the same for both robots
- /modified_occ_grid

## Publications:
- /vision_pattern

