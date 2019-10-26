
# Node: vision_determiner

This node determines the current vision of both of the robots. This vision cannot see in or through walls. Unlike the vision_pattern, the current_vision array has the size of the modified_occ_grid.

## Subscriptions:
- /tb3_0/amcl_pose
- /tb3_1/amcl_pose
- /modified_occupancy_grid
- /vision_pattern

## Publications:
- /tb3_0/current_vision
- /tb3_1/current_vision



