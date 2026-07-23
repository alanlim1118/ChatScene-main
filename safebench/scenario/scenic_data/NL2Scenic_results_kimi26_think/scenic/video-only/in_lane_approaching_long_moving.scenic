"""Scenario Description:

In a top-down simulation view, the ego vehicle, depicted as a red rectangle, travels forward along a grey road marked with dashed white lane lines. The ego vehicle is following its lane and approaching a white and black striped vehicle located to its right, which moves from a blue area onto the main road surface, appearing to drive in the same lane or merge into the ego vehicle's path. Simultaneously, another white and black striped vehicle is visible in the lane to the left, moving parallel to the ego vehicle. The scene captures the ego vehicle navigating through traffic, closing the distance to the vehicle on the right as it proceeds down the road.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town03'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle traveling forward in its lane
ego = new Car with behavior FollowLaneBehavior(target_speed=Range(8, 12))

# Vehicle in the lane to the left, moving parallel to ego
new Car at ego offset by Range(-3.8, -3.2) @ Range(-2, 2),
    with behavior FollowLaneBehavior(target_speed=Range(8, 12))

# Vehicle to the right and ahead, merging into ego's path from the shoulder/blue area
new Car at ego offset by Range(2.5, 4.0) @ Range(15, 25),
    with behavior FollowLaneBehavior(target_speed=Range(6, 10))