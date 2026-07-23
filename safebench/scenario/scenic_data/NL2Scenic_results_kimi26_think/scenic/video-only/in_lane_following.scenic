"""Scenario Description:

In this top-down simulation view, the ego vehicle travels straight along a grey road marked with dashed white lane dividers. The ego vehicle is following a white vehicle with black stripes that is positioned ahead in the same lane, maintaining a consistent following distance as both vehicles proceed from right to left. The scene begins with a blue area visible on the right side of the screen, which disappears as the vehicles advance onto the uniform grey road surface, while telemetry data at the bottom indicates the ego vehicle's speed fluctuates between approximately 60 and 80 km/h.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town07'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

# Target speed range: 60-80 km/h converted to m/s
param OPT_SPEED = Range(16.67, 22.22)
param OPT_FOLLOW_DISTANCE = Range(20, 35)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle traveling along the straight road
ego = new Car with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_SPEED)

# Leading vehicle ahead in the same lane
# Modeled as a police car to reflect the white vehicle with black stripes
lead = new Car at ego offset by 0 @ globalParameters.OPT_FOLLOW_DISTANCE,
    with blueprint "vehicle.dodge_charger.police",
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_SPEED)