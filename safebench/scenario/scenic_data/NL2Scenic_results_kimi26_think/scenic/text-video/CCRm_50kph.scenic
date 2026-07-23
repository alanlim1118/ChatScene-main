"""Scenario Description:

In a top-down simulated driving environment, a white vehicle travels forward along a grey road surface at a constant speed. A red vehicle enters the scene from the left, traveling in the same direction but at a higher velocity, rapidly closing the distance to the white vehicle. As the red vehicle catches up, it approaches the rear of the white vehicle. Finally, the front structure of the red vehicle strikes the rear structure of the white vehicle, resulting in a rear-end collision where the two rectangular vehicle representations overlap.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town05'
model scenic.simulators.carla.model
from scenic.domains.driving.controllers import *

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(5, 10)
param OPT_ADV_SPEED = Range(15, 25)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

behavior AdvBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a random drivable lane
lane = Uniform(*network.lanes)

# Ego spawn point on the lane centerline
egoSpawnPt = new OrientedPoint on lane.centerline

# Adversary spawn point: behind and to the left of the ego
advSpawnPt = new OrientedPoint at egoSpawnPt offset by (-Range(20, 40), Range(1, 2)),
    facing egoSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with color Color(1, 1, 1),
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED)

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with color Color(1, 0, 0),
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED)

require (distance from advSpawnPt to egoSpawnPt) > 10