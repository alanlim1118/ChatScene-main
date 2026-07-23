"""Scenario Description:

A top-down schematic view illustrates a traffic scenario on a paved road with dashed white lane markings where a blue ego vehicle is positioned on the left side of the lane, traveling straight forward towards the right. Directly ahead in the same lane, a pink adversarial vehicle is positioned facing the same direction but is reversing backward, indicated by a left-pointing arrow, moving directly toward the approaching blue car. The scenario depicts a potential collision course as the forward-moving ego vehicle closes the distance to the reversing object ahead.

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

param OPT_EGO_SPEED = Range(5, 10)
param OPT_ADV_THROTTLE = Range(0.3, 0.6)
param OPT_LATERAL_OFFSET = Range(0.5, 1.0)
param OPT_INITIAL_DISTANCE = Range(20, 40)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    do FollowTrajectoryBehavior(trajectory=[lane], target_speed=globalParameters.OPT_EGO_SPEED)

behavior ReverseBehavior(throttle=0.5):
    take SetReverseAction(True)
    while True:
        take SetThrottleAction(throttle)

#################################
# SPATIAL RELATIONS             #
#################################

road = Uniform(*network.roads)
lane = Uniform(*road.lanes)

# Ego positioned on the left side of the lane
egoCenterPt = new OrientedPoint on lane.centerline
egoSpawnPt = new OrientedPoint at (egoCenterPt + (globalParameters.OPT_LATERAL_OFFSET @ (egoCenterPt.heading + 90 deg))),
    facing egoCenterPt.heading

# Adversarial vehicle directly ahead in the same lane, facing same direction
advSpawnPt = new OrientedPoint following lane.orientation from egoCenterPt for globalParameters.OPT_INITIAL_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with heading egoSpawnPt.heading,
    with blueprint EGO_MODEL,
    with color [0.1, 0.4, 0.9],
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with color [1, 0.4, 0.7],
    with behavior ReverseBehavior(throttle=globalParameters.OPT_ADV_THROTTLE)

require road.length > 60