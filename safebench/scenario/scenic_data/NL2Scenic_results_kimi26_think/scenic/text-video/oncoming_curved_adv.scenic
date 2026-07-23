"""Scenario Description:

The ego vehicle navigates a curved rural road lined with trees and a pile of wood debris on the left shoulder under overcast skies. As the vehicle approaches a bend, a white passenger microvan emerges from around a blind corner, drifting entirely into the oncoming lane and cutting off the ego vehicle's path. This results in a sudden head-on collision, causing the camera view to shift abruptly towards the right side of the road, capturing the roadside vegetation and a small structure as the event concludes.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town07'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town07'
param weather = "CloudyNoon"
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_SPEED = Range(8, 12)
MIN_ADV_DISTANCE = 40
MAX_ADV_DISTANCE = 70

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

behavior DriveStraightOncoming():
    # Drive straight towards ego in the oncoming lane
    while True:
        take SetThrottleAction(0.6), SetBrakeAction(0), SetSteerAction(0)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a curved rural road and lane for the ego
road = Uniform(*network.roads)
egoLane = Uniform(*road.lanes)

# Spawn point for ego
egoSpawnPt = new OrientedPoint in egoLane.centerline

# Spawn point for adversary ahead on the same lane (oncoming lane)
adversarySpawnPt = new OrientedPoint on egoLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with behavior EgoBehavior(EGO_SPEED)

# White passenger microvan in the oncoming lane
adversary = new Car at adversarySpawnPt,
    facing (adversarySpawnPt.heading + 180 deg),
    with blueprint "vehicle.volkswagen.t2",
    with color (1, 1, 1),
    with regionContainedIn None,
    with behavior DriveStraightOncoming()

# Ensure adversary is ahead of ego around the bend
require distance from ego to adversary > MIN_ADV_DISTANCE
require distance from ego to adversary < MAX_ADV_DISTANCE
require not (ego can see adversary)

terminate after 15 seconds