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

param EGO_SPEED = Range(3, 5)          # Constant speed for white (ego) vehicle
param ADV_SPEED = EGO_SPEED * Range(1.8, 2.5)  # Red vehicle significantly faster
param INITIAL_GAP = Range(40, 60)      # Initial distance behind ego for adversarial vehicle
param COLLISION_DIST = 2.0             # Distance threshold to detect rear-end collision

#################################
# MONITORS                      #
#################################

monitor CollisionMonitor():
    while True:
        if distance from AdvAgent to ego < COLLISION_DIST:
            record "Rear-end collision occurred"
            terminate
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    """White vehicle travels forward at constant speed."""
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

behavior AdvBehavior():
    """Red vehicle follows same lane at higher speed to cause rear-end collision."""
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight road segment for the scenario
roadSegment = Uniform(*network.roads)
lane = Uniform(*roadSegment.lanes)

# Place ego (white vehicle) on the lane
egoSpawnPt = new OrientedPoint in lane.centerline

# Place adversarial (red vehicle) behind ego in the same lane
advSpawnPt = new OrientedPoint in lane.centerline,
    offset by (-globalParameters.INITIAL_GAP, 0) relative to egoSpawnPt

#################################
# SCENARIO SPECIFICATION        #
#################################

require monitor CollisionMonitor()

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color (1.0, 1.0, 1.0),       # White vehicle
    with behavior EgoBehavior(),
    with regionContainedIn None

AdvAgent = new Car at advSpawnPt,
    with heading egoSpawnPt.heading,   # Same direction as ego
    with blueprint ADV_MODEL,
    with color (1.0, 0.0, 0.0),        # Red vehicle
    with behavior AdvBehavior(),
    with regionContainedIn None

# Ensure adversarial vehicle is behind ego and aligned in same direction
require distance from advSpawnPt to egoSpawnPt >= globalParameters.INITIAL_GAP - 5
require distance from advSpawnPt to egoSpawnPt <= globalParameters.INITIAL_GAP + 5
require abs(advSpawnPt.heading - egoSpawnPt.heading) < 5 deg