"""Scenario Description:

The ego car travels straight ahead through the intersection. Along its path, it passes an adversarial object that is standing still.

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

param OPT_EGO_SPEED = Range(3, 6)
param OPT_BRAKE_DISTANCE = Range(8, 12)  # Distance at which ego brakes for static obstacle
param OPT_ADV_OFFSET = Range(-1.5, 1.5)  # Lateral offset of adversarial object from lane center

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (withinDistanceToObjsInLane(ego, globalParameters.OPT_BRAKE_DISTANCE)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        abort
    terminate

behavior StaticBehavior():
    """Adversarial object remains completely stationary."""
    while True:
        wait

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Ego goes straight through the intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversarial object is placed along the ego's path within the intersection area
advRegion = egoManeuver.connectingLane.region
advSpawnBase = new OrientedPoint in advRegion.centerline
advSpawnPt = new OrientedPoint at advSpawnBase.offsetRotated(0, globalParameters.OPT_ADV_OFFSET),
    with heading egoManeuver.connectingLane.centerline.headingAt(advSpawnBase.position)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior StaticBehavior()

# Ensure ego starts at a reasonable distance before the intersection
require 30 <= (distance from egoSpawnPt to intersection) <= 50

# Ensure the adversarial object is within the connecting lane of the intersection
require advSpawnPt in egoManeuver.connectingLane.region

# Ensure the adversarial object is ahead of the ego along the path
require (distance from egoSpawnPt to advSpawnPt) > 15