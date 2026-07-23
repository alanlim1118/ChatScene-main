"""Scenario Description:

The ego vehicle is positioned at a large intersection, preparing to execute a left turn, when a large blue and green dump truck enters the frame from the left, speeding through the intersection in violation of a red light. The truck crosses the ego vehicle's path and collides violently with the side of the car, filling the camera view with its cabin and chassis as it passes directly in front. Following the impact, the view stabilizes to show the truck has continued through the intersection, while a yellow construction crane truck is visible driving away in the distance on the cross street.

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
DUMP_TRUCK_MODEL = "vehicle.carlamotors.firetruck"  # Large truck blueprint; color set via paint
CRANE_MODEL = "vehicle.mercedes.sprinter"  # Construction-style vehicle blueprint

param OPT_EGO_SPEED = Range(2, 4)
param OPT_DUMP_TRUCK_SPEED = Range(12, 18)  # Speeding through red light
param OPT_CRANE_SPEED = Range(3, 6)
param OPT_BRAKE_DISTANCE = Range(5, 8)

CONST_LEFT_DEG = 90 deg
CONST_TOL_DEG = 20 deg
CONST_MIN_LEFT_DEG = CONST_LEFT_DEG - CONST_TOL_DEG
CONST_MAX_LEFT_DEG = CONST_LEFT_DEG + CONST_TOL_DEG

#################################
# MONITORS                      #
#################################

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(DumpTruck, 100):
            setClosestTrafficLightStatus(DumpTruck, "red")
        wait

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

behavior DumpTruckBehavior():
    """Speed through intersection ignoring red light, then continue straight."""
    do FollowTrajectoryBehavior(trajectory=dumpTrajectory, target_speed=globalParameters.OPT_DUMP_TRUCK_SPEED)
    terminate

behavior CraneBehavior():
    """Drive away on cross street in the distance."""
    do FollowTrajectoryBehavior(trajectory=craneTrajectory, target_speed=globalParameters.OPT_CRANE_SPEED)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Ego performs a left turn
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Dump truck goes straight from the left relative to ego (conflicting straight maneuver)
dumpManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
dumpTrajectory = [dumpManeuver.startLane, dumpManeuver.connectingLane, dumpManeuver.endLane]
dumpInitLane = dumpManeuver.startLane
dumpSpawnPt = new OrientedPoint in dumpInitLane.centerline

# Crane truck drives away on the cross street (same direction as dump truck end lane or adjacent)
craneEndLane = dumpManeuver.endLane
craneTrajectory = [craneEndLane]
craneSpawnPt = new OrientedPoint in craneEndLane.centerline,
    offset by Range(40, 60) along craneEndLane.centerline

egoDir = egoSpawnPt.heading
dumpDir = dumpSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

require monitor TrafficLights()

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

DumpTruck = new Car at dumpSpawnPt,
    with heading dumpSpawnPt.heading,
    with regionContainedIn None,
    with blueprint DUMP_TRUCK_MODEL,
    with color (0, 0.5, 1),  # Blue-green tint
    with behavior DumpTruckBehavior()

CraneTruck = new Car at craneSpawnPt,
    with heading craneSpawnPt.heading,
    with regionContainedIn None,
    with blueprint CRANE_MODEL,
    with color (1, 0.85, 0),  # Yellow construction color
    with behavior CraneBehavior()

# Ensure dump truck approaches from the left of ego
require CONST_MIN_LEFT_DEG < (egoDir - dumpDir) < CONST_MAX_LEFT_DEG

# Positioning constraints
require 25 <= (distance from egoSpawnPt to intersection) <= 40
require 15 <= (distance from dumpSpawnPt to intersection) <= 30