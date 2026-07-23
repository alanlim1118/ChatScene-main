"""Scenario Description:

The ego vehicle travels forward on a straight, paved road in a semi-rural area with buildings on the right and open land on the left under clear skies. As the vehicle nears an intersection, a silver minivan approaches from the right side road. The van proceeds to drive straight across the intersection without yielding, cutting directly in front of the ego vehicle. This action results in a sudden side-impact collision as the van obstructs the ego vehicle's path, forcing a hazardous interaction at the junction.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.mercedes.sprinter"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_ADV_SPEED = Range(8, 12)
param OPT_BRAKE_DIST = Range(10, 15)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior(speed, trajectory):
    try:
        do FollowTrajectoryBehavior(speed, trajectory)
    interrupt when (withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DIST)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 5 seconds
        abort
    terminate

behavior AdvBehavior(speed, trajectory):
    do FollowTrajectoryBehavior(speed, trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))

# Ego travels straight through the intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Adversary approaches from the cross road and goes straight across
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m.startLane != egoInitLane, intersection.maneuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

# Spawn points back from the intersection along the respective lanes
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, egoTrajectory)

AdvAgent = new Car at advSpawnPt,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with color Color(0.75, 0.75, 0.75),
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory)

require 30 <= (distance from ego to intersection) <= 60
require 30 <= (distance from AdvAgent to intersection) <= 60

terminate after 20 seconds