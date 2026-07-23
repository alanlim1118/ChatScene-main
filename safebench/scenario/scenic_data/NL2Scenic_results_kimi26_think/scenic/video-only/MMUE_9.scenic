"""Scenario Description:

From a high-angle aerial view, a blue ego vehicle travels north on a straight urban road towards a T-junction, signaling an intent to turn left. As it approaches the intersection, adversary vehicles, including a grey sedan and a red car, are seen moving from right to left along the perpendicular cross-street. The ego vehicle slows to a stop at the junction line while the cross-traffic also halts, as all vehicles yield to allow a pedestrian to safely cross the road before any further movement occurs.

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

param OPT_EGO_SPEED = Range(5, 10)
param OPT_ADV_SPEED = Range(5, 10)
param OPT_PED_SPEED = Range(1, 3)
param OPT_PED_TRIGGER = Range(10, 20)

OPT_STOP_DISTANCE = 2

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior StopAtJunctionBehavior(speed, stop_reference, stop_distance):
    do FollowLaneBehavior(speed) until (distance from self to stop_reference) <= stop_distance
    take SetThrottleAction(0)
    take SetBrakeAction(1)
    do WaitBehavior()

behavior EgoStopAtJunctionBehavior(speed, trajectory, stop_reference, stop_distance):
    do FollowTrajectoryBehavior(speed, trajectory) until (distance from self to stop_reference) <= stop_distance
    take SetThrottleAction(0)
    take SetBrakeAction(1)
    do WaitBehavior()

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))

# Ego: left turn at T-junction
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Junction line point for ego (end of incoming lane)
egoJunctionPt = new OrientedPoint at egoInitLane.centerline.end,
    with heading egoInitLane.centerline.end.heading

# Adversary maneuver: straight on the perpendicular cross-street
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advSpawnPt1 = new OrientedPoint in advInitLane.centerline
advSpawnPt2 = new OrientedPoint ahead of advSpawnPt1 by -15

# Junction line point for adversaries
advJunctionPt = new OrientedPoint at advInitLane.centerline.end,
    with heading advInitLane.centerline.end.heading

# Pedestrian spawn point on the sidewalk near the junction line, crossing the road
pedSpawnPt = new OrientedPoint at egoInitLane.leftEdge.end,
    with heading egoInitLane.centerline.end.heading + 90 deg

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with color Color.blue,
    with blueprint EGO_MODEL,
    with regionContainedIn None,
    with behavior EgoStopAtJunctionBehavior(globalParameters.OPT_EGO_SPEED, egoTrajectory, egoJunctionPt, OPT_STOP_DISTANCE)

grey_sedan = new Car at advSpawnPt1,
    with color Color.grey,
    with heading advSpawnPt1.heading,
    with regionContainedIn None,
    with behavior StopAtJunctionBehavior(globalParameters.OPT_ADV_SPEED, advJunctionPt, OPT_STOP_DISTANCE)

red_car = new Car at advSpawnPt2,
    with color Color.red,
    with heading advSpawnPt2.heading,
    with regionContainedIn None,
    with behavior StopAtJunctionBehavior(globalParameters.OPT_ADV_SPEED, advJunctionPt, OPT_STOP_DISTANCE)

pedestrian = new Pedestrian at pedSpawnPt,
    with heading pedSpawnPt.heading,
    with regionContainedIn None,
    with behavior CrossingBehavior(ego, globalParameters.OPT_PED_SPEED, globalParameters.OPT_PED_TRIGGER)

require 20 <= (distance to intersection) <= 40
require 10 <= (distance from advSpawnPt1 to intersection) <= 30