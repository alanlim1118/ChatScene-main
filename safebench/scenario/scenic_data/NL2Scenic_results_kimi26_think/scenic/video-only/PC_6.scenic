"""Scenario Description:

From a top-down perspective, the ego vehicle drives straight through a four-way intersection in a city setting characterized by tall glass and concrete buildings on the corners. As the ego vehicle traverses the junction, it navigates past traffic that includes vehicles entering from the perpendicular streets on the left and right, as well as an oncoming vehicle in the opposite lane, some of which appear to be executing turns. The scene also involves pedestrians crossing the street at the marked crosswalks, creating a dynamic environment where the ego vehicle must carefully coordinate its movement to safely clear the intersection and continue along the road.

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

param OPT_EGO_SPEED = Range(5, 15)
param OPT_TRAFFIC_SPEED = Range(5, 12)
param OPT_PED_SPEED = Range(0.5, 1.5)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.OPT_EGO_SPEED)

behavior TrafficBehavior(trajectory):
    do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=globalParameters.OPT_TRAFFIC_SPEED)

behavior PedestrianCrossingBehavior():
    do WalkForwardBehavior(globalParameters.OPT_PED_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way intersection
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego goes straight through the intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Oncoming vehicle from the opposite direction
oncomingManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
oncomingInitLane = oncomingManeuver.startLane
oncomingTrajectory = [oncomingInitLane, oncomingManeuver.connectingLane, oncomingManeuver.endLane]
oncomingSpawnPt = new OrientedPoint in oncomingInitLane.centerline

# Traffic entering from perpendicular streets on the left and right
perpManeuvers = list(filter(lambda m: m is not oncomingManeuver and m.type in (ManeuverType.STRAIGHT, ManeuverType.LEFT_TURN, ManeuverType.RIGHT_TURN), egoManeuver.conflictingManeuvers))
leftManeuver = Uniform(*perpManeuvers)
rightManeuver = Uniform(*filter(lambda m: m is not leftManeuver, perpManeuvers))

leftInitLane = leftManeuver.startLane
leftTrajectory = [leftInitLane, leftManeuver.connectingLane, leftManeuver.endLane]
leftSpawnPt = new OrientedPoint in leftInitLane.centerline

rightInitLane = rightManeuver.startLane
rightTrajectory = [rightInitLane, rightManeuver.connectingLane, rightManeuver.endLane]
rightSpawnPt = new OrientedPoint in rightInitLane.centerline

# Pedestrians at marked crosswalks near the intersection
pedSpawnPt1 = new OrientedPoint at intersection.center offset by (-5, 5), facing toward intersection.center
pedSpawnPt2 = new OrientedPoint at intersection.center offset by (5, -5), facing toward intersection.center

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle
ego = new Car at egoSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory)

# Oncoming vehicle
oncomingCar = new Car at oncomingSpawnPt,
    with heading oncomingSpawnPt.heading,
    with regionContainedIn None,
    with behavior TrafficBehavior(oncomingTrajectory)

# Left entering vehicle
leftCar = new Car at leftSpawnPt,
    with heading leftSpawnPt.heading,
    with regionContainedIn None,
    with behavior TrafficBehavior(leftTrajectory)

# Right entering vehicle
rightCar = new Car at rightSpawnPt,
    with heading rightSpawnPt.heading,
    with regionContainedIn None,
    with behavior TrafficBehavior(rightTrajectory)

# Pedestrians crossing the street
ped1 = new Pedestrian at pedSpawnPt1,
    with heading pedSpawnPt1.heading,
    with regionContainedIn None,
    with behavior PedestrianCrossingBehavior()

ped2 = new Pedestrian at pedSpawnPt2,
    with heading pedSpawnPt2.heading,
    with regionContainedIn None,
    with behavior PedestrianCrossingBehavior()

# Spatial requirements to ensure a dynamic intersection scenario
require 20 <= (distance from egoSpawnPt to intersection) <= 40
require 10 <= (distance from oncomingSpawnPt to intersection) <= 30
require 10 <= (distance from leftSpawnPt to intersection) <= 30
require 10 <= (distance from rightSpawnPt to intersection) <= 30